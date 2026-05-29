# 功能：
# 1. 打开、初始化 CANFD，并只使能 1~6 号关节电机
# 2. 启动 CANFD 连续收发线程
# 3. 启动 MIT 重力补偿线程，使机械臂可手拖示教
# 4. 记录示教轨迹，保存 CSV
# 5. 加载 CSV 轨迹
# 6. PV 模式自动回到轨迹起点
# 7. PV 模式按记录时间节拍复现轨迹
# 8. 支持轻微越界夹紧、轨迹限位汇总、实时状态显示、安全失能
from __future__ import annotations
import csv
import math
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from types import MethodType
from typing import Optional, Sequence, List
from PySide6.QtCore import QObject, Signal, QTimer, Qt
from PySide6.QtGui import QTextCursor
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)
from DMMotor import DMMotor
from Robot import Robot
from USBCANFD import USBCANFD
# =============================================================================
# 常量
# =============================================================================
MODE_MIT = 1
MODE_PV = 2

MODE_NAME = {
    MODE_MIT: "MIT",
    MODE_PV: "PV",
}

POWER_ON_WAIT_S = 1.0
MODE_SWITCH_TIMEOUT_S = 3.0
GRAVITY_COMP_PERIOD_S = 0.001

TRAJ_DIR = Path("trajectories")

DEFAULT_RECORD_HZ = 50.0
DEFAULT_REPLAY_SPEED_SCALE = 1.0
DEFAULT_REPLAY_POINT_STRIDE = 1
DEFAULT_PV_RETURN_TO_START_VEL = 0.25
DEFAULT_PV_RETURN_TIMEOUT_S = 30.0
DEFAULT_PV_POSITION_TOL = 0.035
DEFAULT_PV_LIMIT_SOFT_MARGIN = 0.02
DEFAULT_PV_REPLAY_VEL_MIN = 0.12
DEFAULT_PV_REPLAY_VEL_MAX = 0.80
DEFAULT_PV_VEL_MARGIN = 1.5

# 与之前命令行版保持一致：1、6轴默认不做重力补偿，2、3轴略放大，4、5轴为1
DEFAULT_GRAVITY_TORQUE_SCALE = [0.0, 1.15, 1.1, 1.0, 1.0, 0.0]

@dataclass
class Trajectory:
    path: Optional[Path]
    t: list[float]
    motor_q: list[list[float]]
    dh_q: list[list[float]]

    @property
    def size(self) -> int:
        return len(self.t)

    @property
    def duration(self) -> float:
        if len(self.t) < 2:
            return 0.0
        return self.t[-1] - self.t[0]

# =============================================================================
# 控制器：硬件、轨迹、示教与复现逻辑
# =============================================================================
class TeachReplayController(QObject):
    log_signal = Signal(str)
    trajectory_changed_signal = Signal(str)
    record_state_signal = Signal(bool)
    replay_confirm_request_signal = Signal(str)
    replay_state_signal = Signal(bool)

    def __init__(self):
        super().__init__()

        self.can: Optional[USBCANFD] = None
        self.robot: Optional[Robot] = None

        self.initialized = False
        self.current_mode: Optional[int] = None

        self.command_lock = threading.RLock()
        self.data_lock = threading.RLock()

        self.gravity_stop_event = threading.Event()
        self.gravity_thread: Optional[threading.Thread] = None

        self.record_stop_event = threading.Event()
        self.record_thread: Optional[threading.Thread] = None
        self.record_rows: list[dict[str, float]] = []
        self.recording = False

        self.loaded_traj: Optional[Trajectory] = None
        self.last_traj_path: Optional[Path] = self.get_latest_trajectory_file()

        self.replay_confirm_event = threading.Event()
        self.replay_cancel_event = threading.Event()
        self.replay_waiting_confirm = False
        self.replaying = False

        self.disable_on_exit = True
        self.require_replay_confirm = True
        self.clip_warning_once = True

        self.record_hz = DEFAULT_RECORD_HZ
        self.replay_speed_scale = DEFAULT_REPLAY_SPEED_SCALE
        self.replay_point_stride = DEFAULT_REPLAY_POINT_STRIDE
        self.pv_return_to_start_vel = DEFAULT_PV_RETURN_TO_START_VEL
        self.pv_return_timeout_s = DEFAULT_PV_RETURN_TIMEOUT_S
        self.pv_position_tol = DEFAULT_PV_POSITION_TOL
        self.pv_limit_soft_margin = DEFAULT_PV_LIMIT_SOFT_MARGIN
        self.pv_replay_vel_min = DEFAULT_PV_REPLAY_VEL_MIN
        self.pv_replay_vel_max = DEFAULT_PV_REPLAY_VEL_MAX
        self.pv_vel_margin = DEFAULT_PV_VEL_MARGIN
        self.gravity_torque_scale = list(DEFAULT_GRAVITY_TORQUE_SCALE)

        self._clip_warning_keys: set[tuple[int, str]] = set()

    # -------------------------------------------------------------------------
    # 通用日志与参数
    # -------------------------------------------------------------------------

    def log(self, msg: str):
        ts = time.strftime("%H:%M:%S")
        self.log_signal.emit(f"[{ts}] {msg}")

    def update_runtime_params(
        self,
        *,
        record_hz: float,
        replay_speed_scale: float,
        replay_point_stride: int,
        pv_return_to_start_vel: float,
        pv_return_timeout_s: float,
        pv_position_tol: float,
        pv_limit_soft_margin: float,
        pv_replay_vel_min: float,
        pv_replay_vel_max: float,
        pv_vel_margin: float,
        gravity_torque_scale: Sequence[float],
        disable_on_exit: bool,
        require_replay_confirm: bool,
        clip_warning_once: bool,
    ):
        self.record_hz = max(1.0, float(record_hz))
        self.replay_speed_scale = max(0.05, float(replay_speed_scale))
        self.replay_point_stride = max(1, int(replay_point_stride))
        self.pv_return_to_start_vel = max(0.01, float(pv_return_to_start_vel))
        self.pv_return_timeout_s = max(1.0, float(pv_return_timeout_s))
        self.pv_position_tol = max(0.001, float(pv_position_tol))
        self.pv_limit_soft_margin = max(0.0, float(pv_limit_soft_margin))
        self.pv_replay_vel_min = max(0.001, float(pv_replay_vel_min))
        self.pv_replay_vel_max = max(self.pv_replay_vel_min, float(pv_replay_vel_max))
        self.pv_vel_margin = max(1.0, float(pv_vel_margin))
        self.gravity_torque_scale = [float(x) for x in gravity_torque_scale[:6]]
        if len(self.gravity_torque_scale) < 6:
            self.gravity_torque_scale += [0.0] * (6 - len(self.gravity_torque_scale))
        self.disable_on_exit = bool(disable_on_exit)
        self.require_replay_confirm = bool(require_replay_confirm)
        self.clip_warning_once = bool(clip_warning_once)

    def get_latest_trajectory_file(self) -> Optional[Path]:
        if not TRAJ_DIR.exists():
            return None
        files = sorted(TRAJ_DIR.glob("teach_*.csv"), key=lambda p: p.stat().st_mtime, reverse=True)
        return files[0] if files else None

    # -------------------------------------------------------------------------
    # CANFD补丁：只操作前6个关节电机
    # -------------------------------------------------------------------------

    def patch_can_methods(self):
        assert self.can is not None

        def _fill_send_queue_patched(this) -> int:
            if this.mode_switch_flag != 0:
                for i, motor in enumerate(this.motors):
                    if this.mode_switch_flag == MODE_MIT:
                        cmd = motor.set_mit_command
                    elif this.mode_switch_flag == MODE_PV:
                        cmd = motor.set_pv_command
                    else:
                        cmd = motor.set_pv_command
                    this.canfd_queue[i] = this._new_canfd_frame(motor.PARAM_SET_ID, cmd, flags=0x11)
            else:
                for i, motor in enumerate(this.motors):
                    this.canfd_queue[i] = this._new_canfd_frame(motor.ID_OFFSET, motor.Command, flags=0x11)

            # 本界面只发送前6个关节电机，不操作第7个工具电机
            return this.MOTOR_NUM

        def _canfd_queue_send_thread_6motors(this) -> None:
            this.setQueueSend()
            this.clearQueueSend()
            frame = 0
            this.system_update = 0

            FrameArray = type(this.canfd_queue[0]) * this.MOTOR_NUM

            while this.is_updating:
                frame += 1
                this._fill_send_queue()

                frames = FrameArray(*this.canfd_queue[:this.MOTOR_NUM])
                count = this.MOTOR_NUM

                try:
                    ret = this._zcan.TransmitFD(this.channel_handle_, frames, count)
                except Exception:
                    ret = 0

                with this._lock:
                    this.send_err_num += max(0, count - int(ret))
                    this.send_suc_num += int(ret)
                    this.system_update = frame

                time.sleep(0)

        self.can._fill_send_queue = MethodType(_fill_send_queue_patched, self.can)
        self.can._canfd_queue_send_thread = MethodType(_canfd_queue_send_thread_6motors, self.can)

    # -------------------------------------------------------------------------
    # 初始化、使能、失能
    # -------------------------------------------------------------------------

    def initialize_system(self) -> bool:
        with self.command_lock:
            if self.initialized:
                self.log("[INFO] 系统已经初始化，无需重复初始化")
                return True

            self.can = USBCANFD()
            self.robot = Robot()
            self.patch_can_methods()

            self.log("[1] 打开 CANFD 设备...")
            if not self.can.open_device():
                self.log("[ERR] 打开 CANFD 设备失败")
                return False

            self.log("[2] 初始化 CANFD 设备...")
            if not self.can.init_device():
                self.log("[ERR] 初始化 CANFD 设备失败")
                self.can.close_device()
                return False

            self.log("[3] 启动 CANFD 通道...")
            if not self.can.start_device():
                self.log("[ERR] 启动 CANFD 通道失败")
                self.can.close_device()
                return False

            time.sleep(POWER_ON_WAIT_S)
            self.can.clearRecvBuffer()

            self.log("[4] 使能前 6 个关节电机...")
            if not self.enable_motors_only_before_thread():
                self.log("[ERR] 前 6 个关节电机使能失败")
                self.can.close_device()
                return False

            self.log("[5] 初始化 MIT 零力矩缓存...")
            self.set_all_mit_zero_torque()

            self.log("[6] 启动 CANFD 三线程...")
            self.can.start_can_thread(1)

            self.log("[7] 等待 1~6 号电机反馈...")
            self.wait_motor_feedback(timeout_s=2.0)

            self.log("[8] 启动重力补偿线程...")
            self.gravity_stop_event.clear()
            self.gravity_thread = threading.Thread(
                target=self.gravity_comp_loop,
                name="gravity_comp_loop",
                daemon=True,
            )
            self.gravity_thread.start()

            self.initialized = True
            self.current_mode = None

            self.log("[9] 自动切换到 MIT 示教模式...")
            if self.switch_mode(MODE_MIT):
                self.current_mode = MODE_MIT

            self.log("[OK] 初始化完成，可开始手拖示教和轨迹记录")
            self.log("[INFO] 本程序只操作 1~6 号关节电机，不操作第 7 个工具电机")
            return True

    def enable_motors_only_before_thread(self) -> bool:
        assert self.can is not None

        self.can.stop_can()
        self.can.clearRecvBuffer()

        for motor in self.can.motors:
            data = self.can.send_wait(1, motor.ID, DMMotor.clear_error_command, 100)
            if not motor.read_motor(data):
                self.log(f"[ERR] 电机 {motor.ID} 清错无有效回复")
                return False

            data = self.can.send_wait(1, motor.ID, DMMotor.enable_command, 100)
            if not motor.read_motor(data):
                self.log(f"[ERR] 电机 {motor.ID} 使能无有效回复")
                return False

            if not motor.Enable:
                self.log(f"[ERR] 电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}")
                return False

            self.log(f"[OK] 电机 {motor.ID} 已使能")

        return True

    def disable_motors_only_at_exit(self):
        if self.can is None:
            return

        self.can.stop_can()

        for motor in self.can.motors:
            try:
                data = self.can.send_wait(1, motor.ID, DMMotor.disable_command, 100)
                motor.read_motor(data)
                self.log(f"[EXIT] 电机 {motor.ID} 已发送失能命令，Enable={motor.Enable}, ERR={motor.ERRCODE}")
            except Exception as exc:
                self.log(f"[WARN] 电机 {motor.ID} 失能异常: {exc}")

    def disable_and_stop(self) -> bool:
        with self.command_lock:
            self.log("[SAFE] 准备停止复现/记录/重力补偿，并失能前 6 个电机")

            self.replay_cancel_event.set()
            self.stop_record_no_save()

            self.gravity_stop_event.set()
            if self.gravity_thread is not None and self.gravity_thread.is_alive():
                self.gravity_thread.join(timeout=1.0)

            try:
                self.set_all_mit_zero_torque()
                time.sleep(0.05)
            except Exception as exc:
                self.log(f"[WARN] 清零 MIT 力矩异常: {exc}")

            try:
                self.disable_motors_only_at_exit()
            except Exception as exc:
                self.log(f"[WARN] 失能电机异常: {exc}")

            try:
                if self.can is not None:
                    self.can.stop_can()
            except Exception as exc:
                self.log(f"[WARN] stop_can 异常: {exc}")

            self.initialized = False
            self.current_mode = None
            self.log("[OK] 已停止并失能")
            return True

    def cleanup(self):
        with self.command_lock:
            self.log("[CLEANUP] 程序退出清理...")

            self.replay_cancel_event.set()
            self.stop_record_no_save()

            self.gravity_stop_event.set()
            if self.gravity_thread is not None and self.gravity_thread.is_alive():
                self.gravity_thread.join(timeout=1.0)

            try:
                self.set_all_mit_zero_torque()
                time.sleep(0.05)
            except Exception as exc:
                self.log(f"[WARN] 退出时清零 MIT 力矩异常: {exc}")

            if self.disable_on_exit:
                try:
                    self.disable_motors_only_at_exit()
                except Exception as exc:
                    self.log(f"[WARN] 退出失能异常: {exc}")

            try:
                if self.can is not None:
                    self.can.stop_can()
                    self.can.close_device()
            except Exception as exc:
                self.log(f"[WARN] 关闭 CANFD 设备异常: {exc}")

            self.initialized = False
            self.current_mode = None
            self.log("[END] 清理完成")

    # -------------------------------------------------------------------------
    # 状态读取
    # -------------------------------------------------------------------------

    def get_status_snapshot(self) -> Optional[dict]:
        if self.can is None or self.robot is None:
            return None

        try:
            with self.data_lock:
                motors = []
                for m in self.can.motors:
                    motors.append({
                        "id": m.ID,
                        "mode": m.ModeName,
                        "enable": m.Enable,
                        "err": m.ERRCODE,
                        "pos": float(m.Position),
                        "vel": float(m.Velocity),
                        "tau": float(m.Torque),
                        "recv": int(m.recv_num),
                    })

                q_now = self.robot.motor2dh(self.can.motors)
                q_rad = [float(x) for x in q_now]
                q_deg = [float(x * 180.0 / math.pi) for x in q_now]

                traj_info = "无"
                if self.loaded_traj is not None:
                    traj_info = f"{self.loaded_traj.path} | 点数={self.loaded_traj.size}, 时长={self.loaded_traj.duration:.3f}s"
                elif self.last_traj_path is not None:
                    traj_info = str(self.last_traj_path)

                return {
                    "initialized": self.initialized,
                    "is_updating": bool(self.can.IsUpdating),
                    "current_mode": self.current_mode,
                    "motors": motors,
                    "dh_rad": q_rad,
                    "dh_deg": q_deg,
                    "can_param": list(self.can.CanParam),
                    "recording": self.recording,
                    "replaying": self.replaying,
                    "traj_info": traj_info,
                }
        except Exception as exc:
            self.log(f"[WARN] 状态读取失败: {exc}")
            return None

    def wait_motor_feedback(self, timeout_s: float = 2.0) -> bool:
        assert self.can is not None

        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if all(m.recv_num > 0 for m in self.can.motors):
                return True
            time.sleep(0.005)

        self.log(f"[WARN] 等待反馈超时，recv_num={[m.recv_num for m in self.can.motors]}")
        return False

    # -------------------------------------------------------------------------
    # 命令缓存与模式切换
    # -------------------------------------------------------------------------

    def set_all_mit_zero_torque(self):
        if self.can is None:
            return

        with self.data_lock:
            for motor in self.can.motors:
                motor.MIT.position_set = 0.0
                motor.MIT.velocity_set = 0.0
                motor.MIT.kp_set = 0.0
                motor.MIT.kd_set = 0.0
                motor.MIT.torque_set = 0.0
                motor.set_empty_command_MIT()
                if motor.Mode == MODE_MIT:
                    motor.set()

    @staticmethod
    def refresh_pv_command_cache(motor) -> None:
        if hasattr(motor, "_convert_to_candata_PV") and hasattr(motor, "_pv_command"):
            motor._pv_command = bytearray(
                motor._convert_to_candata_PV(motor.PV.position_set, motor.PV.velocity_lim)
            )
        else:
            if motor.Mode == MODE_PV:
                motor.set()

    def set_pv_hold_current_position(self, velocity_lim: float):
        assert self.can is not None

        with self.data_lock:
            for motor in self.can.motors:
                motor.PV.position_set = float(motor.Position)
                motor.PV.velocity_lim = float(velocity_lim)
                self.refresh_pv_command_cache(motor)

        self.log("[PV] 已准备当前位置保持命令")

    def should_print_clip_warning(self, motor_id: int, side: str) -> bool:
        if not self.clip_warning_once:
            return True

        key = (int(motor_id), str(side))
        if key in self._clip_warning_keys:
            return False

        self._clip_warning_keys.add(key)
        return True

    def sanitize_motor_target_to_limit(self, target_motor_q: Sequence[float]) -> Optional[list[float]]:
        assert self.can is not None

        if len(target_motor_q) != 6:
            self.log("[ERR] target_motor_q 必须是 6 个数")
            return None

        fixed_q: list[float] = []

        for i, motor in enumerate(self.can.motors):
            q = float(target_motor_q[i])
            lo, hi = float(motor.angle_lim[0]), float(motor.angle_lim[1])

            if lo <= q <= hi:
                fixed_q.append(q)
                continue

            if q < lo and (lo - q) <= self.pv_limit_soft_margin:
                if self.should_print_clip_warning(motor.ID, "low"):
                    self.log(
                        f"[WARN] 电机 {motor.ID} 目标角轻微低于下限，已夹到下限: "
                        f"raw={q:.4f}, fixed={lo:.4f}, limit=[{lo:.4f}, {hi:.4f}]"
                    )
                fixed_q.append(lo)
                continue

            if q > hi and (q - hi) <= self.pv_limit_soft_margin:
                if self.should_print_clip_warning(motor.ID, "high"):
                    self.log(
                        f"[WARN] 电机 {motor.ID} 目标角轻微高于上限，已夹到上限: "
                        f"raw={q:.4f}, fixed={hi:.4f}, limit=[{lo:.4f}, {hi:.4f}]"
                    )
                fixed_q.append(hi)
                continue

            self.log(
                f"[ERR] 电机 {motor.ID} 目标角明显超限: "
                f"target={q:.4f}, limit=[{lo:.4f}, {hi:.4f}], "
                f"soft_margin={self.pv_limit_soft_margin:.4f}"
            )
            return None

        return fixed_q

    def set_pv_target_motor_position(
        self,
        target_motor_q: Sequence[float],
        velocity_lim: float | Sequence[float],
    ) -> bool:
        assert self.can is not None

        fixed_target_q = self.sanitize_motor_target_to_limit(target_motor_q)
        if fixed_target_q is None:
            return False

        if isinstance(velocity_lim, (list, tuple)):
            vel_list = [float(v) for v in velocity_lim]
            if len(vel_list) != 6:
                self.log("[ERR] velocity_lim 如果是列表，必须是 6 个数")
                return False
        else:
            vel_list = [float(velocity_lim)] * 6

        with self.data_lock:
            for i, motor in enumerate(self.can.motors):
                motor.PV.position_set = float(fixed_target_q[i])
                motor.PV.velocity_lim = float(vel_list[i])
                self.refresh_pv_command_cache(motor)

        return True

    def switch_mode(self, target_mode: int) -> bool:
        assert self.can is not None

        if target_mode not in (MODE_MIT, MODE_PV):
            self.log(f"[ERR] 不支持的模式: {target_mode}")
            return False

        if not self.can.IsUpdating:
            self.log("[ERR] CANFD 连续收发线程未启动，无法在线切换模式")
            return False

        target_name = MODE_NAME[target_mode]

        if target_mode == MODE_MIT:
            self.log("[PREPARE] 切换 MIT：先清零 MIT 力矩缓存，随后由重力补偿线程写入 MIT 力矩")
            self.set_all_mit_zero_torque()

        elif target_mode == MODE_PV:
            self.log("[PREPARE] 切换 PV：先准备当前位置保持命令")
            self.set_pv_hold_current_position(self.pv_return_to_start_vel)

        for i in range(self.can.MOTOR_NUM):
            self.can.motor_mode[i] = 0

        self.log(f"[SWITCH] 在线切换到 {target_name} 模式，不失能、不停止三线程")
        self.can.mode_switch_flag = target_mode

        deadline = time.time() + MODE_SWITCH_TIMEOUT_S
        while time.time() < deadline:
            modes = [m.Mode for m in self.can.motors]
            if self.can.mode_switch_flag == 0 and all(m == target_mode for m in modes):
                self.current_mode = target_mode
                self.log(f"[OK] 已切换到 {target_name} 模式")
                return True
            time.sleep(0.01)

        self.log(
            f"[ERR] 切换到 {target_name} 模式超时，"
            f"modes={[m.Mode for m in self.can.motors]}, flag={self.can.mode_switch_flag}"
        )
        self.can.mode_switch_flag = 0
        return False

    def switch_to_mit_teach_mode(self) -> bool:
        with self.command_lock:
            if not self.initialized or self.can is None:
                self.log("[ERR] 系统未初始化")
                return False
            return self.switch_mode(MODE_MIT)

    # -------------------------------------------------------------------------
    # 重力补偿线程
    # -------------------------------------------------------------------------

    def gravity_comp_loop(self):
        assert self.can is not None
        assert self.robot is not None

        self.log("[GRAVITY] 重力补偿线程已启动")

        while not self.gravity_stop_event.is_set():
            try:
                with self.data_lock:
                    self.robot.Angle = self.robot.motor2dh(self.can.motors)

                    if not self.robot.set_robot():
                        time.sleep(GRAVITY_COMP_PERIOD_S)
                        continue

                    tau_g_motor = self.robot.Tau_G_Motor

                    # 只有在MIT模式下才更新MIT命令，避免PV复现时覆盖PV命令
                    if all(m.Mode == MODE_MIT for m in self.can.motors):
                        for i, motor in enumerate(self.can.motors):
                            motor.MIT.position_set = 0.0
                            motor.MIT.velocity_set = 0.0
                            motor.MIT.kp_set = 0.0
                            motor.MIT.kd_set = 0.0
                            motor.MIT.torque_set = float(tau_g_motor[i] * self.gravity_torque_scale[i])
                            motor.set()

                time.sleep(GRAVITY_COMP_PERIOD_S)

            except Exception as exc:
                self.log(f"[ERR] 重力补偿线程异常: {exc}")
                time.sleep(0.01)

        self.log("[GRAVITY] 重力补偿线程退出")

    # -------------------------------------------------------------------------
    # 轨迹记录
    # -------------------------------------------------------------------------

    def start_record(self) -> bool:
        with self.command_lock:
            if not self.initialized or self.can is None or self.robot is None:
                self.log("[ERR] 系统未初始化，无法记录")
                return False

            if self.recording:
                self.log("[WARN] 当前已经在记录")
                return False

            if not all(m.Mode == MODE_MIT for m in self.can.motors):
                self.log("[INFO] 当前不是 MIT 模式，先切换到 MIT 示教模式")
                if not self.switch_mode(MODE_MIT):
                    return False

            self.record_rows = []
            self.record_stop_event.clear()

            self.record_thread = threading.Thread(
                target=self.record_worker,
                name="record_worker",
                daemon=True,
            )
            self.recording = True
            self.record_state_signal.emit(True)
            self.record_thread.start()

            self.log("[RECORD] 开始记录轨迹，现在可以手拖机械臂示教")
            return True

    def record_worker(self):
        assert self.can is not None
        assert self.robot is not None

        t0 = time.perf_counter()
        next_t = t0
        sample_idx = 0
        period = 1.0 / max(self.record_hz, 1.0)

        while not self.record_stop_event.is_set():
            now = time.perf_counter()
            if now < next_t:
                time.sleep(min(0.001, next_t - now))
                continue

            with self.data_lock:
                t_rel = now - t0
                motor_q = [float(m.Position) for m in self.can.motors]
                dh_q = [float(x) for x in self.robot.motor2dh(self.can.motors)]

            row: dict[str, float] = {
                "sample": float(sample_idx),
                "t": float(t_rel),
            }

            for i in range(6):
                row[f"motor_{i + 1}"] = motor_q[i]
            for i in range(6):
                row[f"dh_{i + 1}"] = dh_q[i]

            self.record_rows.append(row)

            sample_idx += 1
            next_t += period

        self.recording = False
        self.record_state_signal.emit(False)
        self.log(f"[RECORD] 停止记录，共 {len(self.record_rows)} 个采样点")

    def stop_record_and_save(self) -> bool:
        with self.command_lock:
            if not self.recording:
                self.log("[WARN] 当前没有正在记录的轨迹")
                return False

            self.record_stop_event.set()
            if self.record_thread is not None and self.record_thread.is_alive():
                self.record_thread.join(timeout=2.0)

            path = self.save_trajectory(self.record_rows)
            if path is None:
                return False

            self.load_trajectory(path)
            return True

    def stop_record_no_save(self):
        if self.recording:
            self.record_stop_event.set()
            if self.record_thread is not None and self.record_thread.is_alive():
                self.record_thread.join(timeout=1.0)
        self.recording = False
        self.record_state_signal.emit(False)

    def save_trajectory(self, rows: list[dict[str, float]], path: Optional[Path] = None) -> Optional[Path]:
        if not rows:
            self.log("[WARN] 没有轨迹点，不保存")
            return None

        TRAJ_DIR.mkdir(parents=True, exist_ok=True)

        if path is None:
            path = TRAJ_DIR / f"teach_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

        fieldnames = ["sample", "t"]
        fieldnames += [f"motor_{i + 1}" for i in range(6)]
        fieldnames += [f"dh_{i + 1}" for i in range(6)]

        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        duration = rows[-1]["t"] - rows[0]["t"] if len(rows) >= 2 else 0.0
        self.log(f"[SAVE] 已保存轨迹: {path}")
        self.log(f"[SAVE] 点数={len(rows)}, 时长={duration:.3f}s, 采样频率约={len(rows) / max(duration, 1e-6):.1f}Hz")
        return path

    # -------------------------------------------------------------------------
    # 轨迹加载
    # -------------------------------------------------------------------------

    def load_trajectory(self, path: str | Path) -> Optional[Trajectory]:
        try:
            path = Path(path)

            t: list[float] = []
            motor_q: list[list[float]] = []
            dh_q: list[list[float]] = []

            with path.open("r", newline="", encoding="utf-8") as f:
                reader = csv.DictReader(f)

                for row in reader:
                    t.append(float(row["t"]))
                    motor_q.append([float(row[f"motor_{i + 1}"]) for i in range(6)])

                    if all(f"dh_{i + 1}" in row for i in range(6)):
                        dh_q.append([float(row[f"dh_{i + 1}"]) for i in range(6)])
                    else:
                        dh_q.append([0.0] * 6)

            if len(t) < 2:
                raise ValueError("轨迹点数不足，无法复现")

            traj = Trajectory(path=path, t=t, motor_q=motor_q, dh_q=dh_q)
            self.loaded_traj = traj
            self.last_traj_path = path

            self.log(f"[LOAD] 已加载轨迹: {path}")
            self.log(f"[LOAD] 点数={traj.size}, 时长={traj.duration:.3f}s")
            self.trajectory_changed_signal.emit(f"{path} | 点数={traj.size}, 时长={traj.duration:.3f}s")
            return traj

        except Exception as exc:
            self.log(f"[ERR] 加载轨迹失败: {exc}")
            return None

    def load_latest_trajectory(self) -> bool:
        path = self.get_latest_trajectory_file()
        if path is None:
            self.log("[ERR] 未找到最近轨迹文件")
            return False
        return self.load_trajectory(path) is not None

    # -------------------------------------------------------------------------
    # PV复现
    # -------------------------------------------------------------------------

    def current_motor_q(self) -> list[float]:
        assert self.can is not None
        with self.data_lock:
            return [float(m.Position) for m in self.can.motors]

    @staticmethod
    def max_abs_error(a: Sequence[float], b: Sequence[float]) -> float:
        return max(abs(float(x) - float(y)) for x, y in zip(a, b))

    def wait_until_motor_close(self, target_motor_q: Sequence[float]) -> bool:
        assert self.can is not None

        deadline = time.time() + self.pv_return_timeout_s
        while time.time() < deadline:
            if self.replay_cancel_event.is_set():
                self.log("[REPLAY] 等待回起点过程中收到取消信号")
                return False

            err = self.max_abs_error(self.current_motor_q(), target_motor_q)
            if err <= self.pv_position_tol:
                self.log(f"[PV] 已接近目标，max_err={err:.4f} rad")
                return True

            time.sleep(0.02)

        err = self.max_abs_error(self.current_motor_q(), target_motor_q)
        self.log(f"[WARN] 等待到位超时，max_err={err:.4f} rad")
        return False

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    def compute_segment_velocity_limits(
        self,
        q_prev: Sequence[float],
        q_next: Sequence[float],
        dt: float,
    ) -> list[float]:
        assert self.can is not None

        dt = max(float(dt), 0.005)
        vel_list: list[float] = []

        for i, motor in enumerate(self.can.motors):
            required = abs(float(q_next[i]) - float(q_prev[i])) / dt * self.pv_vel_margin
            motor_limit = min(self.pv_replay_vel_max, float(motor.max_velocity) * 0.5)
            vel_list.append(self.clamp(required, self.pv_replay_vel_min, motor_limit))

        return vel_list

    def print_trajectory_limit_summary(self, points: Sequence[Sequence[float]]) -> None:
        assert self.can is not None

        if not points:
            return

        any_clip = False
        self._clip_warning_keys.clear()

        for i, motor in enumerate(self.can.motors):
            lo, hi = float(motor.angle_lim[0]), float(motor.angle_lim[1])
            low_values = [float(p[i]) for p in points if float(p[i]) < lo]
            high_values = [float(p[i]) for p in points if float(p[i]) > hi]

            if low_values:
                any_clip = True
                min_raw = min(low_values)
                max_exceed = lo - min_raw
                level = "WARN" if max_exceed <= self.pv_limit_soft_margin else "ERR"
                self.log(
                    f"[TRAJ {level}] 电机 {motor.ID} 有 {len(low_values)} 个轨迹点低于下限，"
                    f"min_raw={min_raw:.4f}, limit_low={lo:.4f}, max_exceed={max_exceed:.4f} rad"
                )

            if high_values:
                any_clip = True
                max_raw = max(high_values)
                max_exceed = max_raw - hi
                level = "WARN" if max_exceed <= self.pv_limit_soft_margin else "ERR"
                self.log(
                    f"[TRAJ {level}] 电机 {motor.ID} 有 {len(high_values)} 个轨迹点高于上限，"
                    f"max_raw={max_raw:.4f}, limit_high={hi:.4f}, max_exceed={max_exceed:.4f} rad"
                )

        if any_clip:
            self.log("[TRAJ INFO] 轻微越界点会被夹到软件限位边界；越界点很多时建议重新示教或检查限位")

    def move_to_trajectory_start(self) -> bool:
        with self.command_lock:
            if not self.initialized or self.can is None:
                self.log("[ERR] 系统未初始化")
                return False

            if self.loaded_traj is None:
                if self.last_traj_path is not None:
                    self.load_trajectory(self.last_traj_path)
                if self.loaded_traj is None:
                    self.log("[ERR] 没有可用轨迹，请先记录或加载轨迹")
                    return False

            points = self.loaded_traj.motor_q[::max(1, self.replay_point_stride)]
            if len(points) < 2:
                self.log("[ERR] 轨迹点数不足")
                return False

            first_q = points[0]

            self.print_trajectory_limit_summary(points)

            self.log("[REPLAY] 准备切换到 PV 模式并回到轨迹起点")
            if not self.switch_mode(MODE_PV):
                return False

            self.log("[REPLAY] 发送轨迹起点 PV 目标")
            if not self.set_pv_target_motor_position(first_q, self.pv_return_to_start_vel):
                return False

            return self.wait_until_motor_close(first_q)

    def replay_loaded_trajectory(self) -> bool:
        with self.command_lock:
            if not self.initialized or self.can is None:
                self.log("[ERR] 系统未初始化")
                return False

            if self.loaded_traj is None:
                if self.last_traj_path is not None:
                    self.load_trajectory(self.last_traj_path)
                if self.loaded_traj is None:
                    self.log("[ERR] 没有可用轨迹，请先记录或加载轨迹")
                    return False

            traj = self.loaded_traj
            points = traj.motor_q[::max(1, self.replay_point_stride)]
            times = traj.t[::max(1, self.replay_point_stride)]

            if len(points) < 2:
                self.log("[ERR] 跳点后轨迹点数不足")
                return False

            self.replaying = True
            self.replay_state_signal.emit(True)
            self.replay_cancel_event.clear()
            self.replay_confirm_event.clear()
            self.replay_waiting_confirm = False

            try:
                self.print_trajectory_limit_summary(points)

                self.log("[REPLAY] 准备切换到 PV 模式")
                if not self.switch_mode(MODE_PV):
                    return False

                first_q = points[0]
                self.log("[REPLAY] 先移动到记录轨迹起点")
                if not self.set_pv_target_motor_position(first_q, self.pv_return_to_start_vel):
                    return False

                arrived = self.wait_until_motor_close(first_q)
                if not arrived:
                    self.log("[ERR] 机械臂未能在规定时间内回到轨迹起点，取消复现")
                    return False

                if self.require_replay_confirm:
                    self.replay_waiting_confirm = True
                    self.replay_confirm_request_signal.emit("已回到轨迹起点。确认周围安全后，点击“确认开始播放”。")
                    self.log("[REPLAY] 等待界面确认开始播放...")

                    while not self.replay_confirm_event.is_set():
                        if self.replay_cancel_event.is_set():
                            self.log("[REPLAY] 复现已取消")
                            return False
                        time.sleep(0.05)

                    self.replay_waiting_confirm = False

                self.log(f"[REPLAY] 开始 PV 复现，点数={len(points)}, 原始轨迹时长={times[-1] - times[0]:.3f}s")

                t_replay_start = time.perf_counter()
                t0 = times[0]

                for idx, q in enumerate(points):
                    if self.replay_cancel_event.is_set():
                        self.log("[REPLAY] 收到取消信号，停止复现")
                        return False

                    if idx == 0:
                        vel_lim = [self.pv_return_to_start_vel] * 6
                    else:
                        raw_dt = max(times[idx] - times[idx - 1], 0.001)
                        dt = raw_dt / max(self.replay_speed_scale, 1e-6)
                        vel_lim = self.compute_segment_velocity_limits(points[idx - 1], q, dt)

                    if not self.set_pv_target_motor_position(q, vel_lim):
                        self.log(f"[ERR] 第 {idx} 个轨迹点超限或写入失败，中止复现")
                        return False

                    if idx + 1 < len(points):
                        next_elapsed = (times[idx + 1] - t0) / max(self.replay_speed_scale, 1e-6)
                        sleep_until = t_replay_start + next_elapsed
                        while True:
                            if self.replay_cancel_event.is_set():
                                self.log("[REPLAY] 收到取消信号，停止复现")
                                return False
                            remain = sleep_until - time.perf_counter()
                            if remain <= 0:
                                break
                            time.sleep(min(0.002, remain))

                    # 每大约1秒打印一次进度
                    denom = max(1, int(self.record_hz / max(1, self.replay_point_stride)))
                    if idx % denom == 0:
                        self.log(f"[REPLAY] {idx + 1}/{len(points)}")

                self.set_pv_target_motor_position(points[-1], self.pv_return_to_start_vel)
                self.current_mode = MODE_PV
                self.log("[REPLAY] 轨迹复现完成，PV 模式保持最后一个点")
                return True

            finally:
                self.replay_waiting_confirm = False
                self.replaying = False
                self.replay_state_signal.emit(False)

    def confirm_replay_start(self):
        if self.replay_waiting_confirm:
            self.log("[UI] 已确认开始播放轨迹")
            self.replay_confirm_event.set()

    def cancel_replay(self):
        if self.replaying or self.replay_waiting_confirm:
            self.log("[UI] 请求取消复现")
            self.replay_cancel_event.set()
            self.replay_confirm_event.set()


# =============================================================================
# PySide6界面
# =============================================================================

class MainWindow(QMainWindow):
    command_done_signal = Signal(str, bool)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("机械臂示教记录与PV轨迹复现界面")
        self.resize(1380, 850)

        self.controller = TeachReplayController()
        self.controller.log_signal.connect(self.append_log)
        self.controller.trajectory_changed_signal.connect(self.on_trajectory_changed)
        self.controller.record_state_signal.connect(self.on_record_state_changed)
        self.controller.replay_confirm_request_signal.connect(self.on_replay_confirm_request)
        self.controller.replay_state_signal.connect(self.on_replay_state_changed)
        self.command_done_signal.connect(self.on_command_done)

        self.command_running = False

        self._build_ui()

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_status)
        self.timer.start(200)

        if self.controller.last_traj_path is not None:
            self.traj_label.setText(f"最近轨迹：{self.controller.last_traj_path}")

    # -------------------------------------------------------------------------
    # UI构建
    # -------------------------------------------------------------------------

    def _build_ui(self):
        central = QWidget()
        main_layout = QVBoxLayout(central)

        splitter = QSplitter(Qt.Vertical)

        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)

        left_panel = QWidget()
        left_layout = QVBoxLayout(left_panel)

        self._build_device_group(left_layout)
        self._build_teach_group(left_layout)
        self._build_replay_group(left_layout)
        self._build_param_group(left_layout)
        self._build_option_group(left_layout)
        left_layout.addStretch(1)

        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        self._build_status_group(right_layout)

        top_layout.addWidget(left_panel, 0)
        top_layout.addWidget(right_panel, 1)

        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)

        splitter.addWidget(top_widget)
        splitter.addWidget(self.log_box)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 2)

        main_layout.addWidget(splitter)
        self.setCentralWidget(central)

    def _build_device_group(self, parent_layout):
        group = QGroupBox("设备与安全")
        layout = QGridLayout(group)

        self.btn_init = QPushButton("初始化并进入MIT示教")
        self.btn_safe_stop = QPushButton("安全失能并停止")
        self.btn_status = QPushButton("刷新状态")
        self.btn_exit = QPushButton("退出程序")

        self.btn_init.clicked.connect(lambda: self.run_async("初始化", self.controller.initialize_system))
        self.btn_safe_stop.clicked.connect(lambda: self.run_async("安全失能", self.controller.disable_and_stop))
        self.btn_status.clicked.connect(self.refresh_status)
        self.btn_exit.clicked.connect(self.close)

        layout.addWidget(self.btn_init, 0, 0, 1, 2)
        layout.addWidget(self.btn_safe_stop, 1, 0, 1, 2)
        layout.addWidget(self.btn_status, 2, 0)
        layout.addWidget(self.btn_exit, 2, 1)

        parent_layout.addWidget(group)

    def _build_teach_group(self, parent_layout):
        group = QGroupBox("MIT示教与轨迹记录")
        layout = QGridLayout(group)

        self.btn_mit = QPushButton("切换到MIT示教模式")
        self.btn_start_record = QPushButton("开始记录轨迹")
        self.btn_stop_record = QPushButton("停止记录并保存")
        self.btn_stop_record.setEnabled(False)

        self.btn_mit.clicked.connect(lambda: self.run_async("切换MIT", self.controller.switch_to_mit_teach_mode))
        self.btn_start_record.clicked.connect(self.start_record_clicked)
        self.btn_stop_record.clicked.connect(lambda: self.run_async("停止记录并保存", self.controller.stop_record_and_save))

        layout.addWidget(self.btn_mit, 0, 0, 1, 2)
        layout.addWidget(self.btn_start_record, 1, 0)
        layout.addWidget(self.btn_stop_record, 1, 1)

        parent_layout.addWidget(group)

    def _build_replay_group(self, parent_layout):
        group = QGroupBox("PV轨迹复现")
        layout = QGridLayout(group)

        self.btn_load_latest = QPushButton("加载最近轨迹")
        self.btn_load_file = QPushButton("选择CSV轨迹")
        self.btn_move_start = QPushButton("PV回到轨迹起点")
        self.btn_replay = QPushButton("PV复现轨迹")
        self.btn_confirm_replay = QPushButton("确认开始播放")
        self.btn_cancel_replay = QPushButton("取消复现")

        self.btn_confirm_replay.setEnabled(False)
        self.btn_cancel_replay.setEnabled(False)

        self.btn_load_latest.clicked.connect(lambda: self.run_async("加载最近轨迹", self.controller.load_latest_trajectory))
        self.btn_load_file.clicked.connect(self.load_file_clicked)
        self.btn_move_start.clicked.connect(lambda: self.run_async("回到轨迹起点", self.controller.move_to_trajectory_start))
        self.btn_replay.clicked.connect(lambda: self.run_async("PV复现轨迹", self.controller.replay_loaded_trajectory))
        self.btn_confirm_replay.clicked.connect(self.confirm_replay_clicked)
        self.btn_cancel_replay.clicked.connect(self.controller.cancel_replay)

        layout.addWidget(self.btn_load_latest, 0, 0)
        layout.addWidget(self.btn_load_file, 0, 1)
        layout.addWidget(self.btn_move_start, 1, 0)
        layout.addWidget(self.btn_replay, 1, 1)
        layout.addWidget(self.btn_confirm_replay, 2, 0)
        layout.addWidget(self.btn_cancel_replay, 2, 1)

        self.traj_label = QLabel("轨迹：无")
        self.traj_label.setWordWrap(True)
        layout.addWidget(self.traj_label, 3, 0, 1, 2)

        parent_layout.addWidget(group)

    def _build_param_group(self, parent_layout):
        group = QGroupBox("记录/复现参数")
        layout = QGridLayout(group)

        row = 0

        layout.addWidget(QLabel("记录频率 Hz"), row, 0)
        self.record_hz_spin = QDoubleSpinBox()
        self.record_hz_spin.setRange(1.0, 500.0)
        self.record_hz_spin.setDecimals(1)
        self.record_hz_spin.setSingleStep(10.0)
        self.record_hz_spin.setValue(DEFAULT_RECORD_HZ)
        layout.addWidget(self.record_hz_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("复现速度倍率"), row, 0)
        self.replay_speed_spin = QDoubleSpinBox()
        self.replay_speed_spin.setRange(0.05, 5.0)
        self.replay_speed_spin.setDecimals(2)
        self.replay_speed_spin.setSingleStep(0.1)
        self.replay_speed_spin.setValue(DEFAULT_REPLAY_SPEED_SCALE)
        layout.addWidget(self.replay_speed_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("轨迹跳点 stride"), row, 0)
        self.stride_spin = QSpinBox()
        self.stride_spin.setRange(1, 20)
        self.stride_spin.setValue(DEFAULT_REPLAY_POINT_STRIDE)
        layout.addWidget(self.stride_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("回起点速度 rad/s"), row, 0)
        self.return_vel_spin = QDoubleSpinBox()
        self.return_vel_spin.setRange(0.01, 5.0)
        self.return_vel_spin.setDecimals(3)
        self.return_vel_spin.setSingleStep(0.05)
        self.return_vel_spin.setValue(DEFAULT_PV_RETURN_TO_START_VEL)
        layout.addWidget(self.return_vel_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("回起点超时 s"), row, 0)
        self.return_timeout_spin = QDoubleSpinBox()
        self.return_timeout_spin.setRange(1.0, 120.0)
        self.return_timeout_spin.setDecimals(1)
        self.return_timeout_spin.setSingleStep(1.0)
        self.return_timeout_spin.setValue(DEFAULT_PV_RETURN_TIMEOUT_S)
        layout.addWidget(self.return_timeout_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("到位误差 rad"), row, 0)
        self.position_tol_spin = QDoubleSpinBox()
        self.position_tol_spin.setRange(0.001, 0.5)
        self.position_tol_spin.setDecimals(4)
        self.position_tol_spin.setSingleStep(0.005)
        self.position_tol_spin.setValue(DEFAULT_PV_POSITION_TOL)
        layout.addWidget(self.position_tol_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("限位软容差 rad"), row, 0)
        self.soft_margin_spin = QDoubleSpinBox()
        self.soft_margin_spin.setRange(0.0, 0.2)
        self.soft_margin_spin.setDecimals(4)
        self.soft_margin_spin.setSingleStep(0.005)
        self.soft_margin_spin.setValue(DEFAULT_PV_LIMIT_SOFT_MARGIN)
        layout.addWidget(self.soft_margin_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("复现最小速度"), row, 0)
        self.replay_vel_min_spin = QDoubleSpinBox()
        self.replay_vel_min_spin.setRange(0.001, 5.0)
        self.replay_vel_min_spin.setDecimals(3)
        self.replay_vel_min_spin.setSingleStep(0.05)
        self.replay_vel_min_spin.setValue(DEFAULT_PV_REPLAY_VEL_MIN)
        layout.addWidget(self.replay_vel_min_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("复现最大速度"), row, 0)
        self.replay_vel_max_spin = QDoubleSpinBox()
        self.replay_vel_max_spin.setRange(0.001, 10.0)
        self.replay_vel_max_spin.setDecimals(3)
        self.replay_vel_max_spin.setSingleStep(0.05)
        self.replay_vel_max_spin.setValue(DEFAULT_PV_REPLAY_VEL_MAX)
        layout.addWidget(self.replay_vel_max_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("速度裕量"), row, 0)
        self.vel_margin_spin = QDoubleSpinBox()
        self.vel_margin_spin.setRange(1.0, 5.0)
        self.vel_margin_spin.setDecimals(2)
        self.vel_margin_spin.setSingleStep(0.1)
        self.vel_margin_spin.setValue(DEFAULT_PV_VEL_MARGIN)
        layout.addWidget(self.vel_margin_spin, row, 1)

        row += 1
        layout.addWidget(QLabel("重力补偿比例"), row, 0, 1, 2)

        self.gravity_scale_spins: list[QDoubleSpinBox] = []
        for i, value in enumerate(DEFAULT_GRAVITY_TORQUE_SCALE):
            r = row + 1 + i // 2
            c = (i % 2) * 2
            layout.addWidget(QLabel(f"J{i + 1}"), r, c)
            spin = QDoubleSpinBox()
            spin.setRange(-5.0, 5.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.05)
            spin.setValue(float(value))
            layout.addWidget(spin, r, c + 1)
            self.gravity_scale_spins.append(spin)

        parent_layout.addWidget(group)

    def _build_option_group(self, parent_layout):
        group = QGroupBox("选项")
        layout = QVBoxLayout(group)

        self.check_disable_exit = QCheckBox("退出程序时失能前6个电机")
        self.check_disable_exit.setChecked(True)

        self.check_replay_confirm = QCheckBox("复现前回到起点后需要人工确认")
        self.check_replay_confirm.setChecked(True)

        self.check_clip_once = QCheckBox("轻微越界夹紧提示只显示一次")
        self.check_clip_once.setChecked(True)

        layout.addWidget(self.check_disable_exit)
        layout.addWidget(self.check_replay_confirm)
        layout.addWidget(self.check_clip_once)

        parent_layout.addWidget(group)

    def _build_status_group(self, parent_layout):
        group = QGroupBox("实时状态")
        layout = QVBoxLayout(group)

        self.mode_label = QLabel("当前目标模式：未知")
        self.thread_label = QLabel("CANFD线程：未启动")
        self.record_label = QLabel("记录状态：未记录")
        self.replay_label = QLabel("复现状态：未复现")

        layout.addWidget(self.mode_label)
        layout.addWidget(self.thread_label)
        layout.addWidget(self.record_label)
        layout.addWidget(self.replay_label)

        self.table = QTableWidget(6, 10)
        self.table.setHorizontalHeaderLabels([
            "ID", "Mode", "Enable", "ERR", "Pos(rad)", "Vel", "Torque", "Recv", "DH(rad)", "DH(deg)"
        ])
        self.table.verticalHeader().setVisible(False)

        for r in range(6):
            for c in range(10):
                item = QTableWidgetItem("")
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(r, c, item)

        layout.addWidget(self.table)

        self.can_param_label = QLabel("CAN参数：")
        self.can_param_label.setWordWrap(True)
        layout.addWidget(self.can_param_label)

        parent_layout.addWidget(group)

    # -------------------------------------------------------------------------
    # UI事件
    # -------------------------------------------------------------------------

    def append_log(self, msg: str):
        self.log_box.append(msg)
        self.log_box.moveCursor(QTextCursor.End)

    def update_controller_params(self):
        self.controller.update_runtime_params(
            record_hz=float(self.record_hz_spin.value()),
            replay_speed_scale=float(self.replay_speed_spin.value()),
            replay_point_stride=int(self.stride_spin.value()),
            pv_return_to_start_vel=float(self.return_vel_spin.value()),
            pv_return_timeout_s=float(self.return_timeout_spin.value()),
            pv_position_tol=float(self.position_tol_spin.value()),
            pv_limit_soft_margin=float(self.soft_margin_spin.value()),
            pv_replay_vel_min=float(self.replay_vel_min_spin.value()),
            pv_replay_vel_max=float(self.replay_vel_max_spin.value()),
            pv_vel_margin=float(self.vel_margin_spin.value()),
            gravity_torque_scale=[float(s.value()) for s in self.gravity_scale_spins],
            disable_on_exit=self.check_disable_exit.isChecked(),
            require_replay_confirm=self.check_replay_confirm.isChecked(),
            clip_warning_once=self.check_clip_once.isChecked(),
        )

    def set_buttons_enabled(self, enabled: bool):
        for btn in [
            self.btn_init,
            self.btn_safe_stop,
            self.btn_status,
            self.btn_exit,
            self.btn_mit,
            self.btn_start_record,
            self.btn_load_latest,
            self.btn_load_file,
            self.btn_move_start,
            self.btn_replay,
        ]:
            btn.setEnabled(enabled)

        # 记录停止按钮和确认/取消按钮由状态单独控制
        self.btn_stop_record.setEnabled(self.controller.recording)
        self.btn_confirm_replay.setEnabled(self.controller.replay_waiting_confirm)
        self.btn_cancel_replay.setEnabled(self.controller.replaying or self.controller.replay_waiting_confirm)

    def run_async(self, label: str, func):
        if self.command_running:
            self.append_log("[WARN] 上一个命令仍在执行，请稍后再操作")
            return

        self.update_controller_params()

        self.command_running = True
        self.set_buttons_enabled(False)

        def worker():
            ok = False
            try:
                ok = bool(func())
            except Exception as exc:
                self.controller.log(f"[ERR] {label}异常: {exc}")
                ok = False
            finally:
                self.command_done_signal.emit(label, ok)

        threading.Thread(target=worker, name=f"cmd_{label}", daemon=True).start()

    def on_command_done(self, label: str, ok: bool):
        self.command_running = False
        self.set_buttons_enabled(True)
        self.append_log(f"[DONE] {label} {'成功' if ok else '失败'}")
        self.refresh_status()

    def start_record_clicked(self):
        if self.command_running:
            self.append_log("[WARN] 上一个命令仍在执行，请稍后再操作")
            return

        self.update_controller_params()
        ok = self.controller.start_record()
        self.btn_start_record.setEnabled(not ok)
        self.btn_stop_record.setEnabled(ok)

    def load_file_clicked(self):
        filename, _ = QFileDialog.getOpenFileName(
            self,
            "选择CSV轨迹文件",
            str(TRAJ_DIR),
            "CSV Files (*.csv);;All Files (*)",
        )

        if not filename:
            return

        def command():
            return self.controller.load_trajectory(filename) is not None

        self.run_async("加载轨迹", command)

    def confirm_replay_clicked(self):
        self.controller.confirm_replay_start()
        self.btn_confirm_replay.setEnabled(False)

    def on_trajectory_changed(self, text: str):
        self.traj_label.setText(f"轨迹：{text}")

    def on_record_state_changed(self, recording: bool):
        self.record_label.setText(f"记录状态：{'记录中' if recording else '未记录'}")
        self.btn_start_record.setEnabled(not recording and not self.command_running)
        self.btn_stop_record.setEnabled(recording)

    def on_replay_confirm_request(self, msg: str):
        self.append_log(f"[CONFIRM] {msg}")
        self.btn_confirm_replay.setEnabled(True)
        self.btn_cancel_replay.setEnabled(True)
        QMessageBox.information(self, "轨迹复现确认", msg)

    def on_replay_state_changed(self, replaying: bool):
        self.replay_label.setText(f"复现状态：{'复现中' if replaying else '未复现'}")
        self.btn_cancel_replay.setEnabled(replaying or self.controller.replay_waiting_confirm)
        self.btn_confirm_replay.setEnabled(self.controller.replay_waiting_confirm)

    def refresh_status(self):
        snapshot = self.controller.get_status_snapshot()
        if snapshot is None:
            return

        mode = snapshot.get("current_mode", None)
        if mode is None:
            self.mode_label.setText("当前目标模式：未知")
        else:
            self.mode_label.setText(f"当前目标模式：{MODE_NAME.get(mode, mode)}")

        self.thread_label.setText(f"CANFD线程：{'运行中' if snapshot.get('is_updating') else '未运行'}")
        self.record_label.setText(f"记录状态：{'记录中' if snapshot.get('recording') else '未记录'}")
        self.replay_label.setText(f"复现状态：{'复现中' if snapshot.get('replaying') else '未复现'}")

        traj_info = snapshot.get("traj_info", "无")
        self.traj_label.setText(f"轨迹：{traj_info}")

        motors = snapshot.get("motors", [])
        dh_rad = snapshot.get("dh_rad", [0.0] * 6)
        dh_deg = snapshot.get("dh_deg", [0.0] * 6)

        for r in range(min(6, len(motors))):
            m = motors[r]
            values = [
                str(m["id"]),
                str(m["mode"]),
                str(m["enable"]),
                str(m["err"]),
                f"{m['pos']:.4f}",
                f"{m['vel']:.4f}",
                f"{m['tau']:.4f}",
                str(m["recv"]),
                f"{dh_rad[r]:.4f}",
                f"{dh_deg[r]:.2f}",
            ]
            for c, text in enumerate(values):
                self.table.item(r, c).setText(text)

        can_param = snapshot.get("can_param", [])
        if can_param:
            self.can_param_label.setText(
                "CAN参数：" + " | ".join(f"{i}:{float(v):.2f}" for i, v in enumerate(can_param))
            )

    def closeEvent(self, event):
        if self.command_running:
            QMessageBox.warning(self, "提示", "当前命令仍在执行，请等待完成后再退出。")
            event.ignore()
            return

        reply = QMessageBox.question(
            self,
            "确认退出",
            "是否退出程序？\n如果勾选了“退出程序时失能前6个电机”，程序会先尝试失能电机。",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )

        if reply != QMessageBox.Yes:
            event.ignore()
            return

        self.timer.stop()

        try:
            self.update_controller_params()
            self.controller.cleanup()
        except Exception as exc:
            self.append_log(f"[WARN] 退出清理异常: {exc}")

        event.accept()


def main():
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
