import threading
import time
from typing import List, Optional

import zlgcan
from motor_class import DMMotor   # 导入已有的电机类

STATUS_OK = zlgcan.ZCAN_STATUS_OK
TYPE_CAN = zlgcan.ZCAN_TYPE_CAN
TYPE_CANFD = zlgcan.ZCAN_TYPE_CANFD
DEVICE_TYPE_USBCANFD_MINI = zlgcan.ZCAN_USBCANFD_MINI


class USBCANFD:
    def __init__(self):
        self.zlg = zlgcan.ZCAN()
        self.device_handle = None
        self.channel_handle = None
        self.device_type = DEVICE_TYPE_USBCANFD_MINI
        self.channel_index = 0

        self.MOTOR_NUM = 6
        self.TOOL_NUM = 1
        self.SLAVE_NUM = self.MOTOR_NUM + self.TOOL_NUM

        self.tool_update = 1
        self.is_open = False
        self.is_updating = False
        self.can_param_time = 200   # ms
        self.can_param = [0.0] * 7

        self.send_suc_num = 0
        self.send_err_num = 0
        self.recv_num = 0
        self.system_update = 0

        # 创建电机对象（参数：id, 型号, 初始模式）
        self.motors = [None] * self.MOTOR_NUM
        for i in range(3):
            self.motors[i] = DMMotor(i + 1, 4340, "PV")
        for i in range(3, 6):
            self.motors[i] = DMMotor(i + 1, 4310, "PV")
        # 设置角度限制（与 C# 原代码一致）
        self.motors[1].angle_lim = [0, 213 * 3.1415926 / 180]
        self.motors[2].angle_lim = [0, 182 * 3.1415926 / 180]
        self.motors[4].angle_lim = [-84 * 3.1415926 / 180, 98 * 3.1415926 / 180]

        self.tools = [DMMotor(7, 3507, "PVT")]

        # CANFD 发送队列（复用 zlgcan 定义的结构体）
        self.canfd_queue = [zlgcan.ZCAN_TransmitFD_Data() for _ in range(self.SLAVE_NUM)]
        self.canfd_send_data = zlgcan.ZCAN_TransmitFD_Data()
        self.canfd_send_data.frame.len = 8
        self.canfd_send_data.transmit_type = 1
        self.canfd_send_data.frame.flags = 0x01

        for i in range(self.MOTOR_NUM):
            self._init_queue_item(self.canfd_queue[i], self.motors[i].ID_OFFSET, self.motors[i].Command)
        for i in range(self.TOOL_NUM):
            idx = self.MOTOR_NUM + i
            self._init_queue_item(self.canfd_queue[idx], self.tools[i].ID_OFFSET, self.tools[i].Command)

        self.motor_mode = [0] * self.MOTOR_NUM
        self.motor_lock = [False] * self.MOTOR_NUM
        self.mode_switch_flag = 0

        self.recv_thread = None
        self.send_thread = None
        self.param_thread = None

    def _init_queue_item(self, item, can_id, cmd):
        item.transmit_type = 1
        item.frame.len = 8
        item.frame.can_id = can_id
        item.frame.flags = 0x11
        item.frame._res0 = 1
        item.frame._res1 = 0
        for j, val in enumerate(cmd[:8]):
            item.frame.data[j] = val

    # ---------- 设备操作 ----------
    def open_device(self) -> bool:
        self.device_handle = self.zlg.OpenDevice(self.device_type, 0, 0)
        if self.device_handle is None or self.device_handle == 0:
            print("无法打开设备")
            return False
        self.is_open = True
        return True

    def close_device(self) -> bool:
        self.stop_can()
        if self.is_open and self.device_handle:
            ret = self.zlg.CloseDevice(self.device_handle)
            if ret == STATUS_OK:
                self.is_open = False
        return not self.is_open

    def init_device(self) -> bool:
        if not self._set_canfd_standard(0):
            print("设置CANFD标准失败")
            return False
        if not self._set_custom_baudrate("1.0Mbps(75%),5.0Mbps(75%),(60,00000E2B,00800001)"):
            print("设置波特率失败")
            return False

        config = zlgcan.ZCAN_CHANNEL_INIT_CONFIG()
        config.can_type = TYPE_CANFD
        config.config.canfd.mode = 0
        self.channel_handle = self.zlg.InitCAN(self.device_handle, self.channel_index, config)
        if self.channel_handle is None or self.channel_handle == 0:
            print("初始化CAN失败")
            return False

        if not self._set_resistance_enable(True):
            print("使能终端电阻失败")
            return False
        if not self._set_filter():
            print("滤波设置失败")
            return False
        if self.zlg.ClearBuffer(self.channel_handle) != STATUS_OK:
            print("清空缓冲区失败")
            return False

        self.zlg.ZCAN_SetValue(self.device_handle, "0/set_device_tx_echo", b"0")
        return True

    def start_device(self) -> bool:
        if self.zlg.StartCAN(self.channel_handle) != STATUS_OK:
            print("启动CAN失败")
            return False
        return True

    def canfd_send(self, can_id: int, data: bytes) -> bool:
        self.canfd_send_data.frame.can_id = can_id
        for i, val in enumerate(data[:8]):
            self.canfd_send_data.frame.data[i] = val
        ret = self.zlg.TransmitFD(self.channel_handle, self.canfd_send_data, 1)
        return ret == 1

    def can_send(self, can_id: int, data: bytes) -> bool:
        can_data = zlgcan.ZCAN_Transmit_Data()
        can_data.frame.can_id = self._make_can_id(can_id, 0, 0, 0)
        can_data.frame.can_dlc = 8
        can_data.transmit_type = 1
        for i, val in enumerate(data[:8]):
            can_data.frame.data[i] = val
        ret = self.zlg.Transmit(self.channel_handle, can_data, 1)
        return ret == 1

    # ---------- 线程控制 ----------
    def start_can_thread(self, type_: int):
        self.is_updating = True
        if type_ == 0:
            self.recv_thread = threading.Thread(target=self._can_receive_thread, name="can_receive_thread")
        else:
            self.recv_thread = threading.Thread(target=self._canfd_receive_thread, name="canfd_receive_thread")
        self.recv_thread.daemon = True
        self.recv_thread.start()

        self.send_thread = threading.Thread(target=self._canfd_queue_send_thread, name="canfd_queue_send_thread")
        self.send_thread.daemon = True
        self.send_thread.start()

        self.param_thread = threading.Thread(target=self._can_param_update_thread, name="can_param_update_thread")
        self.param_thread.daemon = True
        self.param_thread.start()

    def stop_can(self):
        self.is_updating = False
        if self.recv_thread and self.recv_thread.is_alive():
            self.recv_thread.join(timeout=1.0)
        if self.send_thread and self.send_thread.is_alive():
            self.send_thread.join(timeout=1.0)
        if self.param_thread and self.param_thread.is_alive():
            self.param_thread.join(timeout=1.0)

    # ---------- 接收线程 ----------
    def _can_receive_thread(self):
        while self.is_updating:
            num = self.zlg.GetReceiveNum(self.channel_handle, TYPE_CAN)
            if num > 0:
                msgs, recv_len = self.zlg.Receive(self.channel_handle, min(num, 100), 50)
                for i in range(recv_len):
                    self._can_data_proc(msgs[i])
            time.sleep(0.001)

    def _canfd_receive_thread(self):
        while self.is_updating:
            num = self.zlg.GetReceiveNum(self.channel_handle, TYPE_CANFD)
            if num > 0:
                msgs, recv_len = self.zlg.ReceiveFD(self.channel_handle, min(num, 100), 50)
                for i in range(recv_len):
                    self._canfd_data_proc(msgs[i])
            time.sleep(0.001)

    def _canfd_data_proc(self, canfd_data: zlgcan.ZCAN_ReceiveFD_Data) -> bool:
        self.recv_num += 1
        # 处理模式切换反馈
        if self.mode_switch_flag != 0:
            idx = (canfd_data.frame.data[0] & 0x0F) - 1
            if 0 <= idx < self.MOTOR_NUM:
                if self.motors[idx].get_motor_mode(canfd_data.frame.data[:8]):
                    self.motor_mode[idx] = self.motors[idx].Mode
                    if all(m == self.motor_mode[0] for m in self.motor_mode):
                        self.mode_switch_flag = 0
                        print("模式切换完成")
            return True

        if 0x11 <= canfd_data.frame.can_id <= 0x16:
            idx = (canfd_data.frame.data[0] & 0x0F) - 1
            if 0 <= idx < self.MOTOR_NUM:
                self.motors[idx].read_motor(canfd_data.frame.data[:8])
            return True
        elif canfd_data.frame.can_id == 0x17:
            self.tools[0].read_motor(canfd_data.frame.data[:8])
            return True
        elif canfd_data.frame.can_id == 0x31:
            # 预留
            return True
        return False

    def _can_data_proc(self, can_data: zlgcan.ZCAN_Receive_Data):
        if 0x10 <= can_data.frame.can_id <= 0x16:
            idx = (can_data.frame.data[0] & 0x0F) - 1
            if 0 <= idx < self.MOTOR_NUM:
                self.motors[idx].read_motor(can_data.frame.data[:8])

    # ---------- 发送线程（队列批量发送）----------
    def _canfd_queue_send_thread(self):
        self._set_queue_send()
        self._clear_queue_send()
        frame_cnt = 0
        self.system_update = 0

        while self.is_updating:
            frame_cnt += 1

            # 模式切换指令
            if self.mode_switch_flag != 0:
                for i in range(self.MOTOR_NUM):
                    self.canfd_queue[i].transmit_type = 1
                    self.canfd_queue[i].frame.len = 8
                    self.canfd_queue[i].frame.can_id = self.motors[i].PARAM_SET_ID
                    cmd = self.motors[i].set_mit_command if self.mode_switch_flag == 1 else self.motors[i].set_pv_command
                    for j, val in enumerate(cmd[:8]):
                        self.canfd_queue[i].frame.data[j] = val
            else:
                for i in range(self.MOTOR_NUM):
                    self.canfd_queue[i].transmit_type = 1
                    self.canfd_queue[i].frame.len = 8
                    self.canfd_queue[i].frame.can_id = self.motors[i].ID_OFFSET
                    for j, val in enumerate(self.motors[i].Command[:8]):
                        self.canfd_queue[i].frame.data[j] = val

            # 工具电机数据
            for i in range(self.TOOL_NUM):
                idx = self.MOTOR_NUM + i
                self.canfd_queue[idx].transmit_type = 1
                self.canfd_queue[idx].frame.len = 8
                self.canfd_queue[idx].frame.can_id = self.tools[i].ID_OFFSET
                for j, val in enumerate(self.tools[i].Command[:8]):
                    self.canfd_queue[idx].frame.data[j] = val

            total = self.SLAVE_NUM if (frame_cnt % self.tool_update == 0) else self.MOTOR_NUM

            # 创建连续结构体数组并批量发送
            msg_array = (zlgcan.ZCAN_TransmitFD_Data * total)()
            for i in range(total):
                # 复制结构体内容（直接赋值）
                msg_array[i].transmit_type = self.canfd_queue[i].transmit_type
                msg_array[i].frame = self.canfd_queue[i].frame
            ret = self.zlg.TransmitFD(self.channel_handle, msg_array[0], total)

            self.send_suc_num += ret
            self.send_err_num += total - ret
            self.system_update = frame_cnt

    # ---------- CAN参数更新线程 ----------
    def _can_param_update_thread(self):
        recv_before = 0
        send_suc_before = 0
        send_err_before = 0
        sys_update_before = 0

        while self.is_updating:
            recv_before = self.recv_num
            send_suc_before = self.send_suc_num
            send_err_before = self.send_err_num
            sys_update_before = self.system_update
            time.sleep(self.can_param_time / 1000.0)

            recv_in = self.recv_num - recv_before
            send_suc_in = self.send_suc_num - send_suc_before
            send_err_in = self.send_err_num - send_err_before
            sys_in = self.system_update - sys_update_before

            self.can_param[0] = (recv_in / self.can_param_time) * 1000
            self.can_param[1] = (recv_in + send_suc_in + send_err_in) * 49.4 / 1000 / self.can_param_time * 100
            self.can_param[2] = self.send_suc_num
            self.can_param[3] = self.send_err_num
            self.can_param[4] = self.recv_num
            self.can_param[5] = self.system_update
            self.can_param[6] = (sys_in / self.can_param_time) * 1000

    # ---------- 用户接口 ----------
    def enable_all(self):
        self.stop_can()
        for m in self.motors:
            data = self.send_wait(1, m.ID, DMMotor.enable_command, 5)
            if data:
                m.read_motor(data)
        for t in self.tools:
            data = self.send_wait(1, t.ID, DMMotor.enable_command, 5)
            if data:
                t.read_motor(data)

    def disable_all(self):
        self.stop_can()
        for m in self.motors:
            data = self.send_wait(1, m.ID, DMMotor.disable_command, 5)
            if data:
                m.read_motor(data)
        for t in self.tools:
            data = self.send_wait(1, t.ID, DMMotor.disable_command, 5)
            if data:
                t.read_motor(data)

    def set_zero(self, motor_id: int):
        self.stop_can()
        data = self.send_wait(1, motor_id, DMMotor.set_zero_command, 5)
        if motor_id > self.MOTOR_NUM:
            idx = motor_id - self.MOTOR_NUM - 1
            if 0 <= idx < self.TOOL_NUM:
                self.tools[idx].read_motor(data)
                self.tools[idx].set_empty_command()
                data = self.send_wait(1, motor_id, self.tools[idx].Command, 5)
                if data:
                    self.tools[idx].read_motor(data)
        else:
            idx = motor_id - 1
            if 0 <= idx < self.MOTOR_NUM:
                self.motors[idx].read_motor(data)
                self.motors[idx].set_empty_command()
                data = self.send_wait(1, motor_id, self.motors[idx].Command, 5)
                if data:
                    self.motors[idx].read_motor(data)

    def get_status_all(self) -> bool:
        self.stop_can()
        for i, m in enumerate(self.motors):
            data = self.send_wait(1, 0x7FF, m.get_mode_command, 5)
            if not m.get_motor_mode(data):
                return False
            self.motor_mode[i] = m.Mode
        if not all(m == self.motor_mode[0] for m in self.motor_mode):
            return False
        for m in self.motors:
            m.set_empty_command()
            data = self.send_wait(1, m.ID, m.Command, 5)
            if data is None:
                return False
            m.read_motor(data)
        for t in self.tools:
            data = self.send_wait(1, 0x7FF, t.get_mode_command, 5)
            t.get_motor_mode(data)
            t.set_empty_command()
            data = self.send_wait(1, t.ID, t.Command, 5)
            if data:
                t.read_motor(data)
        return True

    def set_mode_all(self, mode: int) -> bool:
        self.stop_can()
        for i, m in enumerate(self.motors):
            if mode == 1:
                data = self.send_wait(1, 0x7FF, m.set_mit_command, 50)
            elif mode == 2:
                data = self.send_wait(1, 0x7FF, m.set_pv_command, 50)
            else:
                return False
            m.get_motor_mode(data)
            self.motor_mode[i] = m.Mode
        return all(m == self.motor_mode[0] for m in self.motor_mode)

    def send_wait(self, type_: int, motor_id: int, send_data: bytes, timeout_ms: float) -> Optional[bytearray]:
        self.stop_can()
        self._clear_recv_buffer()
        if type_ == 0:
            self.can_send(motor_id, send_data)
        else:
            self.canfd_send(motor_id, send_data)
        return self._wait_for_recv(timeout_ms)

    def _wait_for_recv(self, timeout_ms: float) -> Optional[bytearray]:
        start = time.perf_counter()
        while (time.perf_counter() - start) * 1000 < timeout_ms:
            if self.zlg.GetReceiveNum(self.channel_handle, TYPE_CAN) > 0:
                msgs, cnt = self.zlg.Receive(self.channel_handle, 1, 0)
                if cnt > 0:
                    frame = msgs[0]
                    res = bytearray(frame.frame.data[:8]) + bytes([frame.frame.can_id & 0xFF])
                    return res
            if self.zlg.GetReceiveNum(self.channel_handle, TYPE_CANFD) > 0:
                msgs, cnt = self.zlg.ReceiveFD(self.channel_handle, 1, 0)
                if cnt > 0:
                    frame = msgs[0]
                    res = bytearray(frame.frame.data[:8]) + bytes([frame.frame.can_id & 0xFF])
                    return res
            time.sleep(0.001)
        return None

    # ---------- 辅助方法 ----------
    @staticmethod
    def delayms(ms: float) -> float:
        if ms <= 0:
            return 0
        time.sleep(ms / 1000.0)
        return ms

    def _set_canfd_standard(self, std: int) -> bool:
        ret = self.zlg.ZCAN_SetValue(self.device_handle, f"{self.channel_index}/canfd_standard", str(std).encode())
        return ret == 1

    def _set_custom_baudrate(self, custom_str: str) -> bool:
        ret = self.zlg.ZCAN_SetValue(self.device_handle, f"{self.channel_index}/baud_rate_custom", custom_str.encode())
        return ret == 1

    def _set_resistance_enable(self, enable: bool) -> bool:
        val = b"1" if enable else b"0"
        ret = self.zlg.ZCAN_SetValue(self.device_handle, f"{self.channel_index}/initenal_resistance", val)
        return ret == 1

    def _set_filter(self) -> bool:
        ret = self.zlg.ZCAN_SetValue(self.device_handle, f"{self.channel_index}/filter_clear", b"0")
        return ret == 1

    def _set_queue_send(self):
        self.zlg.ZCAN_SetValue(self.device_handle, f"{self.channel_index}/set_send_mode", b"1")

    def _clear_queue_send(self):
        self.zlg.ZCAN_SetValue(self.device_handle, f"{self.channel_index}/clear_delay_send_queue", b"0")

    def _clear_recv_buffer(self) -> bool:
        return self.zlg.ClearBuffer(self.channel_handle) == STATUS_OK

    @staticmethod
    def _make_can_id(can_id: int, eff: int, rtr: int, err: int) -> int:
        return can_id | ((1 if eff else 0) << 31) | ((1 if rtr else 0) << 30) | ((1 if err else 0) << 29)

    # ---------- 属性 ----------
    @property
    def IsOpen(self) -> bool:
        return self.is_open

    @property
    def IsUpdating(self) -> bool:
        return self.is_updating

    @property
    def CanParam(self) -> List[float]:
        return self.can_param.copy()

    @property
    def Mode(self) -> int:
        if all(m == 1 for m in self.motor_mode):
            return 1
        if all(m == 2 for m in self.motor_mode):
            return 2
        return 0