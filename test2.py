# test_read_status_only_6motors.py
import time
from USBCANFD import USBCANFD
from DMMotor import DMMotor


TIMEOUT_MS = 50
LOOP_PERIOD_S = 0.2

# 第一次测试建议保持 True：只发送失能命令，不发送力矩、不使能、不切模式
DISABLE_FIRST = True

# 读取反馈需要给电机发一帧“当前模式下的空命令”来换取反馈
READ_FEEDBACK_WITH_ZERO_CMD = True


def print_motor_status(can: USBCANFD):
    print("=" * 110)
    print(
        f"{'ID':>3} | {'Mode':>5} | {'Enable':>6} | {'ERR':>8} | "
        f"{'Pos(rad)':>10} | {'Vel(rad/s)':>11} | {'Torque':>10} | "
        f"{'Tmos':>5} | {'Trotor':>6} | {'Recv':>5}"
    )
    print("-" * 110)

    # 只打印前六个电机，不打印第七个工具电机
    for m in can.motors:
        print(
            f"{m.ID:>3} | {m.ModeName:>5} | {str(m.Enable):>6} | {m.ERRCODE:>8} | "
            f"{m.Position:>10.4f} | {m.Velocity:>11.4f} | {m.Torque:>10.4f} | "
            f"{m.tem_mos:>5} | {m.tem_rotor:>6} | {m.recv_num:>5}"
        )


def query_motor_mode(can: USBCANFD, motor) -> bool:
    data = can.send_wait(1, DMMotor.PARAM_SET_ID, motor.get_mode_command, TIMEOUT_MS)
    ok = motor.get_motor_mode(data)
    if not ok:
        print(f"[WARN] 电机 {motor.ID} 模式查询失败，回复={data}")
    return ok


def disable_motor_once(can: USBCANFD, motor) -> bool:
    data = can.send_wait(1, motor.ID, DMMotor.disable_command, TIMEOUT_MS)
    ok = motor.read_motor(data)
    if not ok:
        print(f"[WARN] 电机 {motor.ID} 失能命令无有效反馈，回复={data}")
    return ok


def read_motor_feedback_once(can: USBCANFD, motor) -> bool:
    # 根据已经查询到的 motor.Mode 生成对应模式下的空命令
    motor.set_empty_command()

    data = can.send_wait(1, motor.ID, motor.Command, TIMEOUT_MS)
    ok = motor.read_motor(data)
    if not ok:
        print(f"[WARN] 电机 {motor.ID} 状态读取失败，回复={data}")
    return ok


def main():
    can = USBCANFD()

    try:
        print("[1] 打开 CANFD 设备...")
        if not can.open_device():
            raise RuntimeError("无法打开 CANFD 设备")

        print("[2] 初始化 CANFD 设备...")
        if not can.init_device():
            raise RuntimeError("初始化 CANFD 失败")

        print("[3] 启动 CANFD 通道...")
        if not can.start_device():
            raise RuntimeError("启动 CANFD 失败")

        print("[OK] 设备已打开并初始化。")
        print("[SAFE] 本脚本只读取 1~6 号电机，不读取第 7 个工具电机。")
        print("[SAFE] 本脚本不会使能电机，不会切换 MIT 模式，不会启动连续发送线程。")

        # 关键修改：只使用 can.motors，也就是 1~6 号电机
        motors_to_test = list(can.motors)

        if DISABLE_FIRST:
            print("[4] 发送一次失能命令，确保 1~6 号电机不处于使能控制状态...")
            for motor in motors_to_test:
                disable_motor_once(can, motor)

        print("[5] 查询 1~6 号电机模式...")
        for motor in motors_to_test:
            query_motor_mode(can, motor)

        print("[6] 开始循环读取 1~6 号电机状态。按 Ctrl+C 停止。")

        while True:
            if READ_FEEDBACK_WITH_ZERO_CMD:
                for motor in motors_to_test:
                    read_motor_feedback_once(can, motor)

            print_motor_status(can)
            time.sleep(LOOP_PERIOD_S)

    except KeyboardInterrupt:
        print("\n用户停止测试。")

    finally:
        print("[END] 停止线程并关闭设备...")
        can.stop_can()
        can.close_device()
        print("[END] 已关闭。")


if __name__ == "__main__":
    main()