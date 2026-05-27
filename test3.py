# test_enable_6motors_2s.py
import time
from USBCANFD import USBCANFD
from DMMotor import DMMotor


TIMEOUT_MS = 50
ENABLE_HOLD_TIME_S = 2.0
CLEAR_ERROR_FIRST = True


def print_motor_status(motor, prefix=""):
    print(
        f"{prefix}电机 {motor.ID}: "
        f"Enable={motor.Enable}, "
        f"ERR={motor.ERRCODE}, "
        f"Pos={motor.Position:.4f} rad, "
        f"Vel={motor.Velocity:.4f} rad/s, "
        f"Torque={motor.Torque:.4f}, "
        f"Tmos={motor.tem_mos}, "
        f"Trotor={motor.tem_rotor}, "
        f"Recv={motor.recv_num}"
    )


def clear_error_once(can: USBCANFD, motor) -> bool:
    data = can.send_wait(1, motor.ID, DMMotor.clear_error_command, TIMEOUT_MS)
    ok = motor.read_motor(data)
    if not ok:
        print(f"[WARN] 电机 {motor.ID} 清错命令无有效反馈，回复={data}")
    else:
        print_motor_status(motor, prefix="[CLEAR] ")
    return ok


def enable_motor_once(can: USBCANFD, motor) -> bool:
    data = can.send_wait(1, motor.ID, DMMotor.enable_command, TIMEOUT_MS)
    ok = motor.read_motor(data)
    if not ok:
        print(f"[WARN] 电机 {motor.ID} 使能命令无有效反馈，回复={data}")
        return False

    print_motor_status(motor, prefix="[ENABLE] ")
    return motor.Enable


def disable_motor_once(can: USBCANFD, motor) -> bool:
    data = can.send_wait(1, motor.ID, DMMotor.disable_command, TIMEOUT_MS)
    ok = motor.read_motor(data)
    if not ok:
        print(f"[WARN] 电机 {motor.ID} 失能命令无有效反馈，回复={data}")
        return False

    print_motor_status(motor, prefix="[DISABLE] ")
    return not motor.Enable


def disable_first_6_motors(can: USBCANFD):
    print("[SAFE] 正在失能 1~6 号电机...")
    for motor in can.motors:
        disable_motor_once(can, motor)


def main():
    can = USBCANFD()
    motors_to_test = list(can.motors)  # 只包含 1~6 号电机，不包含第 7 个工具电机

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
        print("[SAFE] 本脚本不会发送 MIT/PV/PVT 运动命令，不会发送力矩命令，不会启动连续发送线程。")
        print("[SAFE] 本脚本只对 1~6 号电机执行：清错/使能 -> 等待 2s -> 失能。")

        if CLEAR_ERROR_FIRST:
            print("[4] 清除 1~6 号电机错误...")
            for motor in motors_to_test:
                clear_error_once(can, motor)

        print("[5] 使能 1~6 号电机...")
        enable_ok = True
        for motor in motors_to_test:
            if not enable_motor_once(can, motor):
                enable_ok = False

        if not enable_ok:
            print("[WARN] 至少有一个电机使能反馈异常，立即执行失能。")
            disable_first_6_motors(can)
            return

        print(f"[6] 已发送使能命令，等待 {ENABLE_HOLD_TIME_S:.1f} s...")
        time.sleep(ENABLE_HOLD_TIME_S)

        print("[7] 失能 1~6 号电机...")
        disable_ok = True
        for motor in motors_to_test:
            if not disable_motor_once(can, motor):
                disable_ok = False

        if disable_ok:
            print("[OK] 1~6 号电机已完成：使能 -> 等待 2s -> 失能。")
        else:
            print("[WARN] 至少有一个电机失能反馈异常，请检查电机状态。")

    except KeyboardInterrupt:
        print("\n用户中断，立即失能 1~6 号电机。")
        disable_first_6_motors(can)

    finally:
        print("[END] 关闭设备前再次尝试失能 1~6 号电机...")
        try:
            disable_first_6_motors(can)
        except Exception as e:
            print(f"[WARN] 最终失能时出现异常: {e}")

        can.stop_can()
        can.close_device()
        print("[END] 已关闭。")


if __name__ == "__main__":
    main()