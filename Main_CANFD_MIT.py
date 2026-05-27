"""
Python version of Main_CANFD_MIT.cs.

This script is the Python entry point that connects the converted modules:
    - USBCANFD.py
    - Robot.py
    - DMMotor.py
    - TreeStruct.py
    - zlgcan.py

It mimics the control intent of Main_CANFD_MIT.cs:
    1. open/init/start USBCANFD
    2. enable motors
    3. start CANFD receive / queue-send / parameter-update threads
    4. continuously read motor positions
    5. compute robot gravity compensation
    6. write MIT torque commands for joints 2~5

Important:
    The converted USBCANFD.py initializes motors in PV mode by default,
    so this script explicitly switches all six arm motors to MIT mode.
"""

from __future__ import annotations

import time
from typing import Optional

from Robot import Robot
from USBCANFD import USBCANFD


# ---------------------------------------------------------------------------
# User-adjustable parameters
# ---------------------------------------------------------------------------

CONNECT_JOYSTICK = True       # Main_CANFD_MIT.cs calls joystick.connect(); this script skips it if Joystick.py is absent.
FORCE_SWITCH_TO_MIT = True    # Recommended for the converted Python classes.
START_CANFD_THREAD_TYPE = 1   # 1 = CANFD receive thread, matching CAN.start_can_thread(1) in C#.

CONTROL_SLEEP_S = 0.001       # Prevents Python main loop from busy-spinning too hard.
PRINT_INTERVAL_S = 0.2        # Print CAN FPS at this interval.

# Gravity-compensation scale factors, matching the C# example:
# motor[4] = tau_g[4]
# motor[3] = tau_g[3]
# motor[2] = tau_g[2] * 1.1
# motor[1] = tau_g[1] * 1.1
TORQUE_SCALE = {
    4: 1.0,
    3: 1.0,
    2: 1.1,
    1: 1.1,
}


def try_connect_joystick() -> Optional[object]:
    """Try to mimic the joystick.connect() call in the C# program.

    The joystick object is not used in the shown C# control loop, so absence of
    Joystick.py should not prevent the gravity-compensation demo from running.
    """
    if not CONNECT_JOYSTICK:
        return None

    try:
        from Joystick import Joystick  # type: ignore
    except Exception as exc:
        print(f"未找到可用的 Joystick.py，跳过手柄连接：{exc}")
        return None

    joystick = Joystick()
    if hasattr(joystick, "connect"):
        joystick.connect()
    else:
        print("Joystick 对象没有 connect() 方法，已跳过连接。")
    return joystick


def set_all_motors_empty_mit(can: USBCANFD) -> None:
    """Set six arm motors to zero MIT commands."""
    for i in range(6):
        can.motors[i].set_empty_command_MIT()
        can.motors[i].set()


def set_mit_torque(motor, torque: float) -> None:
    """Set one motor's MIT torque command using the converted DMMotor.py API."""
    motor.MIT.position_set = 0.0
    motor.MIT.velocity_set = 0.0
    motor.MIT.torque_set = float(torque)
    motor.MIT.kp_set = 0.0
    motor.MIT.kd_set = 0.0
    motor.set()


def print_can_param(can: USBCANFD, now: float, last_print_time: float) -> float:
    """Print CAN FPS periodically.

    In the converted USBCANFD.py, CanParam[0] corresponds to the C# CAN.CanFps-like value.
    CanParam layout:
        [0] frame rate / receive FPS
        [1] bus load
        [2] send success count
        [3] send error count
        [4] receive count
        [5] system update count
        [6] system update rate
    """
    if now - last_print_time < PRINT_INTERVAL_S:
        return last_print_time

    can_param = can.CanParam
    print(
        f"CAN_FPS={can_param[0]:.1f}, "
        f"BUS_LOAD={can_param[1]:.2f}%, "
        f"SEND_OK={can_param[2]:.0f}, "
        f"SEND_ERR={can_param[3]:.0f}, "
        f"RECV={can_param[4]:.0f}, "
        f"SYS_HZ={can_param[6]:.1f}"
    )
    return now


def init_canfd_for_mit() -> USBCANFD:
    """Open, initialize, start, switch mode, and enable the USBCANFD device."""
    can = USBCANFD()

    if not can.open_device():
        raise RuntimeError("CAN.open_device() failed")

    if not can.init_device():
        can.close_device()
        raise RuntimeError("CAN.init_device() failed")

    if not can.start_device():
        can.close_device()
        raise RuntimeError("CAN.start_device() failed")

    # The converted USBCANFD.py constructs motors as PV by default.
    # For MIT torque commands, switch them to MIT explicitly.
    if FORCE_SWITCH_TO_MIT:
        ok = can.set_mode_all(1)
        if not ok:
            can.close_device()
            raise RuntimeError("CAN.set_mode_all(1) failed; motors did not all switch to MIT mode")

    can.enable_all()
    set_all_motors_empty_mit(can)
    can.start_can_thread(START_CANFD_THREAD_TYPE)

    return can


def shutdown_canfd(can: Optional[USBCANFD]) -> None:
    """Safely stop torque output and close the CANFD device."""
    if can is None:
        return

    try:
        set_all_motors_empty_mit(can)
        time.sleep(0.05)
    except Exception as exc:
        print(f"发送 MIT 零命令时出现异常：{exc}")

    try:
        can.disable_all()
    except Exception as exc:
        print(f"disable_all() 异常：{exc}")

    try:
        can.stop_can()
    except Exception as exc:
        print(f"stop_can() 异常：{exc}")

    try:
        can.close_device()
    except Exception as exc:
        print(f"close_device() 异常：{exc}")


def main() -> None:
    joystick = None
    can: Optional[USBCANFD] = None
    robot = Robot()

    try:
        joystick = try_connect_joystick()
        can = init_canfd_for_mit()

        last_print_time = 0.0

        while True:
            # motor positions -> DH angles
            robot.Angle = robot.motor2dh(can.motors)

            # update kinematics, Jacobian, gravity torque, and external-force torque
            robot.set_robot()

            tau_g_motor = robot.Tau_G_Motor

            # C# equivalent:
            # Console.WriteLine(CAN.CanFps);
            last_print_time = print_can_param(can, time.monotonic(), last_print_time)

            # C# equivalent:
            # CAN.motors[4].torque_set = robot.Tau_G_Motor[4]; CAN.motors[4].set_MIT();
            # CAN.motors[3].torque_set = robot.Tau_G_Motor[3]; CAN.motors[3].set_MIT();
            # CAN.motors[2].torque_set = robot.Tau_G_Motor[2] * 1.1f; CAN.motors[2].set_MIT();
            # CAN.motors[1].torque_set = robot.Tau_G_Motor[1] * 1.1f; CAN.motors[1].set_MIT();
            for motor_index, scale in TORQUE_SCALE.items():
                set_mit_torque(can.motors[motor_index], float(tau_g_motor[motor_index]) * scale)

            time.sleep(CONTROL_SLEEP_S)

    except KeyboardInterrupt:
        print("收到 Ctrl+C，准备停止。")
    finally:
        shutdown_canfd(can)

        # Keep a reference so linters do not mark it as unused.
        _ = joystick


if __name__ == "__main__":
    main()
