# 测试 MIT 模式下的机械臂重力补偿
import time
from USBCANFD import USBCANFD
from Robot import Robot


def main():
    can = USBCANFD()
    robot = Robot()

    if not can.open_device():
        raise RuntimeError("无法打开 CANFD 设备")

    if not can.init_device():
        raise RuntimeError("初始化 CANFD 失败")

    if not can.start_device():
        raise RuntimeError("启动 CANFD 失败")

    # 刚上电后，CANFD设备、电机驱动器、总线通信都需要一点稳定时间
    time.sleep(0.5)

    # 清空启动阶段可能残留的无效帧
    can.clearRecvBuffer()

    # 先尝试读取一次状态/模式，让总线和电机通信预热
    for attempt in range(5):
        if can.get_status_all():
            print("电机状态读取成功，准备切换 MIT 模式")
            break
        print(f"第 {attempt + 1} 次读取电机状态失败，继续等待...")
        time.sleep(0.2)
    else:
        raise RuntimeError("上电后无法读取电机状态，请检查电源、CAN接线、波特率、终端电阻")

    # 切换 1~6 号电机到 MIT 模式
    for attempt in range(5):
        if can.set_mode_all(1):
            print("MIT 模式切换成功")
            break
        print(f"第 {attempt + 1} 次切换 MIT 模式失败，重试...")
        time.sleep(0.2)
    else:
        raise RuntimeError("切换 MIT 模式失败")

    # MIT 空命令，避免刚启动时残留非零力矩
    for motor in can.motors:
        motor.set_empty_command_MIT()
        motor.set()

    # 使能所有电机
    can.enable_all()

    # 启动 CANFD 三线程：接收线程、队列发送线程、CAN 参数刷新线程
    can.start_can_thread(1)

    try:
        while True:
            # 1. 电机角度 -> DH 角
            robot.Angle = robot.motor2dh(can.motors)

            # 2. 更新正运动学、雅可比、重力补偿
            robot.set_robot()

            tau_g_motor = robot.Tau_G_Motor


            # 3. 写入 MIT 力矩命令
            can.motors[4].MIT.torque_set = float(tau_g_motor[4])
            can.motors[4].set()

            can.motors[3].MIT.torque_set = float(tau_g_motor[3])
            can.motors[3].set()

            can.motors[2].MIT.torque_set = float(tau_g_motor[2] * 1.2)
            can.motors[2].set()

            can.motors[1].MIT.torque_set = float(tau_g_motor[1] * 1.2)
            can.motors[1].set()

            # Python 主循环不要完全空转，否则会抢线程调度
            time.sleep(0.001)

    except KeyboardInterrupt:
        print("停止控制")

    finally:
        for motor in can.motors:
            motor.set_empty_command_MIT()
            motor.set()

        time.sleep(0.05)
        can.disable_all()
        can.stop_can()
        can.close_device()


if __name__ == "__main__":
    main()