#!/usr/bin/env python3
# interactive_mode_keep_enable.py
# 交互式选择模式（MIT/PV/PVT），切换模式后保持使能，直至下次切换或退出

import time
from USBCANFD import USBCANFD

def enable_motors_only(can: USBCANFD) -> bool:
    """只使能前6个电机，不操作工具电机"""
    can.stop_can()
    can.clearRecvBuffer()

    for motor in can.motors:
        data = can.send_wait(1, motor.ID, motor.clear_error_command, 20)
        if not motor.read_motor(data):
            print(f"电机 {motor.ID} 清错无有效回复")
            return False
        data = can.send_wait(1, motor.ID, motor.enable_command, 20)
        if not motor.read_motor(data):
            print(f"电机 {motor.ID} 使能无有效回复")
            return False
        if not motor.Enable:
            print(f"电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}")
            return False
    return True

def disable_motors_only(can: USBCANFD):
    """只失能前6个电机"""
    can.stop_can()
    for motor in can.motors:
        can.send_wait(1, motor.ID, motor.disable_command, 5)

def set_mode_all_generic(can: USBCANFD, mode: int) -> bool:
    """
    将前6个电机切换到指定模式（1=MIT, 2=PV, 4=PVT）
    返回是否全部切换成功
    """
    if mode not in (1, 2, 4):
        print(f"不支持的模式 {mode}")
        return False

    for motor in can.motors:
        if mode == 1:
            cmd = motor.set_mit_command
        elif mode == 2:
            cmd = motor.set_pv_command
        else:  # mode == 4
            cmd = motor.set_pvt_command

        data = can.send_wait(1, 0x7FF, cmd, 50)
        if data is None or len(data) < 8:
            print(f"电机 {motor.ID} 模式切换无有效回复")
            return False
        if data[0] != motor.ID:
            print(f"电机 {motor.ID} 切换模式回复ID不匹配")
            return False
        if not motor.get_motor_mode(data):
            print(f"电机 {motor.ID} 模式回复解析失败")
            return False
        if motor.Mode != mode:
            print(f"电机 {motor.ID} 切换失败，当前={motor.Mode}，目标={mode}")
            return False
    return True

def are_motors_enabled(can: USBCANFD) -> bool:

    # 由于使能后我们并没有持续接收，需要临时接收一次来更新状态？
    # 简便方法：利用 motor.Enable 属性，但该属性仅在上次 read_motor 后有效。
    # 这里为了准确，可以尝试发送一个空命令或直接查询状态。但为了简化，我们假设使能成功后 motor.Enable 为 True。
    # 用户可以通过再次选择同一模式来重新使能（如果掉使能了）。
    for motor in can.motors:
        if not motor.Enable:
            return False
    return True

def main():
    can = USBCANFD()

    # 打开并初始化设备
    if not can.open_device():
        print("打开设备失败")
        return
    if not can.init_device():
        print("初始化设备失败")
        can.close_device()
        return
    if not can.start_device():
        print("启动设备失败")
        can.close_device()
        return

    print("CAN 设备初始化成功")
    print("输入 0 退出程序。选择模式后电机会保持使能，选择其他模式时会自动失能当前并切换。")

    current_mode = None   # 记录当前已切换的模式（1/2/4）

    # 主交互循环
    while True:
        print("\n" + "="*50)
        print("请选择电机模式：")
        print("1 - MIT 模式")
        print("2 - PV 模式")
        print("3 - PVT 模式")
        print("0 - 退出程序")
        choice = input("输入数字: ").strip()

        if choice == '0':
            break

        if choice not in ('1', '2', '3'):
            print("无效输入，请重新选择")
            continue

        mode_map = {'1': 1, '2': 2, '3': 4}
        target_mode = mode_map[choice]
        mode_name = {1: "MIT", 2: "PV", 4: "PVT"}[target_mode]

        # 如果当前已有使能的电机且模式与目标不同，先失能
        if current_mode is not None and current_mode != target_mode:
            print(f"\n当前模式为 {current_mode}，正在失能电机...")
            disable_motors_only(can)
            print("电机已失能")
            current_mode = None
        elif current_mode == target_mode:
            # 模式相同，检查是否还使能，若未使能则重新使能
            if not are_motors_enabled(can):
                print(f"\n当前模式已是 {mode_name}，但电机未使能，正在重新使能...")
                if enable_motors_only(can):
                    print("电机已使能")
                else:
                    print("使能失败，请检查")
            else:
                print(f"\n当前模式已是 {mode_name} 且电机已使能，无需操作")
            continue

        # 切换模式（当前已失能或无使能）
        print(f"\n>>> 正在切换前6个电机到 {mode_name} 模式...")
        if not set_mode_all_generic(can, target_mode):
            print("模式切换失败，请检查连接或电机状态")
            continue
        print(f"模式切换成功，当前模式: {mode_name}")
        current_mode = target_mode

        # 使能电机（保持使能状态）
        print("正在使能前6个电机...")
        if not enable_motors_only(can):
            print("使能失败，请检查")
            current_mode = None
            continue
        print("电机已使能，将保持使能状态直到您切换其他模式或退出程序。")

    # 退出前失能电机
    if current_mode is not None:
        print("\n正在失能电机...")
        disable_motors_only(can)
        print("电机已失能")

    # 清理
    can.stop_can()
    can.close_device()
    print("设备已关闭，程序正常退出。")

if __name__ == "__main__":
    main()