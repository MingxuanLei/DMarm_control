#!/usr/bin/env python3
# test_pv_mode_default.py
# 测试 PV 模式（默认目标位置0，速度限制0），使能前六个电机，打印位置反馈

import time
from USBCANFD import USBCANFD

def enable_motors_only(can: USBCANFD) -> bool:
    """只使能前6个电机，不操作工具电机"""
    can.stop_can()
    can.clearRecvBuffer()

    for motor in can.motors:
        # 清错
        data = can.send_wait(1, motor.ID, motor.clear_error_command, 20)
        if not motor.read_motor(data):
            print(f"电机 {motor.ID} 清错无有效回复")
            return False
        # 使能
        data = can.send_wait(1, motor.ID, motor.enable_command, 20)
        if not motor.read_motor(data):
            print(f"电机 {motor.ID} 使能无有效回复")
            return False
        if not motor.Enable:
            print(f"电机 {motor.ID} 使能失败，ERR={motor.ERRCODE}")
            return False
    return True

def main():
    can = USBCANFD()

    # 1. 打开并初始化设备
    if not can.open_device():
        print("打开设备失败")
        return
    if not can.init_device():
        print("初始化设备失败")
        return
    if not can.start_device():
        print("启动设备失败")
        return

    # 2. 切换到 PV 模式（前6个电机）
    print("正在切换前6个电机到 PV 模式...")
    if not can.set_mode_all(2):
        print("切换 PV 模式失败")
        can.close_device()
        return
    print("前6个电机已切换到 PV 模式")

    # 3. 只使能前6个电机
    print("正在使能前6个电机...")
    if not enable_motors_only(can):
        print("使能失败")
        can.close_device()
        return
    print("前6个电机已使能，工具电机未使能")

    # 4. 重启后台发送线程（使能时已 stop_can）
    can.start_can_thread(1)
    print("后台控制线程已启动")
    print("PV 模式使用默认目标：位置 0.0 rad，速度限制 0.0 rad/s")

    time.sleep(0.1)   # 等待第一轮命令生效

    # 5. 循环打印位置和发送频率
    try:
        while True:
            positions = [motor.Position for motor in can.motors]
            # 获取实际发送频率
            can_params = can.CanParam
            send_freq = can_params[6] if len(can_params) > 6 else 0.0
            print(f"[{time.strftime('%H:%M:%S')}] 位置(rad): "
                  f"1:{positions[0]:.4f} 2:{positions[1]:.4f} 3:{positions[2]:.4f} "
                  f"4:{positions[3]:.4f} 5:{positions[4]:.4f} 6:{positions[5]:.4f}  "
                  f"| 控制频率: {send_freq:.1f} Hz")
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n用户中断，正在停止...")

    # 6. 清理：停止线程，失能前6个电机，关闭设备
    can.stop_can()
    for motor in can.motors:
        can.send_wait(1, motor.ID, motor.disable_command, 5)
    can.close_device()
    print("测试结束")

if __name__ == "__main__":
    main()