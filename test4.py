#!/usr/bin/env python3
# test_mit_mode_with_freq.py
# 测试 MIT 模式，读取前六个电机位置，并实时打印 CAN 总线发送频率

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

    # 2. 切换到 MIT 模式（前6个电机）
    print("正在切换前6个电机到 MIT 模式...")
    if not can.set_mode_all(1):
        print("切换 MIT 模式失败")
        can.close_device()
        return
    print("前6个电机已切换到 MIT 模式")

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

    time.sleep(0.1)

    # 5. 循环打印位置和发送频率
    try:
        while True:
            # 读取6个电机位置
            positions = [motor.Position for motor in can.motors]
            # 读取 CAN 参数（线程安全，CanParam 属性会加锁复制）
            can_params = can.CanParam
            send_freq = can_params[6]   # can_param[6] 为实际发送帧率（帧/秒）
            # 格式化输出
            print(f"[{time.strftime('%H:%M:%S')}] 位置(rad): "
                  f"1:{positions[0]:.4f} 2:{positions[1]:.4f} 3:{positions[2]:.4f} "
                  f"4:{positions[3]:.4f} 5:{positions[4]:.4f} 6:{positions[5]:.4f}  "
                  f"| 发送频率: {send_freq:.1f} Hz")
            time.sleep(0.01)
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