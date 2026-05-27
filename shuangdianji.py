import time
import threading
import math
import queue
from zlgcan import *   # 确保 zlgcan.py 在当前目录或 PYTHONPATH 中

# ========== 配置参数 ==========
DEVICE_TYPE = ZCAN_USBCANFD_MINI   # USBCANFD-MINI
DEVICE_INDEX = 0
CAN_CHANNEL = 0
CUSTOM_BAUD = "1.0Mbps(75%),5.0Mbps(75%),(60,00000E2B,00800001)"

# 使能/失能命令 (两个电机相同，仅 CAN ID 不同)
ENABLE_DATA = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC]
DISABLE_DATA = [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD]

# ========== 电机参数定义 ==========
MOTOR1_CONFIG = {
    "id": 1,
    "can_id": 0x01,
    "feedback_id": 0x11,
    "P_MAX": 12.5,
    "V_MAX": 30.0,
    "T_MAX": 10.0,
    "wave_amplitude": 0.8,
}

MOTOR2_CONFIG = {
    "id": 2,
    "can_id": 0x02,
    "feedback_id": 0x12,
    "P_MAX": 12.5,
    "V_MAX": 10.0,
    "T_MAX": 28.0,
    "wave_amplitude": 3.0,
}

MOTORS = [MOTOR1_CONFIG, MOTOR2_CONFIG]

WAVE_PERIOD = 10.0
SEND_FREQ = 500.0
RUN_DURATION = 10

thread_flag = True
cmd_queue = queue.Queue()

motor_status = {
    1: {
        "motor_id": 1,
        "status_str": "未知",
        "position_rad": 0.0,
        "velocity_rps": 0.0,
        "torque_nm": 0.0,
        "mos_temp_c": 0,
        "coil_temp_c": 0,
        "raw_hex": ""
    },
    2: {
        "motor_id": 2,
        "status_str": "未知",
        "position_rad": 0.0,
        "velocity_rps": 0.0,
        "torque_nm": 0.0,
        "mos_temp_c": 0,
        "coil_temp_c": 0,
        "raw_hex": ""
    }
}
status_lock = threading.Lock()


def parse_damiao_feedback(data_bytes, p_max, v_max, t_max):
    if len(data_bytes) < 8:
        return {"error": "数据长度不足8字节", "raw": data_bytes.hex()}

    byte0 = data_bytes[0]
    motor_id = byte0 & 0x0F
    err_code = (byte0 >> 4) & 0x0F
    err_map = {
        0: "失能", 1: "使能", 8: "超压", 9: "欠压",
        10: "过电流", 11: "MOS过温", 12: "电机线圈过温",
        13: "通讯丢失", 14: "过载"
    }
    status_str = err_map.get(err_code, f"未知({err_code})")

    pos_raw = (data_bytes[1] << 8) | data_bytes[2]
    position_rad = (pos_raw / 65535.0) * (2.0 * p_max) - p_max

    vel_raw = ((data_bytes[3] << 4) | (data_bytes[4] >> 4)) & 0x0FFF
    velocity_rps = (vel_raw / 4095.0) * (2.0 * v_max) - v_max

    t_raw = ((data_bytes[4] & 0x0F) << 8) | data_bytes[5]
    torque_nm = (t_raw / 4095.0) * (2.0 * t_max) - t_max

    mos_temp = data_bytes[6]
    coil_temp = data_bytes[7]

    return {
        "motor_id": motor_id,
        "status_str": status_str,
        "position_rad": round(position_rad, 4),
        "velocity_rps": round(velocity_rps, 4),
        "torque_nm": round(torque_nm, 4),
        "mos_temp_c": mos_temp,
        "coil_temp_c": coil_temp,
        "raw_hex": data_bytes.hex()
    }


def pack_mit_torque_frame(torque_nm, p_max, v_max, t_max):
    p_int = int((0.0 - (-p_max)) / (2.0 * p_max) * 65535)
    v_int = int((0.0 - (-v_max)) / (2.0 * v_max) * 4095)
    kp_int = 0
    kd_int = 0
    t_clamped = max(min(torque_nm, t_max), -t_max)
    t_int = int((t_clamped + t_max) / (2.0 * t_max) * 4095)

    data = [0] * 8
    data[0] = (p_int >> 8) & 0xFF
    data[1] = p_int & 0xFF
    data[2] = (v_int >> 4) & 0xFF
    data[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F)
    data[4] = kp_int & 0xFF
    data[5] = (kd_int >> 4) & 0xFF
    data[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F)
    data[7] = t_int & 0xFF
    return data


def init_can_device():
    zcanlib = ZCAN()
    handle = zcanlib.OpenDevice(DEVICE_TYPE, DEVICE_INDEX, 0)
    if handle == INVALID_DEVICE_HANDLE:
        raise Exception("打开设备失败，请检查驱动和设备连接")
    print(f"设备打开成功，句柄: {handle}")

    ret = zcanlib.ZCAN_SetValue(handle, f"{CAN_CHANNEL}/baud_rate_custom", CUSTOM_BAUD.encode("utf-8"))
    if ret != ZCAN_STATUS_OK:
        raise Exception("设置自定义波特率失败")
    print("自定义波特率已设置")

    ret = zcanlib.ZCAN_SetValue(handle, f"{CAN_CHANNEL}/initenal_resistance", b"1")
    if ret != ZCAN_STATUS_OK:
        print("警告：开启终端电阻失败")

    init_cfg = ZCAN_CHANNEL_INIT_CONFIG()
    init_cfg.can_type = ZCAN_TYPE_CANFD
    init_cfg.config.canfd.mode = 0
    chn_handle = zcanlib.InitCAN(handle, CAN_CHANNEL, init_cfg)
    if chn_handle is None or chn_handle == INVALID_CHANNEL_HANDLE:
        raise Exception("初始化 CAN 通道失败")
    print(f"通道初始化成功，句柄: {chn_handle}")

    ret = zcanlib.StartCAN(chn_handle)
    if ret != ZCAN_STATUS_OK:
        raise Exception("启动 CAN 通道失败")
    print("CAN 通道已启动")
    return zcanlib, handle, chn_handle


def send_canfd_frame(zcanlib, chn_handle, can_id, data_bytes, brs=True):
    if len(data_bytes) > 64:
        raise ValueError("CAN FD 最大数据长度为 64 字节")
    tx_msg = ZCAN_TransmitFD_Data()
    tx_msg.frame.can_id = can_id
    tx_msg.frame.len = len(data_bytes)
    tx_msg.frame.flags = 0x01 if brs else 0x00
    for i, b in enumerate(data_bytes):
        tx_msg.frame.data[i] = b
    tx_msg.transmit_type = 0
    ret = zcanlib.TransmitFD(chn_handle, tx_msg, 1)


def sender_thread(zcanlib, chn_handle):
    global thread_flag, cmd_queue
    print("[发送线程] 启动，同步发送使能命令...")

    # ---------- 直接发送使能命令（不走队列） ----------
    for motor in MOTORS:
        send_canfd_frame(zcanlib, chn_handle, motor["can_id"], ENABLE_DATA)

    # 等待使能确认
    print("[发送线程] 等待电机使能确认...")
    timeout_start = time.time()
    both_enabled = False
    while time.time() - timeout_start < 2.0:
        if not thread_flag:
            return
        with status_lock:
            s1 = motor_status[1]["status_str"]
            s2 = motor_status[2]["status_str"]
        if s1 == "使能" and s2 == "使能":
            both_enabled = True
            break
        time.sleep(0.05)
    if both_enabled:
        print("[发送线程] 两个电机均已使能")
    else:
        print("[发送线程] 警告：未检测到全部电机使能，将继续发送")

# ---------- 主发送循环：每次唤醒后排空队列 ----------
    while thread_flag:
        try:
            # 先阻塞等待至少一条命令
            cmd = cmd_queue.get(timeout=0.1)
            send_canfd_frame(zcanlib, chn_handle, cmd["can_id"], cmd["data"])
            # 立即把队列里剩下的全部发送出去，不阻塞
            while not cmd_queue.empty():
                try:
                    cmd = cmd_queue.get_nowait()
                    send_canfd_frame(zcanlib, chn_handle, cmd["can_id"], cmd["data"])
                except queue.Empty:
                    break
        except queue.Empty:
            continue

    # ---------- 退出循环后：直接发送失能命令 ----------
    print("[发送线程] 主循环退出，直接发送失能命令...")
    for motor in MOTORS:
        send_canfd_frame(zcanlib, chn_handle, motor["can_id"], DISABLE_DATA)
        time.sleep(0.002)
    print("[发送线程] 失能命令已发送，线程退出")


def receiver_thread(zcanlib, chn_handle):
    global thread_flag, motor_status

    feedback_map = {motor["feedback_id"]: motor for motor in MOTORS}

    print("[接收线程] 启动，等待反馈...")
    while thread_flag:
        rcv_canfd = zcanlib.GetReceiveNum(chn_handle, ZCAN_TYPE_CANFD)
        if rcv_canfd > 0:
            rcv_fd_msgs, num_fd = zcanlib.ReceiveFD(chn_handle, min(rcv_canfd, 100), 100)
            for i in range(num_fd):
                frame = rcv_fd_msgs[i].frame
                data_bytes = bytes(frame.data[:frame.len])
                can_id = frame.can_id & 0x1FFFFFFF
                if can_id in feedback_map and frame.len >= 8:
                    motor = feedback_map[can_id]
                    status = parse_damiao_feedback(data_bytes, motor["P_MAX"], motor["V_MAX"], motor["T_MAX"])
                    with status_lock:
                        # 只更新非 motor_id 的内容，保留原始 ID 不变
                        s = motor_status[motor["id"]]
                        s["status_str"] = status["status_str"]
                        s["position_rad"] = status["position_rad"]
                        s["velocity_rps"] = status["velocity_rps"]
                        s["torque_nm"] = status["torque_nm"]
                        s["mos_temp_c"] = status["mos_temp_c"]
                        s["coil_temp_c"] = status["coil_temp_c"]
                        s["raw_hex"] = status["raw_hex"]

        rcv_can = zcanlib.GetReceiveNum(chn_handle, ZCAN_TYPE_CAN)
        if rcv_can > 0:
            rcv_msgs, num = zcanlib.Receive(chn_handle, min(rcv_can, 100), 100)
            for i in range(num):
                frame = rcv_msgs[i].frame
                data_bytes = bytes(frame.data[:frame.can_dlc])
                can_id = frame.can_id & 0x1FFFFFFF
                if can_id in feedback_map and frame.can_dlc >= 8:
                    motor = feedback_map[can_id]
                    status = parse_damiao_feedback(data_bytes, motor["P_MAX"], motor["V_MAX"], motor["T_MAX"])
                    with status_lock:
                        s = motor_status[motor["id"]]
                        s["status_str"] = status["status_str"]
                        s["position_rad"] = status["position_rad"]
                        s["velocity_rps"] = status["velocity_rps"]
                        s["torque_nm"] = status["torque_nm"]
                        s["mos_temp_c"] = status["mos_temp_c"]
                        s["coil_temp_c"] = status["coil_temp_c"]
                        s["raw_hex"] = status["raw_hex"]
        time.sleep(0.001)

def main():
    global thread_flag, motor_status

    zcanlib, dev_handle, ch_handle = init_can_device()

    # 接收线程设为 daemon，主线程结束时自动终止
    recv_thread = threading.Thread(target=receiver_thread, args=(zcanlib, ch_handle))
    recv_thread.daemon = True
    recv_thread.start()

    # 发送线程不使用 daemon，等待它完成失能发送后再退出
    send_thread = threading.Thread(target=sender_thread, args=(zcanlib, ch_handle))
    send_thread.start()

    print("\n===== 开始双电机力矩波形控制 =====")
    print(f"电机1 (ID 0x01): 振幅 {MOTOR1_CONFIG['wave_amplitude']} Nm, 周期 {WAVE_PERIOD}s")
    print(f"电机2 (ID 0x02): 振幅 {MOTOR2_CONFIG['wave_amplitude']} Nm, 周期 {WAVE_PERIOD}s")
    print(f"总运行时间: {RUN_DURATION}s, 发送频率 {SEND_FREQ}Hz")

    # 使用 perf_counter 保证高精度
    start_time = time.perf_counter()
    last_print_time = start_time
    period_sec = 1.0 / SEND_FREQ

    # 下一个理想执行时刻
    next_tick = start_time + period_sec

    try:
        while time.perf_counter() - start_time < RUN_DURATION:
            current = time.perf_counter()
            elapsed = current - start_time

            # 计算两个电机的力矩命令，同一时刻入队（保证同步的关键）
            for motor in MOTORS:
                amp = motor["wave_amplitude"]
                torque_cmd = amp * math.sin(2 * math.pi * elapsed / WAVE_PERIOD)
                frame_data = pack_mit_torque_frame(
                    torque_cmd, motor["P_MAX"], motor["V_MAX"], motor["T_MAX"]
                )
                cmd_queue.put({"can_id": motor["can_id"], "data": frame_data})

            # 定期打印状态（每 0.1 秒一次）
            if current - last_print_time >= 0.1:
                with status_lock:
                    s1 = motor_status[1].copy()
                    s2 = motor_status[2].copy()
                print(f"\n[时间 {elapsed:5.1f}s]")
                print(f"  电机1: 状态={s1['status_str']}, 位置={s1['position_rad']:.3f} rad, "
                      f"速度={s1['velocity_rps']:.2f} rad/s, 力矩={s1['torque_nm']:.3f} Nm, "
                      f"MOS={s1['mos_temp_c']}℃, 线圈={s1['coil_temp_c']}℃")
                print(f"  电机2: 状态={s2['status_str']}, 位置={s2['position_rad']:.3f} rad, "
                      f"速度={s2['velocity_rps']:.2f} rad/s, 力矩={s2['torque_nm']:.3f} Nm, "
                      f"MOS={s2['mos_temp_c']}℃, 线圈={s2['coil_temp_c']}℃")
                last_print_time = current

            # ---------- 高精度周期等待 ----------
            next_tick += period_sec
            sleep_until = next_tick - time.perf_counter()
            if sleep_until > 0:
                time.sleep(sleep_until * 0.8)       # 粗略睡眠大部分时间
                while time.perf_counter() < next_tick:   # 剩余时间自旋等待
                    pass
            else:
                # 超时：本次循环耗时过长，放弃追赶，重新校准 next_tick
                next_tick = time.perf_counter() + period_sec

        print("\n程序运行时间已到，正在停止...")

    except KeyboardInterrupt:
        print("\n用户手动中断，正在退出...")
    finally:
        thread_flag = False
        # 等待发送线程完成失能发送（最长等 3 秒）
        send_thread.join(timeout=3.0)
        if ch_handle:
            zcanlib.ResetCAN(ch_handle)
            print("CAN 通道已关闭")
        if dev_handle:
            zcanlib.CloseDevice(dev_handle)
            print("设备已关闭")

if __name__ == "__main__":
    main()