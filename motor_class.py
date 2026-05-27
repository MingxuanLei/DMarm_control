import struct
import math
from typing import Tuple, Optional

class DMMotor:
    # 静态命令字节数组（8字节）
    enable_command = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC])
    disable_command = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD])
    clear_error_command = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB])
    set_zero_command = bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE])

    PARAM_SET_ID = 0x7FF

    def __init__(self, motor_id: int, motor_type: int, mode: str):
        """
        初始化电机
        :param motor_id: 电机ID (0~255)
        :param motor_type: 电机型号 (4340, 4310, 3507 或其他)
        :param mode: 上电模式 ("MIT", "PV", "PVT")
        """
        self.id = motor_id
        self.master_id = motor_id + 0x010

        # 根据型号设置最大参数
        if motor_type == 4340:
            self.max_position = 12.5
            self.max_velocity = 10.0
            self.max_torque = 28.0
        elif motor_type == 4310:
            self.max_position = 12.5
            self.max_velocity = 30.0
            self.max_torque = 10.0
        elif motor_type == 3507:
            self.max_position = 12.5
            self.max_velocity = 50.0
            self.max_torque = 5.0
        else:
            # 默认值
            self.max_position = 12.5
            self.max_velocity = 10.0
            self.max_torque = 28.0

        self.kp_max = 500.0
        self.kd_max = 5.0

        # 当前状态反馈
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.ERR = 0          # 错误码
        self.tem_mos = 0
        self.tem_rotor = 0
        self.enable = False

        # 命令缓冲区
        self.mit_command = bytearray(8)
        self.pv_command = bytearray(8)
        self.pvt_command = bytearray(8)

        self.recv_num = 0

        # 模式切换命令 (前导字节为电机ID)
        self.set_mit_command = bytearray([motor_id, 0x00, 0x55, 10, 0x01, 0x00, 0x00, 0x00])
        self.set_pv_command = bytearray([motor_id, 0x00, 0x55, 10, 0x02, 0x00, 0x00, 0x00])
        self.set_pvt_command = bytearray([motor_id, 0x00, 0x55, 10, 0x04, 0x00, 0x00, 0x00])
        self.get_mode_command = bytearray([motor_id, 0x00, 0x33, 10, 0x00, 0x00, 0x00, 0x00])

        # 模式结构体
        class MIT:
            def __init__(self):
                self.position_set = 0.0
                self.velocity_set = 0.0
                self.torque_set = 0.0
                self.kp_set = 0.0
                self.kd_set = 0.0

        class PV:
            def __init__(self):
                self.position_set = 0.0
                self.velocity_lim = 0.0

        class PVT:
            def __init__(self):
                self.position_set = 0.0
                self.velocity_lim = 0.0
                self.torque_lim = 0.0

        self.MIT = MIT()
        self.PV = PV()
        self.PVT = PVT()

        # 角度限制（未在C#中使用，保留接口）
        self.angle_lim = [-math.pi, math.pi]

        # 当前模式：1=MIT, 2=PV, 4=PVT；负数表示正在切换
        if mode == "MIT":
            self.mode = 1
        elif mode == "PV":
            self.mode = 2
        elif mode == "PVT":
            self.mode = 4
        else:
            self.mode = 2   # 默认PV

        self.set_empty_command()

    # -----------------------------------------------------------------
    # 属性（只读）
    # -----------------------------------------------------------------
    @property
    def ERRCODE(self) -> str:
        """错误码文本描述"""
        err = self.ERR
        if err == 0:
            return "失能"
        elif err == 1:
            return "使能"
        elif err == 0x08:
            return "超压"
        elif err == 0x09:
            return "欠压"
        elif err == 0x0A:
            return "过流"
        elif err == 0x0B:
            return "MOS过热"
        elif err == 0x0C:
            return "线圈过热"
        elif err == 0x0D:
            return "通讯丢失"
        elif err == 0x0E:
            return "过载"
        else:
            return "??"

    @property
    def Position(self) -> float:
        return self.position

    @property
    def Velocity(self) -> float:
        return self.velocity

    @property
    def Torque(self) -> float:
        return self.torque

    @property
    def Command(self) -> bytes:
        """根据当前模式返回要发送的命令字节数组"""
        if self.mode == 1:
            return bytes(self.mit_command)
        elif self.mode == 2:
            return bytes(self.pv_command)
        elif self.mode == 4:
            return bytes(self.pvt_command)
        elif self.mode == -1:
            return bytes(self.set_mit_command)
        elif self.mode == -2:
            return bytes(self.set_pv_command)
        else:
            return bytes(self.disable_command)

    @property
    def ID_OFFSET(self) -> int:
        """根据模式返回CAN ID偏移后的值"""
        if self.mode == 1:
            return self.id
        elif self.mode == 2:
            return self.id + 0x100
        elif self.mode == 4:
            return self.id + 0x300
        elif self.mode in (-1, -2, -4):
            return self.PARAM_SET_ID
        else:
            return self.id

    @property
    def ID(self) -> int:
        return self.id

    @property
    def ID_MASTER(self) -> int:
        return self.master_id

    @property
    def Enable(self) -> bool:
        return self.enable

    @property
    def Mode(self) -> int:
        return self.mode

    @property
    def ModeName(self) -> str:
        if self.mode == 1:
            return "MIT"
        elif self.mode == 2:
            return "PV"
        elif self.mode == 4:
            return "PVT"
        else:
            return "??"

    # -----------------------------------------------------------------
    # 私有命令转换方法 (静态方法)
    # -----------------------------------------------------------------
    @staticmethod
    def _convert_to_candata_MIT(position: float, velocity: float, torque: float,
                                 kp: float, kd: float,
                                 max_pos: float, max_vel: float, max_tor: float,
                                 kp_max: float, kd_max: float) -> bytes:
        """MIT模式编码，返回8字节命令"""
        # 位置 (16位)
        pos_enc = int(((position + max_pos) * 65535) / (max_pos * 2))
        pos_enc = max(0, min(65535, pos_enc))
        b0 = (pos_enc >> 8) & 0xFF
        b1 = pos_enc & 0xFF

        # 速度 (12位)
        vel_enc = int(((velocity + max_vel) * 4095) / (max_vel * 2))
        vel_enc = max(0, min(4095, vel_enc))
        b2 = (vel_enc >> 4) & 0xFF

        # KP (12位)
        kp_enc = int(kp * 4095 / kp_max)
        kp_enc = max(0, min(4095, kp_enc))
        b3 = ((vel_enc & 0x0F) << 4) | ((kp_enc >> 8) & 0x0F)
        b4 = kp_enc & 0xFF

        # KD (12位)
        kd_enc = int(kd * 4095 / kd_max)
        kd_enc = max(0, min(4095, kd_enc))
        b5 = (kd_enc >> 4) & 0xFF

        # 扭矩 (12位)
        tor_enc = int(((torque + max_tor) * 4095) / (max_tor * 2))
        tor_enc = max(0, min(4095, tor_enc))
        b6 = ((kd_enc & 0x0F) << 4) | ((tor_enc >> 8) & 0x0F)
        b7 = tor_enc & 0xFF

        return bytes([b0, b1, b2, b3, b4, b5, b6, b7])

    @staticmethod
    def _convert_to_candata_PV(position: float, velocity: float) -> bytes:
        """PV模式编码，返回8字节命令 (小端两个float)"""
        return struct.pack('<ff', position, velocity)

    @staticmethod
    def _convert_to_candata_PVT(position: float, velocity: int, current: int) -> bytes:
        """PVT模式编码 (position: float, velocity: uint16, current: uint16)，返回8字节"""
        velocity = min(velocity, 5000)
        current = min(current, 5000)
        pos = min(position, 2.35)
        return struct.pack('<fHH', pos, velocity, current)

    @staticmethod
    def _convert_to_candata_PVT_from_torque(position: float, velocity: int, torque: float) -> bytes:
        """PVT模式重载，根据扭矩估算电流"""
        current = int(abs(torque) * 1800)
        return DMMotor._convert_to_candata_PVT(position, velocity, current)

    # -----------------------------------------------------------------
    # 反馈解析
    # -----------------------------------------------------------------
    def read_motor(self, candata: bytes) -> bool:
        """
        解析电机反馈的8字节CAN数据，更新内部状态
        :param candata: 8字节bytes
        :return: 解析是否成功（ID匹配）
        """
        if len(candata) < 8:
            return False

        self.recv_num += 1

        # 检查ID是否匹配 (低4位)
        if (candata[0] & 0x0F) != self.id:
            return False

        self.ERR = (candata[0] >> 4) & 0x0F

        # 位置 (16位无符号)
        pos_raw = (candata[1] << 8) | candata[2]
        self.position = (pos_raw / 65536.0) * self.max_position * 2 - self.max_position

        # 速度 (12位，偏移2048)
        vel_raw = ((candata[3] << 4) | (candata[4] >> 4)) & 0xFFF
        self.velocity = (vel_raw - 2048) / 4096.0 * self.max_velocity * 2

        # 扭矩 (12位，偏移2048)
        tor_raw = (((candata[4] & 0x0F) << 8) | candata[5]) & 0xFFF
        self.torque = (tor_raw - 2048) / 4096.0 * self.max_torque * 2

        self.tem_mos = candata[6]
        self.tem_rotor = candata[7]

        if self.ERR == 0:
            self.enable = False
        elif self.ERR == 1:
            self.enable = True

        return True

    def get_motor_mode(self, candata: bytes) -> bool:
        """
        解析模式切换命令的反馈，更新self.mode
        :param candata: 至少8字节的反馈数据
        :return: 如果是模式切换反馈则返回True
        """
        if len(candata) < 8:
            return False
        if (candata[2] == 0x33 or candata[2] == 0x55) and candata[3] == 10:
            self.mode = candata[4]
            return True
        return False

    # -----------------------------------------------------------------
    # 设置当前目标值并生成命令
    # -----------------------------------------------------------------
    def set(self):
        """根据当前模式及结构体中的目标值，更新对应的命令缓冲区"""
        if self.mode == 1:   # MIT
            self.mit_command = bytearray(self._convert_to_candata_MIT(
                self.MIT.position_set, self.MIT.velocity_set, self.MIT.torque_set,
                self.MIT.kp_set, self.MIT.kd_set,
                self.max_position, self.max_velocity, self.max_torque,
                self.kp_max, self.kd_max
            ))
        elif self.mode == 2:   # PV
            self.pv_command = bytearray(self._convert_to_candata_PV(
                self.PV.position_set, self.PV.velocity_lim
            ))
        elif self.mode == 4:   # PVT
            self.pvt_command = bytearray(self._convert_to_candata_PVT_from_torque(
                self.PVT.position_set,
                int(self.PVT.velocity_lim * 100),
                self.PVT.torque_lim
            ))
        else:
            self.set_empty_command()

    # -----------------------------------------------------------------
    # 清空命令缓冲区
    # -----------------------------------------------------------------
    def set_empty_command(self):
        self._set_empty_command_pv()
        self._set_empty_command_mit()
        self._set_empty_command_pvt()

    def _set_empty_command_pv(self):
        self.pv_command = bytearray(self._convert_to_candata_PV(0.0, 0.0))
        self.PV = self.PV.__class__()   # 重置结构体

    def _set_empty_command_mit(self):
        self.mit_command = bytearray(self._convert_to_candata_MIT(
            0.0, 0.0, 0.0, 0.0, 0.0,
            self.max_position, self.max_velocity, self.max_torque,
            self.kp_max, self.kd_max
        ))
        self.MIT = self.MIT.__class__()

    def _set_empty_command_pvt(self):
        self.pvt_command = bytearray(self._convert_to_candata_PVT(0.0, 0, 0))
        self.PVT = self.PVT.__class__()

    # -----------------------------------------------------------------
    # 模式切换
    # -----------------------------------------------------------------
    def set_mode(self, mode: int):
        """
        请求切换模式（mode: 1=MIT, 2=PV, 4=PVT）
        实际切换需要发送对应命令并从反馈中确认，这里仅设置负标志并清空命令
        """
        self.mode = -mode
        self.set_empty_command()