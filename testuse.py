import ctypes
import os

os.chdir(r"D:\Users\lmxlegion\Desktop\My personal project\DMarm_control")
try:
    dll = ctypes.CDLL("./zlgcan.dll")
except FileNotFoundError as e:
    # 获取更详细错误
    error_code = ctypes.get_last_error()
    print(f"错误码: {error_code}")  # 126 表示依赖缺失
    raise