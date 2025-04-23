import struct
from math import pi as PI

P_RANGE = (-12.5, 12.5) # 4 * PI ~ 12.5
V_RANGE = (-30.0, 30.0)
KP_RANGE = (0.0, 500.0)
KD_RANGE = (0.0, 5.0)
T_RANGE = (-12.0, 12.0)


IQ_RANGE = (-23.0, 23.0)
SPD_RANGE = (-30.0, 30.0)
LIMIT_T_RANGE = (0, 12.0)
FILT_GAIN_RANGE = (0.0, 1.0)
LIMIT_SPD_RANGE = (0.0, 30.0)
LIMIT_CUR_RANGE = (0.0, 23.0)
MECHVEL_RANGE = (-30.0, 30.0)
VBUS_RANGE = (0.0, 60.0)

class MiData:
    """
    小米的电机的通信数据
    """

    def __init__(self, mode:int, data2:int, id:int, data1:bytes | None = None):
        self.mode   = mode
        self.data2  = data2
        self.id     = id
        self.data1  = data1 if data1 is not None else bytes(8)


class YourCeeBridge:

    @staticmethod
    def pack_id(raw: MiData):
        id_feild = raw.mode << 24 | raw.data2 << 8 | raw.id
        # YourCeeUSBCan 的协议
        # 末端三位是 0b100，有特殊的含义
        # 详见 https://wit-motion.yuque.com/wumwnr/docs/nrngkq 关于 `CAN_ID(报文标识符)说明`
        id_value = id_feild << 3 | 0b100
        return struct.pack('>I', id_value)
    
    @staticmethod
    def unpack_id(data: bytes):
        id_value = struct.unpack('>I', data)[0]
        id_feild = id_value >> 3
        mode = (id_feild >> 24) & 0xFF
        data2 = (id_feild >> 8) & 0xFFFF
        id = id_feild & 0xFF
        return MiData(mode, data2, id, None)
    
    @staticmethod
    def pack_AT(raw: MiData):
        id = YourCeeBridge.pack_id(raw)
        data1 = raw.data1
        return b'AT' + id + b'\x08' + data1 + b'\r\n'
    
    @staticmethod
    def unpack_AT(data: bytes):
        id = data[2:6]
        data1 = data[7:-2]
        miData = YourCeeBridge.unpack_id(id)
        miData.data1 = data1
        return miData


def float_to_bytes(value: float, range: tuple[float, float], bytes_len: int):
    min_value, max_value = range
    value = max(min(value, max_value), min_value)
    ratio = (value - min_value) / (max_value - min_value)
    int_value = int(ratio * (2 ** (bytes_len * 8) - 1))
    return int_value

def torque_encode(value: float): return float_to_bytes(value, T_RANGE, 2)
def angle_encode(value: float): return float_to_bytes(value, P_RANGE, 2)
def speed_encode(value: float): return float_to_bytes(value, V_RANGE, 2)
def kp_encode(value: float): return float_to_bytes(value, KP_RANGE, 2)
def kd_encode(value: float): return float_to_bytes(value, KD_RANGE, 2)

def bytes_to_float(value: int, range: tuple[float, float], bytes_len: int):
    min_value, max_value = range
    ratio = value / (2 ** (bytes_len * 8) - 1)
    return ratio * (max_value - min_value) + min_value

def angle_decode(value: int): return bytes_to_float(value, P_RANGE, 2)
def speed_decode(value: int): return bytes_to_float(value, V_RANGE, 2)
def torque_decode(value: int): return bytes_to_float(value, T_RANGE, 2)
def temp_decode(value: int): return value / 10.0

def Req0(target: int, host: int):
    return MiData(
        mode=0,
        data2=host & 0x00FF,
        id=target,
        data1=None
    )

class Res0:
    def __init__(self, raw: MiData):
        self.raw = raw
        assert raw.mode == 0, 'mode error'
        assert raw.id == 0xFE, 'id error'
        self.id = raw.data2 & 0xFFFF
        self.mcu_id = raw.data1
    
    def __str__(self):
        return f'Res0(id={self.id}, mcu_id={self.mcu_id.hex()})'

def Req1(target: int, torque: float, angle: float, speed: float, kp: float, kd: float):
    """
    torque: 没有说明白是大端还是小端

    其他变量在代码中体现了是小端
    """
    torque = torque_encode(torque)
    angle = angle_encode(angle)
    speed = speed_encode(speed)
    kp = kp_encode(kp)
    kd = kd_encode(kd)

    data1 = struct.pack('<HHHH', angle, speed, kp, kd)
    return MiData(
        mode=1,
        data2=torque,
        id=target,
        data1=data1
    )

def extract_bits(n, x, y):
    # 构造掩码
    mask = ((1 << (y - x + 1)) - 1) << x  # 从第 x 位到第 y 位为 1，其余位为 0
    # 提取目标位
    result = (n & mask) >> x
    return result

from enum import Enum

class ModeStatus(Enum):
    unknown = -1
    reset = 0
    cali = 1
    motor = 2


class Res2:
    def __init__(self, raw: MiData):
        self.raw = raw
        assert raw.mode == 2, 'mode error'
        self.host = raw.id
        def bits(x, y): return extract_bits(raw.data2, x-8, y-8)
        self.target = bits(8, 15)
        self.mode_status = Res2.get_mode_status(bits(22, 23))

        ## 异常区域
        self.error = bits(16, 21) != 0
        def check_bit(n): return bits(n, n) == 1
        self.error_undefined = check_bit(21)
        self.error_hall = check_bit(20)
        self.error_magnetic = check_bit(19)
        self.error_overheat = check_bit(18)
        self.error_overcurrent = check_bit(17)
        self.error_undervoltage = check_bit(16)
        
        def decode(x: int, y: int, decode_func: Callable[[int], float]):
            value = struct.unpack('<H', raw.data1[x:y])[0]
            return decode_func(value)
        self.angle = decode(0, 2, angle_decode)
        self.speed = decode(2, 4, speed_decode)
        self.torque = decode(4, 6, torque_decode)
        self.temp = decode(6, 8, temp_decode)

    @staticmethod
    def get_mode_status(value: int):
        for mode in ModeStatus:
            if value == mode.value:
                return mode
        else:
            return ModeStatus.unknown
    
    def __str__(self):
        return f'Res2(host={self.host}, target={self.target}, mode_status={self.mode_status}, error={self.error}, angle={self.angle}, speed={self.speed}, torque={self.torque}, temp={self.temp})'

def Req3(target: int, host: int):
    return MiData(
        mode=3,
        data2=host & 0x00FF,
        id=target,
    )

def Req4(target: int, host: int, fix_error = False):
    data1 = bytes(8)
    if fix_error: data1[0] = 0x01
    return MiData(
        mode=4,
        data2=host & 0x00FF,
        id=target,
        data1=data1
    )

def Req6(target: int, host: int):
    data1 = bytes(8)
    data1[0] = 0x01
    return MiData(
        mode=6,
        data2=host & 0x00FF,
        id=target,
        data1=data1
    )

def Req7(target: int, host: int, new_target: int):
    data2 = (new_target & 0x00FF) << 8 | (host & 0x00FF)
    return MiData(
        mode=7,
        data2=data2,
        id=target,
    )

from typing import Callable

class Args(Enum):
    run_mode = (
        0x7005, 
        lambda x: x[0] & 0xFF,
        lambda x: bytes([x & 0xFF]) + bytes(3),
    )
    iq_ref  = (
        0x7006, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    spd_ref = (
        0x700A, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    limit_torque = (
        0x700B, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    cur_kp = (
        0x7010, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    cur_ki = (
        0x7011, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    cur_filt_gain = (
        0x7014, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    loc_ref = (
        0x7016,
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    limit_spd = (
        0x7017, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    limit_cur = (
        0x7018, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    mechPos = (
        0x7019, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    iqf = (
        0x701A, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    mechVel = (
        0x701B, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    VBUS = (
        0x701C, 
        lambda x: struct.unpack('<f', x)[0],
        lambda x: struct.pack('<f',  float(x)),
    )
    rotation = (
        0x701D,
        lambda x: struct.unpack('<H', x[0:2])[0],
        lambda x: struct.pack('<H', x) + bytes(2),
    )
    def __init__(self, 
                 index: int, 
                 read: Callable[[bytes], int | float], 
                 write: Callable[[int | float], bytes]
        ):
        self.index = index
        self.read = read
        self.write = write
    
    @staticmethod
    def get_index(value: int):
        for arg in Args:
            if value == arg.index:
                return arg
        else:
            raise ValueError(f"Invalid index: {value}")

def Req17(target: int, host: int, arg: Args):
    return MiData(
        mode=17,
        data2=host & 0x00FF,
        id=target,
        data1=struct.pack('<H', arg.index) + bytes(6)
    )

class Res17:
    def __init__(self, raw: MiData):
        self.raw = raw
        assert raw.mode == 17, 'mode error'
        self.host = raw.id
        self.target = raw.data2 & 0x00FF
        self.arg = Args.get_index(struct.unpack('<H', raw.data1[0:2])[0])
        self.param = self.arg.read(raw.data1[4:])
    
    def __str__(self):
        return f'Res17(host={self.host}, target={self.target}, arg={self.arg}, param={self.param})'

def Req18(target: int, host: int, arg: Args, param: float | int):
    return MiData(
        mode=18,
        data2=host & 0x00FF,
        id=target,
        data1=struct.pack('<H', arg.index) + bytes(2) + arg.write(param) 
    )

class Res21:
    def __init__(self, raw: MiData):
        self.raw = raw
        assert raw.mode == 21, 'mode error'
        self.target = raw.id
        self.host = raw.data2 & 0x00FF

        error_code = struct.unpack('<I', raw.data1[0:4])[0]
        self.error = False if error_code == 0 else True
        self.warning_value = struct.unpack('<I', raw.data1[4:8])[0]
        if not self.error: return
        def check_bit(n): return extract_bits(error_code, n, n) == 1
        self.error_a_phase_overcurrent = check_bit(16)
        self.error_b_phase_overcurrent = check_bit(15)
        self.error_c_phase_overcurrent = check_bit(14)
        self.error_encoder_not_calibrated = check_bit(7)
        self.error_overload = False if extract_bits(error_code, 8, 15) == 0 else True
        self.error_overvoltage = check_bit(3)
        self.error_undervoltage = check_bit(2)
        self.error_driver_chip = check_bit(1)
        self.error_motor_overheat = check_bit(0)
    
    def __str__(self):
        return f'Res21(host={self.host}, target={self.target}, error={self.error}, warning_value={self.warning_value})'

def get_req(bytes: bytes, decode: Callable[[bytes], MiData]):
    data = decode(bytes)
    match data.mode:
        case 0: return Res0(data)
        case 2: return Res2(data)
        case 17: return Res17(data)
        case 21: return Res21(data)

class RunMode(Enum):
    control = 0 # 运控模式
    current = 3 # 电流模式
    speed = 2 # 速度模式
    position = 1 # 位置模式

import serial
import logging

logger = logging.getLogger(__name__)

class YourCeePort:
    def __init__(self, port: str, mode = 'AT', host = 0, timeout=1):
        self.serial = serial.Serial(port, 921600, timeout=timeout)
        self.mode = mode
        self.host = host
    
    @property
    def mode(self): return self._mode
    @mode.setter
    def mode(self, value: str):
        self._mode = value
        match self.mode:
            case 'AT':
                self.serial.write(b'AT+AT\r\n')
                self.encode = YourCeeBridge.pack_AT
                def deocde(x: bytes): return get_req(x, YourCeeBridge.unpack_AT)
                self.decode  = deocde
                self.size = 2 + 4 + 1 + 8 + 2
    
    def get_run_mode(self, target: int): 
        data = self.read_single_arg(target, Args.run_mode)
        for mode in RunMode:
            if data == mode.value:
                return mode
        else:
            raise ValueError(f"Invalid run mode: {data}")

    def set_run_mode(self, target: int, value: RunMode):
        self.write_single_arg(target, Args.run_mode, value.value)
        data = self.read()

    
    def write(self, data: MiData): 
        bytes = self.encode(data)
        logger.info(f'write: {bytes.hex()}')
        return self.serial.write(bytes)
    def read(self): 
        bytes = self.serial.read(self.size)
        if len(bytes) < self.size: return None
        data = self.decode(bytes)
        logger.info(f'read : {bytes.hex()}\t = {data}')
        return data
    
    def request_id(self, target: int):
        self.write(Req0(target, self.host))
    
    def wait_for_id(self):
        data: Res0 | None = self.read()
        if data is None: return None
        return data.id
    
    def search_id(self):
        for i in range(256):
            self.request_id(i)
            id = self.wait_for_id()
            if id is not None:
                return id
        else:
            raise ValueError("No ID found")
    
    def enable(self, target: int):
        self.write(Req3(target, self.host))
        data = self.read()
    
    def disable(self, target: int, fix_error = False):
        self.write(Req4(target, self.host, fix_error))
        data = self.read()

    def write_single_arg(self, target: int, arg: Args, param: float | int):
        self.write(Req18(target, self.host, arg, param))
        data = self.read()
        if data is None: return None
        # return data.param
    
    def read_single_arg(self, target: int, arg: Args):
        self.write(Req17(target, self.host, arg))
        data:Res17 = self.read()
        if data is None: return None
        return data.param
    
    def control(self, target: int, torque: float, angle: float, speed: float, kp: float, kd: float):
        self.write(Req1(target, torque, angle, speed, kp, kd))
        data = self.read()
        if data is None: return None
        # return data.param

    def set_zero(self, target: int):
        self.write(Req6(target, self.host))
        data = self.read()

    def set_id(self, target: int, new_target: int):
        self.write(Req7(target, self.host, new_target))
        data = self.read()
        if data is None: return None
        # return data.param

    def close(self):
        self.serial.close()

def test_control_mode():
    """
    默认模式
    """
    import time
    port = YourCeePort('COM10')
    id = port.search_id()
    print(f'ID: {id}')
    port.set_run_mode(id, RunMode.control)
    time.sleep(1)
    port.enable(id)
    time.sleep(1)
    # 我就看不懂
    port.control(id, 5, 3, 10, 20, 5)
    time.sleep(3)
    port.disable(id)
    time.sleep(1)
    port.close()

def test_cur_mode():
    import time
    port = YourCeePort('COM10')
    id = port.search_id()
    print(f'ID: {id}')
    port.set_run_mode(id, RunMode.current)
    port.enable(id)
    port.write_single_arg(id, Args.iq_ref, 0.5)
    time.sleep(3)
    port.disable(id)
    port.close()

def test_vel_mode():
    import time
    port = YourCeePort('COM10', 'AT', 0xFD)
    id = port.search_id()
    print(f'ID: {id}')
    port.set_run_mode(id, RunMode.speed)
    port.enable(id)
    # 为啥没啥反应
    port.write_single_arg(id, Args.limit_cur, 5.0)
    print("set spd_ref")
    port.write_single_arg(id, Args.spd_ref, 1.0)
    time.sleep(3)
    port.disable(id)
    port.close()

def test_pos_mode():
    import time
    port = YourCeePort('COM10', 'AT', 0xFD)
    id = port.search_id()
    print(f'ID: {id}')
    port.set_run_mode(id, RunMode.position)
    port.enable(id)
    # 我就看不懂
    port.write_single_arg(id, Args.limit_spd, 5.0)
    print("set loc_ref")
    port.write_single_arg(id, Args.loc_ref, -12.0)
    time.sleep(3)
    port.disable(id)
    port.close()

def test_set_id():
    import time
    port = YourCeePort('COM10', 'AT', 0xFD)
    id = port.search_id()
    print(f'ID: {id}')
    new_id = id + 1
    port.set_id(id, new_id)
    check_id = port.search_id()
    assert check_id == new_id, f"ID not match: {check_id}"

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    # test_control_mode()
    # test_cur_mode()
    test_vel_mode()
    # test_pos_mode()
    # test_set_id()