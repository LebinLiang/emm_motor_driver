# 树莓派Python串口通讯控制 - 面向对象版本
# 基于EMM V5电机驱动协议，使用单个串口 /dev/ttyUSB0 控制多个电机

import time
import serial
import struct
from typing import Optional, Tuple


class SerialCommun:
    """串口通讯管理类"""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200, timeout: float = 0.1):
        self.uart = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        self.timeout = timeout
    
    def send_data(self, cmd: bytearray) -> None:
        self.uart.write(cmd)
    
    def receive_data(self) -> Tuple[str, int]:
        i = 0
        rxCmd = bytearray(128)
        start_time = time.time()
        last_recv_time = start_time
        
        while True:
            if self.uart.in_waiting > 0:
                if i < 128:
                    rxCmd[i] = ord(self.uart.read(1))
                    i += 1
                    last_recv_time = time.time()
            else:
                current_time = time.time()
                if (current_time - last_recv_time) > 0.02:  # 缩短超时时间以提高响应
                    hex_data = ' '.join(['{:02x}'.format(b) for b in rxCmd[:i]])
                    hex_data = hex_data.strip('00 ')
                    if hex_data and hex_data[0] != '0':
                        hex_data = '0' + hex_data
                    return hex_data, len(hex_data.replace(' ', '')) // 2
                if (current_time - start_time) > self.timeout:
                    return "", 0
    
    def close(self) -> None:
        if self.uart.is_open:
            self.uart.close()
    
    def __del__(self):
        self.close()


class Motor:
    """电机控制类"""
    
    FUNC_CODES = {
        'S_VER': 0x1F, 'S_RL': 0x20, 'S_PID': 0x21, 'S_VBUS': 0x24,
        'S_CPHA': 0x27, 'S_ENCL': 0x31, 'S_TPOS': 0x33, 'S_VEL': 0x35,
        'S_CPOS': 0x36, 'S_PERR': 0x37, 'S_FLAG': 0x3A, 'S_ORG': 0x3B,
        'S_Conf': 0x42, 'S_State': 0x43
    }
    
    CW = 0
    CCW = 1
    CHECKSUM = 0x6B
    
    def __init__(self, addr: int, serial_comm: SerialCommun):
        self.addr = addr
        self.serial_comm = serial_comm
        self.current_pos = 0.0
        self.is_enabled = False
    
    def _send_command(self, cmd: bytearray) -> None:
        self.serial_comm.send_data(cmd)
    
    def enable(self, sync: bool = False) -> None:
        cmd = bytearray(6)
        cmd[0] = self.addr
        cmd[1] = 0xF3
        cmd[2] = 0xAB
        cmd[3] = 0x01
        cmd[4] = 0x01 if sync else 0x00
        cmd[5] = self.CHECKSUM
        self._send_command(cmd)
        self.is_enabled = True
        time.sleep(0.005)
    
    def disable(self, sync: bool = False) -> None:
        cmd = bytearray(6)
        cmd[0] = self.addr
        cmd[1] = 0xF3
        cmd[2] = 0xAB
        cmd[3] = 0x00
        cmd[4] = 0x01 if sync else 0x00
        cmd[5] = self.CHECKSUM
        self._send_command(cmd)
        self.is_enabled = False
        time.sleep(0.005)
    
    def reset_position(self) -> None:
        cmd = bytearray(4)
        cmd[0] = self.addr
        cmd[1] = 0x0A
        cmd[2] = 0x6D
        cmd[3] = self.CHECKSUM
        self._send_command(cmd)
        self.current_pos = 0.0
        time.sleep(0.005)
    
    def velocity_control(self, direction: int, velocity: int, acceleration: int = 50, sync: bool = False) -> None:
        cmd = bytearray(16)
        cmd[0] = self.addr
        cmd[1] = 0xF6
        cmd[2] = direction
        cmd[3] = (velocity >> 8) & 0xFF
        cmd[4] = velocity & 0xFF
        cmd[5] = acceleration
        cmd[6] = 0x01 if sync else 0x00
        cmd[7] = self.CHECKSUM
        self._send_command(cmd[:8])
        time.sleep(0.005)
    
    def position_control(self, direction: int, velocity: int, acceleration: int, 
                        pulses: int, relative: bool = True, sync: bool = False) -> None:
        cmd = bytearray(16)
        cmd[0] = self.addr
        cmd[1] = 0xFD
        cmd[2] = direction
        cmd[3] = (velocity >> 8) & 0xFF
        cmd[4] = velocity & 0xFF
        cmd[5] = acceleration
        cmd[6] = (pulses >> 24) & 0xFF
        cmd[7] = (pulses >> 16) & 0xFF
        cmd[8] = (pulses >> 8) & 0xFF
        cmd[9] = pulses & 0xFF
        cmd[10] = 0x01 if relative else 0x00
        cmd[11] = 0x01 if sync else 0x00
        cmd[12] = self.CHECKSUM
        self._send_command(cmd[:13])
        
        if relative:
            angle_change = (pulses / 65536.0) * 360.0
            if direction == self.CW:
                self.current_pos += angle_change
            else:
                self.current_pos -= angle_change
        time.sleep(0.005)
    
    def stop_now(self, sync: bool = False) -> None:
        cmd = bytearray(5)
        cmd[0] = self.addr
        cmd[1] = 0xFE
        cmd[2] = 0x98
        cmd[3] = 0x01 if sync else 0x00
        cmd[4] = self.CHECKSUM
        self._send_command(cmd)
        time.sleep(0.005)
    
    def read_system_params(self, param: str) -> None:
        if param not in self.FUNC_CODES:
            return
        cmd = bytearray(16)
        cmd[0] = self.addr
        cmd[1] = self.FUNC_CODES[param]
        cmd[2] = 0x6B
        self._send_command(cmd[:3])
        # time.sleep(0.005) # Removed sleep to speed up
    
    def get_current_position(self) -> float:
        self.read_system_params('S_CPOS')
        data, count = self.serial_comm.receive_data()
        if data:
            data_hex = data.split()
            if count > 0 and len(data_hex) >= 7:
                if int(data_hex[0], 16) == self.addr and int(data_hex[1], 16) == 0x36:
                    pos = struct.unpack('>I', bytes.fromhex(''.join(data_hex[3:7])))[0]
                    angle = float(pos) * 360.0 / 65536.0
                    if int(data_hex[2], 16):
                        angle = -angle
                    self.current_pos = angle
                    return angle
        return self.current_pos
    
    def get_velocity(self) -> float:
        self.read_system_params('S_VEL')
        data, count = self.serial_comm.receive_data()
        if data:
            data_hex = data.split()
            if count > 0 and len(data_hex) >= 4:
                if int(data_hex[0], 16) == self.addr and int(data_hex[1], 16) == 0x35:
                    velocity = struct.unpack('>H', bytes.fromhex(''.join(data_hex[2:4])))[0]
                    return float(velocity)
        return 0.0
    
    def get_status(self) -> dict:
        return {
            'position': self.get_current_position(),
            'velocity': self.get_velocity()
        }


class MotorController:
    """电机控制器"""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        self.serial_comm = SerialCommun(port, baudrate)
        self.motors = {}
    
    def add_motor(self, addr: int, name: str = None) -> Motor:
        motor = Motor(addr, self.serial_comm)
        self.motors[addr] = motor
        if name:
            self.motors[name] = motor
        return motor
    
    def get_motor(self, addr: int) -> Optional[Motor]:
        return self.motors.get(addr)
    
    def synchronous_motion(self) -> None:
        cmd = bytearray(4)
        cmd[0] = 0x00
        cmd[1] = 0xFF
        cmd[2] = 0x66
        cmd[3] = Motor.CHECKSUM
        self.serial_comm.send_data(cmd)
        time.sleep(0.005)

    def synchronous_position_control(self, motor_commands: list) -> None:
        for cmd_dict in motor_commands:
            motor = self.get_motor(cmd_dict['addr'])
            if motor:
                motor.position_control(
                    direction=cmd_dict.get('direction', Motor.CW),
                    velocity=cmd_dict.get('velocity', 500),
                    acceleration=cmd_dict.get('acceleration', 50),
                    pulses=cmd_dict.get('pulses', 3600),
                    relative=cmd_dict.get('relative', True),
                    sync=True
                )
        self.synchronous_motion()

    def synchronous_velocity_control(self, motor_commands: list) -> None:
        for cmd_dict in motor_commands:
            motor = self.get_motor(cmd_dict['addr'])
            if motor:
                motor.velocity_control(
                    direction=cmd_dict.get('direction', Motor.CW),
                    velocity=cmd_dict.get('velocity', 500),
                    acceleration=cmd_dict.get('acceleration', 50),
                    sync=True
                )
        self.synchronous_motion()
    
    def close(self) -> None:
        self.serial_comm.close()
