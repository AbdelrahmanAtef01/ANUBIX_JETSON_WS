import serial
import time
import threading


class Command():
    """Communication message:
        Header(int):Frame Header
        Len(int):length
        ID(int):Gripper ID
        Code(int):Function code(6-write,3-read)
        Number_High(int):Instruction number high byte
        Number_LOW(int):Instruction number low byte
        Value_High(int):Parameter high byte
        Value_LOW(int):Parameter low byte

    """
    Header = 254
    Len = 8
    ID = 14
    Code = 0
    Zero = 0
    Number_High = 0
    Number_LOW = 0
    Value_High = 0
    Value_LOW = 0
    cmd_list = [Header, Header, Len, ID, Code, Number_High, Number_LOW, Value_High, Value_LOW]


class Gripper(Command):

    def __init__(self, port, baudrate=115200, id=14):
        self.lock = threading.Lock()
        self.port = port
        self.baudrate = baudrate
        self.ser = serial.Serial(port, baudrate, timeout=5)
        self.cmd_list[3] = id

    def check_value(self, value, lower, upper, index=1):
        valid_values = list(range(lower, upper + 1))
        if isinstance(value, list):
            for i, val in enumerate(value):
                if val not in valid_values:
                    raise ValueError(
                        f"The {index} input value at position {i + 1} is invalid. Valid values between [{lower},{upper}]")
                    return False
            return True
        else:
            if value in valid_values:
                return True
            else:
                if index == 1:
                    raise ValueError(f"The first input value can be selected as: [{lower},{upper}]")
                else:
                    raise ValueError(f"The second input value can be selected as: [{lower},{upper}]")
                return False

    def __byte_deal(self, value1, value2):
        high_byte1 = (value1 >> 8) & 0xFF
        low_byte1 = value1 & 0xFF
        high_byte2 = (value2 >> 8) & 0xFF
        low_byte2 = value2 & 0xFF
        return [high_byte1, low_byte1, high_byte2, low_byte2]

    def __crc16_modbus(self, data: bytes) -> bytes:
        crc = 0xFFFF
        for pos in data:
            crc ^= pos
            for _ in range(8):
                if (crc & 0x0001) != 0:
                    crc >>= 1
                    crc ^= 0xA001
                else:
                    crc >>= 1
        return crc.to_bytes(2, byteorder='big')

    def __send_cmd(self, cmd, is_special_interface=False):
        with self.lock:
            send_data = cmd + self.__crc16_modbus(cmd)
            self.ser.write(send_data)
            self.ser.flush()
            time.sleep(0.04)
            recv_data = self.ser.read(11)
            if not recv_data:
                raise TimeoutError("Reading data timeout")
            if len(recv_data) == 11:
                data = recv_data[0:9]
                crc_data = recv_data[9:]
                if self.__crc16_modbus(data) == crc_data:
                    response = data + crc_data
                    if is_special_interface:
                        result = int(response.hex()[14:16], 16)
                    else:
                        result = int(response.hex()[14:18], 16)
                    return result
                else:
                    return -2
            else:
                return -1

    def set_gripper_value(self, value, speed=100):
        if self.check_value(value, 0, 100):
            if self.check_value(speed, 1, 100, index=2):
                self.set_gripper_speed(speed)
                self.cmd_list[4] = 6
                tmp = self.__byte_deal(11, value)
                for i in range(5, 9):
                    self.cmd_list[i] = tmp[i - 5]
                cmd = bytes(self.cmd_list)
                return self.__send_cmd(cmd)

    def set_gripper_speed(self, value):
        if self.check_value(value, 1, 100):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(32, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def set_gripper_enable(self, value):
        if self.check_value(value, 0, 1):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(10, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def set_gripper_calibration(self):
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(13, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_gripper_value(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(12, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_gripper_status(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(14, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_firmware_version(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(1, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_modified_version(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(2, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_Id(self, value):
        if self.check_value(value, 1, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(3, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            self.cmd_list[3] = value
            return self.__send_cmd(cmd)

    def get_gripper_Id(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(4, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_baud(self, value=0):
        if self.check_value(value, 0, 5):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(5, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_baud(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(6, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_torque(self, value):
        if self.check_value(value, 0, 100):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(27, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_torque(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(28, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_stop(self):
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(39, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_P(self, value):
        if self.check_value(value, 0, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(15, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_P(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(16, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_D(self, value):
        if self.check_value(value, 0, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(17, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_D(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(18, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_I(self, value):
        if self.check_value(value, 0, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(19, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_I(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(20, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_mini_pressure(self, value):
        if self.check_value(value, 0, 254):
            self.cmd_list[4] = 6
            tmp = self.__byte_deal(25, value)
            for i in range(5, 9):
                self.cmd_list[i] = tmp[i - 5]
            cmd = bytes(self.cmd_list)
            return self.__send_cmd(cmd)

    def get_gripper_mini_pressure(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(26, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_output(self, value=0):
        if self.check_value(value, 0, 3):
            if value == 2:
                value = 16
            elif value == 3:
                value = 17
        else:
            return None
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(29, value)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def get_gripper_speed(self):
        self.cmd_list[4] = 3
        tmp = self.__byte_deal(33, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_state(self, value, speed=100):
        if self.check_value(value, 0, 1):
            if self.check_value(speed, 1, 100):
                self.set_gripper_speed(speed)
                if value == 1:
                    return self.set_gripper_value(100)
                elif value == 0:
                    return self.set_gripper_value(0)

    def set_gripper_pause(self):
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(37, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def set_gripper_resume(self):
        self.cmd_list[4] = 6
        tmp = self.__byte_deal(38, 0)
        for i in range(5, 9):
            self.cmd_list[i] = tmp[i - 5]
        cmd = bytes(self.cmd_list)
        return self.__send_cmd(cmd)

    def close(self):
        self.ser.close()
