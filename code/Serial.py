import serial
import serial.tools.list_ports
import time

class SerialCommunication:
    def __init__(self):
        """
        初始化串口通信

        参数:
            port: 串口号，如果为None则自动检测
            baudrate: 波特率，默认115200
            timeout: 超时时间(秒)，默认1
        """
        self.port = "/dev/ttyUSB0"  # 默认串口号
        self.baudrate = 115200
        self.timeout = 1
        self.serial = None

        # 如果没有指定端口，自动检测STM32
        if self.port is None:
            self.port = self._find_stm32_port()
            if self.port is None:
                raise Exception("未找到串口设备，请手动指定端口")

    def _find_stm32_port(self):
        """
        自动检测可用的串口设备

        返回:
            找到的串口号，如果未找到则返回None
        """
        ports = serial.tools.list_ports.comports()
        if ports:
            # 返回第一个可用的串口
            return ports[0].device
        return None

    def connect(self):
        """
        连接到串口设备

        返回:
            bool: 连接是否成功
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(2)  # 等待串口稳定
            print(f"成功连接到 {self.port}，波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def disconnect(self):
        """
        断开串口连接
        """
        if self.serial and self.serial.is_open:
            self.serial.close()
            print("已断开连接")

    def send_data(self, data):
        """
        发送数据到串口设备

        参数:
            data: 要发送的数据，可以是字符串或字节

        返回:
            bool: 发送是否成功
        """
        if not self.serial or not self.serial.is_open: ###如果串口未连接
            print("串口未连接")
            return False

        try:
            if isinstance(data, str):
                # 如果是字符串，转换为字节
                data = data.encode('utf-8')
            self.serial.write(data)
            return True
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False

    def receive_data(self, length=1):
        """
        从串口接收数据

        参数:
            length: 要接收的数据长度，默认1

        返回:
            bytes: 接收到的数据，如果出错则返回None
        """
        if not self.serial or not self.serial.is_open:
            print("串口未连接")
            return None

        try:
            data = self.serial.read(length)
            if len(data) < length:
                print(f"警告: 只接收到 {len(data)} 字节，请求 {length} 字节")
            return data
        except Exception as e:
            print(f"接收数据失败: {e}")
            return None

    def send_command(self, command, expected_response=None, timeout=2):
        """
        发送命令并等待响应

        参数:
            command: 要发送的命令字符串
            expected_response: 期望的响应，如果为None则不检查
            timeout: 等待响应的超时时间(秒)

        返回:
            bytes: 接收到的响应，如果出错或超时则返回None
        """
        if not self.send_data(command):
            return None

        start_time = time.time()
        response = b''

        while time.time() - start_time < timeout:
            data = self.receive_data(1)
            if data:
                response += data
                if expected_response and response == expected_response.encode('utf-8'):
                    return response
            time.sleep(0.01)  # 短暂休眠，减少CPU占用

        return response if response else None

    def read_until(self, delimiter=b'\n', timeout=2):
        """
        读取数据直到遇到指定的分隔符

        参数:
            delimiter: 分隔符，默认为换行符
            timeout: 超时时间(秒)

        返回:
            bytes: 读取到的数据(不包括分隔符)，如果超时则返回已读取的数据
        """
        if not self.serial or not self.serial.is_open:
            print("串口未连接")
            return None

        response = b''
        start_time = time.time()

        while time.time() - start_time < timeout:
            data = self.receive_data(1)
            if data:
                response += data
                if response.endswith(delimiter):
                    return response[:-len(delimiter)]  # 不包含分隔符
            time.sleep(0.01)

        return response

# 使用示例
if __name__ == "__main__":
    try:
        # 创建串口通信对象
        instance = SerialCommunication(baudrate=115200)  # 使用较高的波特率提高通信速度

        # 连接串口设备x
        if instance.connect():
            # 发送简单的测试命令
            print("发送测试命令...")
            response = instance.send_command("TEST", expected_response="OK", timeout=3)
            if response:
                print(f"接收到响应: {response.decode('utf-8')}")

            # 发送数据读取命令
            print("发送读取数据命令...")
            instance.send_data("READ_DATA\n")

            # 读取响应，直到遇到换行符
            data = instance.read_until(timeout=2)
            if data:
                print(f"接收到的数据: {data.decode('utf-8')}")

            # 发送控制命令
            print("发送控制命令...")
            instance.send_data("CONTROL:FORWARD\n")

            # 保持连接一段时间
            print("等待5秒...")
            time.sleep(5)

    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 确保断开连接
        if 'serial_comm' in locals():
            instance.disconnect()
