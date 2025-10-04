import serial
import serial.tools.list_ports
import time
import queue
from threading import Event, Thread
import platform  # 新增：用于检测操作系统


##########################
#一些名词/缩写
#ins ---- instance实例
#flag --- 标志位
#event --- 事件
#queue --- 队列
#thread --- 线程
#data --- 数据
#id --- id
#baudrate --- 波特率
#debunce --- 防抖
#mapping --- 映射
#stream --- 任务流
##########################
# 全局事件和队列（减小队列大小，RPi内存优化）
running_thread_event = Event()
data_queue = queue.Queue(maxsize=50)  # 减小maxsize，避免RPi内存压力
id_event = Event()
detection_event = Event()
text_event = Event()
id_data = None

# 模拟 start_thread 和 oled
def start_thread(name, target):
    thread = Thread(target=target, name=name, daemon=True)  # daemon=True，确保RPi退出时清理
    thread.start()
    print(f"线程 '{name}' 已启动")

class Oled:
    def __init__(self):
        pass

    def update_display(self, text):
        print(f"OLED 显示: {text}")

oled = Oled()

class SerialCommunication:
    def __init__(self, port=None, baudrate=115200, timeout=0.5):  # 统一115200 + 更短timeout
        self.port = port  # 移除默认 '/dev/ttyUSB0'，让自动检测生效
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.buf = bytearray()  # 合并：内部缓冲区（原 ReadLine.buf）

        # 始终进行自动检测如果未指定端口
        if self.port is None:
            self.port = self._find_stm32_port()
            if self.port is None:
                raise Exception("未找到串口设备，请手动指定端口")

    def _find_stm32_port(self):
        """
        增强自动检测：优先STM32相关端口；无则根据OS检测默认端口（Windows: COM, Linux: ttyUSB）
        """
        ports = serial.tools.list_ports.comports()
        os_name = platform.system()  # 检测OS: 'Windows' 或 'Linux'

        # 步骤1: 优先找STM32相关端口
        for port in ports:
            if 'STM32' in port.description or 'USB Serial' in port.description:
                print(f"检测到STM32端口: {port.device}")
                return port.device

        # 步骤2: 无STM32，则根据OS找默认串口
        if os_name == 'Windows':
            for port in ports:
                if port.device.startswith('COM'):  # Windows COM端口
                    print(f"Windows: 检测到COM端口: {port.device}")
                    return port.device
        elif os_name == 'Linux':
            for port in ports:
                if port.device.startswith('/dev/ttyUSB'):  # Linux ttyUSB端口
                    print(f"Linux: 检测到ttyUSB端口: {port.device}")
                    return port.device

        # 步骤3: 最终回退到第一个可用端口
        if ports:
            print(f"回退到第一个可用端口: {ports[0].device}")
            return ports[0].device
        return None

    def connect(self):
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout
            )
            time.sleep(0.5)  # RPi：缩短稳定等待
            self.buf.clear()  # 合并：初始化缓冲区
            print(f"成功连接到 {self.port}，波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def reconnect(self):  # RPi稳定性：添加重连
        self.disconnect()
        return self.connect()

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.buf.clear()  # 合并：清理缓冲区
            print("已断开连接")

    def send_data(self, data):
        if not self.serial or not self.serial.is_open:
            print("串口未连接，尝试重连...")
            if not self.reconnect():
                return False
        try:
            if isinstance(data, str):
                data = data.encode('utf-8')
            self.serial.write(data)
            return True
        except Exception as e:
            print(f"发送数据失败: {e}")
            return False

    def readline(self):
        """合并 ReadLine 逻辑：高效读取一行"""
        if not self.serial or not self.serial.is_open:
            return b''
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i + 1]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            i = max(1, min(2048, self.serial.in_waiting))  # 批量读取
            data = self.serial.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i + 1]
                self.buf[0:] = data[i + 1:]
                return r
            else:
                self.buf.extend(data)
            if not data:
                time.sleep(0.005)  # RPi优化：更短sleep，降CPU忙等
        return b''  # 防无限循环（实际不会到这里）

    def in_waiting(self):
        if not self.serial or not self.serial.is_open:
            return 0
        try:
            return self.serial.in_waiting
        except Exception:
            return 0

    def is_open(self):
        return self.serial is not None and self.serial.is_open

class Seri:
    def __init__(self, serial_comm):
        self.serial_comm = serial_comm
        self.current_data = None
        self.current_step = "idle"
        self.debounce_interval = 0.07  # RPi：更紧凑防抖
        self.last_debounce_time = time.time()

    def read_response(self):
        if not self.serial_comm or not self.serial_comm.is_open():
            return None
        try:
            if self.serial_comm.in_waiting() > 0:
                response_bytes = self.serial_comm.readline()
                if response_bytes:
                    decoded_response = response_bytes.decode("utf-8").strip()

                    # 防抖
                    current_time = time.time()
                    if current_time - self.last_debounce_time < self.debounce_interval:
                        return None
                    self.last_debounce_time = current_time

                    return decoded_response
            return None
        except Exception as e:
            print(f"读取错误: {e}")
            return None

    def put_queue_stream(self):
        print("串口读取线程已启动...")
        while running_thread_event.is_set():
            if not self.serial_comm or not self.serial_comm.is_open():
                print("串口读取线程：串口未连接，尝试重连...")
                self.serial_comm.reconnect()
                continue
            response = self.read_response()
            if response:
                try:
                    data_queue.put(response, timeout=0.05)  # 缩短timeout，提高实时
                except queue.Full:
                    print("警告：数据队列已满，数据可能丢失。")
            else:
                time.sleep(0.005)  # RPi：更短sleep，平衡CPU/响应
        print("串口读取线程已停止。")

    def read_queue_stream(self):
        print("数据处理线程已启动...")
        while running_thread_event.is_set():
            try:
                self.current_data = data_queue.get(timeout=0.05)  # 缩短timeout
                current_time = time.strftime('%H:%M:%S')

                if self.current_data.startswith("#"):
                    print(f"time: {current_time} 处理：触发停止信号！")
                    running_thread_event.clear()
                    detection_event.clear()
                    id_event.clear()

                elif self.current_data.startswith("$"):
                    print(f"time: {current_time} 处理：触发检测任务！数据: {self.current_data}")
                    detection_event.set()
                    if ":TYPE1" in self.current_data:
                        print("特定信号检查：执行 yolo 检测任务")
                        text_event.set()

                elif self.current_data.startswith("@"):
                    print(f"time: {current_time} 处理：触发识别任务！数据: {self.current_data}")
                    global id_data
                    id_data = self.current_data
                    id_event.set()

                else:
                    print(f"time: {current_time} 处理：收到未识别数据: {self.current_data}")

            except queue.Empty:
                continue
            except Exception as e:
                print(f"处理队列数据时出错: {e}")
        print("数据处理线程已停止。")

    def id_stream(self):
        print("ID处理线程已启动...")
        id_mapping = {
            "@1": "1", "@2": "2", "@3": "3", "@4": "4",
            "@5": "5", "@6": "6", "@7": "7", "@8": "8", "@9": "9"
        }
        while running_thread_event.is_set():
            if id_event.wait(timeout=0.05):  # RPi：缩短等待，提高响应
                global id_data
                if id_data:
                    print(f"ID处理: 识别到数据: {id_data}")
                    if id_data.startswith("@"):
                        mapped_id = id_mapping.get(id_data.strip(), None)
                        if mapped_id:
                            print(f"ID处理: 匹配到 {mapped_id}")
                            oled.update_display(f"id:{mapped_id}")
                        else:
                            print(f"ID处理: 未识别的ID: {id_data}")
                            oled.update_display("未知ID")
                    else:
                        print(f"ID处理: 数据格式不匹配 '@' 开头: {id_data}")
                else:
                    print("ID处理: id_event触发，但 id_data 不是预期的ID格式。")
                id_event.clear()
        print("ID处理线程已停止。")

    def detect_finished(self):
        self.serial_comm.send_data("5|0|0")
        print("检测任务完成，发送下位机停止命令。")

    def test(self):
        running_thread_event.set()
        start_thread("put_queue_stream", self.put_queue_stream)
        start_thread("read_queue_stream", self.read_queue_stream)
        start_thread("id_stream", self.id_stream)
        if self.serial_comm and self.serial_comm.is_open():
            self.serial_comm.send_data("1|25|0")
            print("测试：已发送开始信号 '1|0|0'")

# 使用示例（RPi：默认USB端口，统一115200）
if __name__ == "__main__":
    serial_ins = SerialCommunication(baudrate=115200)  # 统一115200，自动检测端口
    try:
        if serial_ins.connect():
            seri = Seri(serial_ins)
            seri.test()
            while running_thread_event.is_set():
                if detection_event.wait(timeout=0.5):  # 缩短主循环等待
                    print("主循环：检测事件触发，执行 detect_finished")
                    seri.detect_finished()
                    detection_event.clear()
                time.sleep(0.5)  # RPi：更短间隔
        else:
            print("串口连接失败，无法启动测试")

    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        running_thread_event.clear()
        if 'serial_comm' in locals():
            serial_ins.disconnect()