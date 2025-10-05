from ultralytics import YOLO
import cv2
import os
import time
import serial
import serial.tools.list_ports
import queue
from threading import Event, Thread
import platform
import signal
import sys
from collections import Counter
import pygame  # 新增：用于语音播报
import shutil  # 用于清空文件夹
import subprocess  # 新增：启动 pigpiod
import pigpio  # pigpio库，用于稳定PWM

# 条件导入 RPi.GPIO（仅 RPi/Linux）
RPI_GPIO_AVAILABLE = False
try:
    import RPi.GPIO as GPIO

    if platform.system() == 'Linux':
        RPI_GPIO_AVAILABLE = True
except ImportError:
    print("RPi.GPIO 不可用（Windows 模拟模式）")

# 全局事件和队列
running_flag = Event()
data_queue = queue.Queue(maxsize=50)
id_event = Event()
detection_event = Event()
text_event = Event()
cast_event = Event()  # 新增：语音播报事件
id_data = None
display_text = [None] * 4

# 全局变量：用于信号处理
shutdown_flag = False

# 创建保存图片的文件夹
save_folder = 'saved_images'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

# 创建保存检测结果的文件夹
output_folder = 'detected_images'
if not os.path.exists(output_folder):
    os.makedirs(output_folder)


def signal_handler(sig, frame):
    global shutdown_flag
    print("\n收到 Ctrl+C 中断信号，正在优雅退出...")
    shutdown_flag = True
    running_flag.clear()
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)


def start_thread(name, target):
    thread = Thread(target=target, name=name, daemon=True)
    thread.start()
    print(f"线程 '{name}' 已启动")


def start_pigpiod():
    """自动启动 pigpiod（如果未运行）"""
    try:
        # 检查是否已运行
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        if 'pigpiod' not in result.stdout:
            print("[Pan] pigpiod 未运行，自动启动...")
            subprocess.Popen(['sudo', 'pigpiod'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1)  # 等启动
            print("[Pan] pigpiod 已启动")
        else:
            print("[Pan] pigpiod 已运行")
        return True
    except Exception as e:
        print(f"[Pan] 启动 pigpiod 失败: {e}")
        return False


class Pan:
    DEFAULT_ANGLE = 90
    _using_pigpio = False  # 全局标志

    def __init__(self):
        self._simulated = not RPI_GPIO_AVAILABLE
        if self._simulated:
            print("[Pan] 模拟模式（非 RPi 环境）")
            return
        # 新增：自动启动 pigpiod
        if not start_pigpiod():
            print("[Pan] pigpiod 启动失败，回退 RPi.GPIO")
            self._init_rpi_gpio()
            return
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise Exception("pigpio未连接")
            self.pin_1 = 18  # 水平舵机
            self.pin_2 = 13  # 垂直舵机
            self.pin_light = 4  # 补光灯
            self.pi.set_mode(self.pin_1, pigpio.OUTPUT)
            self.pi.set_mode(self.pin_2, pigpio.OUTPUT)
            self.pi.set_mode(self.pin_light, pigpio.OUTPUT)
            self.pi.write(self.pin_light, 0)  # 灯初始关
            # pigpio PWM：频率50Hz，范围500-2500us (对应0-180°)
            self._set_servo_pulse(self.pin_1, 1500)  # 中位1500us
            self._set_servo_pulse(self.pin_2, 1500)
            Pan._using_pigpio = True
            print("[Pan] pigpio 初始化完成（硬件PWM）")
        except Exception as e:
            print(f"[Pan] pigpio失败，回退RPi.GPIO: {e}")
            self._init_rpi_gpio()

    def _init_rpi_gpio(self):
        """回退到 RPi.GPIO"""
        import RPi.GPIO as GPIO
        self.GPIO = GPIO
        self.pin_1 = 18
        self.pin_2 = 13
        self.pin_light = 4
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_1, GPIO.OUT)
        GPIO.setup(self.pin_2, GPIO.OUT)
        GPIO.setup(self.pin_light, GPIO.OUT)
        self.pwm_1 = GPIO.PWM(self.pin_1, 50)
        self.pwm_2 = GPIO.PWM(self.pin_2, 50)
        self.pwm_1.start(0)
        self.pwm_2.start(0)
        GPIO.output(self.pin_light, GPIO.LOW)
        Pan._using_pigpio = False
        print("[Pan] RPi.GPIO 初始化完成（软件PWM，可能有抖动）")

    def _set_servo_pulse(self, pin, pulse_us):
        """pigpio设置脉宽"""
        self.pi.set_servo_pulsewidth(pin, pulse_us)

    def _angle_to_pulse(self, angle):
        angle = max(0, min(180, angle))
        return 500 + (angle * 2000 / 180)  # 0°=500us, 180°=2500us

    def set_angle(self, pin, angle):
        if self._simulated:
            print(f"[Pan 模拟] 设置角度: {angle}°")
            time.sleep(0.5)
            return
        if Pan._using_pigpio:
            pulse = self._angle_to_pulse(angle)
            self._set_servo_pulse(pin, pulse)
        else:
            # RPi.GPIO 回退
            duty = 2 + (angle / 18)
            if pin == self.pin_1:
                self.pwm_1.ChangeDutyCycle(duty)
            else:
                self.pwm_2.ChangeDutyCycle(duty)
        time.sleep(0.5)  # 等转动

    def pan_left(self, angle=DEFAULT_ANGLE):
        if self._simulated:
            print(f"[Pan 模拟] 云台左转 (垂直: {angle}°)")
            return
        self.set_angle(self.pin_1, 0)
        self.set_angle(self.pin_2, angle)

    def pan_right(self, angle=DEFAULT_ANGLE):
        if self._simulated:
            print(f"[Pan 模拟] 云台右转 (垂直: {angle}°)")
            return
        self.set_angle(self.pin_1, 180)
        self.set_angle(self.pin_2, angle)

    def pan_center(self, angle=DEFAULT_ANGLE):
        if self._simulated:
            print(f"[Pan 模拟] 云台居中 (垂直: {angle}°)")
            return
        self.set_angle(self.pin_1, 90)
        self.set_angle(self.pin_2, angle)

    def light_on(self):
        if self._simulated:
            print("[Pan 模拟] 补光灯开启")
            return
        if Pan._using_pigpio:
            self.pi.write(self.pin_light, 1)
        else:
            self.GPIO.output(self.pin_light, self.GPIO.HIGH)

    def light_off(self):
        if self._simulated:
            print("[Pan 模拟] 补光灯关闭")
            return
        if Pan._using_pigpio:
            self.pi.write(self.pin_light, 0)
        else:
            self.GPIO.output(self.pin_light, self.GPIO.LOW)

    def cleanup(self):
        if self._simulated:
            print("[Pan 模拟] 清理完成")
            return
        if Pan._using_pigpio:
            self._set_servo_pulse(self.pin_1, 0)
            self._set_servo_pulse(self.pin_2, 0)
            self.pi.write(self.pin_light, 0)
            self.pi.stop()
            print("[Pan] pigpio 已释放")
            # 可选：停止 pigpiod
            subprocess.run(['sudo', 'pkill', 'pigpiod'], capture_output=True)
        else:
            self.pwm_1.ChangeDutyCycle(0)
            self.pwm_2.ChangeDutyCycle(0)
            self.pwm_1.stop()
            self.pwm_2.stop()
            self.GPIO.output(self.pin_light, self.GPIO.LOW)
            self.GPIO.cleanup()
            print("[Pan] RPi.GPIO 已释放")


class Oled:
    def __init__(self):
        try:
            from luma.core.interface.serial import i2c
            from luma.core.render import canvas
            from luma.oled.device import ssd1306
            self.canvas = canvas
            self.serial = i2c(port=1, address=0x3C)
            self.device = ssd1306(self.serial)
            self._real_hardware = True
            print("OLED 硬件初始化成功")
        except ImportError as e:
            print(f"OLED 库未安装（Windows 模拟模式）: {e}")
            self._real_hardware = False
        except Exception as e:
            print(f"OLED 硬件初始化失败（模拟模式）: {e}")
            self._real_hardware = False

    def show_text(self, t1=None, t2=None, t3=None, t4=None, t5=None, t6=None):
        if self._real_hardware:
            with self.canvas(self.device) as draw:
                draw.rectangle(self.device.bounding_box, outline="white", fill="black")
                draw.text((10, 0), str(t1 or ""), fill="white")
                draw.text((10, 10), str(t2 or ""), fill="white")
                draw.text((10, 20), str(t3 or ""), fill="white")
                draw.text((10, 30), str(t4 or ""), fill="white")
                draw.text((10, 40), "status: running", fill="white")
                draw.text((10, 50), f"time: {time.strftime('%H:%M:%S')}", fill="white")
            time.sleep(2)
        else:
            print(f"OLED 模拟显示: {t1}, {t2}, {t3}, {t4}, status: running, time: {time.strftime('%H:%M:%S')}")

    def threading_text(self):
        print("OLED 显示线程已启动...")
        while running_flag.is_set():
            if text_event.is_set():
                print("text_event ok")
                current_text = display_text.copy()
                self.show_text(*current_text)
                text_event.clear()
            time.sleep(1)

    def threading_start(self):
        self.threading = Thread(target=self.threading_text, daemon=True)
        self.threading.start()

    def update_display(self, t1=None, t2=None, t3=None, t4=None):
        global display_text
        display_text[0] = t1
        display_text[1] = t2
        display_text[2] = t3
        display_text[3] = t4
        text_event.set()


class SerialCommunication:
    def __init__(self, port=None, baudrate=115200, timeout=0.5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.buf = bytearray()
        if self.port is None:
            self.port = self._find_stm32_port()
            if self.port is None:
                raise Exception("未找到串口设备，请手动指定端口")

    def _find_stm32_port(self):
        ports = serial.tools.list_ports.comports()
        os_name = platform.system()
        for port in ports:
            if 'STM32' in port.description or 'USB Serial' in port.description:
                print(f"检测到 STM32 端口: {port.device}")
                return port.device
        if os_name == 'Windows':
            for port in ports:
                if port.device.startswith('COM'):
                    print(f"Windows: 检测到 COM 端口: {port.device}")
                    return port.device
        elif os_name == 'Linux':
            for port in ports:
                if port.device.startswith('/dev/ttyUSB'):
                    print(f"Linux: 检测到 ttyUSB 端口: {port.device}")
                    return port.device
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
            time.sleep(0.5)
            self.buf.clear()
            print(f"成功连接到 {self.port}，波特率: {self.baudrate}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False

    def reconnect(self):
        self.disconnect()
        return self.connect()

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.serial.close()
            self.buf.clear()
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
        if not self.serial or not self.serial.is_open:
            return b''
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i + 1]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            i = max(1, min(2048, self.serial.in_waiting))
            data = self.serial.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i + 1]
                self.buf[0:] = data[i + 1:]
                return r
            else:
                self.buf.extend(data)
            if not data:
                time.sleep(0.005)
        return b''

    def in_waiting(self):
        if not self.serial or not self.serial.is_open:
            return 0
        try:
            return self.serial.in_waiting
        except Exception:
            return 0

    def is_open(self):
        return self.serial is not None and self.serial.is_open


class Boardcast:
    def __init__(self):
        pygame.init()
        pygame.mixer.init()
        self.current_place = ""
        self.current_name = ""

    def _play_sound(self, place, name):
        """通用的播放声音方法"""
        print(f"尝试播放: {name}")
        sound_path = f"../sound/{place}/{name}.mp3"
        if not os.path.exists(sound_path):
            print(f"错误: 文件不存在 - {sound_path}")
            return
        try:
            pygame.mixer.music.load(sound_path)
            pygame.mixer.music.play()
            print(f"开始播放 {name}")
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)  # 降低 CPU 使用率
            print(f"播放完成 {name}")
        except Exception as e:
            print(f"播放声音失败: {e}")

    def threading_sound(self):
        print("语音播报线程已启动...")
        while running_flag.is_set():
            if cast_event.is_set():
                place = self.current_place
                name = self.current_name
                self._play_sound(place, name)
                cast_event.clear()
            time.sleep(0.1)
        print("语音播报线程已停止")

    def update_sound(self, place, name):
        self.current_place = place
        self.current_name = name
        cast_event.set()


class Seri:
    def __init__(self, serial_comm, boardcast):
        self.serial_comm = serial_comm
        self.boardcast = boardcast
        self.current_data = None
        self.current_step = "idle"
        self.debounce_interval = 0.5  # 保持 0.5s
        self.last_debounce_time = time.time()

        # 新增：事件级防抖和去重
        self.last_detection_trigger_time = 0
        self.last_id_trigger_time = 0
        self.last_detection_data = None  # 上次检测信号内容
        self.last_id_data = None  # 上次 ID 信号内容

    def read_response(self):
        if not self.serial_comm or not self.serial_comm.is_open():
            return None
        try:
            if self.serial_comm.in_waiting() > 0:
                response_bytes = self.serial_comm.readline()
                if response_bytes:
                    decoded_response = response_bytes.decode("utf-8", errors='ignore').strip()
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
        while running_flag.is_set():
            if not self.serial_comm or not self.serial_comm.is_open():
                print("串口读取线程：串口未连接，尝试重连...")
                self.serial_comm.reconnect()
                continue
            response = self.read_response()
            if response:
                try:
                    data_queue.put(response, timeout=0.05)
                except queue.Full:
                    print("警告：数据队列已满，数据可能丢失。")
            else:
                time.sleep(0.01)
        print("串口读取线程已停止。")

    def read_queue_stream(self):
        print("数据处理线程已启动...")
        while running_flag.is_set():
            try:
                self.current_data = data_queue.get(timeout=0.05)
                current_time = time.strftime('%H:%M:%S')
                now = time.time()  # 新增：当前时间戳
                # 新增：噪声过滤（忽略重复调试消息）
                if "Turn=1 init success" in self.current_data or "sensor calibration" in self.current_data or "DataNotReady" in self.current_data:
                    print(f"time: {current_time} 忽略噪声: {self.current_data}")
                    continue
                if self.current_data.startswith("#"):
                    print(f"time: {current_time} 处理：触发停止信号！")
                    running_flag.clear()
                    detection_event.clear()
                    id_event.clear()
                    text_event.clear()
                elif self.current_data.startswith("$"):
                    print(f"time: {current_time} 处理：触发检测任务！数据: {self.current_data}")
                    # 新增：事件级防抖 + 内容去重
                    if (now - self.last_detection_trigger_time >= self.debounce_interval and
                            self.current_data != self.last_detection_data):
                        detection_event.set()
                        self.last_detection_trigger_time = now
                        self.last_detection_data = self.current_data
                        print("检测事件已触发（去重后）")
                        if ":TYPE1" in self.current_data:
                            print("特定信号检查：执行 YOLO 检测任务")
                            text_event.set()
                    else:
                        print(
                            f"检测信号忽略（防抖/重复）：间隔 {now - self.last_detection_trigger_time:.2f}s, 上次数据: {self.last_detection_data}")
                elif self.current_data.startswith("@"):
                    print(f"time: {current_time} 处理：触发识别任务！数据: {self.current_data}")
                    global id_data
                    id_data = self.current_data
                    # 新增：事件级防抖 + 内容去重
                    if (now - self.last_id_trigger_time >= self.debounce_interval and
                            self.current_data != self.last_id_data):
                        id_event.set()
                        self.last_id_trigger_time = now
                        self.last_id_data = self.current_data
                        print("ID 事件已触发（去重后）")
                    else:
                        print(
                            f"ID 信号忽略（防抖/重复）：间隔 {now - self.last_id_trigger_time:.2f}s, 上次数据: {self.last_id_data}")
                else:
                    print(f"time: {current_time} 处理：收到未识别数据: {self.current_data}")
            except queue.Empty:
                continue
            except Exception as e:
                print(f"处理队列数据时出错: {e}")
        print("数据处理线程已停止。")

    def id_stream(self):
        print("ID 处理线程已启动...")
        id_mapping = {
            "@1": "1", "@2": "2", "@3": "3", "@4": "4",
            "@5": "5", "@6": "6", "@7": "7", "@8": "8", "@9": "9"
        }
        while running_flag.is_set():
            if id_event.wait(timeout=0.05):
                global id_data
                if id_data:
                    print(f"ID 处理: 识别到数据: {id_data}")
                    if id_data.startswith("@"):
                        # 增强解析：提取第一个@数字
                        import re
                        match = re.search(r'@(\d+)', id_data)
                        if match:
                            mapped_id = match.group(1)
                            if mapped_id in id_mapping.values():
                                print(f"ID 处理: 匹配到 {mapped_id}")
                                oled_ins.update_display(f"id:{mapped_id}")
                                self.boardcast.update_sound("point", mapped_id)  # 新增：播报 ID
                                id_data = None  # 清空
                                continue
                        mapped_id = id_mapping.get(id_data.strip(), None)
                        if mapped_id:
                            print(f"ID 处理: 匹配到 {mapped_id}")
                            oled_ins.update_display(f"id:{mapped_id}")
                            self.boardcast.update_sound("point", mapped_id)  # 新增：播报 ID
                        else:
                            print(f"ID 处理: 未识别的 ID: {id_data}")
                            oled_ins.update_display("未知 ID")
                    else:
                        print(f"ID 处理: 数据格式不匹配 '@' 开头: {id_data}")
                else:
                    print("ID 处理: id_event 触发，但 id_data 不是预期的 ID 格式。")
                id_event.clear()
        print("ID 处理线程已停止。")

    def detect_finished(self):
        self.serial_comm.send_data("5|0|0")
        print("检测任务完成，发送下位机停止命令。")

    def test(self):
        running_flag.set()
        start_thread("put_queue_stream", self.put_queue_stream)
        start_thread("read_queue_stream", self.read_queue_stream)
        start_thread("id_stream", self.id_stream)
        start_thread("cast", self.boardcast.threading_sound)  # 新增：启动语音播报线程
        if self.serial_comm and self.serial_comm.is_open():
            self.serial_comm.send_data("1|0|0")
            print("测试：已发送开始信号 '1|0|0'")


class RobotController:
    def __init__(self):
        # 新增：确保 pigpiod 启动（在 Pan 前）
        start_pigpiod()
        self.pan = Pan()
        self.serial_comm = SerialCommunication(baudrate=115200)
        self.oled = Oled()
        self.boardcast = Boardcast()  # 语音播报实例
        self.seri = Seri(self.serial_comm, self.boardcast)  # 传入 boardcast
        self.oled.threading_start()
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # 用V4L2后端优化RPi
        self.is_executing = False  # 执行锁，防止重入
        if not self.cap.isOpened():
            print("❌ 无法找到可用摄像头，请检查设备或权限")
            exit()
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 最小缓冲
        model_path = "/home/pi/Desktop/code/2025.pt"
        if not os.path.exists(model_path):
            print(f"❌ 模型文件 {model_path} 不存在")
            exit()
        try:
            self.model = YOLO(model_path)
        except Exception as e:
            print(f"❌ 加载模型失败: {e}")
            exit()
        print("[RobotController] 所有组件初始化完成")

    def clear_buffer(self, num_frames=8):
        """丢弃旧帧，确保读新帧"""
        for _ in range(num_frames):
            self.cap.grab()  # 高效丢帧
        print(f"[Buffer] 已丢弃 {num_frames} 旧帧")

    def perform_yolo_detection(self, save_count, max_saves=6):
        """执行 YOLO 检测并保存结果（最多 max_saves 张）"""
        if save_count >= max_saves:
            return save_count, []

        # 清缓冲取新帧
        self.clear_buffer(8)  # 每张清8帧
        ret, frame = self.cap.retrieve()  # 解码新帧
        if not ret:
            # 备用read
            ret, frame = self.cap.read()
            if not ret:
                print("❌ 无法读取摄像头画面")
                return save_count, []

        # 验证时间戳（调试用）
        ts = self.cap.get(cv2.CAP_PROP_POS_MSEC)
        print(f"[Frame TS] {ts:.0f}ms")  # 应递增，无重复

        results = self.model(frame, verbose=False)
        annotated_frame = results[0].plot()
        classes = results[0].boxes.cls.tolist()
        class_names = [self.model.names[int(cls)] for cls in classes]
        print(f"[YOLO] 检测到类别: {', '.join(class_names) if class_names else '无'}")
        # 播报检测到的类别（顺序播放）
        for name in class_names:
            self.boardcast.update_sound("shape", name)  # 播报 YOLO 检测类别
        image_filename = os.path.join(save_folder, f"image_{int(time.time() * 1000)}.jpg")  # 用时间戳命名
        cv2.imwrite(image_filename, annotated_frame)
        save_count += 1
        print(f"✅ 保存画面为 {image_filename} ({save_count}/{max_saves})")
        time.sleep(0.3)  # 减sleep
        return save_count, class_names

    def reprocess_saved_images(self):
        """重新检测 saved_images 文件夹中的图片"""
        # 按修改时间降序排序，取最新的 6 张
        image_files = [f for f in os.listdir(save_folder) if f.endswith(('.jpg', '.jpeg', '.png'))]
        if len(image_files) == 0:
            print("❌ saved_images 文件夹中没有图片")
            return
        # 获取文件完整路径，按修改时间降序排序
        full_paths = [(os.path.join(save_folder, f), os.path.getmtime(os.path.join(save_folder, f))) for f in
                      image_files]
        full_paths.sort(key=lambda x: x[1], reverse=True)  # 最新的在前
        latest_images = full_paths[:6]
        image_files = [os.path.basename(path) for path, mtime in latest_images]
        print(f"🎬 开始对最新的 {len(image_files)} 张图片进行 YOLO 重新检测")
        for i, image_file in enumerate(image_files):
            image_path = os.path.join(save_folder, image_file)
            frame = cv2.imread(image_path)
            if frame is None:
                print(f"❌ 无法读取图片: {image_path}")
                continue
            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot()
            classes = results[0].boxes.cls.tolist()
            class_names = [self.model.names[int(cls)] for cls in classes]
            print(f"图片 {image_file} 识别到的类别: {', '.join(class_names) if class_names else '无'}")
            # 播报重新检测的类别
            for name in class_names:
                self.boardcast.update_sound("shape", name)  # 播报重新检测类别
            self.oled.update_display(f"Image {i + 1}", f"Classes: {', '.join(class_names) if class_names else 'None'}")
            output_path = os.path.join(output_folder, f"detected_{image_file}")
            cv2.imwrite(output_path, annotated_frame)
            print(f"✅ 保存检测结果为 {output_path}")

    def print_detection_stats(self, side, all_classes):
        """打印检测统计：类别计数"""
        if not all_classes:
            print(f"[Stats] {side} 侧检测统计: 无检测到对象")
            self.oled.update_display(f"{side} Stats", "No objects")
            return
        class_counter = Counter(all_classes)
        stats_str = f"{side} 侧检测统计: {dict(class_counter)}"
        print(f"[Stats] {stats_str}")
        # 播报统计结果（示例：播报总计）
        self.boardcast.update_sound("stats", f"{side.lower()}_stats")  # 播报统计
        self.oled.update_display(f"{side} Total: {len(all_classes)} objs",
                                 f"Classes: {dict(list(class_counter.items())[:2])}")

    def execute_detection_sequence(self):
        """执行检测序列：左转连续拍3张（每张立即检测显示） → 左统计输出 → 右转连续拍3张（每张立即检测显示） → 右统计输出 → 归中（不拍照） → 重新检测"""
        if self.is_executing:  # 如果已在执行，忽略
            print("[Detection] 序列已在执行中，忽略新触发")
            return
        self.is_executing = True  # 加锁
        try:
            # 清空 saved_images 文件夹，确保只处理当前序列的图片
            if os.path.exists(save_folder):
                shutil.rmtree(save_folder)
                os.makedirs(save_folder)
                print(f"[Detection] 已清空 {save_folder} 文件夹，开始新序列")

            self.pan.light_on()  # 开灯稳定
            print("[Detection] 开始检测序列...")
            save_count = 0
            max_saves = 6
            left_classes = []
            right_classes = []

            # 左转 + 等转动 + 清缓冲
            self.pan.pan_left()
            time.sleep(1.0)  # 等转动完成
            self.clear_buffer(5)  # 清转动前旧帧
            print("[Detection] 云台左转 - 连续捕获 3 张图片")
            for shot in range(1, 4):
                print(f"[Detection] 左转 - 第 {shot} 张")
                save_count, classes = self.perform_yolo_detection(save_count, max_saves)
                self.oled.update_display(f"Left {shot}", f"Classes: {', '.join(classes) if classes else 'None'}")
                left_classes.extend(classes)
                time.sleep(0.5)  # 每张间稍等
            self.print_detection_stats("Left", left_classes)

            # 右转 + 等 + 清
            self.pan.pan_right()
            time.sleep(1.0)
            self.clear_buffer(5)
            print("[Detection] 云台右转 - 连续捕获 3 张图片")
            for shot in range(1, 4):
                print(f"[Detection] 右转 - 第 {shot} 张")
                save_count, classes = self.perform_yolo_detection(save_count, max_saves)
                self.oled.update_display(f"Right {shot}", f"Classes: {', '.join(classes) if classes else 'None'}")
                right_classes.extend(classes)
                time.sleep(0.5)
            self.print_detection_stats("Right", right_classes)

            self.pan.pan_center()
            time.sleep(0.5)
            self.clear_buffer(3)  # 居中清
            print("[Detection] 云台居中")
            self.oled.update_display("Center", "No detection")
            print("[Detection] 重新检测保存的图片")
            self.reprocess_saved_images()
            self.seri.detect_finished()
            print("[Detection] 序列完成")
        finally:
            self.is_executing = False  # 解锁
            self.pan.light_off()

    def start(self):
        if self.serial_comm.connect():
            self.seri.test()
            while running_flag.is_set():
                if detection_event.wait(timeout=0.5):
                    if not self.is_executing:  # 双重检查锁
                        self.execute_detection_sequence()
                    detection_event.clear()  # 始终 clear，避免积压
                time.sleep(0.5)
        else:
            print("串口连接失败，无法启动测试")

    def cleanup(self):
        running_flag.clear()
        self.pan.cleanup()
        self.serial_comm.disconnect()
        self.cap.release()
        cv2.destroyAllWindows()
        pygame.mixer.quit()  # 清理 pygame 资源
        print("[RobotController] 资源已清理")


if __name__ == "__main__":
    global oled_ins
    oled_ins = Oled()
    controller = RobotController()
    try:
        controller.start()
    except KeyboardInterrupt:
        print("\n捕获 KeyboardInterrupt，正在优雅退出...")
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        controller.cleanup()
        print("程序已退出")
# 执行命令（简化）