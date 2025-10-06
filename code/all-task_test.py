# pan_yolo.py
# 合并 Pan 云台转动和 YOLO 检测模块
import shutil
import cv2
import time
import os
from collections import Counter
from ultralytics import YOLO
import platform
import subprocess
import threading
import queue
import serial
import serial.tools.list_ports
import pygame
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
import signal
import sys
import pigpio

# 全局变量和配置
RPI_GPIO_AVAILABLE = False
running_flag = threading.Event()
data_queue = queue.Queue(maxsize=50)
id_event = threading.Event()
detection_event = threading.Event()
text_event = threading.Event()
cast_event = threading.Event()
id_data = None
display_text = [None] * 4
shutdown_flag = False
audio_queue = queue.Queue(maxsize=10)  # 新增：音频队列

# 文件夹配置
save_folder = 'saved_images'
output_folder = 'detected_images'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)
if not os.path.exists(output_folder):
    os.makedirs(output_folder)


# 信号处理
def signal_handler(sig, frame):
    global shutdown_flag
    print("\n收到 Ctrl+C 中断信号，正在优雅退出...")
    shutdown_flag = True
    running_flag.clear()
    pygame.mixer.quit()
    sys.exit(0)


def start_pigpiod():
    try:
        result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
        if 'pigpiod' not in result.stdout:
            print("[Pan] pigpiod 未运行，自动启动...")
            subprocess.Popen(['sudo', 'pigpiod'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(1)
            print("[Pan] pigpiod 已启动")
        else:
            print("[Pan] pigpiod 已运行")
        return True
    except Exception as e:
        print(f"[Pan] 启动 pigpiod 失败: {e}")
        return False


signal.signal(signal.SIGINT, signal_handler)

# 条件导入 RPi.GPIO
try:
    import RPi.GPIO as GPIO

    if platform.system() == 'Linux':
        RPI_GPIO_AVAILABLE = True
except ImportError:
    print("RPi.GPIO 不可用（Windows 模拟模式）")


def start_thread(name, target):
    thread = threading.Thread(target=target, name=name, daemon=True)
    thread.start()
    print(f"[Thread] {name} 已启动")
    return thread


def clear_folder(folder_path):
    for filename in os.listdir(folder_path):
        file_path = os.path.join(folder_path, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print(f'删除 {file_path} 失败: {e}')


class Oled:
    def __init__(self):
        try:
            self.serial = i2c(port=1, address=0x3C)
            self.device = ssd1306(self.serial)
            self._real_hardware = True
            print("[Oled] 硬件初始化成功")
        except ImportError as e:
            print(f"[Oled] 库未安装（Windows 模拟模式）: {e}")
            self._real_hardware = False
        except Exception as e:
            print(f"[Oled] 硬件初始化失败（模拟模式）: {e}")
            self._real_hardware = False

    def show_text(self, t1=None, t2=None, t3=None, t4=None, t5=None, t6=None):
        print(f"[Oled] show_text called with: {t1}, {t2}, {t3}, {t4}")
        if self._real_hardware:
            try:
                with canvas(self.device) as draw:
                    draw.rectangle(self.device.bounding_box, outline="white", fill="black")
                    draw.text((10, 0), str(t1 or ""), fill="white")
                    draw.text((10, 10), str(t2 or ""), fill="white")
                    draw.text((10, 20), str(t3 or ""), fill="white")
                    draw.text((10, 30), str(t4 or ""), fill="white")
                    draw.text((10, 40), "status: running", fill="white")
                    draw.text((10, 50), f"time: {time.strftime('%H:%M:%S')}", fill="white")
                print("[Oled] 硬件显示成功")
            except Exception as e:
                print(f"[Oled] 硬件显示失败: {e}")
        else:
            print(f"[Oled] 模拟显示: {t1}, {t2}, {t3}, {t4}, status: running, time: {time.strftime('%H:%M:%S')}")
        time.sleep(0.5)

    def threading_text(self):
        print("[Oled] 显示线程已启动...")
        while running_flag.is_set():
            if text_event.is_set():
                current_text = display_text.copy()
                print(f"[Oled] Thread processing display_text: {current_text}")
                self.show_text(*current_text)
                text_event.clear()
            time.sleep(0.1)
        print("[Oled] 显示线程已停止")

    def threading_start(self):
        self.threading = threading.Thread(target=self.threading_text, daemon=True)
        self.threading.start()
        print("[Oled] 线程已启动")

    def update_display(self, t1=None, t2=None, t3=None, t4=None):
        global display_text
        display_text = [t1, t2, t3, t4]
        print(f"[Oled] update_display set to: {display_text}")
        text_event.set()


class Boardcast:
    def __init__(self):
        try:
            pygame.init()
            pygame.mixer.init()
            self.audio_initialized = True
            print("[Boardcast] pygame.mixer 初始化成功")
        except Exception as e:
            print(f"[Boardcast] pygame.mixer 初始化失败: {e}")
            self.audio_initialized = False

    def _play_sound(self, place, name):
        if not self.audio_initialized:
            print("[Boardcast] 音频未初始化，跳过播放")
            return
        sound_path = f"/home/pi/Desktop/sound/{place}/{name}.mp3"
        print(f"[Boardcast] 尝试播放: {sound_path}")
        if not os.path.exists(sound_path):
            print(f"[Boardcast] 错误: 文件不存在 - {sound_path}")
            return
        try:
            pygame.mixer.music.load(sound_path)
            pygame.mixer.music.play()
            print(f"[Boardcast] 开始播放 {name}")
            while pygame.mixer.music.get_busy():
                pygame.time.Clock().tick(10)
            print(f"[Boardcast] 播放完成 {name}")
        except Exception as e:
            print(f"[Boardcast] 播放声音失败: {e}")

    def threading_sound(self):
        print("[Boardcast] 语音播报线程已启动...")
        while running_flag.is_set():
            try:
                place, name = audio_queue.get(timeout=0.1)
                self._play_sound(place, name)
                audio_queue.task_done()
            except queue.Empty:
                continue
        print("[Boardcast] 语音播报线程已停止")

    def update_sound(self, place, name):
        if not self.audio_initialized:
            print("[Boardcast] 音频未初始化，跳过更新")
            return
        try:
            audio_queue.put((place, name), timeout=0.05)
            print(f"[Boardcast] update_sound: {place}/{name} 已加入队列")
        except queue.Full:
            print(f"[Boardcast] 警告：音频队列已满，丢弃 {place}/{name}")


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


class Seri:
    def __init__(self, serial_comm, boardcast, pan_yolo):
        self.serial_comm = serial_comm
        self.boardcast = boardcast
        self.pan_yolo = pan_yolo  # 引用 PanYolo 实例
        self.current_data = None
        self.current_step = "idle"

    def read_response(self):
        if not self.serial_comm or not self.serial_comm.is_open():
            return None
        try:
            if self.serial_comm.in_waiting() > 0:
                response_bytes = self.serial_comm.readline()
                if response_bytes:
                    return response_bytes.decode("utf-8", errors='ignore').strip()
            return None
        except Exception as e:
            print(f"[Seri] 读取错误: {e}")
            return None

    def put_queue_stream(self):
        print("[Seri] 串口读取线程已启动...")
        while running_flag.is_set():
            if not self.serial_comm or not self.serial_comm.is_open():
                print("[Seri] 串口未连接，尝试重连...")
                self.serial_comm.reconnect()
                continue
            response = self.read_response()
            if response:
                try:
                    data_queue.put(response, timeout=0.05)
                except queue.Full:
                    print("[Seri] 警告：数据队列已满，数据可能丢失")
            time.sleep(0.005)
        print("[Seri] 串口读取线程已停止")

    def read_queue_stream(self):
        print("[Seri] 数据处理线程已启动...")
        while running_flag.is_set():
            try:
                self.current_data = data_queue.get(timeout=0.01)
                current_time = time.strftime('%H:%M:%S')
                if "Turn=1 init success" in self.current_data or "sensor calibration" in self.current_data or "DataNotReady" in self.current_data:
                    print(f"[Seri] time: {current_time} 忽略噪声: {self.current_data}")
                    continue
                if self.current_data.startswith("#"):
                    print(f"[Seri] time: {current_time} 处理：触发停止信号！")
                    running_flag.clear()
                    detection_event.clear()
                    id_event.clear()
                    text_event.clear()
                elif self.current_data.startswith("$"):
                    print(f"[Seri] time: {current_time} 处理：触发检测任务！数据: {self.current_data}")
                    detection_event.set()
                    if "$" in self.current_data:
                        print("[Seri] 特定信号检查：执行 YOLO 检测任务")
                        # 触发 execute_detection_sequence
                        threading.Thread(target=self.pan_yolo.execute_detection_sequence, daemon=True).start()
                        text_event.set()
                elif self.current_data.startswith("msg=@"):
                    global id_data
                    id_data = self.current_data
                    print(f"[Seri] time: {current_time} 处理：触发识别任务！数据: {self.current_data}")
                    id_event.set()
                else:
                    print(f"[Seri] time: {current_time} 处理：收到未识别数据: {self.current_data}")
            except queue.Empty:
                continue
            except Exception as e:
                print(f"[Seri] 处理队列数据时出错: {e}")
        print("[Seri] 数据处理线程已停止")

    def id_stream(self):
        print("[Seri] ID 处理线程已启动...")
        import re
        while running_flag.is_set():
            if id_event.wait(timeout=0.05):
                global id_data
                if id_data and id_data.startswith("msg=@"):
                    print(f"[Seri] ID 处理: 识别到数据: {id_data}")
                    match = re.search(r'msg=@(\d)', id_data)
                    if match:
                        mapped_id = match.group(1)
                        if mapped_id in {'1', '2', '3', '4', '5', '6', '7', '8', '9'}:
                            print(f"[Seri] ID 处理: 匹配到 {mapped_id}")
                            self.boardcast.update_sound("point", mapped_id)
                            id_data = None
                            id_event.clear()
                            continue
                        else:
                            print(f"[Seri] ID 处理: 未识别的 ID 数字: {mapped_id}")
                    else:
                        print(f"[Seri] ID 处理: 无有效一位数字在 {id_data}")
                else:
                    print(f"[Seri] ID 处理: id_event 触发，但 id_data 无效: {id_data}")
                id_event.clear()
        print("[Seri] ID 处理线程已停止")

    def detect_finished(self):
        self.serial_comm.send_data("5|0|0")
        print("[Seri] 检测任务完成，发送下位机停止命令")

    def test(self):
        start_thread("put_queue_stream", self.put_queue_stream)
        start_thread("read_queue_stream", self.read_queue_stream)
        start_thread("id_stream", self.id_stream)
        start_thread("cast", self.boardcast.threading_sound)
        if self.serial_comm and self.serial_comm.is_open():
            self.serial_comm.send_data("1|0|0")
            print("[Seri] 测试：已发送开始信号 '1|0|0'")


class PanYolo:
    DEFAULT_ANGLE = 80
    _using_pigpio = False

    def __init__(self, model_path="/home/pi/Desktop/code/1989.pt"):
        self._simulated = not RPI_GPIO_AVAILABLE
        if self._simulated:
            print("[PanYolo] 模拟模式（非 RPi 环境）")
            return

        start_pigpiod()
        if not start_pigpiod():
            print("[PanYolo] pigpiod 启动失败，回退 RPi.GPIO")
            self._init_rpi_gpio()
            return
        try:
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise Exception("pigpio未连接")
            self.pin_1 = 18
            self.pin_2 = 13
            self.pin_light = 4
            self.pi.set_mode(self.pin_1, pigpio.OUTPUT)
            self.pi.set_mode(self.pin_2, pigpio.OUTPUT)
            self.pi.set_mode(self.pin_light, pigpio.OUTPUT)
            self.pi.write(self.pin_light, 0)
            self._set_servo_pulse(self.pin_1, 1500)
            self._set_servo_pulse(self.pin_2, 1500)
            PanYolo._using_pigpio = True
            print("[PanYolo] pigpio 初始化完成（硬件PWM）")
        except Exception as e:
            print(f"[PanYolo] pigpio失败，回退RPi.GPIO: {e}")
            self._init_rpi_gpio()

        if not os.path.exists(model_path):
            print(f"❌ 模型文件 {model_path} 不存在")
            exit()
        try:
            self.model = YOLO(model_path)
            self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
            if not self.cap.isOpened():
                print("❌ 无法找到可用摄像头，请检查设备或权限")
                exit()
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            print("[PanYolo] YOLO 和摄像头初始化完成")
        except Exception as e:
            print(f"❌ 加载模型失败: {e}")
            exit()

        self.oled = Oled()
        self.boardcast = Boardcast()
        self.oled.threading_start()
        start_thread("cast", self.boardcast.threading_sound)

    def _init_rpi_gpio(self):
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
        PanYolo._using_pigpio = False
        print("[PanYolo] RPi.GPIO 初始化完成（软件PWM，可能有抖动）")

    def _set_servo_pulse(self, pin, pulse_us):
        self.pi.set_servo_pulsewidth(pin, pulse_us)

    def _angle_to_pulse(self, angle):
        angle = max(0, min(180, angle))
        return 500 + (angle * 2000 / 180)

    def set_angle(self, pin, angle):
        if self._simulated:
            print(f"[PanYolo 模拟] 设置角度: {angle}°")
            time.sleep(0.5)
            return
        if PanYolo._using_pigpio:
            pulse = self._angle_to_pulse(angle)
            self._set_servo_pulse(pin, pulse)
        else:
            duty = 2 + (angle / 18)
            if pin == self.pin_1:
                self.pwm_1.ChangeDutyCycle(duty)
            else:
                self.pwm_2.ChangeDutyCycle(duty)
        time.sleep(0.5)

    def pan_left(self, angle=DEFAULT_ANGLE):
        if self._simulated:
            print(f"[PanYolo 模拟] 云台左转 (垂直: {angle}°)")
            return
        self.set_angle(self.pin_1, 0)
        self.set_angle(self.pin_2, angle)

    def pan_right(self, angle=DEFAULT_ANGLE):
        if self._simulated:
            print(f"[PanYolo 模拟] 云台右转 (垂直: {angle}°)")
            return
        self.set_angle(self.pin_1, 180)
        self.set_angle(self.pin_2, angle)

    def pan_center(self, angle=DEFAULT_ANGLE):
        if self._simulated:
            print(f"[PanYolo 模拟] 云台居中 (垂直: {angle}°)")
            return
        self.set_angle(self.pin_1, 90)
        self.set_angle(self.pin_2, angle)

    def light_on(self):
        if self._simulated:
            print("[PanYolo 模拟] 补光灯开启")
            return
        if PanYolo._using_pigpio:
            self.pi.write(self.pin_light, 1)
        else:
            self.GPIO.output(self.pin_light, self.GPIO.HIGH)

    def light_off(self):
        if self._simulated:
            print("[PanYolo 模拟] 补光灯关闭")
            return
        if PanYolo._using_pigpio:
            self.pi.write(self.pin_light, 0)
        else:
            self.GPIO.output(self.pin_light, self.GPIO.LOW)

    def cleanup(self):
        if self._simulated:
            print("[PanYolo 模拟] 清理完成")
            return
        if PanYolo._using_pigpio:
            self._set_servo_pulse(self.pin_1, 0)
            self._set_servo_pulse(self.pin_2, 0)
            self.pi.write(self.pin_light, 0)
            self.pi.stop()
            print("[PanYolo] pigpio 已释放")
            subprocess.run(['sudo', 'pkill', 'pigpiod'], capture_output=True)
        else:
            self.pwm_1.ChangeDutyCycle(0)
            self.pwm_2.ChangeDutyCycle(0)
            self.pwm_1.stop()
            self.pwm_2.stop()
            self.GPIO.output(self.pin_light, self.GPIO.LOW)
            self.GPIO.cleanup()
            print("[PanYolo] RPi.GPIO 已释放")
        self.cap.release()
        cv2.destroyAllWindows()
        print("[PanYolo] 资源已清理")

    def clear_buffer(self, num_frames=8):
        for _ in range(num_frames):
            self.cap.grab()
        print(f"[PanYolo] 已丢弃 {num_frames} 旧帧")

    def yolo_detection(self, save_count, max_saves=6):
        if save_count >= max_saves:
            return save_count, []
        self.clear_buffer(8)
        ret, frame = self.cap.retrieve()
        if not ret:
            ret, frame = self.cap.read()
            if not ret:
                print("[PanYolo] ❌ 无法读取摄像头画面")
                return save_count, []
        ts = self.cap.get(cv2.CAP_PROP_POS_MSEC)
        print(f"[PanYolo] Frame TS: {ts:.0f}ms")
        try:
            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot()
            classes = results[0].boxes.cls.tolist() if results[0].boxes else []
            class_names = [self.model.names[int(cls)] for cls in classes] if classes else []
            print(f"[PanYolo] 检测到类别: {', '.join(class_names) if class_names else '无'}")
            if class_names:
                self.boardcast.update_sound("shape", class_names[0])
        except Exception as e:
            print(f"[PanYolo] YOLO 检测失败: {e}")
            class_names = []
        image_filename = os.path.join(save_folder, f"image_{int(time.time() * 1000)}.jpg")
        cv2.imwrite(image_filename, annotated_frame)
        save_count += 1
        print(f"[PanYolo] ✅ 保存画面为 {image_filename} ({save_count}/{max_saves})")
        time.sleep(0.2)
        return save_count, class_names

    def reprocess_saved_images(self):
        image_files = [f for f in os.listdir(save_folder) if f.endswith(('.jpg', '.jpeg', '.png'))]
        if len(image_files) == 0:
            print("[PanYolo] ❌ saved_images 文件夹中没有图片")
            return
        full_paths = [(os.path.join(save_folder, f), os.path.getmtime(os.path.join(save_folder, f))) for f in
                      image_files]
        full_paths.sort(key=lambda x: x[1], reverse=True)
        latest_images = full_paths[:6]
        image_files = [os.path.basename(path) for path, mtime in latest_images]
        print(f"[PanYolo] 🎬 开始对最新的 {len(image_files)} 张图片进行 YOLO 重新检测")
        for i, image_file in enumerate(image_files):
            image_path = os.path.join(save_folder, image_file)
            frame = cv2.imread(image_path)
            if frame is None:
                print(f"[PanYolo] ❌ 无法读取图片: {image_path}")
                continue
            try:
                results = self.model(frame, verbose=False)
                annotated_frame = results[0].plot()
                classes = results[0].boxes.cls.tolist() if results[0].boxes else []
                class_names = [self.model.names[int(cls)] for cls in classes] if classes else []
                print(f"[PanYolo] 图片 {image_file} 识别到的类别: {', '.join(class_names) if class_names else '无'}")
                if class_names:
                    self.boardcast.update_sound("shape", class_names[0])
                output_path = os.path.join(output_folder, f"detected_{image_file}")
                cv2.imwrite(output_path, annotated_frame)
                print(f"[PanYolo] ✅ 保存检测结果为 {output_path}")
            except Exception as e:
                print(f"[PanYolo] 图片 {image_file} 检测失败: {e}")

    def print_detection_stats(self, side, all_classes):
        if not all_classes:
            print(f"[PanYolo] [Stats] {side} 侧检测统计: 无检测到对象")
            self.oled.update_display(f"{side} Stats", "No objects")
            return
        class_counter = Counter(all_classes)
        stats_str = f"{side} 侧检测统计: {dict(class_counter)}"
        print(f"[PanYolo] [Stats] {stats_str}")
        most_common = class_counter.most_common(1)
        if most_common:
            most_common_class, max_count = most_common[0]
            print(f"[PanYolo] [Stats] {side} 侧最多类别: {most_common_class} (出现 {max_count} 次)")
            self.oled.update_display(f"{side} Stats", f"Most: {most_common_class} ({max_count})")
        else:
            self.oled.update_display(f"{side} Stats", "No dominant class")
        self.boardcast.update_sound("stats", f"{side.lower()}_stats")

    def execute_detection_sequence(self):
        try:
            clear_folder(save_folder)
            print(f"[PanYolo] 已清空 {save_folder} 文件夹，开始新序列")
            print("[PanYolo] 开始检测序列...")
            save_count = 0
            max_saves = 6
            left_classes = []
            right_classes = []
            self.pan_left()
            time.sleep(1.0)
            self.clear_buffer(5)
            print("[PanYolo] 云台左转 - 连续捕获 3 张图片")
            for shot in range(1, 4):
                print(f"[PanYolo] 左转 - 第 {shot} 张")
                save_count, classes = self.yolo_detection(save_count, max_saves)
                classes = classes if isinstance(classes, list) else []
                self.oled.update_display(f"Left {shot}", f"Classes: {', '.join(classes) if classes else 'None'}")
                left_classes.extend(classes)
                time.sleep(0.5)
            if left_classes:
                left_counter = Counter(left_classes)
                most_common = left_counter.most_common(1)
                if most_common:
                    most_common_class, max_count = most_common[0]
                    print(f"[PanYolo] Left 最多类别: {most_common_class} (出现 {max_count} 次)")
                    self.boardcast.update_sound("person", most_common_class)
                    self.oled.update_display("Left", f"Most: {most_common_class} ({max_count})")
                else:
                    print(f"[PanYolo] Left 统计失败，无有效类别")
                    self.oled.update_display("Left", "No dominant class")
            else:
                print(f"[PanYolo] Left 无检测到对象")
                self.oled.update_display("Left", "No objects")

            self.pan_right()
            time.sleep(1.0)
            self.clear_buffer(5)
            print("[PanYolo] 云台右转 - 连续捕获 3 张图片")
            for shot in range(1, 4):
                print(f"[PanYolo] 右转 - 第 {shot} 张")
                save_count, classes = self.yolo_detection(save_count, max_saves)
                classes = classes if isinstance(classes, list) else []
                self.oled.update_display(f"Right {shot}", f"Classes: {', '.join(classes) if classes else 'None'}")
                right_classes.extend(classes)
                time.sleep(0.3)
            if right_classes:
                right_counter = Counter(right_classes)
                most_common = right_counter.most_common(1)
                if most_common:
                    most_common_class, max_count = most_common[0]
                    print(f"[PanYolo] Right 最多类别: {most_common_class} (出现 {max_count} 次)")
                    self.boardcast.update_sound("person", most_common_class)
                    self.oled.update_display("Right", f"Most: {most_common_class} ({max_count})")
                else:
                    print(f"[PanYolo] Right 统计失败，无有效类别")
                    self.oled.update_display("Right", "No dominant class")
            else:
                print(f"[PanYolo] Right 无检测到对象")
                self.oled.update_display("Right", "No objects")

            self.pan_center()
            time.sleep(0.5)
            self.clear_buffer(3)
            print("[PanYolo] 云台居中")
            self.oled.update_display("Center")
            print("[PanYolo] 序列完成")
            serial_comm.send_data("5|0|0")


        except Exception as e:
            print(f"[PanYolo] 执行序列失败: {e}")
            raise
        finally:
            self.light_off()


if __name__ == "__main__":
    print("测试 pan_yolo.py")
    running_flag.set()
    try:
        serial_comm = SerialCommunication()
        serial_comm.connect()
        pan_yolo = PanYolo()
        seri = Seri(serial_comm, pan_yolo.boardcast, pan_yolo)
        seri.test()  # 启动串口线程
        while running_flag.is_set():
            time.sleep(1)  # 主线程保持运行，等待串口指令
    except Exception as e:
        print(f"[Main] 测试过程中发生错误: {e}")
    finally:
        running_flag.clear()
        pan_yolo.cleanup()
        serial_comm.disconnect()
        print("pan_yolo 测试完成")