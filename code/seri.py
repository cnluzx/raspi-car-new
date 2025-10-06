# seri.py
import time
import re
from collections import Counter
import queue  # 新增：导入 queue 以使用 queue.Empty 和 queue.Full
from utils import data_queue, running_flag, id_event, detection_event, text_event, id_data


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
                    # 新增：事件级防抖 + 内容去重
                    if (now - self.last_detection_trigger_time >= self.debounce_interval and
                            self.current_data != self.last_detection_data):
                        print(f"time: {current_time} 处理：触发检测任务！数据: {self.current_data}")
                        detection_event.set()
                        self.last_detection_trigger_time = now
                        self.last_detection_data = self.current_data
                        if ":TYPE1" in self.current_data:
                            print("特定信号检查：执行 YOLO 检测任务")
                            text_event.set()
                    else:
                        print(
                            f"time: {current_time} 检测信号忽略（防抖/重复）：间隔 {now - self.last_detection_trigger_time:.2f}s, 上次数据: {self.last_detection_data}")
                elif self.current_data.startswith("@"):
                    global id_data
                    id_data = self.current_data
                    # 新增：事件级防抖 + 内容去重
                    if (now - self.last_id_trigger_time >= self.debounce_interval and
                            self.current_data != self.last_id_data):
                        print(f"time: {current_time} 处理：触发识别任务！数据: {self.current_data}")
                        id_event.set()
                        self.last_id_trigger_time = now
                        self.last_id_data = self.current_data
                    else:
                        print(
                            f"time: {current_time} ID 信号忽略（防抖/重复）：间隔 {now - self.last_id_trigger_time:.2f}s, 上次数据: {self.last_id_data}")
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
        from utils import running_flag, id_event, id_data
        while running_flag.is_set():
            if id_event.wait(timeout=0.05):
                global id_data
                if id_data:
                    print(f"ID 处理: 识别到数据: {id_data}")
                    if id_data.startswith("@"):
                        # 增强解析：提取第一个@数字
                        match = re.search(r'@(\d+)', id_data)
                        if match:
                            mapped_id = match.group(1)
                            if mapped_id in id_mapping.values():
                                print(f"ID 处理: 匹配到 {mapped_id}")
                                # 假设 oled_ins 已导入或全局
                                print(f"模拟 OLED 更新: id:{mapped_id}")
                                self.boardcast.update_sound("point", mapped_id)  # 新增：播报 ID
                                id_data = None  # 清空
                                continue
                        mapped_id = id_mapping.get(id_data.strip(), None)
                        if mapped_id:
                            print(f"ID 处理: 匹配到 {mapped_id}")
                            print(f"模拟 OLED 更新: id:{mapped_id}")
                            self.boardcast.update_sound("point", mapped_id)  # 新增：播报 ID
                        else:
                            print(f"ID 处理: 未识别的 ID: {id_data}")
                            print("模拟 OLED 更新: 未知 ID")
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
        from utils import start_thread
        running_flag.set()
        start_thread("put_queue_stream", self.put_queue_stream)
        start_thread("read_queue_stream", self.read_queue_stream)
        start_thread("id_stream", self.id_stream)
        start_thread("cast", self.boardcast.threading_sound)  # 新增：启动语音播报线程
        if self.serial_comm and self.serial_comm.is_open():
            self.serial_comm.send_data("1|0|0")
            print("测试：已发送开始信号 '1|0|0'")
        # 移除 time.sleep 和 clear，让它一直运行，直到外部停止


if __name__ == "__main__":
    print("测试 seri.py")
    from serial_comm import SerialCommunication
    from broadcast import Boardcast

    try:
        serial_comm = SerialCommunication(baudrate=115200)
        serial_comm.connect()
        boardcast = Boardcast()
        seri = Seri(serial_comm, boardcast)
        seri.test()
        # 一直运行，直到 KeyboardInterrupt
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("收到 Ctrl+C，停止测试...")
            from utils import running_flag

            running_flag.clear()
            time.sleep(1)  # 等待线程停止
    except Exception as e:
        print(f"seri 测试异常: {e}")
    finally:
        if 'serial_comm' in locals():
            serial_comm.disconnect()
        print("seri 测试完成")