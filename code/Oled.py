from Variable import *

"""
###oled模块

功能:在oled上更新显示
t1,t2,t3,t4为第n行显示内容

使用方式
step1: oled = Oled()
step2: oled.threading_start()
step3: oled.update_display(t1=None, t2=None, t3=None,t4=None) # 延时显示2s


"""


class Oled:
    def __init__(self):
        from luma.core.interface.serial import i2c, spi
        from luma.core.render import canvas
        from luma.oled.device import ssd1306, ssd1325, ssd1331, sh1106
        from time import sleep
        self.canvas = canvas
        self.serial = i2c(port=1, address=0x3C)  # 初始化端口
        self.device_module = {'ssd1306': ssd1306}
        self.device = self.device_module['ssd1306'](self.serial)

    # 调用显示函数
    def show_text(self, t1=None, t2=None, t3=None, t4=None, t5=None, t6=None):
        with self.canvas(self.device) as draw:
            draw.rectangle(self.device.bounding_box, outline="white", fill="black")
            draw.text((10, 0), str(t1), fill="white")
            draw.text((10, 10), str(t2), fill="white")
            draw.text((10, 20), str(t3), fill="white")
            draw.text((10, 30), str(t4), fill="white")
            draw.text((10, 40), "status: running", fill="white")
            draw.text((10, 50), f"time: {time.strftime('%H:%M:%S')}", fill="white")
        time.sleep(2)  # 延时显示3s

    def threading_text(self):
        global display_text
        print("oled显示线程已启动...")
        while running_thread_event.is_set():
            if text_event.is_set():
                print("text_event ok")
                current_text = display_text.copy()
                self.show_text(*current_text)
                text_event.clear()
            time.sleep(1)

    def update_display(t1=None, t2=None, t3=None, t4=None):
        """更新显示内容的公共方法（线程安全）"""
        display_text[0] = t1
        display_text[1] = t2
        display_text[2] = t3
        display_text[3] = t4
        text_event.set()  # 触发显示更新


if __name__ == '__main__':
    oled = Oled()
    running_thread_event.set()

    # 先启动线程，再触发事件
    oled.threading_start()

    # 等待一小段时间确保线程已启动
    time.sleep(0.1)

    # 保持主线程运行，避免守护线程被终止
    try:
        while True:
            # 触发第一次显示
            oled.update_display("detect--succcessfully!", "shape:rec", "point:12", "ok")
            time.sleep(1)
    except KeyboardInterrupt:
        running_thread_event.clear()
        print("程序退出")
