import time
import RPi.GPIO as GPIO

class Pan:
    DEFAULT_ANGLE = 120

    def __init__(self):
        # GPIO 引脚
        self.pin_1 = 18  # 水平舵机
        self.pin_2 = 13  # 垂直舵机
        self.pin_light = 4  # 补光灯（可选）

        # 初始化 GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin_1, GPIO.OUT)
        GPIO.setup(self.pin_2, GPIO.OUT)
        GPIO.setup(self.pin_light, GPIO.OUT)

        # 初始化 PWM，频率 50Hz
        self.pwm_1 = GPIO.PWM(self.pin_1, 50)
        self.pwm_2 = GPIO.PWM(self.pin_2, 50)
        self.pwm_1.start(0)
        self.pwm_2.start(0)

        print("[Pan] 初始化完成")

    def _angle_to_duty(self, angle):
        """角度转占空比"""
        angle = max(0, min(180, angle))  # 限制在0~180度
        duty = 2 + (angle / 18)
        return duty

    def set_angle(self, pin_pwm, angle):
        duty = self._angle_to_duty(angle)
        pin_pwm.ChangeDutyCycle(duty)
        time.sleep(0.5)
        pin_pwm.ChangeDutyCycle(0)  # 停止发送信号，减少抖动
        ###上面舵机固定角度
    # ---------- 云台动作 ----------
    def pan_left(self, angle=DEFAULT_ANGLE):
        self.set_angle(self.pwm_1, 0)
        self.set_angle(self.pwm_2, angle)

    def pan_right(self, angle=DEFAULT_ANGLE):
        self.set_angle(self.pwm_1, 180)
        self.set_angle(self.pwm_2, angle)

    def pan_center(self, angle=DEFAULT_ANGLE):
        self.set_angle(self.pwm_1, 90)
        self.set_angle(self.pwm_2, angle)

    def pan_turn(self, hor_angle, ver_angle):
        self.set_angle(self.pwm_1, hor_angle)
        self.set_angle(self.pwm_2, ver_angle)

    # ---------- 补光灯 ----------
    def light_on(self):
        GPIO.output(self.pin_light, GPIO.HIGH)

    def light_off(self):
        GPIO.output(self.pin_light, GPIO.LOW)

    # ---------- 安全退出 ----------
    def cleanup(self):
        self.pwm_1.stop()
        self.pwm_2.stop()
        GPIO.cleanup()
        print("[Pan] GPIO 已释放")

    # ---------- 测试方法 ----------
    def test(self):
        try:
            self.pan_left()
            time.sleep(2)
            self.pan_right()
            time.sleep(2)
            self.pan_center()
            time.sleep(2)
        finally:
            self.cleanup()


if __name__ == "__main__":
    pan = Pan()
    pan.test()
