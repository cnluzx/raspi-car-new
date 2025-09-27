import sys
import pyttsx3


###调用方式： subprocess.call(["python", "tts.py", phrase])
###         subprocess.call(["python", "tts.py", "I am your assistant. How can I help you?"])
###         subprocess.Popen(["python", "tts.py", "I am your assistant"])

def init_engine():
    """初始化 pyttsx3 引擎"""
    engine = pyttsx3.init()
    return engine

def say(text):
    """使用 pyttsx3 播放语音"""
    engine = init_engine()
    engine.say(text)
    engine.runAndWait()

if __name__ == "__main__":
    # 获取命令行参数   执行文件
    if len(sys.argv) < 2:
        print("Error: No text provided for speech.")
        sys.exit(1)

    text_to_speak = sys.argv[1]  # 获取传递的文本
    print(sys.executable)  # 输出当前使用的Python解释器路径
    say(text_to_speak)
