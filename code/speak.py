import subprocess
import time
import os
import sys
import pyttsx3

current_dir = os.path.dirname(os.path.abspath(__file__))
tts_script_path = os.path.join(current_dir, "tts.py")
# 使用sys.executable确保使用正确的Python解释器

subprocess.call([sys.executable, tts_script_path, "I am your assistant"])
subprocess.call([sys.executable, tts_script_path, "nihao，I am Lu"])


time.sleep(3)