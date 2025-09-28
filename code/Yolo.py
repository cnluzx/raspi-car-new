from ultralytics import YOLO
import cv2
import os

# 创建保存图片的文件夹
save_folder = 'saved_images'
if not os.path.exists(save_folder):
    os.makedirs(save_folder)

# 打开摄像头 (默认使用设备 1，可以尝试改成 0 或其他)
cap = cv2.VideoCapture(1)
if not cap.isOpened():
    print("❌ 无法找到可用摄像头，请检查设备或权限")
    exit()

# 自动设置低分辨率以提高性能
cap.set(3, 640)  # 宽
cap.set(4, 480)  # 高

# 加载 YOLOv8n 模型
model = YOLO("1989.pt")  #

print("🎬 实时视频流检测开始，按 'q' 退出，按 's' 保存当前画面")

while True:
    ret, frame = cap.read()
    if not ret:
        print("❌ 无法读取摄像头画面")
        break

    # YOLO 推理
    results = model(frame, verbose=False)

    # 可视化结果
    annotated_frame = results[0].plot()

    # 提取并显示识别到的类别
    classes = results[0].boxes.cls.tolist()  # 识别到的类别 ID
    class_names = [model.names[int(cls)] for cls in classes]  # 类别名称
    print("识别到的类别:", ', '.join(class_names))  # 输出类别

    # 显示画面
    cv2.imshow("YOLOv8 实时检测", annotated_frame)

    # 按 's' 键保存当前画面
    if cv2.waitKey(1) & 0xFF == ord('s'):
        image_filename = os.path.join(save_folder, f"image_{cv2.getTickCount()}.jpg")
        cv2.imwrite(image_filename, annotated_frame)
        print(f"✅ 保存画面为 {image_filename}")

    # 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()
