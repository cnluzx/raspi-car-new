# coding=utf-8
import cv2 as cv
import numpy as np
import time
import os
from typing import Tuple, Optional

class StereoDepthProcessor:
    """双目深度处理器类，封装了双目视觉深度计算的相关功能"""

    def __init__(self, intrinsics_path: str = 'intrinsics.yml', extrinsics_path: str = 'extrinsics.yml'):
        """
        初始化双目深度处理器

        Args:
            intrinsics_path: 内参文件路径
            extrinsics_path: 外参文件路径
        """
        # 窗口名称
        self.depth_win_title = 'Depth'
        self.cam_win_title = '3D Cam'

        # 初始化窗口
        self._init_windows()

        # 加载相机参数
        self._load_camera_parameters(intrinsics_path, extrinsics_path)

        # 初始化摄像头
        self.cap = cv.VideoCapture(1)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开摄像头")

        # 图像尺寸
        self.size = (320, 240)

        # 初始化映射
        self._init_rectification_maps()

        # 立体匹配器
        self.stereo = None
        self._init_stereo_matcher()

        # 3D点云数据
        self.threeD = None

        # 帧率计算
        self.frame_count = 0
        self.last_time = time.time()
        self.fps = 0

    def _init_windows(self) -> None:
        """初始化显示窗口"""
        cv.namedWindow(self.depth_win_title)
        cv.namedWindow(self.cam_win_title)

        cv.createTrackbar("num", self.depth_win_title, 2, 10, lambda x: None)  # 默认值设为2
        cv.createTrackbar("blockSize", self.depth_win_title, 15, 255, lambda x: None)  # 默认值设为15

        cv.moveWindow(self.cam_win_title, 100, 50)
        cv.moveWindow(self.depth_win_title, 800, 50)

        # 设置鼠标回调 - 为两个窗口都设置回调
        cv.setMouseCallback(self.depth_win_title, self._mouse_callback, None)
        cv.setMouseCallback(self.cam_win_title, self._mouse_callback, None)

    def _load_camera_parameters(self, intrinsics_path: str, extrinsics_path: str) -> None:
        """加载相机参数"""
        # 加载内参
        fs = cv.FileStorage(intrinsics_path, cv.FILE_STORAGE_READ)
        self.M1 = fs.getNode('M1').mat()
        self.D1 = fs.getNode('D1').mat()
        self.M2 = fs.getNode('M2').mat()
        self.D2 = fs.getNode('D2').mat()
        fs.release()

        # 加载外参
        fs = cv.FileStorage(extrinsics_path, cv.FILE_STORAGE_READ)
        self.R = fs.getNode('R').mat()
        self.T = fs.getNode('T').mat()
        self.R1 = fs.getNode('R1').mat()
        self.P1 = fs.getNode('P1').mat()
        self.R2 = fs.getNode('R2').mat()
        self.P2 = fs.getNode('P2').mat()
        self.Q = fs.getNode('Q').mat()
        fs.release()

    def _init_rectification_maps(self) -> None:
        """初始化校正映射"""
        self.left_map1, self.left_map2 = cv.initUndistortRectifyMap(
            self.M1, self.D1, self.R1, self.P1, self.size, cv.CV_16SC2)
        self.right_map1, self.right_map2 = cv.initUndistortRectifyMap(
            self.M2, self.D2, self.R2, self.P2, self.size, cv.CV_16SC2)

    def _init_stereo_matcher(self) -> None:
        """初始化立体匹配器"""
        num = cv.getTrackbarPos("num", self.depth_win_title)
        block_size = cv.getTrackbarPos("blockSize", self.depth_win_title)
        block_size = max(5, block_size if block_size % 2 == 1 else block_size + 1)

        self.stereo = cv.StereoBM_create(numDisparities=16 * num, blockSize=block_size)

    def _mouse_callback(self, event: int, x: int, y: int, flags: int, param: Optional[any]) -> None:
        """
        鼠标点击回调函数，处理两个窗口的点击事件

        Args:
            event: 鼠标事件类型
            x: 点击位置的x坐标
            y: 点击位置的y坐标
            flags: 鼠标事件标志
            param: 额外参数
        """
        if event == cv.EVENT_LBUTTONDOWN and self.threeD is not None:
            # 获取窗口名称
            win_name = cv.getWindowProperty(param, cv.WND_PROP_VISIBLE)

            # 根据不同的窗口处理坐标
            if param == self.cam_win_title:
                # 如果点击的是原图像窗口，需要确定点击的是左图还是右图
                if x < 320:  # 左图部分
                    # 原图像的左图部分对应于深度图的相同位置
                    depth_x, depth_y = x, y
                else:  # 右图部分
                    # 原图像的右图部分也对应于深度图的相同位置
                    depth_x, depth_y = x - 320, y
            else:
                # 如果点击的是深度图窗口，直接使用坐标
                depth_x, depth_y = x, y

            # 确保坐标在有效范围内
            if 0 <= depth_x < self.size[0] and 0 <= depth_y < self.size[1]:
                # 获取点击位置的深度值
                depth = self.threeD[depth_y, depth_x, 2]

                # 如果深度值是有效值（不是无穷大）
                if not np.isinf(depth):
                    print(f"位置 ({x}, {y}) 的距离: {depth:.2f} mm")

                    # 在图像上标记点击位置
                    if param == self.cam_win_title:
                        # 在原图像上绘制标记
                        cv.circle(self.current_frame, (x, y), 3, (0, 0, 255), -1)
                        cv.putText(self.current_frame, f"{depth:.2f}mm",
                                  (x + 5, y - 5), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                        cv.imshow(self.cam_win_title, self.current_frame)
                else:
                    print(f"位置 ({x}, {y}) 的距离: 无法计算")

    def _update_stereo_matcher(self) -> None:
        """更新立体匹配器参数"""
        num = cv.getTrackbarPos("num", self.depth_win_title)
        block_size = cv.getTrackbarPos("blockSize", self.depth_win_title)
        block_size = max(5, block_size if block_size % 2 == 1 else block_size + 1)

        self.stereo.setNumDisparities(16 * num)
        self.stereo.setBlockSize(block_size)

    def _calculate_fps(self) -> None:
        """计算帧率"""
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_time >= 1.0:
            self.fps = self.frame_count / (current_time - self.last_time)
            self.frame_count = 0
            self.last_time = current_time

    def process_frame(self, frame: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        处理单帧图像

        Args:
            frame: 输入的双目图像帧

        Returns:
            Tuple[np.ndarray, np.ndarray]: (处理后的图像, 深度图像)
        """
        try:
            # 保存当前帧用于鼠标回调
            self.current_frame = frame.copy()

            # 调整图像大小
            frame = cv.resize(frame, (640, 240), interpolation=cv.INTER_LINEAR)

            # 分割左右图像
            frame_left = frame[0:240, 0:320]
            frame_right = frame[0:240, 320:640]

            # 校正图像
            left_rectified = cv.remap(frame_left, self.left_map1, self.left_map2, cv.INTER_LINEAR)
            right_rectified = cv.remap(frame_right, self.right_map1, self.right_map2, cv.INTER_LINEAR)

            # 拼接校正后的图像
            rectified = np.concatenate([left_rectified, right_rectified], axis=1)
            image = np.concatenate([frame, rectified])

            # 转换为灰度图
            imgL = cv.cvtColor(left_rectified, cv.COLOR_BGR2GRAY)
            imgR = cv.cvtColor(right_rectified, cv.COLOR_BGR2GRAY)

            # 更新立体匹配器参数
            self._update_stereo_matcher()

            # 计算视差图
            disparity = self.stereo.compute(imgL, imgR)

            # 归一化视差图用于显示
            disp = cv.normalize(disparity, disparity, alpha=0, beta=255, norm_type=cv.NORM_MINMAX, dtype=cv.CV_8U)

            # 计算深度图
            self.threeD = cv.reprojectImageTo3D(disparity.astype(np.float32) / 16., self.Q)

            # 在深度图像上添加距离信息
            depth_display = disp.copy()
            center_y, center_x = depth_display.shape[0] // 2, depth_display.shape[1] // 2
            center_depth = self.threeD[center_y, center_x, 2]
            if not np.isinf(center_depth):
                cv.putText(depth_display, f"中心距离: {center_depth:.2f} mm",
                          (10, 30), cv.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 添加网格线
            for i in range(1, 30):
                cv.line(image, (0, 16 * i), (640, 16 * i), (0, 255, 0), 1)

            # 添加FPS信息
            self._calculate_fps()
            cv.putText(image, f"FPS: {self.fps:.1f}", (10, image.shape[0] - 10),
                      cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            return image, depth_display

        except Exception as e:
            print(f"处理帧时出错: {e}")
            return frame, np.zeros((240, 320), dtype=np.uint8)

    def run(self) -> None:
        """运行主循环"""
        try:
            while True:
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    print("无法获取帧，请检查摄像头连接")
                    time.sleep(1)  # 等待一秒后继续尝试
                    continue

                # 处理帧
                image, depth_display = self.process_frame(frame)

                # 显示结果
                cv.imshow(self.cam_win_title, image)
                cv.imshow(self.depth_win_title, depth_display)

                # 检查退出键
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            print("程序被用户中断")
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """清理资源"""
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    try:
        processor = StereoDepthProcessor()
        processor.run()
    except Exception as e:
        print(f"程序运行出错: {e}")
