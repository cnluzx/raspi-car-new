# coding=utf-8
import cv2
import numpy as np
import os

# 全局变量
rectifyImageL = None
rectifyImageR = None
validROIL = None  # 图像校正之后，会对图像进行裁剪，这里的validROI就是指裁剪之后的区域
validROIR = None
Q = None
xyz = None  # 三维坐标
bm = None
blockSize = 0
uniquenessRatio = 0
numDisparities = 0

# 鼠标相关变量
origin = None  # 鼠标按下的起始点
selection = None  # 定义矩形选框
selectObject = False  # 是否选择对象


def stereo_match_sgbm(val):
    """SGBM匹配算法"""
    global bm, blockSize, uniquenessRatio, numDisparities, validROIL, validROIR, xyz, Q

    if bm is None:
        bm = cv2.StereoBM_create(numDisparities=16, blockSize=9)

    bm.setBlockSize(2 * blockSize + 5)  # SAD窗口大小，5~21之间为宜
    bm.setROI1(validROIL)
    bm.setROI2(validROIR)
    bm.setPreFilterCap(31)
    bm.setMinDisparity(0)  # 最小视差，默认值为0, 可以是负值，int型
    bm.setNumDisparities(numDisparities * 16 + 16)  # 视差窗口，即最大视差值与最小视差值之差,窗口大小必须是16的整数倍，int型
    bm.setTextureThreshold(10)
    bm.setUniquenessRatio(uniquenessRatio)  # uniquenessRatio主要可以防止误匹配
    bm.setSpeckleWindowSize(100)
    bm.setSpeckleRange(32)
    bm.setDisp12MaxDiff(-1)

    disp = bm.compute(rectifyImageL, rectifyImageR)  # 输入图像必须为灰度图
    disp8 = cv2.normalize(disp, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # 计算出的视差是CV_16S格式
    xyz = cv2.reprojectImageTo3D(disp, Q, handleMissingValues=True)
    # 在实际求距离时，ReprojectTo3D出来的X / W, Y / W, Z / W都要乘以16(也就是W除以16)，才能得到正确的三维坐标信息。
    xyz = xyz * 16

    cv2.imshow("disparity", disp8)


def onMouse(event, x, y, flags, param):
    """鼠标操作回调"""
    global origin, selection, selectObject, xyz

    if selectObject:
        selection.x = min(x, origin[0])
        selection.y = min(y, origin[1])
        selection.width = abs(x - origin[0])
        selection.height = abs(y - origin[1])

    if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左按钮按下的事件
        origin = (x, y)
        selection = (x, y, 0, 0)
        selectObject = True
        if xyz is not None:
            print(f"Point at {origin} in world coordinate is: {xyz[y, x]}")

    elif event == cv2.EVENT_LBUTTONUP:  # 鼠标左按钮释放的事件
        selectObject = False
        if selection[2] > 0 and selection[3] > 0:
            pass


def main():
    global rectifyImageL, rectifyImageR, validROIL, validROIR, Q, xyz, selection

    # 初始化选择矩形
    selection = (0, 0, 0, 0)

    # 读取图像
    viewLeft = cv2.imread("D:\\table\\last_project\\raspi-car-new\\stereo\\data\\0left.jpg")
    viewRight = cv2.imread("D:\\table\\last_project\\raspi-car-new\\stereo\\data\\0left.jpg")

    if viewLeft is None or viewRight is None:
        print("无法读取图像")
        return

    cv2.imshow("viewLeft", viewLeft)
    cv2.waitKey(40)
    print("done")

    imageSize = viewLeft.shape[:2][::-1]  # 获取图像尺寸 (width, height)

    # 读取相机参数
    fs = cv2.FileStorage("intrinsics.yml", cv2.FILE_STORAGE_READ)
    if fs.isOpened():
        print("read")
        intrMatFirst = fs.getNode("M1").mat()
        distCoeffsFirst = fs.getNode("D1").mat()
        intrMatSec = fs.getNode("M2").mat()
        distCoffesSec = fs.getNode("D2").mat()
        R = fs.getNode("R").mat()
        T = fs.getNode("T").mat()
        Q = fs.getNode("Q").mat()
        print("M1", intrMatFirst, "\n", distCoeffsFirst)
        fs.release()
    else:
        print("无法打开参数文件")
        return

    print("stereo rectify...")
    # 立体校正
    R1, R2, P1, P2, Q, validROIL, validROIR = cv2.stereoRectify(
        intrMatFirst, distCoeffsFirst, intrMatSec, distCoffesSec, imageSize, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=-1, newImageSize=imageSize)

    # 初始化校正映射
    rmapFirst = cv2.initUndistortRectifyMap(intrMatFirst, distCoeffsFirst, R1, P1, imageSize, cv2.CV_16SC2)
    rmapSec = cv2.initUndistortRectifyMap(intrMatSec, distCoffesSec, R2, P2, imageSize, cv2.CV_16SC2)

    # 应用校正映射
    rectifyImageL = cv2.remap(viewLeft, rmapFirst[0], rmapFirst[1], cv2.INTER_LINEAR)
    rectifyImageR = cv2.remap(viewRight, rmapSec[0], rmapSec[1], cv2.INTER_LINEAR)

    # 转换为灰度图
    rectifyImageL = cv2.cvtColor(rectifyImageL, cv2.COLOR_BGR2GRAY)
    rectifyImageR = cv2.cvtColor(rectifyImageR, cv2.COLOR_BGR2GRAY)

    cv2.imshow("remap_left", rectifyImageL)
    cv2.imshow("remap_right", rectifyImageR)

    # 创建窗口和滑动条
    cv2.namedWindow("disparity", cv2.WINDOW_NORMAL)

    # 创建滑动条
    cv2.createTrackbar("BlockSize:\n", "disparity", 0, 8, stereo_match_sgbm)
    cv2.createTrackbar("UniquenessRatio:\n", "disparity", 0, 50, stereo_match_sgbm)
    cv2.createTrackbar("NumDisparities:\n", "disparity", 0, 16, stereo_match_sgbm)

    # 设置鼠标回调
    cv2.setMouseCallback("disparity", onMouse, 0)

    # 初始调用立体匹配算法
    stereo_match_sgbm(0)

    # 等待按键
    cv2.waitKey(0)

    # 清理资源
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
