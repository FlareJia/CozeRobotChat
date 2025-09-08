import cv2
import pyrealsense2 as rs
import numpy as np

def save_stable_frame(target_frame=50):
    # 初始化相机管道
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 配置彩色流参数
    config.enable_stream(
        rs.stream.color, 
        width=640, 
        height=480, 
        format=rs.format.bgr8, 
        framerate=30
    )
    
    try:
        # 启动相机
        pipeline.start(config)
        print(f"相机启动成功，将在第{target_frame}帧保存图像...")
        
        frame_count = 0
        saved = False
        
        while not saved:
            # 获取一帧数据
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            
            # 跳过无效帧
            if not color_frame:
                continue
            
            # 计数有效帧
            frame_count += 1
            print(f"已采集: {frame_count}/{target_frame}帧", end='\r')
            
            # 达到目标帧时保存
            if frame_count == target_frame:
                # 转换为OpenCV格式
                color_image = np.asanyarray(color_frame.get_data())
                
                # 保存图像
                save_path = "stable_color_image.jpg"
                cv2.imwrite(save_path, color_image)
                print(f"\n成功保存第{target_frame}帧图像至: {save_path}")
                saved = True
                
    except KeyboardInterrupt:
        print("\n用户中断操作")
    finally:
        # 停止相机
        pipeline.stop()
        print("相机已关闭")

if __name__ == "__main__":
    # 保存第50帧（可修改数字调整目标帧）
    save_stable_frame(target_frame=50)

    
        
#在 Python 中调用 RealSense 摄像头需要使用 `pyrealsense2` 库（Intel 官方提供的 Python 绑定），以下是一个简单的示例文件，可同时获取彩色图像和深度图像并显示：


# 使用说明：
#1. **确保已安装依赖**：  
#   如果还未安装 `pyrealsense2`，先执行以下命令：  
#   ```bash
#   sudo apt install python3-pyrealsense2
#   ```

#2. **运行脚本**：  
#   将上述代码保存为 `realsense_camera.py`，然后运行：  
#   ```bash
#   python3 realsense_camera.py
#   ```

#3. **功能说明**：  
#   - 脚本会同时显示两个窗口：彩色图像（`Color Image`）和彩色化的深度图像（`Depth Image`）。  
#   - 深度图像中，不同颜色代表不同距离（红色表示较远，蓝色表示较近）。  
#   - 按 `q` 键可退出程序并关闭相机。


# 可自定义的参数：
#- **分辨率和帧率**：修改 `config.enable_stream` 中的参数（如 `640, 480` 改为 `1280, 720`，但需相机支持）。  
#- **深度图像颜色映射**：将 `cv2.COLORMAP_JET` 改为其他类型（如 `cv2.COLORMAP_HSV`）。  
#- **只获取单一类型图像**：删除不需要的流配置（如只保留彩色流或深度流）。

#如果需要获取更详细的信息（如相机内参、深度值计算等），可以参考 Intel 官方的 [pyrealsense2 文档](https://dev.intelrealsense.com/docs/python2)。
