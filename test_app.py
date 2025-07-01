
import signal
import sys
import time
from flask import Flask, render_template, Response

# 关键：直接从您现有的文件中导入摄像头处理器
# 请确保此文件与 unified_camera_handler.py 在同一目录下
try:
    from unified_camera_handler import UnifiedCamera
except ImportError:
    print("错误: 无法找到 unified_camera_handler.py。")
    print("请将本测试文件与 unified_camera_handler.py 放在同一个文件夹下。")
    sys.exit(1)

# --- Flask 应用初始化 ---
app = Flask(__name__)

# --- 全局摄像头实例 ---
# 在应用启动时只初始化一次摄像头，这是正确的做法。
print("正在初始化摄像头，请稍候...")
camera = UnifiedCamera()
print("摄像头初始化完毕。")


# --- 视频流生成器 ---
def generate_frames():
    """
    一个生成器函数，不断地从摄像头处理器获取
    已经编码好的JPEG帧，并按照multipart格式产出。
    """
    while True:
        # 1. 从摄像头处理器获取JPEG字节流。
        #    get_jpeg_frame()内部是线程安全的，它只是返回一个预先编码好的字节对象。
        frame_bytes = camera.get_jpeg_frame()

        # 2. 如果获取失败（比如摄像头刚启动还没准备好），则跳过这一帧。
        if frame_bytes is None:
            time.sleep(0.01) # 短暂等待，避免空转消耗CPU
            continue
        
        # 3. 使用yield关键字，将每一帧作为HTTP multipart响应的一部分发送出去。
        #    浏览器看到这个格式，就会流式地更新<img>标签的内容。
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# --- Flask 路由 ---
@app.route('/')
def index():
    """渲染主页，这个页面只包含一个视频画面。"""
    return render_template('test_index.html')

@app.route('/video_feed')
def video_feed():
    """这个路由专门提供视频流。"""
    return Response(generate_frames(), 
                    mimetype='multipart/x-mixed-replace; boundary=frame')

# --- 程序退出时的清理工作 ---
def cleanup(sig, frame):
    """确保在按 Ctrl+C 退出时，能安全地释放摄像头资源。"""
    print("\n接收到关停信号，正在释放摄像头...")
    if camera:
        camera.release()
    print("清理完毕，程序退出。")
    sys.exit(0)

# --- 主程序入口 ---
if __name__ == '__main__':
    # 绑定信号处理，实现优雅退出
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)

    print("\n--- 摄像头视频流测试服务已启动 ---")
    print("* 请在浏览器中打开: http://127.0.0.1:8888")
    print("* 按 Ctrl+C 停止服务。")
    print("---------------------------------\n")
    
    # 运行Flask应用
    # debug=False 和 threaded=True 是生产环境中处理多请求的推荐配置
    app.run(host='0.0.0.0', port=8888, debug=False, threaded=True)