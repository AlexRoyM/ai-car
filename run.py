import signal
from main import app, socketio, initialize_app, cleanup
from config import FLASK_PORT

if __name__ == '__main__':
    signal.signal(signal.SIGINT, cleanup)
    signal.signal(signal.SIGTERM, cleanup)
    
    initialize_app()
    
    print("\n--- AI小车控制服务已启动 (WebSocket模式) ---")
    print(f"* 访问地址 1: http://127.0.0.1:{FLASK_PORT}")
    #print(f"* 请将浏览器指向你小车的实际IP地址, 例如: http://192.168.10.103:{FLASK_PORT}")
    print("------------------------------------------\n")

    # 使用 socketio.run() 启动服务器
    socketio.run(app, host='0.0.0.0', port=FLASK_PORT)
