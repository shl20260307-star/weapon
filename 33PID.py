import cv2
import numpy as np
import serial
import struct
import threading
import time
from collections import deque
from ultralytics import YOLO
import tkinter as tk
from tkinter import ttk
import math
from pynput import keyboard
from queue import Queue
from simple_pid import PID

class YOLOTracker:
    def __init__(self, serial_port='COM27'):
        # 初始化YOLOv8n模型
        self.model = YOLO('yolo11n.pt')
        self.track_class = "person"  # 默认跟踪人
        
        # 云台控制参数
        self.pan_angle = 0    # 当前水平角度 (0-360)
        self.tilt_angle = 30  # 当前俯仰角度 (0-60)
        self.fov = (60, 40)   # 摄像头视野(水平,垂直)
        self.img_size = (1920, 1080)
        self.smoothing_factor = 0.2  # 平滑系数
        self.reverse_pan = True  # 默认水平方向反转
        
        # 摄像头安装偏差补偿参数 (单位:像素)
        self.camera_offset_x = -20  # 向左偏差50mm（转换为像素值，假设1mm≈1像素）
        self.camera_offset_y = 0   # 垂直方向无偏差
        
        # 手动控制参数
        self.manual_control = False  # 手动控制标志
        self.step = 1               # 手动控制步长
        self.min_tilt = 0            # 最小俯仰角度
        self.max_tilt = 60           # 最大俯仰角度
        self.tilt_zero = 30          # 俯仰轴零点位置
        # -------------------------------------------------------------





        # PID
        



        # -------------------------------------------------------------
        self.pan_pid = PID(Kp=0.008, Ki=0, Kd=0, setpoint=0)  # 水平PIDCXCS
        self.tilt_pid = PID(Kp=0.008, Ki=0, Kd=0, setpoint=0)  # 俯仰PID
        # -------------------------------------------------------------






        # 限制输出范围（根据实际需求调整）
        self.pan_pid.output_limits = (-30, 30)  # 限制最大角度变化量
        self.tilt_pid.output_limits = (-15, 15)
        # -------------------------------------------------------------
        # GPIO控制状态
        self.gpio_state = {
            '25': False,
            '26': False,
            '27_pulse': False
        }
        
        # 串口通信
        self.serial_lock = threading.Lock()
        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=115200,
                timeout=0.5,
                write_timeout=0.5
            )
            time.sleep(2)  # 等待串口初始化
            print("串口连接成功")
        except Exception as e:
            print(f"串口连接失败: {e}")
            self.ser = None
            
        # 创建GUI界面
        self.root = tk.Tk()
        self.root.geometry("1024x768")  # 初始窗口大小（示例值）
        self.root.resizable(True, True)  # 允许水平和垂直缩放
        self.root.title("YOLO云台跟踪系统")
        self.setup_ui()
        
        # 视频帧队列 (用于线程间通信)
        self.frame_queue = Queue(maxsize=1)
        
        # 先初始化视频捕获对象
        self.cap = None
        try:
            self.cap = cv2.VideoCapture(1)
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"摄像头实际分辨率: {actual_width}x{actual_height}")  # 调试用
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_size[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_size[1])
        except Exception as e:
            print(f"摄像头初始化失败: {e}")
            
        # 启动键盘监听
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        
        # 运行状态标志
        self.running = True
        self.tracking = False  # 跟踪状态
        
        # 启动视频处理线程
        self.video_thread = threading.Thread(target=self.video_processing_thread, daemon=True)
        self.video_thread.start()
        
        # 启动主循环
        self.root.after(10, self.update_gui)
        self.root.mainloop()
      
    def setup_ui(self):
        """设置GUI界面"""
        # 视频显示区域
        self.video_frame = ttk.Label(self.root)
        self.video_frame.pack(fill=tk.BOTH, expand=True)  # 填充并扩展
        
        # 控制面板
        self.control_frame = ttk.Frame(self.root)
        self.control_frame.pack(pady=10)
        
        # 控制按钮
        ttk.Button(self.control_frame, text="开始跟踪", command=self.start_tracking).pack(side=tk.LEFT, padx=5)
        
        ttk.Button(self.control_frame, text="归零位置", command=self.go_home).pack(side=tk.LEFT, padx=5)
        
        # 目标类别选择
        self.class_var = tk.StringVar(value="person")
        ttk.Label(self.control_frame, text="跟踪目标:").pack(side=tk.LEFT, padx=5)
        ttk.Combobox(self.control_frame, 
                    textvariable=self.class_var,
                    values=["person", "bicycle", "bottle", "vase"]).pack(side=tk.LEFT)
        
        # 状态显示
        self.status_var = tk.StringVar(value="状态: 准备就绪")
        ttk.Label(self.root, textvariable=self.status_var, font=('Arial', 12)).pack()
        
        # 角度显示
        self.angle_var = tk.StringVar(value="当前角度: 水平0° 俯仰30°")
        ttk.Label(self.root, textvariable=self.angle_var, font=('Arial', 12)).pack()
        
        # 控制模式显示
        self.mode_var = tk.StringVar(value="模式: 自动跟踪")
        ttk.Label(self.root, textvariable=self.mode_var, font=('Arial', 12)).pack()
        
        # 键盘控制提示
        self.help_var = tk.StringVar(
            value="键盘控制:\nA/D:旋转轴±10°(手动)\nW/S:俯仰轴±10°(手动)\nR:归零(任何模式)\nV:切换手动/自动\nZ/X:GPIO25/26(任何模式)\nC:GPIO27脉冲(任何模式)")
        ttk.Label(self.root, textvariable=self.help_var, font=('Arial', 10), justify=tk.LEFT).pack()

    def calculate_shortest_path(self, current, target):
        """计算最短旋转路径"""
        diff = (target - current) % 360
        return diff - 360 if diff > 180 else diff

    def smart_rotate(self, direction):
        """智能旋转(就近)"""
        step = self.step if direction == '+' else -self.step
        current = self.pan_angle
        target1 = (current + step) % 360
        target2 = (current - step) % 360
        
        if abs(self.calculate_shortest_path(current, target1)) <= \
           abs(self.calculate_shortest_path(current, target2)):
            self.pan_angle = target1
        else:
            self.pan_angle = target2

    def smart_home(self):
        """智能归零"""
        current_pan = self.pan_angle
        shortest_move = self.calculate_shortest_path(current_pan, 0)
        self.pan_angle = (current_pan + shortest_move) % 360
        self.tilt_angle = self.tilt_zero

    def send_angles(self):
        """发送角度到ESP32"""
        if not self.ser or not self.ser.is_open:
            print("串口未连接")
            return
            
        try:
            with self.serial_lock:
                # 新增GPIO控制字节 (bit0:GPIO25, bit1极GPIO26, bit2:GPIO27脉冲)
                gpio_byte = (self.gpio_state['25'] << 0) | \
                           (self.gpio_state['26'] << 1) | \
                           (self.gpio_state['27_pulse'] << 2)
                
                data = struct.pack(">iiBB",
                                 int(self.pan_angle * 100),  # 放大100倍
                                 int(self.tilt_angle * 100),  # 放大100倍
                                 gpio_byte,
                                 0xFF)
                self.ser.write(data)
                self.ser.flush()
                print(f"已发送: 旋转{self.pan_angle}°, 俯仰{self.tilt_angle}°, GPIO25:{self.gpio_state['25']}, GPIO26:{self.gpio_state['26']}, GPIO27脉冲:{self.gpio_state['27_pulse']}")
                
                # 更新状态显示
                self.angle_var.set(f"当前角度: 水平{self.pan_angle:.1f}° 俯仰{self.tilt_angle:.1f}°")
                
                # 如果是脉冲信号,发送后立即复位
                if self.gpio_state['27_pulse']:
                    self.gpio_state['27_pulse'] = False
        except Exception as e:
            print(f"发送失败: {e}")

    def on_press(self, key):
        """键盘按键处理"""
        try:
            char = key.char.lower()
            
            if char == 'v':  # 切换手动/自动模式
                self.manual_control = not self.manual_control
                if self.manual_control:
                    self.mode_var.set("模式: 手动控制")
                    self.stop_tracking()
                else:
                    self.mode_var.set("模式: 自动跟踪")
                self.send_angles()
                return
            
            # R键归零 - 任何模式下都可用
            if char == 'r':
                self.smart_home()
                self.send_angles()
                return
            
            # GPIO控制 - 任何模式下都可用
            if char == 'z':  # GPIO25
                self.gpio_state['25'] = True
                self.send_angles()
            elif char == 'x':  # GPIO26 
                self.gpio_state['26'] = True
                self.send_angles()  


            elif char == 'c':  # GPIO27脉冲
                self.gpio_state['27_pulse'] = True
                self.send_angles()
            
            # 手动控制模式下处理按键 (A/D/W/S)
            if self.manual_control:
                if char == 'a':  # 智能+
                    self.smart_rotate('+')
                elif char == 'd':  # 智能-
                    self.smart_rotate('-')
                elif char == 'w':  # 俯仰+
                    self.tilt_angle = min(self.tilt_angle + self.step, self.max_tilt)
                elif char == 's':  # 俯仰-
                    self.tilt_angle = max(self.tilt_angle - self.step, self.min_tilt)
                
                self.send_angles()
            
        except AttributeError:
            pass

    def on_release(self, key):
        """键盘释放处理"""
        try:
        
            char = key.char.lower()
            if char == 'z':  # GPIO25释放
                self.gpio_state['25'] = False
                self.send_angles()
            elif char == 'x':  # GPIO26释放
                self.gpio_state['26'] = False
                self.send_angles()
        except AttributeError:
            pass

    def calculate_angles(self, target_x, target_y):
        """计算目标位置对应的云台角度"""
        if self.manual_control:  # 手动控制优先
            return
            
        img_center_x = self.img_size[0] / 2 + self.camera_offset_x  # 补偿摄像头安装偏差
        img_center_y = self.img_size[1] / 2 + self.camera_offset_y
        
        # 计算偏移量 (像素坐标 → 角度偏移)
        offset_x = target_x - img_center_x
        offset_y = target_y - img_center_y
        
        # 转换为角度偏移 (考虑视野FOV)
        pan_offset = (offset_x / (self.img_size[0]/2)) * (self.fov[0] / 2)
        tilt_offset = -(offset_y / (self.img_size[1]/2)) * (self.fov[1] / 2)  # Y轴向下为正
        
        # 水平方向默认反转
        pan_offset = -pan_offset
        #
        # PID计算控制量 ---------------------------------------------------
        pan_output = self.pan_pid(offset_x)
        tilt_output = self.tilt_pid(offset_y)
        # 更新角度（直接应用PID输出，无需手动平滑
        self.pan_angle = (self.pan_angle + pan_output) % 360
        self.tilt_angle = max(0, min(60, self.tilt_angle + tilt_output))
        # ---------------------------------------------------
        # 应用平滑过渡
        #self.pan_angle = (self.pan_angle + pan_offset * self.smoothing_factor) % 360
        #self.tilt_angle = max(0, min(60, self.tilt_angle + tilt_offset * self.smoothing_factor))
        
        # 发送新角度
        self.send_angles()

    def video_processing_thread(self):
        """视频处理线程"""
        while self.running:
            if not self.cap:
                time.sleep(0.1)
                continue
                
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.1)
                continue
            
            # YOLO目标检测
            results = self.model(frame, verbose=False)
            annotated_frame = results[0].plot()
            
            # 绘制红色十字准星（考虑摄像头安装偏差）
            center_x = int(self.img_size[0] / 2 + self.camera_offset_x)
            center_y = int(self.img_size[1] / 2 + self.camera_offset_y)
            
            # 绘制十字线
            cross_size = 20  # 十字线大小
            color = (0, 0, 255)  # 红色
            thickness = 2
            
            # 水平线
            cv2.line(annotated_frame, 
                    (center_x - cross_size, center_y), 
                    (center_x + cross_size, center_y), 
                    color, thickness)
            # 垂直线
            cv2.line(annotated_frame, 
                    (center_x, center_y - cross_size), 
                    (center_x, center_y + cross_size), 
                    color, thickness)
            
            # 如果正在跟踪且检测到目标且非手动模式
            if self.tracking and not self.manual_control and len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    if self.model.names[int(box.cls)] == self.track_class:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        center_x = (x1 + x2) // 2
                        center_y = (y1 + y2) // 2
                        
                        # 绘制目标中心点
                        cv2.circle(annotated_frame, (center_x, center_y), 10, (0, 0, 255), -1)
                        
                        # 计算并更新云台角度
                        self.calculate_angles(center_x, center_y)
                        break
            
            # 转换图像格式并放入队列
            img = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
            
            
            # 清空队列并放入最新帧
            while not self.frame_queue.empty():
                try:
                    self.frame_queue.get_nowait()
                except:
                    pass
            self.frame_queue.put(img)

    def update_gui(self):
        """更新GUI显示"""
        if not self.frame_queue.empty():
            img = self.frame_queue.get()
            label_width = self.video_frame.winfo_width()
            label_height = self.video_frame.winfo_height()
            if label_width > 0 and label_height > 0:
                h, w = img.shape[:2]
                ratio = min(label_width/w, label_height/h)
                new_size = (int(w*ratio), int(h*ratio))
                img = cv2.resize(img, new_size)


            photo = tk.PhotoImage(data=cv2.imencode('.png', img)[1].tobytes())
            self.video_frame.config(image=photo)
            self.video_frame.image = photo
        
        if self.running:
            self.root.after(10, self.update_gui)

    def start_tracking(self):
        if not self.manual_control:  # 只有在非手动模式下才能启动跟踪
            self.tracking = True
            self.track_class = self.class_var.get()
            self.status_var.set(f"状态: 正在跟踪 {self.track_class}")
            print(f"开始跟踪: {self.track_class}")

    def stop_tracking(self):
        self.tracking = True
        self.status_var.set("状态: 跟踪已停止")
        print("跟踪已停止")

    def go_home(self):
        self.pan_angle = 0
        self.tilt_angle = 30
        self.send_angles()
        self.status_var.set("状态: 已归零")
        print("云台已归零")

    def __del__(self):
        self.running = False
        if hasattr(self, 'cap') and self.cap:
            self.cap.release()
        if hasattr(self, 'ser') and self.ser:
            self.ser.close()
        if hasattr(self, 'listener') and self.listener:
            self.listener.stop()

if __name__ == "__main__":
    tracker = YOLOTracker()