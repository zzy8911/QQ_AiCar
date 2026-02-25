import sys
import json
import socket
import time
from collections import deque
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
import pyqtgraph as pg

# =================================================================
# 配置常量
# =================================================================
ESP32_CONTROL_PORT = 7749  
PC_RECEIVE_PORT = 7750     
UI_REFRESH_MS = 30  

# =================================================================
# 1. UDP 接收线程
# =================================================================
class UdpReceiver(QThread):
    pid_signal = pyqtSignal(dict) 

    def __init__(self):
        super().__init__()
        self.is_running = True
        self.max_history = 500 
        self.start_time = time.time()
        
        self.time_data = deque(maxlen=self.max_history)
        self.pitch_data = deque(maxlen=self.max_history)
        self.left_speed_data = deque(maxlen=self.max_history)
        self.right_speed_data = deque(maxlen=self.max_history)
        self.latest_payload = {}

    def run(self):
        try:
            self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_sock.bind(('', PC_RECEIVE_PORT))
            self.udp_sock.settimeout(0.5)
        except Exception as e:
            print(f"UDP Bind Error: {e}")
            return

        while self.is_running:
            try:
                data, _ = self.udp_sock.recvfrom(2048)
                msg = json.loads(data.decode())
                cmd = msg.get("cmd")

                if cmd == "telemetry":
                    p = msg.get("payload", {})
                    t = time.time() - self.start_time
                    self.time_data.append(t)
                    self.pitch_data.append(p.get("pitch", 0.0))
                    self.left_speed_data.append(p.get("motor_left_speed", 0.0))
                    self.right_speed_data.append(p.get("motor_right_speed", 0.0))
                    self.latest_payload = p

                elif cmd == "pid":
                    self.pid_signal.emit(msg.get("payload", {}))

            except (socket.timeout, json.JSONDecodeError):
                continue
            except Exception as e:
                if self.is_running: print("Receive Error:", e)

        if hasattr(self, 'udp_sock'):
            self.udp_sock.close()

    def stop(self):
        self.is_running = False
        self.wait()

# =================================================================
# 2. 完整 UI 界面
# =================================================================
class PIDControl(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.udp_receiver = UdpReceiver()
        self.udp_receiver.pid_signal.connect(self.update_pid_fields)
        self.udp_receiver.start()

        self.timer = QTimer()
        self.timer.timeout.connect(self.refresh_gui)
        self.timer.start(UI_REFRESH_MS)

    def init_ui(self):
        self.setWindowTitle("ESP32 PID Tuner - CPU Optimized")
        self.resize(1100, 800)

        main_layout = QtWidgets.QHBoxLayout(self)
        left_panel = QtWidgets.QVBoxLayout()
        right_panel = QtWidgets.QVBoxLayout()

        # --- PID 组建工厂 ---
        def make_pid_group(title):
            box = QtWidgets.QGroupBox(title)
            grid = QtWidgets.QGridLayout()
            p, i, d = QtWidgets.QLineEdit("0.0"), QtWidgets.QLineEdit("0.0"), QtWidgets.QLineEdit("0.0")
            for e in (p, i, d): 
                e.setFixedWidth(80)
                e.returnPressed.connect(self.on_send) # 回车发送
            
            grid.addWidget(QtWidgets.QLabel("P"), 0, 0); grid.addWidget(p, 0, 1)
            grid.addWidget(QtWidgets.QLabel("I"), 1, 0); grid.addWidget(i, 1, 1)
            grid.addWidget(QtWidgets.QLabel("D"), 2, 0); grid.addWidget(d, 2, 1)
            box.setLayout(grid)
            return box, p, i, d

        self.stb_box, self.stb_p, self.stb_i, self.stb_d = make_pid_group("Stability PID (直立)")
        self.vel_box, self.vel_p, self.vel_i, self.vel_d = make_pid_group("Velocity PID (速度)")
        self.steer_box, self.steer_p, self.steer_i, self.steer_d = make_pid_group("Steering PID (转向)")

        left_panel.addWidget(self.stb_box)
        left_panel.addWidget(self.vel_box)
        left_panel.addWidget(self.steer_box)

        # --- 系统与连接设置 ---
        sys_box = QtWidgets.QGroupBox("System & Connection")
        sys_grid = QtWidgets.QGridLayout()
        self.midpoint_edit = QtWidgets.QLineEdit("0.0")
        self.ip_edit = QtWidgets.QLineEdit("192.168.1.6")
        
        self.midpoint_edit.returnPressed.connect(self.on_send)
        self.ip_edit.returnPressed.connect(self.on_send)

        sys_grid.addWidget(QtWidgets.QLabel("Balance Midpoint:"), 0, 0)
        sys_grid.addWidget(self.midpoint_edit, 0, 1)
        sys_grid.addWidget(QtWidgets.QLabel("ESP32 IP:"), 1, 0)
        sys_grid.addWidget(self.ip_edit, 1, 1)
        sys_box.setLayout(sys_grid)
        left_panel.addWidget(sys_box)

        # 按钮样式统一：高度一致，普通字体
        self.btn_send = QtWidgets.QPushButton("Send PID")
        self.btn_get = QtWidgets.QPushButton("Get PID From ESP32")
        self.btn_send.setFixedHeight(30)
        self.btn_get.setFixedHeight(30)
        
        self.btn_send.clicked.connect(self.on_send)
        self.btn_get.clicked.connect(lambda: self.send_raw({"cmd": "get_pid"}))
        
        left_panel.addWidget(self.btn_send)
        left_panel.addWidget(self.btn_get)
        left_panel.addStretch()

        # --- 右侧波形与数值 ---
        self.graph_pitch = pg.PlotWidget(title="Pitch Angle (deg)")
        self.graph_pitch.showGrid(x=True, y=True)
        self.curve_pitch = self.graph_pitch.plot(pen=pg.mkPen('g', width=2))

        self.graph_speed = pg.PlotWidget(title="Wheel Speed")
        self.graph_speed.showGrid(x=True, y=True)
        self.curve_l_speed = self.graph_speed.plot(pen=pg.mkPen('r', width=2), name="Left")
        self.curve_r_speed = self.graph_speed.plot(pen=pg.mkPen('b', width=2), name="Right")

        right_panel.addWidget(self.graph_pitch, 2)
        right_panel.addWidget(self.graph_speed, 2)

        state_box = QtWidgets.QGroupBox("Live Telemetry")
        state_grid = QtWidgets.QGridLayout()
        self.val_pitch = QtWidgets.QLabel("0.0")
        self.val_l_spd = QtWidgets.QLabel("0.0")
        self.val_r_spd = QtWidgets.QLabel("0.0")
        state_grid.addWidget(QtWidgets.QLabel("Pitch:"), 0, 0); state_grid.addWidget(self.val_pitch, 0, 1)
        state_grid.addWidget(QtWidgets.QLabel("L Speed:"), 0, 2); state_grid.addWidget(self.val_l_spd, 0, 3)
        state_grid.addWidget(QtWidgets.QLabel("R Speed:"), 0, 4); state_grid.addWidget(self.val_r_spd, 0, 5)
        state_box.setLayout(state_grid)
        right_panel.addWidget(state_box, 1)

        main_layout.addLayout(left_panel, 1)
        main_layout.addLayout(right_panel, 3)

    def update_pid_fields(self, p):
        def fill(lp, li, ld, src):
            lp.setText(f"{src.get('p', 0):.4f}")
            li.setText(f"{src.get('i', 0):.4f}")
            ld.setText(f"{src.get('d', 0):.4f}")
        fill(self.stb_p, self.stb_i, self.stb_d, p.get("stb", {}))
        fill(self.vel_p, self.vel_i, self.vel_d, p.get("vel", {}))
        fill(self.steer_p, self.steer_i, self.steer_d, p.get("steer", {}))
        self.midpoint_edit.setText(f"{p.get('midpoint', 0):.3f}")

    def refresh_gui(self):
        if not self.udp_receiver.time_data: return
        t = list(self.udp_receiver.time_data)
        self.curve_pitch.setData(t, list(self.udp_receiver.pitch_data))
        self.curve_l_speed.setData(t, list(self.udp_receiver.left_speed_data))
        self.curve_r_speed.setData(t, list(self.udp_receiver.right_speed_data))

        if t:
            self.graph_pitch.setXRange(t[-1] - 5, t[-1], padding=0)
            self.graph_speed.setXRange(t[-1] - 5, t[-1], padding=0)

        lp = self.udp_receiver.latest_payload
        self.val_pitch.setText(f"{lp.get('pitch', 0):.2f}°")
        self.val_l_spd.setText(f"{lp.get('motor_left_speed', 0):.2f}")
        self.val_r_spd.setText(f"{lp.get('motor_right_speed', 0):.2f}")

    def on_send(self):
        try:
            msg = {
                "cmd": "set_pid",
                "payload": {
                    "stb": {"p": float(self.stb_p.text()), "i": float(self.stb_i.text()), "d": float(self.stb_d.text())},
                    "vel": {"p": float(self.vel_p.text()), "i": float(self.vel_i.text()), "d": float(self.vel_d.text())},
                    "steer": {"p": float(self.steer_p.text()), "i": float(self.steer_i.text()), "d": float(self.steer_d.text())},
                    "midpoint": float(self.midpoint_edit.text())
                }
            }
            self.send_raw(msg)
            print(f"[{time.strftime('%H:%M:%S')}] Sent PID.")
        except ValueError:
            print("Invalid input!")

    def send_raw(self, msg):
        ip = self.ip_edit.text()
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(json.dumps(msg).encode(), (ip, ESP32_CONTROL_PORT))
            sock.close()
        except Exception as e:
            print(f"Send Error: {e}")

    def closeEvent(self, e):
        self.udp_receiver.stop()
        e.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = PIDControl()
    win.show()
    sys.exit(app.exec_())
