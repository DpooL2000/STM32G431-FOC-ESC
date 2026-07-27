import sys
import struct
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import (QApplication, QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QPushButton, QComboBox, QLabel)
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
import serial
import serial.tools.list_ports

# --- SERIAL WORKER THREAD ---
class SerialReader(QThread):
    new_data = pyqtSignal(float, float, float, float, float, float, float)
    
    def __init__(self):
        super().__init__()
        self.port = None
        self.serial_conn = None
        self.running = False
        
    def connect_port(self, port_name):
        try:
            self.serial_conn = serial.Serial(port_name, 2000000, timeout=1)
            self.running = True
            self.start()
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            return False
            
    def disconnect_port(self):
        self.running = False
        self.wait()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            
    def run(self):
        buffer = bytearray()
        while self.running:
            try:
                waiting = self.serial_conn.in_waiting
                if waiting > 0:
                    buffer.extend(self.serial_conn.read(waiting))
                
                # Parse 30-byte structs: <H = uint16, 7x floats
                while len(buffer) >= 30:
                    sync = struct.unpack('<H', buffer[0:2])[0]
                    if sync == 0xABCD:
                        _, iu, iv, iw, ia, ib, l_live, l_min = struct.unpack('<Hfffffff', buffer[0:30])
                        self.new_data.emit(iu, iv, iw, ia, ib, l_live, l_min)
                        buffer = buffer[30:] 
                    else:
                        buffer = buffer[1:] 
            except Exception as e:
                print(f"Read error: {e}")
                break

# --- MAIN GUI ---
class HFIDashboard(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Full Diagnostic HFI Plotter")
        self.resize(1200, 900)
        
        self.is_paused = False
        self.max_points = 1000
        
        # Data Arrays
        self.t_axis = np.arange(self.max_points)
        self.d_iu = np.full(self.max_points, 0.0)
        self.d_iv = np.full(self.max_points, 0.0)
        self.d_iw = np.full(self.max_points, 0.0)
        self.d_ia = np.full(self.max_points, 0.0)
        self.d_ib = np.full(self.max_points, 0.0)
        self.d_l  = np.full(self.max_points, 54.0)
        
        self.init_ui()
        self.reader = SerialReader()
        self.reader.new_data.connect(self.handle_new_data)
        
        # 60fps GUI update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plots)
        self.timer.start(16) 

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)
        
        # --- TOP CONTROL BAR ---
        control_layout = QHBoxLayout()
        self.combo_ports = QComboBox()
        self.refresh_ports()
        
        self.btn_refresh = QPushButton("Refresh Ports")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        self.btn_pause = QPushButton("Pause Plot")
        self.btn_pause.setCheckable(True)
        self.btn_pause.clicked.connect(self.toggle_pause)

        self.lbl_l_min = QLabel("Min Ld: -- µH")
        self.lbl_l_min.setStyleSheet("font-size: 28px; font-weight: bold; color: #ff0055; margin-left: 20px;")
        
        control_layout.addWidget(self.combo_ports)
        control_layout.addWidget(self.btn_refresh)
        control_layout.addWidget(self.btn_connect)
        control_layout.addWidget(self.btn_pause)
        control_layout.addStretch()
        control_layout.addWidget(self.lbl_l_min)
        layout.addLayout(control_layout)

        # --- AVERAGES READOUT BAR ---
        avg_layout = QHBoxLayout()
        avg_style = "font-size: 16px; font-weight: bold; color: black; padding: 5px;"
        
        self.lbl_avg_iu = QLabel("Avg IU: 0.00 A")
        self.lbl_avg_iu.setStyleSheet(avg_style)
        self.lbl_avg_iv = QLabel("Avg IV: 0.00 A")
        self.lbl_avg_iv.setStyleSheet(avg_style)
        self.lbl_avg_iw = QLabel("Avg IW: 0.00 A")
        self.lbl_avg_iw.setStyleSheet(avg_style)
        self.lbl_avg_ia = QLabel("Avg I_Alpha: 0.00 A")
        self.lbl_avg_ia.setStyleSheet(avg_style)
        self.lbl_avg_ib = QLabel("Avg I_Beta: 0.00 A")
        self.lbl_avg_ib.setStyleSheet(avg_style)

        avg_layout.addWidget(self.lbl_avg_iu)
        avg_layout.addWidget(self.lbl_avg_iv)
        avg_layout.addWidget(self.lbl_avg_iw)
        avg_layout.addWidget(self.lbl_avg_ia)
        avg_layout.addWidget(self.lbl_avg_ib)
        avg_layout.addStretch()
        layout.addLayout(avg_layout)
        
        # --- PYQTGRAPH SETUP ---
        pg.setConfigOption('background', '#121212')
        pg.setConfigOption('foreground', '#d3d3d3')
        
        self.p1 = pg.PlotWidget(title="Phase Currents (U, V, W)")
        self.p1.addLegend()
        self.p1.showGrid(x=True, y=True, alpha=0.3)
        self.c_iu = self.p1.plot(pen=pg.mkPen('#ff3333', width=2), name="I_U")
        self.c_iv = self.p1.plot(pen=pg.mkPen('#33ff33', width=2), name="I_V")
        self.c_iw = self.p1.plot(pen=pg.mkPen('#3333ff', width=2), name="I_W")
        layout.addWidget(self.p1)

        self.p2 = pg.PlotWidget(title="Clarke Transform (Alpha, Beta)")
        self.p2.addLegend()
        self.p2.showGrid(x=True, y=True, alpha=0.3)
        self.c_ia = self.p2.plot(pen=pg.mkPen('#00ffcc', width=2), name="I_Alpha")
        self.c_ib = self.p2.plot(pen=pg.mkPen('#cc00ff', width=2), name="I_Beta")
        layout.addWidget(self.p2)

        self.p3 = pg.PlotWidget(title="Live Saliency Wave (µH)")
        self.p3.showGrid(x=True, y=True, alpha=0.3)
        self.c_l = self.p3.plot(pen=pg.mkPen('#ff0055', width=2))
        layout.addWidget(self.p3)

    def refresh_ports(self):
        self.combo_ports.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.combo_ports.addItem(p.device)

    def toggle_connection(self):
        if self.btn_connect.text() == "Connect":
            port = self.combo_ports.currentText()
            if self.reader.connect_port(port):
                self.btn_connect.setText("Disconnect")
                self.combo_ports.setEnabled(False)
        else:
            self.reader.disconnect_port()
            self.btn_connect.setText("Connect")
            self.combo_ports.setEnabled(True)

    def toggle_pause(self):
        self.is_paused = self.btn_pause.isChecked()
        if self.is_paused:
            self.btn_pause.setText("Resume Plot")
            self.btn_pause.setStyleSheet("background-color: #aa0000; color: white;")
        else:
            self.btn_pause.setText("Pause Plot")
            self.btn_pause.setStyleSheet("")
            self.d_iu.fill(self.d_iu[-1]); self.d_iv.fill(self.d_iv[-1]); self.d_iw.fill(self.d_iw[-1])
            self.d_ia.fill(self.d_ia[-1]); self.d_ib.fill(self.d_ib[-1]); self.d_l.fill(self.d_l[-1])

    def handle_new_data(self, iu, iv, iw, ia, ib, l_live, l_min):
        if not self.is_paused:
            self.d_iu[:-1], self.d_iv[:-1], self.d_iw[:-1] = self.d_iu[1:], self.d_iv[1:], self.d_iw[1:]
            self.d_ia[:-1], self.d_ib[:-1], self.d_l[:-1]  = self.d_ia[1:], self.d_ib[1:], self.d_l[1:]
            
            self.d_iu[-1], self.d_iv[-1], self.d_iw[-1] = iu, iv, iw
            self.d_ia[-1], self.d_ib[-1], self.d_l[-1]  = ia, ib, l_live
            
            if l_min > 0:
                self.lbl_l_min.setText(f"Min Ld: {l_min:.2f} µH")

    def update_plots(self):
        if not self.is_paused:
            self.c_iu.setData(self.t_axis, self.d_iu)
            self.c_iv.setData(self.t_axis, self.d_iv)
            self.c_iw.setData(self.t_axis, self.d_iw)
            self.c_ia.setData(self.t_axis, self.d_ia)
            self.c_ib.setData(self.t_axis, self.d_ib)
            self.c_l.setData(self.t_axis, self.d_l)
            
            # Update Averages
            self.lbl_avg_iu.setText(f"Avg IU: {np.mean(self.d_iu):.3f} A")
            self.lbl_avg_iv.setText(f"Avg IV: {np.mean(self.d_iv):.3f} A")
            self.lbl_avg_iw.setText(f"Avg IW: {np.mean(self.d_iw):.3f} A")
            self.lbl_avg_ia.setText(f"Avg I_Alpha: {np.mean(self.d_ia):.3f} A")
            self.lbl_avg_ib.setText(f"Avg I_Beta: {np.mean(self.d_ib):.3f} A")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = HFIDashboard()
    window.show()
    sys.exit(app.exec_())