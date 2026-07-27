import sys
import serial
import serial.tools.list_ports
import struct
import math
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore
from collections import deque

# Fallback port if auto-detect fails
FALLBACK_PORT = 'COM25'   
BAUD_RATE = 2000000 
BUFFER_SIZE = 200   # 0.2 seconds of history

class TelemetryReader(QtCore.QThread):
    def __init__(self, port, baud, gui_ref):
        super().__init__()
        self.port = port
        self.baud = baud
        self.gui = gui_ref
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.ser.reset_input_buffer()
            
            sync_word = b'\xaa\xcc\xbb\xdd'
            
            # Format: 2 uint32, 10 floats, 1 uint8 = 4+4+40+1 = 49 bytes
            struct_fmt = '<IIffffffffffB' 
            struct_size = struct.calcsize(struct_fmt) 
            
            buffer = bytearray()

            while self.running:
                # 1. READ IN CHUNKS
                if self.ser.in_waiting:
                    buffer.extend(self.ser.read(self.ser.in_waiting))

                # 2. PROCESS ALL COMPLETE FRAMES IN MEMORY
                while len(buffer) >= struct_size:
                    # Find the sync word instantly in RAM
                    sync_idx = buffer.find(sync_word)

                    if sync_idx == -1:
                        buffer = buffer[-3:]
                        break
                    
                    if sync_idx + struct_size <= len(buffer):
                        frame = buffer[sync_idx : sync_idx + struct_size]
                        data = struct.unpack(struct_fmt, frame)
                        
                        est_ang = data[10]
                        ol_ang = data[11]
                        
                        # Calculate Phase Lag (OL - Est) wrapped to [-pi, pi]
                        lag_rad = (ol_ang - est_ang + math.pi) % (2 * math.pi) - math.pi
                        
                        # Dump straight to GUI deques
                        self.gui.cyc_data.append(data[1])
                        self.gui.ia_data.append(data[2])
                        self.gui.ib_data.append(data[3])
                        
                        self.gui.id_meas_data.append(data[4])
                        self.gui.iq_meas_data.append(data[5])
                        
                        self.gui.id_targ_data.append(data[6])
                        self.gui.iq_targ_data.append(data[7])
                        
                        self.gui.vd_data.append(data[8])
                        self.gui.vq_data.append(data[9])
                        
                        self.gui.est_ang_data.append(est_ang)
                        self.gui.ol_ang_data.append(ol_ang)
                        self.gui.lag_data.append(lag_rad)
                        self.gui.current_state = data[12]
                        
                        # Delete the processed frame from the buffer EXACTLY as you had it
                        del buffer[:sync_idx + struct_size]
                    else:
                        del buffer[:sync_idx]
                        break

        except Exception as e:
            print(f"Serial Error: {e}")
            self.gui.lbl_info.setText(f"Serial Error: {e}")

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.quit()
        self.wait()

class FOCAnalyzer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOC Master Telemetry (Ultimate Diagnostics)")
        self.resize(1200, 1000) 

        # Thread-safe, fixed-length buffers
        self.cyc_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        
        self.ia_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.ib_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        
        self.id_meas_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.iq_meas_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.id_targ_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.iq_targ_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        
        self.vd_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.vq_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        
        self.est_ang_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.ol_ang_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        self.lag_data = deque([0.0]*BUFFER_SIZE, maxlen=BUFFER_SIZE)
        
        self.current_state = 0
        self.states = ["OFF", "ALIGN", "SPOOL", "BLEND", "FOC"]
        
        self.reader = None
        self.is_paused = False

        self.init_ui()

        # GUI strictly updates at 30 FPS
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(33) 

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        # --- Top Control Bar ---
        control_layout = QtWidgets.QHBoxLayout()
        
        self.port_combo = QtWidgets.QComboBox()
        ports = serial.tools.list_ports.comports()
        for p in ports:
            self.port_combo.addItem(p.device)
        if self.port_combo.count() == 0:
            self.port_combo.addItem(FALLBACK_PORT)
            
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)
        
        self.btn_pause = QtWidgets.QPushButton("Pause Plot")
        self.btn_pause.clicked.connect(self.toggle_pause)
        
        control_layout.addWidget(QtWidgets.QLabel("COM Port:"))
        control_layout.addWidget(self.port_combo)
        control_layout.addWidget(self.btn_connect)
        control_layout.addWidget(self.btn_pause)
        control_layout.addStretch()

        layout.addLayout(control_layout)

        # --- Info Labels ---
        self.lbl_info = QtWidgets.QLabel("State: OFF | Loop Overhead: --- CPU Cycles | Loop Time: --- µs")
        self.lbl_info.setStyleSheet("font-size: 16px; font-weight: bold; color: green;")
        layout.addWidget(self.lbl_info)

        self.lbl_lag = QtWidgets.QLabel("Live Phase Lag: 0.00°")
        self.lbl_lag.setStyleSheet("font-size: 16px; font-weight: bold; color: red;")
        layout.addWidget(self.lbl_lag)

        # --- Plots ---
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.GraphicsLayoutWidget()
        
        # 1. Angle Tracking (Y-Range Locked to standard circle)
        self.ang_plot = self.plot_widget.addPlot(title="Tracking: Est Angle (Green) vs OL Angle (Yellow)")
        self.ang_plot.showGrid(x=True, y=True)
        self.ang_plot.setYRange(-1, 7)
        self.ol_curve = self.ang_plot.plot(pen=pg.mkPen('y', width=2))
        self.est_curve = self.ang_plot.plot(pen=pg.mkPen('g', width=2, style=QtCore.Qt.DashLine))
        self.plot_widget.nextRow()
        
        # 2. Phase Lag (Y-Range Locked to +/- Pi)
        self.lag_plot = self.plot_widget.addPlot(title="Phase Lag Error (Rad)")
        self.lag_plot.showGrid(x=True, y=True)
        self.lag_plot.setYRange(-math.pi, math.pi)
        self.lag_curve = self.lag_plot.plot(pen=pg.mkPen('r', width=2))
        self.plot_widget.nextRow()

        # 3. Alpha / Beta Currents (AUTO-SCALING - Y-axis UNLOCKED)
        self.ab_plot = self.plot_widget.addPlot(title="Clarke Currents: I_Alpha (Cyan) vs I_Beta (Magenta)")
        self.ab_plot.showGrid(x=True, y=True)
        self.ia_curve = self.ab_plot.plot(pen=pg.mkPen('c', width=2))
        self.ib_curve = self.ab_plot.plot(pen=pg.mkPen('m', width=2))
        self.plot_widget.nextRow()
        
        # 4. D-Axis & Q-Axis Currents (AUTO-SCALING - Y-axis UNLOCKED)
        self.dq_plot = self.plot_widget.addPlot(title="Park Currents: I_d (Yellow - Reactive) vs I_q (Magenta - Torque)")
        self.dq_plot.showGrid(x=True, y=True)
        self.id_meas_curve = self.dq_plot.plot(pen=pg.mkPen('y', width=2))
        self.iq_meas_curve = self.dq_plot.plot(pen=pg.mkPen('m', width=2))
        self.plot_widget.nextRow()

        # 5. Voltage Output (AUTO-SCALING - Y-axis UNLOCKED)
        self.v_plot = self.plot_widget.addPlot(title="Active Voltages: V_D (Cyan) vs V_Q (Magenta)")
        self.v_plot.showGrid(x=True, y=True)
        self.vd_curve = self.v_plot.plot(pen=pg.mkPen('c', width=2))
        self.vq_curve = self.v_plot.plot(pen=pg.mkPen('m', width=2))

        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    def toggle_connection(self):
        if self.reader is None or not self.reader.running:
            port = self.port_combo.currentText()
            self.reader = TelemetryReader(port, BAUD_RATE, self)
            self.reader.start()
            self.btn_connect.setText("Disconnect")
            self.lbl_info.setText("Connecting...")
        else:
            self.reader.stop()
            self.reader = None
            self.btn_connect.setText("Connect")
            self.lbl_info.setText("Disconnected.")

    def toggle_pause(self):
        self.is_paused = not self.is_paused
        if self.is_paused:
            self.btn_pause.setText("Resume Plot")
        else:
            self.btn_pause.setText("Pause Plot")

    def update_display(self):
        if self.is_paused or self.reader is None or not self.reader.running:
            return

        # Convert deques to lists instantly for plotting
        self.ol_curve.setData(list(self.ol_ang_data))
        self.est_curve.setData(list(self.est_ang_data))
        self.lag_curve.setData(list(self.lag_data))
        
        self.ia_curve.setData(list(self.ia_data))
        self.ib_curve.setData(list(self.ib_data))

        self.id_meas_curve.setData(list(self.id_meas_data))
        self.iq_meas_curve.setData(list(self.iq_meas_data))
        
        self.vd_curve.setData(list(self.vd_data))
        self.vq_curve.setData(list(self.vq_data))

        # Update Top Labels
        lag_list = list(self.lag_data)
        current_lag = sum(lag_list) / len(lag_list) if len(lag_list) > 0 else 0.0
        self.lbl_lag.setText(f"Avg Phase Lag: {current_lag:.2f} rad  ({math.degrees(current_lag):.1f} deg)")

        # Calculate average cycles
        cyc_list = list(self.cyc_data)
        if len(cyc_list) > 0:
            avg_cycles = sum(cyc_list) / len(cyc_list)
        else:
            avg_cycles = 0

        cpu_time_us = (avg_cycles / 170.0) 
        
        state_str = self.states[self.current_state] if self.current_state < 5 else "UNKNOWN"
        self.lbl_info.setText(f"State: {state_str} | Loop Overhead: {int(avg_cycles)} Cycles | Computation Time: {cpu_time_us:.2f} µs (Max budget is 50 µs)")

    def closeEvent(self, event):
        if self.reader:
            self.reader.stop()
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = FOCAnalyzer()
    window.show()
    sys.exit(app.exec_())