import sys
import serial
import numpy as np
import pyqtgraph as pg
from PyQt5 import QtWidgets, QtCore

# --- CONFIGURATION ---
COM_PORT = 'COM25'      # CHANGE THIS TO YOUR ACTUAL COM PORT
BAUD_RATE = 2000000
# 5kHz telemetry means 10 samples per 500Hz wave.
# A buffer of 150 holds 15 full sine waves. Fast and zero lag.
BUFFER_SIZE = 150      
INJECTION_FREQ = 500.0 

class SerialReader(QtCore.QThread):
    new_data = QtCore.pyqtSignal(float, float)

    def __init__(self, port, baud):
        super().__init__()
        self.port = port
        self.baud = baud
        self.running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            while self.running:
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if "V_applied:" in line and "I_measured:" in line:
                        try:
                            parts = line.split(',')
                            v_val = float(parts[0].split(':')[1])
                            i_val = float(parts[1].split(':')[1])
                            self.new_data.emit(v_val, i_val)
                        except (ValueError, IndexError):
                            pass 
        except Exception as e:
            print(f"Serial Error: {e}")

    def stop(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()
        self.quit()
        self.wait()

class SPWMAnalyzer(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FOC SPWM Inductance Decoder (Filtered)")
        self.resize(1000, 600)

        self.v_data = np.zeros(BUFFER_SIZE)
        self.i_data = np.zeros(BUFFER_SIZE)
        self.filtered_L = None
        self.filter_alpha = 0.05 # LPF Convergence speed (Lower = smoother, slower)

        self.init_ui()
        
        self.reader = SerialReader(COM_PORT, BAUD_RATE)
        self.reader.new_data.connect(self.update_buffers)
        self.reader.start()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_display)
        self.timer.start(33) 

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        # --- Control Panel ---
        ctrl_layout = QtWidgets.QHBoxLayout()
        
        self.lbl_R = QtWidgets.QLabel("Stator Resistance (R) in Ohms:")
        self.input_R = QtWidgets.QDoubleSpinBox()
        self.input_R.setDecimals(4)
        self.input_R.setSingleStep(0.001)
        self.input_R.setValue(0.0930) 

        self.btn_stop = QtWidgets.QPushButton("Stop / Close")
        self.btn_stop.clicked.connect(self.close)

        ctrl_layout.addWidget(self.lbl_R)
        ctrl_layout.addWidget(self.input_R)
        ctrl_layout.addStretch()
        ctrl_layout.addWidget(self.btn_stop)
        
        # --- Big Math Display ---
        math_layout = QtWidgets.QHBoxLayout()
        self.lbl_Z = QtWidgets.QLabel("Z: 0.000 Ω")
        self.lbl_Z.setStyleSheet("font-size: 20px; font-weight: bold; color: #FFA500;")
        
        # VBox for the L outputs
        l_box = QtWidgets.QVBoxLayout()
        self.lbl_L_converged = QtWidgets.QLabel("L (Converged): Calculating...")
        self.lbl_L_converged.setStyleSheet("font-size: 28px; font-weight: bold; color: #00FF00;")
        
        self.lbl_L_raw = QtWidgets.QLabel("L (Raw Jitter): ---")
        self.lbl_L_raw.setStyleSheet("font-size: 14px; color: #888888;")
        
        l_box.addWidget(self.lbl_L_converged)
        l_box.addWidget(self.lbl_L_raw)

        math_layout.addWidget(self.lbl_Z)
        math_layout.addLayout(l_box)

        # --- Plots ---
        pg.setConfigOptions(antialias=True)
        self.plot_widget = pg.GraphicsLayoutWidget()
        
        self.v_plot = self.plot_widget.addPlot(title="Applied SPWM Voltage (V)")
        self.v_plot.showGrid(x=True, y=True)
        self.v_curve = self.v_plot.plot(pen=pg.mkPen('y', width=2))
        
        self.plot_widget.nextRow()
        
        self.i_plot = self.plot_widget.addPlot(title="Measured Stator Current (A)")
        self.i_plot.showGrid(x=True, y=True)
        self.i_curve = self.i_plot.plot(pen=pg.mkPen('c', width=2))

        layout.addLayout(ctrl_layout)
        layout.addLayout(math_layout)
        layout.addWidget(self.plot_widget)
        self.setLayout(layout)

    def update_buffers(self, v, i):
        self.v_data[:-1] = self.v_data[1:]
        self.v_data[-1] = v
        self.i_data[:-1] = self.i_data[1:]
        self.i_data[-1] = i

    def update_display(self):
        self.v_curve.setData(self.v_data)
        self.i_curve.setData(self.i_data)

        v_amp = (np.max(self.v_data) - np.min(self.v_data)) / 2.0
        i_amp = (np.max(self.i_data) - np.min(self.i_data)) / 2.0

        if i_amp < 0.05:
            self.lbl_Z.setText("Z: waiting for current...")
            self.lbl_L_converged.setText("L: Motor idle")
            return

        Z = v_amp / i_amp
        self.lbl_Z.setText(f"Z: {Z:.4f} Ω")
        R = self.input_R.value()
        
        if Z > R:
            X_L = np.sqrt(Z**2 - R**2)
            L_raw = X_L / (2.0 * np.pi * INJECTION_FREQ)
            
            # Apply the discrete first-order filter
            if self.filtered_L is None:
                self.filtered_L = L_raw
            else:
                self.filtered_L = (1.0 - self.filter_alpha) * self.filtered_L + self.filter_alpha * L_raw

            self.lbl_L_converged.setText(f"L (Converged): {self.filtered_L * 1e6:.2f} µH")
            self.lbl_L_raw.setText(f"L (Raw Jitter): {L_raw * 1e6:.2f} µH")
        else:
            self.lbl_L_converged.setText("L: Math Error (Z < R)")
            self.lbl_L_converged.setStyleSheet("font-size: 28px; font-weight: bold; color: #FF0000;")

    def closeEvent(self, event):
        self.reader.stop()
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = SPWMAnalyzer()
    window.show()
    sys.exit(app.exec_())