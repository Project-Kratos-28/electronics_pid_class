import sys
import serial
import serial.tools.list_ports
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QDoubleSpinBox, QPushButton, QComboBox, QGroupBox, QGridLayout
)
from PyQt6.QtCore import QTimer, pyqtSignal, QThread, Qt

class SerialReaderThread(QThread):
    """Background thread to handle non-blocking UART reads."""
    data_received = pyqtSignal(float, float, float)

    def __init__(self, ser):
        super().__init__()
        self.ser = ser
        self.running = True

    def run(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line.startswith("DATA"):
                    parts = line.split()
                    if len(parts) == 4:
                        rpm = float(parts[1])
                        setpoint = float(parts[2])
                        duty = float(parts[3])
                        self.data_received.emit(rpm, setpoint, duty)
            except Exception:
                pass

    def stop(self):
        self.running = False
        self.wait()

class PIDTunerApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STM32 SpeedMotor PID Tuner")
        self.resize(1000, 700)

        self.ser = None
        self.reader_thread = None

        # Data buffers for live plotting
        self.max_samples = 300
        self.time_data = list(range(self.max_samples))
        self.actual_rpm_data = [0.0] * self.max_samples
        self.target_rpm_data = [0.0] * self.max_samples
        self.duty_data = [0.0] * self.max_samples

        self.init_ui()

    def init_ui(self):
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)

        # Control Panel (Left Side)
        control_panel = QVBoxLayout()
        control_panel.addWidget(self.create_connection_box())
        control_panel.addWidget(self.create_pid_box())
        control_panel.addStretch()

        # Plot Area (Right Side)
        plot_layout = QVBoxLayout()
        
        # RPM Plot
        self.plot_rpm = pg.PlotWidget(title="Speed (RPM)")
        self.plot_rpm.showGrid(x=True, y=True)
        self.plot_rpm.addLegend()
        self.curve_target = self.plot_rpm.plot(pen=pg.mkPen('r', width=2, style=Qt.PenStyle.DashLine), name="Target RPM")
        self.curve_actual = self.plot_rpm.plot(pen=pg.mkPen('g', width=2), name="Actual RPM")
        plot_layout.addWidget(self.plot_rpm)

        # Duty Cycle Plot
        self.plot_duty = pg.PlotWidget(title="PID Output Duty Cycle (%)")
        self.plot_duty.showGrid(x=True, y=True)
        self.plot_duty.setYRange(-105, 105)
        self.curve_duty = self.plot_duty.plot(pen=pg.mkPen('b', width=2), name="Output %")
        plot_layout.addWidget(self.plot_duty)

        main_layout.addLayout(control_panel, 1)
        main_layout.addLayout(plot_layout, 3)

    def create_connection_box(self):
        box = QGroupBox("Serial Connection")
        layout = QGridLayout()

        self.port_combo = QComboBox()
        self.refresh_ports()

        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "57600", "115200", "230400", "921600"])
        self.baud_combo.setCurrentText("115200")

        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connection)

        layout.addWidget(QLabel("Port:"), 0, 0)
        layout.addWidget(self.port_combo, 0, 1)
        layout.addWidget(QLabel("Baud:"), 1, 0)
        layout.addWidget(self.baud_combo, 1, 1)
        layout.addWidget(self.btn_connect, 2, 0, 1, 2)
        box.setLayout(layout)
        return box

    def create_pid_box(self):
        box = QGroupBox("PID & Motor Control")
        layout = QGridLayout()

        self.spin_kp = self.create_spinbox(0.0, 500.0, 2.0, 0.1)
        self.spin_ki = self.create_spinbox(0.0, 500.0, 1.0, 0.1)
        self.spin_kd = self.create_spinbox(0.0, 100.0, 0.0, 0.01)
        self.spin_target = self.create_spinbox(-5000.0, 5000.0, 40.0, 5.0)

        self.btn_send = QPushButton("Update Parameters")
        self.btn_send.clicked.connect(self.send_parameters)

        self.btn_estop = QPushButton("EMERGENCY STOP")
        self.btn_estop.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 14px;")
        self.btn_estop.clicked.connect(self.emergency_stop)

        layout.addWidget(QLabel("Kp:"), 0, 0)
        layout.addWidget(self.spin_kp, 0, 1)
        layout.addWidget(QLabel("Ki:"), 1, 0)
        layout.addWidget(self.spin_ki, 1, 1)
        layout.addWidget(QLabel("Kd:"), 2, 0)
        layout.addWidget(self.spin_kd, 2, 1)
        layout.addWidget(QLabel("Target RPM:"), 3, 0)
        layout.addWidget(self.spin_target, 3, 1)
        layout.addWidget(self.btn_send, 4, 0, 1, 2)
        layout.addWidget(self.btn_estop, 5, 0, 1, 2)
        box.setLayout(layout)
        return box

    def create_spinbox(self, min_v, max_v, default, step):
        sb = QDoubleSpinBox()
        sb.setRange(min_v, max_v)
        sb.setValue(default)
        sb.setSingleStep(step)
        sb.setDecimals(3)
        return sb

    def refresh_ports(self):
        self.port_combo.clear()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo.addItems(ports)

    def toggle_connection(self):
        if self.ser is None or not self.ser.is_open:
            port = self.port_combo.currentText()
            baud = int(self.baud_combo.currentText())
            if not port:
                return
            try:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.reader_thread = SerialReaderThread(self.ser)
                self.reader_thread.data_received.connect(self.update_plots)
                self.reader_thread.start()
                self.btn_connect.setText("Disconnect")
            except Exception as e:
                print(f"Connection failed: {e}")
        else:
            if self.reader_thread:
                self.reader_thread.stop()
            self.ser.close()
            self.ser = None
            self.btn_connect.setText("Connect")

    def send_parameters(self):
        if self.ser and self.ser.is_open:
            kp = self.spin_kp.value()
            ki = self.spin_ki.value()
            kd = self.spin_kd.value()
            target = self.spin_target.value()
            cmd = f"SET {kp:.3f} {ki:.3f} {kd:.3f} {target:.2f}\n"
            self.ser.write(cmd.encode('utf-8'))

    def emergency_stop(self):
        self.spin_target.setValue(0.0)
        self.send_parameters()

    def update_plots(self, actual_rpm, target_rpm, duty):
        self.actual_rpm_data.pop(0)
        self.target_rpm_data.pop(0)
        self.duty_data.pop(0)

        self.actual_rpm_data.append(actual_rpm)
        self.target_rpm_data.append(target_rpm)
        self.duty_data.append(duty)

        self.curve_actual.setData(self.time_data, self.actual_rpm_data)
        self.curve_target.setData(self.time_data, self.target_rpm_data)
        self.curve_duty.setData(self.time_data, self.duty_data)

    def closeEvent(self, event):
        if self.reader_thread:
            self.reader_thread.stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PIDTunerApp()
    window.show()
    sys.exit(app.exec())