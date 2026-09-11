import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import threading
import collections
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

class PIDTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Live PID & Speed Tuner")
        
        # Data storage for plotting
        self.max_data_points = 100
        self.rpm_data = collections.deque(maxlen=self.max_data_points)
        self.target_data = collections.deque(maxlen=self.max_data_points)
        
        self.serial_port = None
        self.is_reading = False
        
        self.setup_ui()
        self.update_plot()

    def setup_ui(self):
        # --- Serial Connection Frame ---
        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.pack(fill="x", padx=10, pady=5)
        
        self.port_cb = ttk.Combobox(conn_frame, values=[port.device for port in serial.tools.list_ports.comports()])
        self.port_cb.pack(side="left", padx=5, pady=5)
        
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5, pady=5)

        # --- PID and Target Controls Frame ---
        control_frame = ttk.LabelFrame(self.root, text="Tuning Controls")
        control_frame.pack(fill="x", padx=10, pady=5)

        self.kp_val = tk.DoubleVar(value=2.0)
        self.ki_val = tk.DoubleVar(value=1.0)
        self.kd_val = tk.DoubleVar(value=0.0)
        self.target_val = tk.DoubleVar(value=30.0)

        self.create_slider(control_frame, "Target RPM", self.target_val, 0, 300)
        self.create_slider(control_frame, "Kp", self.kp_val, 0, 10)
        self.create_slider(control_frame, "Ki", self.ki_val, 0, 10)
        self.create_slider(control_frame, "Kd", self.kd_val, 0, 10)

        ttk.Button(control_frame, text="Send Parameters", command=self.send_params).pack(pady=10)

        # --- Matplotlib Graph Frame ---
        graph_frame = ttk.Frame(self.root)
        graph_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.set_title("Live RPM vs Target RPM")
        self.ax.set_xlabel("Time (ticks)")
        self.ax.set_ylabel("RPM")
        self.line_rpm, = self.ax.plot([], [], label="Actual RPM", color="blue")
        self.line_target, = self.ax.plot([], [], label="Target RPM", color="red", linestyle="--")
        self.ax.legend()

        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def create_slider(self, parent, label_text, variable, from_, to_):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", pady=2)
        ttk.Label(frame, text=label_text, width=10).pack(side="left")
        ttk.Scale(frame, from_=from_, to=to_, variable=variable, command=lambda _: self.send_params()).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Label(frame, textvariable=variable, width=5).pack(side="left")

    def toggle_connection(self):
        if self.serial_port and self.serial_port.is_open:
            self.is_reading = False
            self.serial_port.close()
            self.connect_btn.config(text="Connect")
        else:
            try:
                self.serial_port = serial.Serial(self.port_cb.get(), 115200, timeout=1)
                self.is_reading = True
                self.connect_btn.config(text="Disconnect")
                threading.Thread(target=self.read_serial, daemon=True).start()
            except Exception as e:
                print(f"Connection error: {e}")

    def send_params(self):
        if self.serial_port and self.serial_port.is_open:
            # Format: Kp,Ki,Kd,Target\n
            command = f"{self.kp_val.get():.3f},{self.ki_val.get():.3f},{self.kd_val.get():.3f},{self.target_val.get():.2f}\n"
            self.serial_port.write(command.encode('utf-8'))

    def read_serial(self):
        while self.is_reading:
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line.startswith("RPM:"):
                    rpm = float(line.split(":")[1].strip())
                    self.rpm_data.append(rpm)
                    self.target_data.append(self.target_val.get())
            except Exception:
                pass

    def update_plot(self):
        if len(self.rpm_data) > 0:
            x_data = range(len(self.rpm_data))
            self.line_rpm.set_data(x_data, self.rpm_data)
            self.line_target.set_data(x_data, self.target_data)
            self.ax.set_xlim(0, self.max_data_points)
            self.ax.set_ylim(min(0, min(self.rpm_data)-10), max(self.target_data) + 50)
            self.canvas.draw()
        
        self.root.after(100, self.update_plot)

if __name__ == "__main__":
    root = tk.Tk()
    app = PIDTunerApp(root)
    root.mainloop()