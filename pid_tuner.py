import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import collections
import struct
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from pyocd.core.helpers import ConnectHelper
from pyocd.debug.elf.symbols import ELFSymbolProvider

def float_to_u32(val):
    return struct.unpack('<I', struct.pack('<f', float(val)))[0]

def u32_to_float(val):
    return struct.unpack('<f', struct.pack('<I', val))[0]

class SWDPIDTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 Live SWD PID Tuner (Nucleo-F410RB)")
        self.root.geometry("800x850")

        self.session = None
        self.target = None
        self.is_reading = False
        self.elf_path = ""
        self.swd_lock = threading.Lock()

        # Symbol memory addresses
        self.addrs = {}

        # Plot data
        self.max_data_points = 100
        self.rpm_data = collections.deque(maxlen=self.max_data_points)
        self.target_data = collections.deque(maxlen=self.max_data_points)
        self.duty_data = collections.deque(maxlen=self.max_data_points)

        # Telemetry variables
        self.latest_rpm = 0.0
        self.latest_target = 0.0
        self.latest_duty = 0.0

        self.setup_ui()
        self.update_plot()

    def setup_ui(self):
        # --- Top Connection Frame ---
        top_frame = ttk.Frame(self.root)
        top_frame.pack(fill="x", padx=10, pady=5)

        # ELF Selection
        file_frame = ttk.LabelFrame(top_frame, text="1. ELF Binary")
        file_frame.pack(side="left", fill="x", expand=True, padx=5)

        self.file_label = ttk.Label(file_frame, text="No .elf loaded", width=25)
        self.file_label.pack(side="left", padx=5, pady=5)
        ttk.Button(file_frame, text="Browse", command=self.select_elf).pack(side="right", padx=5)

        # Target Connection
        conn_frame = ttk.LabelFrame(top_frame, text="2. Target SWD")
        conn_frame.pack(side="right", fill="x", padx=5)

        self.target_mcu = ttk.Combobox(
            conn_frame, 
            values=["stm32f410rb", "stm32f410rbtx", "stm32f411re", "stm32f411ceux", "stm32f446re", "cortex_m"],
            width=14
        )
        self.target_mcu.set("stm32f410rbtx")
        self.target_mcu.pack(side="left", padx=5, pady=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect SWD", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5, pady=5)

        # --- Emergency Stop & Action Bar ---
        action_frame = ttk.Frame(self.root)
        action_frame.pack(fill="x", padx=10, pady=5)

        # Big Bright Red Emergency Stop Button
        self.estop_btn = tk.Button(
            action_frame, 
            text="🚨 EMERGENCY STOP 🚨", 
            bg="#cc0000", 
            fg="white", 
            font=("Helvetica", 12, "bold"),
            activebackground="#ff0000",
            activeforeground="white",
            command=self.emergency_stop
        )
        self.estop_btn.pack(side="left", fill="x", expand=True, padx=5)

        ttk.Button(action_frame, text="Send Parameters", command=self.send_params).pack(side="right", padx=5, ipady=4)
        ttk.Button(action_frame, text="Clear Graph", command=self.clear_plot).pack(side="right", padx=5, ipady=4)

        # --- PID & Target Tuning Controls ---
        control_frame = ttk.LabelFrame(self.root, text="Tuning Controls")
        control_frame.pack(fill="x", padx=10, pady=5)

        self.target_val = tk.DoubleVar(value=0.0)
        self.kp_val = tk.DoubleVar(value=0.0)
        self.ki_val = tk.DoubleVar(value=0.0)
        self.kd_val = tk.DoubleVar(value=0.0)

        self.create_slider_row(control_frame, "Target RPM", self.target_val, 0.0, 300.0, resolution=1.0)
        self.create_slider_row(control_frame, "Kp", self.kp_val, 0.0, 10.0, resolution=0.01)
        self.create_slider_row(control_frame, "Ki", self.ki_val, 0.0, 10.0, resolution=0.01)
        self.create_slider_row(control_frame, "Kd", self.kd_val, 0.0, 10.0, resolution=0.01)

        # --- Real-Time Telemetry Dashboard ---
        telem_frame = ttk.LabelFrame(self.root, text="Live Telemetry Readout")
        telem_frame.pack(fill="x", padx=10, pady=5)

        card1 = ttk.Frame(telem_frame)
        card1.pack(side="left", expand=True, padx=10, pady=5)
        ttk.Label(card1, text="Actual Speed", font=("Helvetica", 9)).pack()
        self.lbl_curr_rpm = ttk.Label(card1, text="0.0 RPM", font=("Helvetica", 14, "bold"), foreground="blue")
        self.lbl_curr_rpm.pack()

        card2 = ttk.Frame(telem_frame)
        card2.pack(side="left", expand=True, padx=10, pady=5)
        ttk.Label(card2, text="Target Speed", font=("Helvetica", 9)).pack()
        self.lbl_targ_rpm = ttk.Label(card2, text="0.0 RPM", font=("Helvetica", 14, "bold"), foreground="red")
        self.lbl_targ_rpm.pack()

        card3 = ttk.Frame(telem_frame)
        card3.pack(side="left", expand=True, padx=10, pady=5)
        ttk.Label(card3, text="Motor Output", font=("Helvetica", 9)).pack()
        self.lbl_duty = ttk.Label(card3, text="0.0 %", font=("Helvetica", 14, "bold"), foreground="green")
        self.lbl_duty.pack()

        # --- Real-Time Plot ---
        graph_frame = ttk.Frame(self.root)
        graph_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.fig, self.ax = plt.subplots(figsize=(6, 3.8))
        self.fig.tight_layout(pad=2.0)
        self.ax.set_title("Live RPM Response (via SWD)")
        self.ax.set_xlabel("Ticks (50ms sample)")
        self.ax.set_ylabel("RPM")
        self.ax.grid(True, linestyle=":", alpha=0.6)

        self.line_rpm, = self.ax.plot([], [], label="Actual RPM", color="blue", linewidth=1.8)
        self.line_target, = self.ax.plot([], [], label="Target RPM", color="red", linestyle="--", linewidth=1.5)
        self.ax.legend(loc="upper left")

        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def create_slider_row(self, parent, label_text, variable, from_, to_, resolution=0.01):
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=3, padx=5)

        ttk.Label(row, text=label_text, width=12, anchor="w", font=("Helvetica", 10, "bold")).pack(side="left")

        scale = ttk.Scale(row, from_=from_, to=to_, variable=variable)
        scale.pack(side="left", fill="x", expand=True, padx=5)
        scale.bind("<ButtonRelease-1>", lambda _: self.send_params())

        # Entry for direct numerical input
        entry = ttk.Entry(row, textvariable=variable, width=8)
        entry.pack(side="left", padx=5)
        entry.bind("<Return>", lambda _: self.send_params())

    def select_elf(self):
        filename = filedialog.askopenfilename(filetypes=[("ELF files", "*.elf")])
        if filename:
            self.elf_path = filename
            self.file_label.config(text=filename.split('/')[-1])

    def toggle_connection(self):
        if self.session and self.session.is_open:
            self.is_reading = False
            self.session.close()
            self.session = None
            self.connect_btn.config(text="Connect SWD")
        else:
            if not self.elf_path:
                messagebox.showerror("Error", "Please select your project .elf file first!")
                return
            try:
                mcu = self.target_mcu.get()
                self.session = ConnectHelper.session_with_chosen_probe(
                    target_override=mcu,
                    options={"elf": self.elf_path}
                )
                self.session.open()
                self.target = self.session.target

                if self.target.is_halted():
                    self.target.resume()

                if hasattr(self.target, 'elf') and self.target.elf:
                    provider = self.target.elf
                else:
                    self.target.elf = self.elf_path
                    provider = ELFSymbolProvider(self.target.elf)

                symbols = ["g_kp", "g_ki", "g_kd", "g_target_rpm", "g_current_rpm", "g_output_duty"]
                for sym in symbols:
                    addr = provider.get_symbol_value(sym)
                    if addr is not None:
                        self.addrs[sym] = addr

                if "g_current_rpm" not in self.addrs:
                    raise ValueError("Crucial symbol 'g_current_rpm' not found in ELF file.")

                self.is_reading = True
                self.connect_btn.config(text="Disconnect SWD")
                threading.Thread(target=self.read_swd_loop, daemon=True).start()
            except Exception as e:
                messagebox.showerror("SWD Connection Error", str(e))

    def emergency_stop(self):
        """Instantly zero out Target RPM and push to MCU immediately."""
        self.target_val.set(0.0)
        if self.target and self.is_reading:
            try:
                with self.swd_lock:
                    if "g_target_rpm" in self.addrs:
                        self.target.write32(self.addrs["g_target_rpm"], float_to_u32(0.0))
                print("[SWD E-STOP EXECUTED] Target RPM reset to 0.0")
            except Exception as e:
                messagebox.showerror("E-Stop Write Failure", str(e))
        self.send_params()

    def send_params(self):
        if self.target and self.is_reading:
            try:
                with self.swd_lock:
                    if "g_kp" in self.addrs:
                        self.target.write32(self.addrs["g_kp"], float_to_u32(self.kp_val.get()))
                    if "g_ki" in self.addrs:
                        self.target.write32(self.addrs["g_ki"], float_to_u32(self.ki_val.get()))
                    if "g_kd" in self.addrs:
                        self.target.write32(self.addrs["g_kd"], float_to_u32(self.kd_val.get()))
                    if "g_target_rpm" in self.addrs:
                        self.target.write32(self.addrs["g_target_rpm"], float_to_u32(self.target_val.get()))
                print(f"[SWD Write OK] Target={self.target_val.get():.1f}, Kp={self.kp_val.get():.2f}, Ki={self.ki_val.get():.2f}, Kd={self.kd_val.get():.2f}")
            except Exception as e:
                messagebox.showerror("Write Error", f"SWD Parameter Update Failed: {e}")

    def clear_plot(self):
        self.rpm_data.clear()
        self.target_data.clear()
        self.duty_data.clear()

    def read_swd_loop(self):
        while self.is_reading:
            try:
                with self.swd_lock:
                    raw_rpm = self.target.read32(self.addrs["g_current_rpm"])
                    raw_target = self.target.read32(self.addrs["g_target_rpm"]) if "g_target_rpm" in self.addrs else 0
                    raw_duty = self.target.read32(self.addrs["g_output_duty"]) if "g_output_duty" in self.addrs else 0

                self.latest_rpm = u32_to_float(raw_rpm)
                self.latest_target = u32_to_float(raw_target)
                self.latest_duty = u32_to_float(raw_duty)

                self.rpm_data.append(self.latest_rpm)
                self.target_data.append(self.latest_target)
                self.duty_data.append(self.latest_duty)

                time.sleep(0.05)
            except Exception:
                time.sleep(0.05)

    def update_plot(self):
        # Update Telemetry Labels
        self.lbl_curr_rpm.config(text=f"{self.latest_rpm:.1f} RPM")
        self.lbl_targ_rpm.config(text=f"{self.latest_target:.1f} RPM")
        self.lbl_duty.config(text=f"{self.latest_duty:.1f} %")

        # Update Waveform
        if len(self.rpm_data) > 0:
            x_data = range(len(self.rpm_data))
            self.line_rpm.set_data(x_data, self.rpm_data)
            self.line_target.set_data(x_data, self.target_data)
            self.ax.set_xlim(0, self.max_data_points)

            min_y = min(0, min(self.rpm_data) - 5)
            max_y = max(max(self.target_data), max(self.rpm_data)) + 15
            self.ax.set_ylim(min_y, max_y)
            self.canvas.draw_idle()

        self.root.after(100, self.update_plot)

if __name__ == "__main__":
    root = tk.Tk()
    app = SWDPIDTunerApp(root)
    root.mainloop()