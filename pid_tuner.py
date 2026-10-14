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
        self.root.title("STM32 Live SWD PID Tuner (Nucleo-F410RB / pyocd)")

        self.session = None
        self.target = None
        self.is_reading = False
        self.elf_path = ""
        self.swd_lock = threading.Lock()  # Thread lock to prevent SWD bus collisions

        # Symbol memory addresses
        self.addrs = {}

        # Plot data
        self.max_data_points = 100
        self.rpm_data = collections.deque(maxlen=self.max_data_points)
        self.target_data = collections.deque(maxlen=self.max_data_points)

        self.setup_ui()
        self.update_plot()

    def setup_ui(self):
        # --- File Selection Frame ---
        file_frame = ttk.LabelFrame(self.root, text="Project ELF File")
        file_frame.pack(fill="x", padx=10, pady=5)

        self.file_label = ttk.Label(file_frame, text="No .elf file selected", width=50)
        self.file_label.pack(side="left", padx=5, pady=5)

        ttk.Button(file_frame, text="Browse .elf", command=self.select_elf).pack(side="left", padx=5)

        # --- SWD Connection Frame ---
        conn_frame = ttk.LabelFrame(self.root, text="ST-Link SWD Connection")
        conn_frame.pack(fill="x", padx=10, pady=5)

        # Configured for Nucleo-F410RB
        self.target_mcu = ttk.Combobox(
            conn_frame, 
            values=["stm32f410rb", "stm32f410rbtx", "stm32f411re", "stm32f411ceux", "stm32f446re", "cortex_m"]
        )
        self.target_mcu.set("stm32f410rb")
        self.target_mcu.pack(side="left", padx=5, pady=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect SWD", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5, pady=5)

        # --- PID Tuning Controls ---
        control_frame = ttk.LabelFrame(self.root, text="Tuning Controls")
        control_frame.pack(fill="x", padx=10, pady=5)

        # Default sliders to 0.0 for safe startup
        self.kp_val = tk.DoubleVar(value=0.0)
        self.ki_val = tk.DoubleVar(value=0.0)
        self.kd_val = tk.DoubleVar(value=0.0)
        self.target_val = tk.DoubleVar(value=0.0)

        self.create_slider(control_frame, "Target RPM", self.target_val, 0, 300)
        self.create_slider(control_frame, "Kp", self.kp_val, 0, 10)
        self.create_slider(control_frame, "Ki", self.ki_val, 0, 10)
        self.create_slider(control_frame, "Kd", self.kd_val, 0, 10)

        ttk.Button(control_frame, text="Send Parameters", command=self.send_params).pack(pady=10)

        # --- Plot Frame ---
        graph_frame = ttk.Frame(self.root)
        graph_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.set_title("Live RPM vs Target RPM (via SWD)")
        self.ax.set_xlabel("Ticks")
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
        scale = ttk.Scale(frame, from_=from_, to=to_, variable=variable)
        scale.pack(side="left", fill="x", expand=True, padx=5)
        scale.bind("<ButtonRelease-1>", lambda _: self.send_params())
        ttk.Label(frame, textvariable=variable, width=5).pack(side="left")

    def select_elf(self):
        filename = filedialog.askopenfilename(filetypes=[("ELF files", "*.elf")])
        if filename:
            self.elf_path = filename
            self.file_label.config(text=filename.split('/')[-1])

    def toggle_connection(self):
        if self.session and self.session.is_open:
            self.is_reading = False
            self.session.close()
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

                # Ensure target is running (un-halt if attached in halted state)
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
                    if addr is None:
                        if sym in ["g_kp", "g_ki", "g_kd", "g_target_rpm", "g_current_rpm"]:
                            raise ValueError(f"Symbol '{sym}' not found in ELF file. Ensure extern \"C\" is used.")
                    else:
                        self.addrs[sym] = addr

                self.is_reading = True
                self.connect_btn.config(text="Disconnect SWD")
                threading.Thread(target=self.read_swd_loop, daemon=True).start()
            except Exception as e:
                messagebox.showerror("SWD Error", str(e))

    def send_params(self):
        if self.target and self.is_reading:
            try:
                with self.swd_lock:
                    self.target.write32(self.addrs["g_kp"], float_to_u32(self.kp_val.get()))
                    self.target.write32(self.addrs["g_ki"], float_to_u32(self.ki_val.get()))
                    self.target.write32(self.addrs["g_kd"], float_to_u32(self.kd_val.get()))
                    self.target.write32(self.addrs["g_target_rpm"], float_to_u32(self.target_val.get()))
                print(f"[SWD Success] Set Target={self.target_val.get():.1f}, Kp={self.kp_val.get():.1f}, Ki={self.ki_val.get():.1f}, Kd={self.kd_val.get():.1f}")
            except Exception as e:
                messagebox.showerror("Write Error", f"SWD Write Failed: {e}")

    def read_swd_loop(self):
        while self.is_reading:
            try:
                with self.swd_lock:
                    raw_rpm = self.target.read32(self.addrs["g_current_rpm"])
                    raw_target = self.target.read32(self.addrs["g_target_rpm"])

                current_rpm = u32_to_float(raw_rpm)
                target_rpm = u32_to_float(raw_target)

                self.rpm_data.append(current_rpm)
                self.target_data.append(target_rpm)
                time.sleep(0.05)
            except Exception:
                time.sleep(0.05)

    def update_plot(self):
        if len(self.rpm_data) > 0:
            x_data = range(len(self.rpm_data))
            self.line_rpm.set_data(x_data, self.rpm_data)
            self.line_target.set_data(x_data, self.target_data)
            self.ax.set_xlim(0, self.max_data_points)
            
            min_y = min(0, min(self.rpm_data) - 10)
            max_y = max(max(self.target_data), max(self.rpm_data)) + 20
            self.ax.set_ylim(min_y, max_y)
            self.canvas.draw()
        self.root.after(100, self.update_plot)

if __name__ == "__main__":
    root = tk.Tk()
    app = SWDPIDTunerApp(root)
    root.mainloop()
