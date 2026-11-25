import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import threading
import collections
import struct
import time
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from pyocd.core.helpers import ConnectHelper
from pyocd.core.target import Target
from pyocd.debug.elf.elf import ELFBinaryFile
from pyocd.debug.elf.symbols import ELFSymbolProvider
from pyocd.target import TARGET


class PIDTunerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32F446RE Live SWD PID & Position Tuner")

        # Data storage for plotting
        self.max_data_points = 100
        self.angle_data = collections.deque(maxlen=self.max_data_points)
        self.target_data = collections.deque(maxlen=self.max_data_points)

        self.session = None
        self.target = None
        self.sym_addrs = {}
        self.val_labels = {}
        self.is_reading = False
        self.swd_lock = threading.Lock()

        self.setup_ui()
        self.update_plot()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def setup_ui(self):
        # --- SWD Target & ELF Connection Frame ---
        conn_frame = ttk.LabelFrame(self.root, text="SWD Connection (NUCLEO-F446RE / ST-Link)")
        conn_frame.pack(fill="x", padx=10, pady=5)

        # Row 0: Board / MCU Target Type
        row0 = ttk.Frame(conn_frame)
        row0.pack(fill="x", padx=5, pady=3)
        ttk.Label(row0, text="Board / MCU:", width=12).pack(side="left")

        stm32_targets = sorted(
            set([t for t in TARGET.keys() if t.startswith("stm32")] + ["stm32f446re", "stm32f446retx"])
        )

        self.board_var = tk.StringVar(value="stm32f446re")
        self.board_cb = ttk.Combobox(row0, textvariable=self.board_var, values=stm32_targets, width=28)
        self.board_cb.pack(side="left", padx=5)

        self.connect_btn = ttk.Button(row0, text="Connect", command=self.toggle_connection)
        self.connect_btn.pack(side="left", padx=5)

        self.status_label = ttk.Label(row0, text="Disconnected", foreground="red")
        self.status_label.pack(side="left", padx=8)

        # Row 1: ELF File Selector
        row1 = ttk.Frame(conn_frame)
        row1.pack(fill="x", padx=5, pady=3)
        ttk.Label(row1, text="ELF File:", width=12).pack(side="left")

        self.elf_path_var = tk.StringVar()
        ttk.Entry(row1, textvariable=self.elf_path_var).pack(side="left", fill="x", expand=True, padx=5)
        ttk.Button(row1, text="Browse...", command=self.browse_elf).pack(side="left", padx=5)

        # --- PID and Target Controls Frame ---
        control_frame = ttk.LabelFrame(self.root, text="Elbow Position Tuning Controls (AS5600 Raw Counts)")
        control_frame.pack(fill="x", padx=10, pady=5)

        self.kp_val = tk.DoubleVar(value=0.0)
        self.ki_val = tk.DoubleVar(value=0.0)
        self.kd_val = tk.DoubleVar(value=0.0)
        self.target_val = tk.DoubleVar(value=2048.0)

        self.create_slider(control_frame, "target", "Target Pos", self.target_val, 200, 3800)
        self.create_slider(control_frame, "kp", "Kp", self.kp_val, 0, 10)
        self.create_slider(control_frame, "ki", "Ki", self.ki_val, 0, 10)
        self.create_slider(control_frame, "kd", "Kd", self.kd_val, 0, 10)

        ttk.Button(control_frame, text="Send Parameters", command=self.send_params).pack(pady=8)

        # --- Matplotlib Graph Frame ---
        graph_frame = ttk.Frame(self.root)
        graph_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.fig, self.ax = plt.subplots(figsize=(6, 4))
        self.ax.set_title("Live Elbow Position vs Target (AS5600 via SWD)")
        self.ax.set_xlabel("Samples")
        self.ax.set_ylabel("AS5600 Counts (0 - 4095)")
        self.line_angle, = self.ax.plot([], [], label="Actual Position", color="blue")
        self.line_target, = self.ax.plot([], [], label="Target Position", color="red", linestyle="--")
        self.ax.legend()

        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

    def create_slider(self, parent, key, label_text, variable, from_, to_):
        frame = ttk.Frame(parent)
        frame.pack(fill="x", pady=2)
        ttk.Label(frame, text=label_text, width=12).pack(side="left")

        val_label = ttk.Label(frame, text=f"{variable.get():.2f}", width=7)
        self.val_labels[key] = val_label

        def on_slide(val):
            v = round(float(val), 3)
            variable.set(v)
            val_label.config(text=f"{v:.2f}")
            self.send_params()

        ttk.Scale(frame, from_=from_, to=to_, variable=variable, command=on_slide).pack(
            side="left", fill="x", expand=True, padx=5
        )
        val_label.pack(side="left")

    def set_slider_display(self, key, var, value):
        var.set(value)
        if key in self.val_labels:
            self.val_labels[key].config(text=f"{value:.2f}")

    def browse_elf(self):
        path = filedialog.askopenfilename(
            title="Select Compiled Firmware ELF",
            filetypes=[("ELF Files", "*.elf"), ("All Files", "*.*")],
        )
        if path:
            self.elf_path_var.set(path)

    def resolve_symbols(self, elf_path):
        try:
            elf_bin = ELFBinaryFile(elf_path)
            provider = ELFSymbolProvider(elf_bin)
        except Exception:
            provider = ELFSymbolProvider(elf_path)

        symbol_candidates = {
            "kp": ["g_kp", "kp", "Kp"],
            "ki": ["g_ki", "ki", "Ki"],
            "kd": ["g_kd", "kd", "Kd"],
            "target": ["g_target_angle", "target_angle", "target"],
            "angle": ["g_current_angle", "current_angle", "angle"],
        }
        resolved = {}
        missing = []
        for key, names in symbol_candidates.items():
            addr = None
            for name in names:
                try:
                    val = provider.get_symbol_value(name)
                    if val is not None:
                        addr = val
                        break
                except Exception:
                    continue
            if addr is None:
                missing.append(f"{key}: {' / '.join(names)}")
            else:
                resolved[key] = addr
        return resolved, missing

    def read_float(self, addr):
        with self.swd_lock:
            raw = self.target.read32(addr)
        return struct.unpack("<f", struct.pack("<I", raw))[0]

    def write_float(self, addr, value):
        raw = struct.unpack("<I", struct.pack("<f", float(value)))[0]
        with self.swd_lock:
            self.target.write32(addr, raw)

    def toggle_connection(self):
        if self.session is not None:
            self.disconnect()
            return

        elf_path = self.elf_path_var.get().strip()
        board_type = self.board_var.get().strip().lower()

        if not elf_path:
            messagebox.showerror("Missing ELF", "Please select your compiled .elf file first.")
            return
        if not board_type:
            messagebox.showerror("Missing Board", "Please select or type your STM32 board/MCU target.")
            return

        try:
            resolved, missing = self.resolve_symbols(elf_path)
            if missing:
                messagebox.showerror(
                    "Symbols Not Found in ELF",
                    "Could not find these global float variables in the .elf file:\n\n" + "\n".join(missing),
                )
                return

            self.sym_addrs = resolved

            active_target = board_type
            try:
                self.session = ConnectHelper.session_with_chosen_probe(
                    return_first=True,
                    options={
                        "target_override": board_type,
                        "connect_mode": "attach",
                        "frequency": 4000000,
                    },
                )
            except Exception:
                active_target = "cortex_m"
                self.session = ConnectHelper.session_with_chosen_probe(
                    return_first=True,
                    options={
                        "target_override": "cortex_m",
                        "connect_mode": "attach",
                        "frequency": 4000000,
                    },
                )

            if self.session is None:
                raise RuntimeError("No SWD probe detected. Plug in the NUCLEO-F446RE USB cable.")

            self.session.open()
            self.target = self.session.target

            if self.target.get_state() == Target.State.HALTED:
                self.target.resume()

            # Sync initial sliders and readouts with current SRAM values on the F446RE
            self.set_slider_display("kp", self.kp_val, round(self.read_float(self.sym_addrs["kp"]), 3))
            self.set_slider_display("ki", self.ki_val, round(self.read_float(self.sym_addrs["ki"]), 3))
            self.set_slider_display("kd", self.kd_val, round(self.read_float(self.sym_addrs["kd"]), 3))
            self.set_slider_display("target", self.target_val, round(self.read_float(self.sym_addrs["target"]), 2))

            self.is_reading = True
            self.connect_btn.config(text="Disconnect")
            self.status_label.config(text=f"Connected ({active_target})", foreground="green")
            threading.Thread(target=self.read_swd_loop, daemon=True).start()

        except Exception as e:
            self.disconnect()
            messagebox.showerror("SWD Connection Error", str(e))

    def disconnect(self):
        self.is_reading = False
        if self.session is not None:
            try:
                self.session.close()
            except Exception:
                pass
            self.session = None
            self.target = None
        self.connect_btn.config(text="Connect")
        self.status_label.config(text="Disconnected", foreground="red")

    def send_params(self):
        if self.target is not None and self.is_reading:
            try:
                self.write_float(self.sym_addrs["kp"], self.kp_val.get())
                self.write_float(self.sym_addrs["ki"], self.ki_val.get())
                self.write_float(self.sym_addrs["kd"], self.kd_val.get())
                self.write_float(self.sym_addrs["target"], self.target_val.get())
            except Exception as e:
                print(f"SWD write error: {e}")

    def read_swd_loop(self):
        while self.is_reading and self.target is not None:
            try:
                angle = self.read_float(self.sym_addrs["angle"])
                self.angle_data.append(angle)
                self.target_data.append(self.target_val.get())
            except Exception:
                pass
            time.sleep(0.02)  # ~50 Hz live sampling over ST-Link SWD

    def update_plot(self):
        if len(self.angle_data) > 0:
            x_data = range(len(self.angle_data))
            self.line_angle.set_data(x_data, self.angle_data)
            self.line_target.set_data(x_data, self.target_data)
            self.ax.set_xlim(0, self.max_data_points)
            y_min = min(0, min(self.angle_data) - 100, min(self.target_data) - 100)
            y_max = max(4095, max(self.angle_data) + 100, max(self.target_data) + 100)
            self.ax.set_ylim(y_min, y_max)
            self.canvas.draw()

        self.root.after(50, self.update_plot)

    def on_closing(self):
        self.disconnect()
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = PIDTunerApp(root)
    root.mainloop()
