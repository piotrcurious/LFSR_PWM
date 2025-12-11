import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from scipy.fft import fft
import re

# --- GLOBAL CONFIGURATION ---
LFSR_WIDTH = 16
PWM_BASE_PERIOD = 255
ERROR_CALC_LENGTH = 512
VISUAL_SEGMENT_CAP = 512
RC_FILTER_ALPHA = 0.1

parsed_lfsr_config = []

def parse_c_code(c_code_content):
    global parsed_lfsr_config
    parsed_lfsr_config = []
    pattern = re.compile(
        r"\{\s*(0x[0-9a-fA-F]+)(?:u)?\s*,\s*(0x[0-9a-fA-F]+)(?:u)?\s*\}\s*,?",
        re.MULTILINE | re.DOTALL
    )
    matches = pattern.findall(c_code_content)
    if not matches:
        return False
    for i, (mask_str, seed_str) in enumerate(matches):
        try:
            mask = int(mask_str, 16)
            seed = int(seed_str, 16)
            parsed_lfsr_config.append({
                'mask': mask,
                'seed': seed,
                'period': PWM_BASE_PERIOD,
            })
        except ValueError as e:
            print(f"Skipping malformed entry at index {i}: {mask_str}, {seed_str}. Error: {e}")
            continue
    return len(parsed_lfsr_config) == 256

def run_lfsr_sequence(mask, seed, sequence_len, width=16):
    state = seed
    threshold = seed
    signal = []
    def popcount_parity(n):
        return bin(n).count('1') % 2
    for _ in range(sequence_len):
        signal.append(1 if state > threshold else 0)
        feedback = popcount_parity(state & mask)
        state = (state >> 1) | (feedback << (width - 1))
    return np.array(signal, dtype=int)

def simulate_rc_integrator(signal, alpha):
    if signal.size == 0:
        return np.array([])
    filtered_signal = np.zeros_like(signal, dtype=float)
    beta = 1.0 - alpha
    current_state = 0.0
    for k in range(signal.size):
        current_state = alpha * signal[k] + beta * current_state
        filtered_signal[k] = current_state
    return filtered_signal

def generate_lfsr_pwm_signal(level, sequence_len):
    if not (0 <= level <= 255) or not parsed_lfsr_config or level >= len(parsed_lfsr_config):
        return np.array([]), 0.0, 0.0
    entry = parsed_lfsr_config[level]
    mask = entry['mask']
    seed = entry['seed']
    lfsr_signal = run_lfsr_sequence(mask, seed, sequence_len)
    calc_ratio = np.mean(lfsr_signal)
    D_ideal = level / 255.0
    calc_error = calc_ratio - D_ideal
    return lfsr_signal, calc_ratio, calc_error

def calculate_spectral_response(signal, sampling_rate=1.0):
    if signal.size == 0:
        return np.array([]), np.array([])
    N = signal.size
    signal_centered = signal - np.mean(signal)
    yf = fft(signal_centered)
    psd = np.abs(yf[:N//2])**2
    xf = np.linspace(0.0, sampling_rate/2.0, N//2)
    psd_linear = psd / N
    return xf, psd_linear

def generate_ordinary_pwm_signal(level, period, sequence_len):
    if not (0 <= level <= 255):
        return np.array([])
    D_ideal = level / 255.0
    if period <= 0: period = 1
    on_steps = int(round(D_ideal * period))
    off_steps = period - on_steps
    ordinary_signal = np.concatenate((np.ones(on_steps, dtype=int), np.zeros(off_steps, dtype=int)))
    num_cycles = int(np.ceil(sequence_len / period))
    signal_full = np.tile(ordinary_signal, num_cycles)[:sequence_len]
    return signal_full

# --- GUI ---
class PWM_Analyzer_App:
    def __init__(self, master):
        self.master = master
        master.title("LFSR PWM Analyzer")
        master.geometry("1200x780")
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.seq_len_exp_var = tk.IntVar(value=13)
        self.setup_parsing_ui()

    def on_closing(self):
        try:
            if hasattr(self, 'fig'):
                plt.close(self.fig)
        except Exception:
            pass
        self.master.destroy()

    def setup_parsing_ui(self):
        for widget in self.master.winfo_children():
            widget.destroy()
        parse_frame = ttk.Frame(self.master, padding=8)
        parse_frame.pack(fill=tk.BOTH, expand=True)
        ttk.Label(parse_frame, text="16-bit LFSR PWM Analysis Tool", font=('Arial', 12, 'bold')).pack(pady=(6,8))
        ttk.Label(parse_frame, text="Select the C header file (.h) containing the LFSR table:", font=('Arial', 9)).pack(pady=(0,6))
        ttk.Button(parse_frame, text="Browse and Load LUT File", command=self.load_file_and_process).pack(pady=6)
        self.filename_label_var = tk.StringVar(value="No file selected.")
        ttk.Label(parse_frame, textvariable=self.filename_label_var, foreground="blue", font=('Arial', 9)).pack(pady=(4,2))

    def load_file_and_process(self):
        file_path = filedialog.askopenfilename(
            defaultextension=".h",
            filetypes=[("C Header Files (LUT)", "*.h"), ("C Source Files", "*.c"), ("All Files", "*.*")],
            title="Select LFSR PWM Look-Up Table File"
        )
        if not file_path:
            self.filename_label_var.set("File selection cancelled.")
            return
        self.filename_label_var.set(f"Selected: {file_path}")
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                c_code_content = f.read()
            if parse_c_code(c_code_content) and len(parsed_lfsr_config) == 256:
                self.setup_analyzer_ui()
            else:
                messagebox.showerror("Parsing Error",
                    f"Could not parse the LUT data or found only {len(parsed_lfsr_config)} entries. "
                    "Ensure the file contains exactly 256 entries in the format: { 0xMASK, 0xSEED },")
        except FileNotFoundError:
            messagebox.showerror("File Error", "The selected file was not found.")
        except Exception as e:
            messagebox.showerror("File Read Error", f"An unexpected error occurred while reading the file: {e}")

    def on_mouse_wheel(self, event):
        current_value = self.level_var.get()
        step = 1
        if event.delta > 0:
            new_value = min(255, current_value + step)
        else:
            new_value = max(0, current_value - step)
        self.level_var.set(new_value)
        self.update_plots(new_value)
        return "break"

    def update_seq_len(self, *args):
        exp = int(round(self.seq_len_exp_var.get()))
        length = 2**exp
        self.seq_len_label_var.set(f"2^{exp} = {length}")
        self.update_plots(self.level_var.get())

    def setup_analyzer_ui(self):
        for widget in self.master.winfo_children():
            widget.destroy()

        # smaller global plot fonts and sizes
        plt.rcParams.update({
            'font.size': 8,
            'axes.titlesize': 9,
            'axes.labelsize': 8,
            'legend.fontsize': 8,
            'xtick.labelsize': 7,
            'ytick.labelsize': 7,
        })

        self.level_var = tk.IntVar(value=127)
        self.lfsr_calc_ratio_var = tk.StringVar(value="N/A")
        self.lfsr_calc_error_var = tk.StringVar(value="N/A")
        self.seq_len_label_var = tk.StringVar()

        main_analysis_frame = ttk.Frame(self.master)
        main_analysis_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=1, padx=6, pady=6)

        # Figure with tight margins
        self.fig, (self.ax_sig, self.ax_spec, self.ax_error) = plt.subplots(
            3, 1, figsize=(10, 6.5),
            gridspec_kw={'height_ratios': [2, 3, 2]}
        )
        # minimize margins visually (subplots_adjust uses fraction of figure)
        self.fig.subplots_adjust(left=0.02, right=0.98, top=0.975, bottom=0.03, hspace=0.18)

        self.canvas = FigureCanvasTkAgg(self.fig, master=main_analysis_frame)
        self.canvas_widget = self.canvas.get_tk_widget()

        main_analysis_frame.grid_columnconfigure(0, weight=4)
        main_analysis_frame.grid_columnconfigure(1, weight=0)
        main_analysis_frame.grid_rowconfigure(0, weight=1)

        self.canvas_widget.grid(row=0, column=0, sticky="nsew", padx=(0,6))

        # Controls: compact padding
        controls_frame = ttk.Frame(main_analysis_frame, padding=4, width=180)
        controls_frame.grid(row=0, column=1, sticky="ne")

        # PWM Level Control (compact)
        ttk.Label(controls_frame, text="PWM Level", anchor='center').pack(pady=(2,2))
        self.level_label = ttk.Label(controls_frame, textvariable=self.level_var, font=('Arial', 10, 'bold'))
        self.level_label.pack(pady=(0,4))

        # shorter vertical slider
        self.level_slider = ttk.Scale(controls_frame, from_=255, to=0,
                                      orient=tk.VERTICAL, length=220,
                                      variable=self.level_var, command=self.update_plots)
        self.level_slider.pack(pady=2, fill=tk.Y)
        self.level_slider.bind('<MouseWheel>', self.on_mouse_wheel)

        ttk.Separator(controls_frame, orient='horizontal').pack(fill='x', pady=6)

        ttk.Label(controls_frame, text="FFT Length (2^N)").pack(pady=(2,2))
        self.seq_len_label = ttk.Label(controls_frame, textvariable=self.seq_len_label_var, font=('Arial', 9))
        self.seq_len_label.pack(pady=(0,4))

        self.seq_len_slider = ttk.Scale(controls_frame, from_=9, to=13,
                                        orient=tk.HORIZONTAL,
                                        variable=self.seq_len_exp_var, command=self.update_seq_len, length=140)
        self.seq_len_slider.pack(pady=2, fill=tk.X)

        self.update_seq_len()

        # Info bar (bottom) — simplified to only show values, less padding
        info_frame = ttk.Frame(self.master, padding=(6,4))
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=(2,6))

        ttk.Label(info_frame, text="Duty:", font=('Arial', 9)).pack(side=tk.LEFT, padx=(4,6))
        ttk.Label(info_frame, textvariable=self.lfsr_calc_ratio_var, foreground='green', font=('Arial', 9, 'bold')).pack(side=tk.LEFT)

        ttk.Label(info_frame, text=" | Err(%):", font=('Arial', 9)).pack(side=tk.LEFT, padx=(8,6))
        ttk.Label(info_frame, textvariable=self.lfsr_calc_error_var, foreground='green', font=('Arial', 9, 'bold')).pack(side=tk.LEFT)

        self.update_plots(self.level_var.get())

    def update_plots(self, level_val):
        level = int(float(level_val))
        self.level_var.set(level)
        sequence_len = 2**int(round(self.seq_len_exp_var.get()))

        lfsr_signal, calc_ratio, calc_error = generate_lfsr_pwm_signal(level, sequence_len)

        if level < len(parsed_lfsr_config):
            period = parsed_lfsr_config[level]['period']
            ordinary_signal = generate_ordinary_pwm_signal(level, period, sequence_len)
        else:
            period = 1
            ordinary_signal = np.array([])

        lfsr_filtered = simulate_rc_integrator(lfsr_signal, RC_FILTER_ALPHA)
        ord_filtered = simulate_rc_integrator(ordinary_signal, RC_FILTER_ALPHA)

        # Update info text
        self.lfsr_calc_ratio_var.set(f"{calc_ratio:.6f}")
        self.lfsr_calc_error_var.set(f"{calc_error*100:.3f}")

        # --- Time Domain ---
        self.ax_sig.clear()
        segment_len = min(4 * period + 20, lfsr_signal.size, VISUAL_SEGMENT_CAP)
        # thinner, simpler labels
        self.ax_sig.plot(lfsr_signal[:segment_len], drawstyle='steps-post',
                         label='LFSR', linewidth=0.6, alpha=0.4)
        self.ax_sig.plot(lfsr_filtered[:segment_len],
                         label='LFSR filt', linewidth=1.0)
        self.ax_sig.plot(ord_filtered[:segment_len],
                         label='Ord filt', linestyle='--', linewidth=0.8)
        self.ax_sig.set_title(f'Filtered Output — Level {level}', pad=6)
        self.ax_sig.set_xlabel('Time (steps)', labelpad=6)
        self.ax_sig.set_ylabel('Voltage', labelpad=6)
        self.ax_sig.set_ylim([-0.1, 1.1])
        self.ax_sig.set_xlim(0, max(1, segment_len))
        self.ax_sig.set_yticks(np.linspace(0, 1, 5))
        self.ax_sig.grid(axis='y', alpha=0.35, linewidth=0.4)
        # compact legend, small font, translucent background off
        self.ax_sig.legend(loc='upper right', fontsize=8, frameon=False, handlelength=1.2)

        # --- Frequency Domain ---
        self.ax_spec.clear()
        xf_lfsr, psd_lfsr = calculate_spectral_response(lfsr_signal)
        xf_ord, psd_ord = calculate_spectral_response(ordinary_signal)
        if xf_lfsr.size:
            self.ax_spec.plot(xf_lfsr, 10*np.log10(psd_lfsr + 1e-15), label='LFSR PSD', linewidth=0.8)
        if xf_ord.size:
            self.ax_spec.plot(xf_ord, 10*np.log10(psd_ord + 1e-15), label='Ord PSD', linewidth=0.8, alpha=0.8)
        self.ax_spec.set_title('Spectral Response (PSD)', pad=6)
        self.ax_spec.set_xlabel('Normalized f / f_s', labelpad=6)
        self.ax_spec.set_ylabel('Power (dB)', labelpad=6)
        self.ax_spec.grid(True, which="major", ls="-", alpha=0.35, linewidth=0.4)
        self.ax_spec.set_xlim(0, 0.5)
        self.ax_spec.legend(loc='upper right', fontsize=8, frameon=False)

        # --- Error plot ---
        self.ax_error.clear()
        all_levels = np.arange(256)
        all_lfsr_quant_errors = np.array([
            generate_lfsr_pwm_signal(l, ERROR_CALC_LENGTH)[2]
            for l in all_levels
        ])
        ord_quant_errors = np.array([
            (l / 255.0) - (round((l / 255.0) * entry['period']) / entry['period'])
            for l, entry in enumerate(parsed_lfsr_config)
        ])
        self.ax_error.plot(all_levels, ord_quant_errors * 100, label='Ord err', linewidth=0.7, alpha=0.7)
        self.ax_error.plot(all_levels, all_lfsr_quant_errors * 100, label='LFSR err', linewidth=0.7)
        current_error_for_marker = generate_lfsr_pwm_signal(level, ERROR_CALC_LENGTH)[2]
        self.ax_error.plot(level, current_error_for_marker * 100, 'o', markersize=6, color='green', label='Current')
        self.ax_error.set_title('Quantization Error (%)', pad=6)
        self.ax_error.set_xlabel('Level', labelpad=6)
        self.ax_error.set_ylabel('Error (%)', labelpad=6)
        self.ax_error.grid(True, which="both", ls="-", alpha=0.28, linewidth=0.35)
        self.ax_error.legend(loc='lower left', fontsize=8, frameon=False)

        # final tight adjust (keeps margins minimal)
        self.fig.subplots_adjust(left=0.02, right=0.98, top=0.975, bottom=0.03, hspace=0.18)
        self.canvas.draw()

if __name__ == "__main__":
    root = tk.Tk()
    app = PWM_Analyzer_App(root)
    root.mainloop()
