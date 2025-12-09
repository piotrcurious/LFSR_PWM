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
MAX_FFT_LENGTH = 8192 # Length of the sequence for spectral analysis (must be a power of 2 for clean FFT)

# Global variable to store parsed data
parsed_lfsr_config = []

# --- PARSING FUNCTION ---
def parse_c_code(c_code_content):
    """
    Parses the C array content to extract mask, period, ratio, and error.
    
    Expected format: { mask, period }, // Level L: Ratio R, err E, LC C
    """
    global parsed_lfsr_config
    parsed_lfsr_config = []
    
    # Regex to capture all four values
    # Group 1: mask (hex)
    # Group 2: period (octal/decimal)
    # Group 3: Ratio (float)
    # Group 4: Error (e-notation float)
    pattern = re.compile(
        # Note: The regex for Ratio and Error is slightly broad to accommodate various C comment formats.
        r"\{\s*(0x[0-9a-fA-F]+u?),\s*(\d+u?)\s*\},.*?Ratio\s*([0-9.]+),\s*err\s*([0-9eE+-.]+)",
        re.MULTILINE | re.DOTALL
    )
    
    matches = pattern.findall(c_code_content)
    
    if not matches:
        return False

    for mask_str, period_str, ratio_str, error_str in matches:
        try:
            # Convert strings to appropriate types
            mask = int(mask_str.replace('u', ''), 16)
            
            period_clean = period_str.replace('u', '')

            # --- Robust Period Parsing Logic ---
            # This logic handles C-style octal and gracefully falls back to decimal (base 10)
            # if invalid octal digits (8 or 9) are present, which caused the error.
            if period_str.startswith('0x'):
                # Hexadecimal case (e.g., 0x10u)
                period = int(period_clean, 16)
            elif period_str.startswith('0') and len(period_clean) > 1:
                # C-style potential octal notation (e.g., 001u or 028u). Try base 8 first.
                try:
                    period = int(period_clean, 8)
                except ValueError:
                    # If base 8 fails (e.g., contains digits 8 or 9), fall back to decimal.
                    period = int(period_clean, 10)
            else:
                # Standard decimal (e.g., 16u, 255u)
                period = int(period_clean, 10)
            # --- End Robust Period Parsing Logic ---
                
            ratio = float(ratio_str)
            
            # Fix for the previous issue: Strip the comma and whitespace before conversion.
            clean_error_str = error_str.rstrip(',').strip()
            error = float(clean_error_str)
            
            parsed_lfsr_config.append({
                'mask': mask, 
                'period': period, 
                'ratio': ratio,
                'error': error
            })
        except ValueError as e:
            # Log the issue but continue parsing other entries
            print(f"Skipping malformed entry: {mask_str}, {period_str}, {ratio_str}, {error_str}. Error: {e}")
            continue
            
    # The LUT should ideally have 256 entries for levels 0-255
    return len(parsed_lfsr_config) == 256 

# --- SIMULATION FUNCTIONS ---

def generate_lfsr_pwm_signal(level, sequence_len):
    """
    Generates an idealized LFSR PWM signal based on the LUT entry's period and ratio.
    The pulses are randomly distributed within the period to simulate the 
    noise-shaping (dithering) effect in the spectral domain.
    """
    if not (0 <= level <= 255) or not parsed_lfsr_config or level >= len(parsed_lfsr_config):
        return np.array([]), 0.0, 0.0

    entry = parsed_lfsr_config[level]
    period = entry['period']
    D_actual = entry['ratio']
    error = entry['error']
    
    if period <= 0: period = 1
    
    # The number of 'on' steps is the actual ratio multiplied by the period
    on_steps = int(round(D_actual * period))
    
    # LFSR PWM Simulation: Randomly distribute the 'on' steps within the period
    # This randomness is the key difference from Ordinary PWM for spectral analysis.
    indices = np.random.choice(period, size=on_steps, replace=False)
    lfsr_signal = np.zeros(period, dtype=int)
    lfsr_signal[indices] = 1
    
    # Extend the randomized signal to the required sequence length
    num_cycles = int(np.ceil(sequence_len / period))
    signal_full = np.tile(lfsr_signal, num_cycles)[:sequence_len]
    
    return signal_full, D_actual, error

def calculate_spectral_response(signal, sampling_rate=1.0):
    """Calculates the power spectral density (PSD) using FFT."""
    if signal.size == 0:
        return np.array([]), np.array([])

    N = signal.size
    yf = fft(signal)
    
    # Calculate Power Spectral Density (PSD)
    psd = np.abs(yf[:N//2])**2
    
    # Frequency points
    xf = np.linspace(0.0, sampling_rate/2.0, N//2)
    
    # Normalized power
    psd_linear = psd / N**2
    
    return xf, psd_linear

def generate_ordinary_pwm_signal(level, period, sequence_len):
    """
    Generates an ordinary, counter-based (DPWM) signal using the LFSR period.
    This is used as a baseline comparison to show the difference in spectral content.
    """
    if not (0 <= level <= 255) or not parsed_lfsr_config or level >= len(parsed_lfsr_config):
        return np.array([])
    
    # Use the intended duty cycle L/255 for the quantization calculation
    D_ideal = level / 255.0
    
    if period <= 0: period = 1
        
    # Ordinary PWM quantization using the LUT's Period P
    on_steps = int(round(D_ideal * period))
    off_steps = period - on_steps
    
    # Ordinary PWM has a single contiguous pulse
    ordinary_signal = np.concatenate((np.ones(on_steps, dtype=int), np.zeros(off_steps, dtype=int)))
    
    # Extend the signal
    num_cycles = int(np.ceil(sequence_len / period))
    signal_full = np.tile(ordinary_signal, num_cycles)[:sequence_len]
    
    return signal_full

# --- TKINTER GUI CLASS ---
class PWM_Analyzer_App:
    def __init__(self, master):
        self.master = master
        master.title("LFSR PWM Look-Up Table Analyzer")
        master.geometry("1200x800")
        self.setup_parsing_ui()

    def setup_parsing_ui(self):
        """Sets up the initial UI to select the C code file."""
        for widget in self.master.winfo_children():
            widget.destroy()

        parse_frame = ttk.Frame(self.master, padding="20")
        parse_frame.pack(fill=tk.BOTH, expand=True)

        ttk.Label(parse_frame, text="16-bit LFSR PWM Analysis Tool", 
                  font=('Arial', 14, 'bold')).pack(pady=10)
        ttk.Label(parse_frame, text="Select the C source file (.c or .txt) containing the LFSR PWM Look-Up Table:", 
                  font=('Arial', 10)).pack(pady=15)

        # UI: Button to open file dialog
        ttk.Button(parse_frame, text="Browse and Load LUT File", command=self.load_file_and_process).pack(pady=10)

        # Label to show the selected file path
        self.filename_label_var = tk.StringVar(value="No file selected.")
        ttk.Label(parse_frame, textvariable=self.filename_label_var, foreground="blue").pack(pady=5)

    def load_file_and_process(self):
        """Opens a file dialog, reads the selected file, and processes its content."""
        
        # Open file dialog
        file_path = filedialog.askopenfilename(
            defaultextension=".c",
            filetypes=[("C Source Files", "*.c"), ("Text Files", "*.txt"), ("All Files", "*.*")],
            title="Select LFSR PWM Look-Up Table File"
        )
        
        if not file_path:
            self.filename_label_var.set("File selection cancelled.")
            return

        self.filename_label_var.set(f"Selected: {file_path}")

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                c_code_content = f.read()
            
            # Attempt to parse the code
            if parse_c_code(c_code_content) and len(parsed_lfsr_config) == 256:
                self.setup_analyzer_ui()
            else:
                messagebox.showerror("Parsing Error", 
                    f"Could not parse the LUT data or found only {len(parsed_lfsr_config)} entries. "
                    "Please ensure the file contains the full, correct C array (256 entries) in the format: "
                    "`{ mask, period }, // Level L: Ratio R, err E, LC C`")
                    
        except FileNotFoundError:
            messagebox.showerror("File Error", "The selected file was not found.")
        except Exception as e:
            messagebox.showerror("File Read Error", f"An unexpected error occurred while reading the file: {e}")

    def setup_analyzer_ui(self):
        """Sets up the main analysis GUI after successful parsing."""
        for widget in self.master.winfo_children():
            widget.destroy()

        # --- Data Variables ---
        self.level_var = tk.IntVar(value=127) 
        self.lfsr_duty_cycle_var = tk.StringVar(value="N/A")
        self.lfsr_period_var = tk.StringVar(value="N/A")
        self.lfsr_error_var = tk.StringVar(value="N/A")

        # --- Matplotlib Figure Setup ---
        # Create a subplot for Time, Spectrum, and Error
        self.fig, (self.ax_sig, self.ax_spec, self.ax_error) = plt.subplots(3, 1, figsize=(11, 7), 
                                                                           gridspec_kw={'height_ratios': [2, 3, 2]})
        self.fig.tight_layout(pad=3.0)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.master)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.pack(side=tk.TOP, fill=tk.BOTH, expand=1)

        # --- Control and Info Frames ---
        control_frame = ttk.Frame(self.master)
        control_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)

        info_frame = ttk.Frame(self.master)
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=2)
        
        # Slider
        ttk.Label(control_frame, text="PWM Level (0-255):").pack(side=tk.LEFT, padx=5)
        self.level_slider = ttk.Scale(control_frame, from_=0, to=255, 
                                      orient=tk.HORIZONTAL, length=400, 
                                      variable=self.level_var, command=self.update_plots)
        self.level_slider.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        
        # Current Level Display
        self.level_label = ttk.Label(control_frame, textvariable=self.level_var, font=('Arial', 10, 'bold'))
        self.level_label.pack(side=tk.LEFT, padx=5)
        
        # Info Displays
        ttk.Label(info_frame, text="LFSR Period (P):").pack(side=tk.LEFT, padx=5)
        ttk.Label(info_frame, textvariable=self.lfsr_period_var, font=('Arial', 10, 'bold', 'underline')).pack(side=tk.LEFT, padx=10)

        ttk.Label(info_frame, text="| LFSR Duty Cycle (Ratio):").pack(side=tk.LEFT, padx=5)
        ttk.Label(info_frame, textvariable=self.lfsr_duty_cycle_var, font=('Arial', 10, 'bold', 'underline')).pack(side=tk.LEFT, padx=10)
        
        ttk.Label(info_frame, text="| Quantization Error:").pack(side=tk.LEFT, padx=5)
        ttk.Label(info_frame, textvariable=self.lfsr_error_var, font=('Arial', 10, 'bold', 'underline')).pack(side=tk.LEFT, padx=10)
        
        self.update_plots(self.level_var.get())
        
    def update_plots(self, level_val):
        """Updates all plots based on the current PWM Level."""
        level = int(float(level_val))
        
        # --- 1. Signal Generation ---
        np.random.seed(42) # Ensure random distribution is consistent for comparison
        lfsr_signal, D_lfsr, error_lfsr = generate_lfsr_pwm_signal(level, MAX_FFT_LENGTH)
        
        period = parsed_lfsr_config[level]['period']
        ordinary_signal = generate_ordinary_pwm_signal(level, period, MAX_FFT_LENGTH)
        
        # --- 2. Update Info Variables ---
        self.lfsr_period_var.set(f"{period}")
        self.lfsr_duty_cycle_var.set(f"{D_lfsr:.6f}")
        self.lfsr_error_var.set(f"{error_lfsr:.2e}")

        # --- 3. Plot Signal Waveform (Time Domain) ---
        self.ax_sig.clear()
        segment_len = min(4 * period + 20, lfsr_signal.size, 500) # Plot a few cycles or max 500 steps
        
        # Plot LFSR signal
        self.ax_sig.plot(lfsr_signal[:segment_len], drawstyle='steps-post', label='LFSR PWM', color='tab:blue')
        # Plot Ordinary signal (offset for visual separation)
        self.ax_sig.plot(ordinary_signal[:segment_len] + 1.1, drawstyle='steps-post', label='Ordinary PWM', color='tab:red', alpha=0.7)

        self.ax_sig.set_title(f'Waveform at Level {level} (Target Duty: {level/255.0:.4f}, Period: {period})')
        self.ax_sig.set_xlabel('Time Step (k)')
        self.ax_sig.set_ylabel('Amplitude (0/1)')
        self.ax_sig.set_ylim([-0.2, 2.3])
        self.ax_sig.set_yticks([0.0, 1.0, 1.1, 2.1], ['0', '1 (LFSR)', '0 (Ord)', '1 (Ord)'])
        self.ax_sig.grid(axis='y', alpha=0.5)
        self.ax_sig.legend(loc='upper right')

        # --- 4. Plot Spectral Response (Frequency Domain) ---
        self.ax_spec.clear()
        
        xf_lfsr, psd_lfsr = calculate_spectral_response(lfsr_signal)
        xf_ord, psd_ord = calculate_spectral_response(ordinary_signal)
        
        self.ax_spec.plot(xf_lfsr, psd_lfsr, label='LFSR PWM (Noise Shaped)', color='tab:blue')
        self.ax_spec.plot(xf_ord, psd_ord, label='Ordinary PWM (Harmonics)', color='tab:red', alpha=0.7)
        
        self.ax_spec.set_title('Spectral Response (Power Spectral Density - Linear Scale)')
        self.ax_spec.set_xlabel('Normalized Frequency ($f/f_s$)')
        self.ax_spec.set_ylabel('Normalized Power')
        self.ax_spec.grid(True, which="major", ls="-", alpha=0.5)
        self.ax_spec.set_xlim(0, 0.5)
        self.ax_spec.legend(loc='upper right')
        
        # --- 5. Plot Error vs. Ideal Duty Cycle ---
        self.ax_error.clear()
        
        all_levels = np.arange(256)
        all_lfsr_errors = np.array([entry['error'] for entry in parsed_lfsr_config])
        
        # Calculate Ordinary PWM Error based on the specific LFSR period P (best case quantization for P)
        ord_quant_errors = np.array([
            (l / 255.0) - (round((l / 255.0) * entry['period']) / entry['period'])
            for l, entry in enumerate(parsed_lfsr_config)
        ])
        
        # LFSR Error Plot
        self.ax_error.plot(all_levels, all_lfsr_errors * 100, label='LFSR PWM Error', color='tab:blue')
        
        # Ordinary PWM Quantization Error Plot
        self.ax_error.plot(all_levels, ord_quant_errors * 100, label='Ordinary PWM Quantization Error', color='tab:red', alpha=0.7)
        
        # Current Level Marker
        self.ax_error.plot(level, error_lfsr * 100, 'o', color='green', label=f'Current Level {level} Error')
        
        self.ax_error.set_title('Quantization Error vs. Ideal Duty Cycle')
        self.ax_error.set_xlabel('PWM Level (0-255)')
        self.ax_error.set_ylabel('Error (%)')
        self.ax_error.grid(True, which="both", ls="-", alpha=0.5)
        self.ax_error.legend(loc='lower left')
        
        self.fig.tight_layout(pad=3.0)
        self.canvas.draw()

# --- Run the application ---
if __name__ == "__main__":
    root = tk.Tk()
    app = PWM_Analyzer_App(root)
    root.mainloop()
