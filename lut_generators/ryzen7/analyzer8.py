import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from scipy.fft import fft
import re

# --- GLOBAL CONFIGURATION ---
LFSR_WIDTH = 16
# Using 255 as the default period, as the per-level period is no longer in the LUT array
PWM_BASE_PERIOD = 255 
MAX_FFT_LENGTH = 8192 # Length of the sequence for spectral analysis (must be a power of 2 for clean FFT)

# RC Filter Time Constant (Alpha) for the simulated integrator
# y[k] = alpha * u[k] + (1 - alpha) * y[k-1]. 
# A smaller alpha means a longer time constant (more filtering).
RC_FILTER_ALPHA = 0.01 

# Global variable to store parsed data
parsed_lfsr_config = []

# --- PARSING FUNCTION (Unchanged) ---
def parse_c_code(c_code_content):
    """
    Parses the C array content to extract mask, seed, ratio, and error.
    
    Expected format: { mask, seed }, // Level L Ratio R, err E, LC C
    """
    global parsed_lfsr_config
    parsed_lfsr_config = []
    
    # Regex to capture:
    # Group 1: mask (hex)
    # Group 2: seed (hex)
    # Group 3: Ratio (float)
    # Group 4: Error (e-notation float)
    pattern = re.compile(
        # Look for two hex values in the array: { 0x...u, 0x...u }
        r"\{\s*(0x[0-9a-fA-F]+u?),\s*(0x[0-9a-fA-F]+u?)\s*\},.*?Ratio\s*([0-9.]+),\s*err\s*([0-9eE+-.]+)",
        re.MULTILINE | re.DOTALL
    )
    
    matches = pattern.findall(c_code_content)
    
    if not matches:
        return False

    for mask_str, seed_str, ratio_str, error_str in matches:
        try:
            # Convert strings to appropriate types
            mask = int(mask_str.replace('u', ''), 16)
            seed = int(seed_str.replace('u', ''), 16)
            
            # Since the period is not in the file, we default to the base PWM period (255)
            period = PWM_BASE_PERIOD 

            ratio = float(ratio_str)
            
            # Clean up and convert error string
            clean_error_str = error_str.rstrip(',').strip()
            error = float(clean_error_str)
            
            parsed_lfsr_config.append({
                'mask': mask, 
                'seed': seed,
                'period': period, # Using the global base period for simulation
                'ratio': ratio,
                'error': error
            })
        except ValueError as e:
            # Log the issue but continue parsing other entries
            print(f"Skipping malformed entry: {mask_str}, {seed_str}, {ratio_str}, {error_str}. Error: {e}")
            continue
            
    # The LUT should ideally have 256 entries for levels 0-255
    return len(parsed_lfsr_config) == 256 

# --- LFSR CORE FUNCTION (Unchanged) ---

def run_lfsr_sequence(mask, seed, level, sequence_len, width=16):
    """
    Runs a deterministic 16-bit Fibonacci LFSR, comparing the MSB of its state 
    against the PWM 'level' (0-255) to generate the deterministic PWM signal.
    
    This simulates the core noise-shaping behavior of LFSR PWM.
    """
    # LFSR state must not be zero to generate a sequence
    state = seed if seed != 0 else 1 
    signal = []
    
    # The target PWM level (0-255) is compared against the MSB 8 bits of the LFSR state.
    
    for _ in range(sequence_len):
        
        # 1. Determine PWM output (Noise-Shaped PWM based on LFSR state comparison)
        # Compare the most significant 8 bits of the LFSR state against the level
        # If the LFSR state is 'low' (i.e., <= level), output high (1).
        lfsr_comparison_value = (state >> (width - 8)) 
        
        if lfsr_comparison_value <= level:
            signal.append(1)
        else:
            signal.append(0)
            
        # 2. Calculate LFSR next state (Fibonacci/Standard LFSR shift and XOR)
        
        # Get the LSB (used as the feedback signal)
        lfsr_out = state & 1
        
        # Shift the state right
        state >>= 1
        
        # Apply XOR feedback using the mask if the LSB was 1
        if lfsr_out:
            state ^= mask
            
        # Ensure state stays within the 16-bit range
        state &= ((1 << width) - 1)
        
        # Prevent zero state lock (should only happen for all-zero start)
        if state == 0:
            state = 1
            
    return np.array(signal, dtype=int)

# --- NEW: RC INTEGRATOR SIMULATION ---

def simulate_rc_integrator(signal, alpha):
    """
    Simulates a first-order RC low-pass filter (integrator) using 
    y[k] = alpha * u[k] + (1 - alpha) * y[k-1].
    
    Args:
        signal (np.array): The digital PWM input signal (0s and 1s).
        alpha (float): The filter constant (0 < alpha < 1).
    
    Returns:
        np.array: The analog filtered output signal (0.0 to 1.0).
    """
    if signal.size == 0:
        return np.array([])
    
    # Initialize output array
    filtered_signal = np.zeros_like(signal, dtype=float)
    
    # Initial state (set to the DC value of the input for a stable start)
    # Using the first value is also fine, but setting to the mean is slightly more stable
    # filtered_signal[0] = signal[0] * alpha 
    
    # Apply the difference equation
    beta = 1.0 - alpha
    current_state = 0.0 # Start from 0, or average duty cycle for faster convergence
    
    for k in range(signal.size):
        current_state = alpha * signal[k] + beta * current_state
        filtered_signal[k] = current_state
        
    return filtered_signal

# --- SIMULATION FUNCTIONS (Modified to use deterministic LFSR) ---

def generate_lfsr_pwm_signal(level, sequence_len):
    """
    Generates a deterministic LFSR PWM signal using the mask and seed from the LUT,
    and returns the parsed ratio and error for comparison.
    """
    if not (0 <= level <= 255) or not parsed_lfsr_config or level >= len(parsed_lfsr_config):
        return np.array([]), 0.0, 0.0

    entry = parsed_lfsr_config[level]
    mask = entry['mask']
    seed = entry['seed']
    period = entry['period'] # Base period (255)
    
    # Display values from LUT (Ratio and Error)
    D_lfsr_parsed = entry['ratio']
    error_lfsr_parsed = entry['error']

    # --- Use the deterministic LFSR sequence generator ---
    lfsr_signal = run_lfsr_sequence(mask, seed, level, sequence_len, LFSR_WIDTH)
    
    return lfsr_signal, D_lfsr_parsed, error_lfsr_parsed

def calculate_spectral_response(signal, sampling_rate=1.0):
    """Calculates the power spectral density (PSD) using FFT."""
    if signal.size == 0:
        return np.array([]), np.array([])

    N = signal.size
    # We must subtract the mean (DC component) before FFT to properly see the noise floor/harmonics
    # The spectral response of the raw signal is the most telling for noise shaping analysis
    signal_centered = signal - np.mean(signal) 
    
    yf = fft(signal_centered)
    
    # Calculate Power Spectral Density (PSD)
    # Note: We take the squared magnitude of the positive frequency components
    psd = np.abs(yf[:N//2])**2
    
    # Frequency points
    xf = np.linspace(0.0, sampling_rate/2.0, N//2)
    
    # Normalized power
    psd_linear = psd / N # Simple power normalization, not true PSD in V^2/Hz, but for comparison it works
    
    return xf, psd_linear

def generate_ordinary_pwm_signal(level, period, sequence_len):
    """
    Generates an ordinary, counter-based (DPWM) signal using the fixed period.
    This is used as a baseline comparison.
    """
    if not (0 <= level <= 255) or not parsed_lfsr_config or level >= len(parsed_lfsr_config):
        return np.array([])
    
    # Use the intended duty cycle L/255 for the quantization calculation
    D_ideal = level / 255.0
    
    if period <= 0: period = 1
        
    # Ordinary PWM quantization using the LUT's fixed Period P
    on_steps = int(round(D_ideal * period))
    off_steps = period - on_steps
    
    # Ordinary PWM has a single contiguous pulse
    ordinary_signal = np.concatenate((np.ones(on_steps, dtype=int), np.zeros(off_steps, dtype=int)))
    
    # Extend the signal
    num_cycles = int(np.ceil(sequence_len / period))
    signal_full = np.tile(ordinary_signal, num_cycles)[:sequence_len]
    
    return signal_full

# --- TKINTER GUI CLASS (Modified) ---
class PWM_Analyzer_App:
    def __init__(self, master):
        self.master = master
        master.title("LFSR PWM Look-Up Table Analyzer")
        master.geometry("1200x800")
        
        # Bind the cleanup function to the window close protocol
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        self.setup_parsing_ui()

    def on_closing(self):
        """Handle cleanup when the main window is closed to ensure a clean exit."""
        try:
            # Explicitly close the Matplotlib figure to release resources
            if hasattr(self, 'fig'):
                plt.close(self.fig)
        except Exception:
            # Ignore errors if the figure was already destroyed or not initialized
            pass
        
        # Destroy the Tkinter window
        self.master.destroy()

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
        ttk.Label(parse_frame, text="NOTE: This version uses the parsed 'seed' and 'mask' to generate a deterministic LFSR sequence and simulates an RC integrator filter.", 
                  font=('Arial', 10, 'italic'), foreground='blue').pack(pady=5)


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
                    "`{ mask, seed }, // Level L Ratio R, err E, LC C`")
                    
        except FileNotFoundError:
            messagebox.showerror("File Error", "The selected file was not found.")
        except Exception as e:
            messagebox.showerror("File Read Error", f"An unexpected error occurred while reading the file: {e}")

    def on_mouse_wheel(self, event):
        """Adjusts the vertical slider position based on mouse wheel movement, step-by-step."""
        current_value = self.level_var.get()
        step = 1 # Force a 1-unit step for precise level selection (0-255 granularity)
        
        # Determine direction based on OS/Tkinter delta
        if event.delta > 0:
            # Positive delta (scroll up) means increase level (lower position on inverted scale)
            new_value = min(255, current_value + step)
        else:
            # Negative delta (scroll down) means decrease level (higher position on inverted scale)
            new_value = max(0, current_value - step)
            
        self.level_var.set(new_value)
        # Call update_plots with the new value
        self.update_plots(new_value) 
        
        # Prevents default scrolling behavior if inside a larger scrollable area
        return "break"

    def setup_analyzer_ui(self):
        """Sets up the main analysis GUI after successful parsing."""
        for widget in self.master.winfo_children():
            widget.destroy()

        # --- Data Variables ---
        self.level_var = tk.IntVar(value=127) 
        self.lfsr_duty_cycle_var = tk.StringVar(value="N/A")
        self.lfsr_period_var = tk.StringVar(value="N/A")
        self.lfsr_error_var = tk.StringVar(value="N/A")
        
        # Main Layout Frame (Holds plots and slider side-by-side)
        main_analysis_frame = ttk.Frame(self.master)
        # Managed by PACK relative to the root window (self.master)
        main_analysis_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=1, padx=10, pady=10) 

        # --- Matplotlib Figure Setup ---
        # Create a subplot for Time, Spectrum, and Error
        self.fig, (self.ax_sig, self.ax_spec, self.ax_error) = plt.subplots(3, 1, figsize=(10, 7), 
                                                                           gridspec_kw={'height_ratios': [2, 3, 2]})
        self.fig.tight_layout(pad=3.0)
        
        # CRITICAL FIX: Make main_analysis_frame the master/parent of the canvas widget.
        self.canvas = FigureCanvasTkAgg(self.fig, master=main_analysis_frame) 
        self.canvas_widget = self.canvas.get_tk_widget()
        
        # --- Use GRID for plots and slider for robust layout within main_analysis_frame ---
        main_analysis_frame.grid_columnconfigure(0, weight=4) # Plots column, takes most space
        main_analysis_frame.grid_columnconfigure(1, weight=1) # Slider column, fixed width
        main_analysis_frame.grid_rowconfigure(0, weight=1)

        # Plots (Column 0, child of main_analysis_frame, uses GRID)
        self.canvas_widget.grid(row=0, column=0, sticky="nsew", padx=(0, 10))

        # Right side: Vertical Slider and Info (Column 1, child of main_analysis_frame, uses GRID)
        slider_frame = ttk.Frame(main_analysis_frame, padding="10")
        slider_frame.grid(row=0, column=1, sticky="ns") # Stick vertically
        
        # Slider Label (child of slider_frame, uses PACK)
        ttk.Label(slider_frame, text="PWM Level (0-255):", anchor='center').pack(pady=(0, 5))
        
        # Current Level Display (child of slider_frame, uses PACK)
        # Display the integer value of the variable
        self.level_label = ttk.Label(slider_frame, textvariable=self.level_var, font=('Arial', 12, 'bold'))
        self.level_label.pack(pady=5)
        
        # Vertical Slider (child of slider_frame, uses PACK)
        # Note: The command passes a float, which we handle in update_plots
        self.level_slider = ttk.Scale(slider_frame, from_=255, to=0, 
                                            orient=tk.VERTICAL, length=400, 
                                            variable=self.level_var, command=self.update_plots)
        self.level_slider.pack(pady=10, fill=tk.Y, expand=True)

        # Bind mouse wheel to the slider for incremental changes
        self.level_slider.bind('<MouseWheel>', self.on_mouse_wheel)

        # --- Info Displays (At the bottom of the master window, uses PACK) ---
        info_frame = ttk.Frame(self.master)
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=5)
        
        ttk.Label(info_frame, text="LFSR Period (P):").pack(side=tk.LEFT, padx=5)
        ttk.Label(info_frame, textvariable=self.lfsr_period_var, font=('Arial', 10, 'bold', 'underline')).pack(side=tk.LEFT, padx=10)

        ttk.Label(info_frame, text="| LFSR Duty Cycle (Ratio):").pack(side=tk.LEFT, padx=5)
        ttk.Label(info_frame, textvariable=self.lfsr_duty_cycle_var, font=('Arial', 10, 'bold', 'underline')).pack(side=tk.LEFT, padx=10)
        
        ttk.Label(info_frame, text="| Quantization Error:").pack(side=tk.LEFT, padx=5)
        ttk.Label(info_frame, textvariable=self.lfsr_error_var, font=('Arial', 10, 'bold', 'underline')).pack(side=tk.LEFT, padx=10)
        
        self.update_plots(self.level_var.get())
        
    def update_plots(self, level_val):
        """Updates all plots based on the current PWM Level."""
        
        # CRITICAL FIX: Ensure level is an integer and update the IntVar explicitly 
        # so the label displays the clean integer value.
        level = int(float(level_val))
        self.level_var.set(level)
        
        # --- 1. Signal Generation ---
        lfsr_signal, D_lfsr, error_lfsr = generate_lfsr_pwm_signal(level, MAX_FFT_LENGTH)
        
        if level < len(parsed_lfsr_config):
            period = parsed_lfsr_config[level]['period']
            ordinary_signal = generate_ordinary_pwm_signal(level, period, MAX_FFT_LENGTH)
        else:
            period = 1 
            ordinary_signal = np.array([])
            
        # --- 2. RC Integrator Simulation ---
        lfsr_filtered = simulate_rc_integrator(lfsr_signal, RC_FILTER_ALPHA)
        ord_filtered = simulate_rc_integrator(ordinary_signal, RC_FILTER_ALPHA)
        
        # --- 3. Update Info Variables ---
        self.lfsr_period_var.set(f"{period} (Base)")
        self.lfsr_duty_cycle_var.set(f"{D_lfsr:.6f}")
        self.lfsr_error_var.set(f"{error_lfsr:.2e}")

        # --- 4. Plot Signal Waveform (Time Domain) ---
        self.ax_sig.clear()
        segment_len = min(4 * period + 20, lfsr_signal.size, 500) # Plot a few cycles or max 500 steps
        
        # Plot LFSR raw signal (digital steps, lower opacity)
        self.ax_sig.plot(lfsr_signal[:segment_len], drawstyle='steps-post', 
                         label='LFSR PWM (Raw Digital)', color='tab:blue', alpha=0.3, linewidth=0.5)
                         
        # Plot LFSR filtered output (analog line)
        self.ax_sig.plot(lfsr_filtered[:segment_len], 
                         label='LFSR PWM (Filtered Output)', color='tab:blue', linewidth=2)
                         
        # Plot Ordinary filtered output (analog line)
        self.ax_sig.plot(ord_filtered[:segment_len], 
                         label='Ordinary PWM (Filtered Output)', color='tab:red', linestyle='--', linewidth=1.5)

        self.ax_sig.set_title(f'Filtered Output Simulation at Level {level} (RC Filter $\\alpha$={RC_FILTER_ALPHA})')
        self.ax_sig.set_xlabel('Time Step (k)')
        self.ax_sig.set_ylabel('Filtered Output Voltage (0-1)')
        self.ax_sig.set_ylim([-0.1, 1.1])
        self.ax_sig.set_yticks(np.linspace(0, 1, 6))
        self.ax_sig.grid(axis='y', alpha=0.5)
        self.ax_sig.legend(loc='upper right')

        # --- 5. Plot Spectral Response (Frequency Domain) ---
        # Plotting PSD of the RAW signal (centered) is the best way to show noise shaping.
        self.ax_spec.clear()
        
        xf_lfsr, psd_lfsr = calculate_spectral_response(lfsr_signal)
        xf_ord, psd_ord = calculate_spectral_response(ordinary_signal)
        
        self.ax_spec.plot(xf_lfsr, 10*np.log10(psd_lfsr + 1e-15), label='LFSR PWM (Noise Shaped)', color='tab:blue') # Use dB scale
        self.ax_spec.plot(xf_ord, 10*np.log10(psd_ord + 1e-15), label='Ordinary PWM (Harmonics)', color='tab:red', alpha=0.7) # Use dB scale
        
        self.ax_spec.set_title('Spectral Response (Power Spectral Density - dB Scale)')
        self.ax_spec.set_xlabel('Normalized Frequency ($f/f_s$)')
        self.ax_spec.set_ylabel('Normalized Power (dB)')
        self.ax_spec.grid(True, which="major", ls="-", alpha=0.5)
        self.ax_spec.set_xlim(0, 0.5)
        self.ax_spec.legend(loc='upper right')
        
        # --- 6. Plot Error vs. Ideal Duty Cycle ---
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
