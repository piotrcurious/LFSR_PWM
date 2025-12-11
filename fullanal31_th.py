import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from scipy.fft import fft
import re

# --- GLOBAL CONFIGURATION (Now variables, not constants) ---
LFSR_WIDTH = 16
PWM_BASE_PERIOD = 255
# These are now dynamic, initialized to their original defaults
ERROR_CALC_LENGTH = 512
VISUAL_SEGMENT_CAP = 512
RC_FILTER_ALPHA = 0.1 # Alpha = dt / (RC + dt). dt=1 (step size). RC = tau

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
    """Generates LFSR signal and calculates duty cycle ratio and error."""
    if not (0 <= level <= 255) or not parsed_lfsr_config or level >= len(parsed_lfsr_config):
        return np.array([]), 0.0, 0.0
    entry = parsed_lfsr_config[level]
    mask = entry['mask']
    seed = entry['seed']
    
    # Ensure sequence length is adequate for the full period if requested
    if sequence_len > (2**LFSR_WIDTH - 1):
        sequence_len = (2**LFSR_WIDTH - 1)
        
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
    # 16-bit LFSR has a maximum period of 2^16 - 1 = 65535
    LFSR_TRUE_PERIOD = 2**LFSR_WIDTH - 1
    
    def __init__(self, master):
        self.master = master
        master.title("LFSR PWM Analyzer")
        master.geometry("1200x780")
        self.master.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # UI variables
        self.seq_len_exp_var = tk.IntVar(value=13)
        self.level_var = tk.IntVar(value=127)
        self.steady_state_scaled_var = tk.BooleanVar(value=False) # New control for dual axis
        
        # New variables for dynamic global configuration
        self.tau_var = tk.DoubleVar(value=9.0) # RC/dt = tau_s, where dt=1
        self.error_calc_len_exp_var = tk.IntVar(value=9) # For ERROR_CALC_LENGTH = 2^N
        
        # Pre-calculated steady-state error (will be populated after file load)
        self.steady_state_errors = np.array([])
        
        # Debouncing mechanism variable
        self.plot_update_job = None
        
        # Set initial global values from new variables (using simple setters)
        self._update_rc_alpha_global(self.tau_var.get())
        self._set_initial_error_calc_length(self.error_calc_len_exp_var.get())
        
        self.setup_parsing_ui()

    def on_closing(self):
        try:
            # Cancel any pending plot updates before exit
            if self.plot_update_job:
                self.master.after_cancel(self.plot_update_job)
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
                # Calculate steady-state errors once after successful load
                self.calculate_steady_state_errors()
                self.setup_analyzer_ui()
            else:
                messagebox.showerror("Parsing Error",
                    f"Could not parse the LUT data or found only {len(parsed_lfsr_config)} entries. "
                    "Ensure the file contains exactly 256 entries in the format: {{ 0xMASK, 0xSEED }},")
        except FileNotFoundError:
            messagebox.showerror("File Error", "The selected file was not found.")
        except Exception as e:
            messagebox.showerror("File Read Error", f"An unexpected error occurred while reading the file: {e}")

    def calculate_steady_state_errors(self):
        """Calculates the true, full-period steady-state error for all 256 levels."""
        print(f"Calculating steady-state errors over {self.LFSR_TRUE_PERIOD} steps...")
        errors = []
        for l in range(256):
            # Sequence length is set to the full LFSR period
            _, _, true_error = generate_lfsr_pwm_signal(l, self.LFSR_TRUE_PERIOD)
            errors.append(true_error)
        self.steady_state_errors = np.array(errors)
        print("Steady-state errors calculation complete.")

    # --- Debouncing Logic ---
    def debounce_plot_update(self, delay_ms=50):
        """Cancels a pending plot update and schedules a new one."""
        if self.plot_update_job:
            self.master.after_cancel(self.plot_update_job)
        
        # The actual plotting function which pulls current state from all variables
        def scheduled_plot_update():
            self.plot_update_job = None
            self.update_plots() # Now called without arguments, reads self.level_var internally
            
        self.plot_update_job = self.master.after(delay_ms, scheduled_plot_update)

    # --- Mouse Wheel Handlers ---
    def on_mouse_wheel_level(self, event):
        return self._handle_mouse_wheel(event, self.level_var, 0, 255, 1)
    
    def on_mouse_wheel_tau(self, event):
        return self._handle_mouse_wheel(event, self.tau_var, 1.0, 100.0, 0.5)

    def on_mouse_wheel_error_len(self, event):
        return self._handle_mouse_wheel(event, self.error_calc_len_exp_var, 5, 13, 1)
        
    def on_mouse_wheel_fft_len(self, event):
        # Range for FFT length exponent (N) is 9 to 13
        return self._handle_mouse_wheel(event, self.seq_len_exp_var, 9, 13, 1)


    def _handle_mouse_wheel(self, event, variable, min_val, max_val, step):
        current_value = variable.get()
        delta = 0

        # 1. Try to get delta from Windows/macOS event type
        if hasattr(event, 'delta'):
            if event.delta > 0:
                delta = 1
            elif event.delta < 0:
                delta = -1
        
        # 2. Try Linux/X11 style event (Button-4/Button-5)
        if delta == 0 and hasattr(event, 'num'):
            if event.num == 4: # Scroll up/forward
                delta = 1
            elif event.num == 5: # Scroll down/backward
                delta = -1
                
        if delta == 0:
            return "break"
            
        new_value = current_value + delta * step
        
        # Clamp value
        new_value = max(min_val, min(max_val, new_value))
        
        # Ensure integer variables only get integer values
        if isinstance(variable, tk.IntVar):
             new_value = int(round(new_value))
        
        # 1. Update the variable and associated non-plotting state/labels IMMEDIATELY
        variable.set(new_value)

        if variable == self.tau_var:
            self._update_rc_alpha_global(new_value)
            self.tau_label_var.set(f"$\\tau={new_value:.1f}$")
        elif variable == self.error_calc_len_exp_var:
            self._set_initial_error_calc_length(new_value)
            self.error_len_label_var.set(f"{int(new_value)}")
        elif variable == self.seq_len_exp_var:
            self.seq_len_label_var.set(f"{int(new_value)}")
        
        # 2. Debounce the expensive plot update
        self.debounce_plot_update()
            
        return "break" # Prevent the parent widget from scrolling

    # --- Update Logic for Sliders (called by slider command) ---
    def update_level_and_plots(self, value):
        """Called by the Level slider command. Ensures level is an integer."""
        # FIX: Explicitly round and set the IntVar to ensure clean integer display
        new_level = int(round(float(value)))
        self.level_var.set(new_level)
        self.debounce_plot_update()

    def update_seq_len(self, *args):
        exp = int(round(self.seq_len_exp_var.get()))
        self.seq_len_exp_var.set(exp) # Correct rounding
        self.seq_len_label_var.set(f"{exp}") # Only show N
        self.debounce_plot_update()

    def _update_rc_alpha_global(self, tau):
        global RC_FILTER_ALPHA
        # Alpha = dt / (RC + dt). With dt=1, RC=tau: Alpha = 1 / (tau + 1)
        RC_FILTER_ALPHA = 1.0 / (float(tau) + 1.0)
    
    def update_tau_and_plots(self, tau_val):
        tau = float(tau_val)
        self._update_rc_alpha_global(tau)
        self.tau_var.set(f"{tau:.1f}")
        self.tau_label_var.set(f"$\\tau={tau:.1f}$") # Show tau value
        self.debounce_plot_update()

    def _set_initial_error_calc_length(self, exp_val):
        global ERROR_CALC_LENGTH
        exp = int(round(float(exp_val)))
        ERROR_CALC_LENGTH = 2**exp

    def update_error_calc_len_and_plots(self, exp_val):
        exp = int(round(float(exp_val)))
        self._set_initial_error_calc_length(exp) # Update global
        self.error_calc_len_exp_var.set(exp) # Correct rounding
        self.error_len_label_var.set(f"{exp}") # Only show N
        self.debounce_plot_update()

    # --- UI Setup ---
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

        # Define all StringVar/LabelVar here, before calling update methods
        self.lfsr_calc_ratio_var = tk.StringVar(value="N/A")
        self.lfsr_calc_error_var = tk.StringVar(value="N/A")
        self.seq_len_label_var = tk.StringVar() # For FFT N
        self.tau_label_var = tk.StringVar()      # For Tau value
        self.error_len_label_var = tk.StringVar() # For Error N
        
        # --- Style Definition for Color-Coding and general appearance ---
        style = ttk.Style()
        # Custom styles for vertical scales
        style.configure('Red.Vertical.TScale', troughcolor='#FFDDE0', background='#DC143C') # Red for Level
        style.configure('Blue.Vertical.TScale', troughcolor='#DDEEFF', background='#1E90FF') # Blue for Tau
        style.configure('Green.Vertical.TScale', troughcolor='#DDFFEE', background='#3CB371') # Green for Error Length
        style.configure('Orange.Vertical.TScale', troughcolor='#FFEEDD', background='#FF8C00') # Orange for FFT Length


        main_analysis_frame = ttk.Frame(self.master)
        main_analysis_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=1, padx=6, pady=6)

        # Figure setup
        self.fig, (self.ax_sig, self.ax_spec, self.ax_error) = plt.subplots(
            3, 1, figsize=(10, 6.5),
            gridspec_kw={'height_ratios': [2, 3, 2]}
        )
        # Initialize the twin axis for steady-state error scaling
        self.ax_error_twin = self.ax_error.twinx() 
        self.ax_error_twin.set_ylabel('Steady-State Error (%)', color='purple', labelpad=6)
        self.ax_error_twin.tick_params(axis='y', colors='purple')
        self.ax_error_twin.spines['right'].set_color('purple')

        self.fig.subplots_adjust(left=0.02, right=0.98, top=0.975, bottom=0.03, hspace=0.18)

        self.canvas = FigureCanvasTkAgg(self.fig, master=main_analysis_frame)
        self.canvas_widget = self.canvas.get_tk_widget()

        # Grid setup for main frame (Plots on left, Controls on right)
        main_analysis_frame.grid_columnconfigure(0, weight=4) # Plots column
        main_analysis_frame.grid_columnconfigure(1, weight=0) # Controls column (minimal width)
        main_analysis_frame.grid_rowconfigure(0, weight=1)

        self.canvas_widget.grid(row=0, column=0, sticky="nsew", padx=(0,6))

        # Controls: Refactored to a compact, vertically-stacked frame
        controls_frame = ttk.Frame(main_analysis_frame, padding=(4, 8))
        controls_frame.grid(row=0, column=1, sticky="nsew", padx=(0, 0)) # Sticky 'nsew' ensures vertical expansion
        
        # Give all four rows (sliders) equal vertical weight for scaling
        controls_frame.grid_rowconfigure((0, 1, 2, 3), weight=1) 
        controls_frame.grid_columnconfigure(0, weight=1) 

        # Helper function to create control group
        def create_control_group(parent, row, text_label, var, min_val, max_val, command, wheel_handler, style_name, label_var=None):
            frame = ttk.Frame(parent, padding=2)
            # Use 'nsew' to ensure the frame expands vertically within the controls_frame's row
            frame.grid(row=row, column=0, sticky='nsew', pady=(0, 6)) 
            frame.grid_columnconfigure(0, weight=1)
            frame.grid_rowconfigure(2, weight=1) # Give the slider row weight for expansion

            # Title label (smaller font)
            ttk.Label(frame, text=text_label, anchor='center', font=('Arial', 8, 'bold')).grid(row=0, column=0, pady=(2,1), sticky='ew')

            # Value label (use value_label to show current setting)
            if label_var:
                value_label = ttk.Label(frame, textvariable=label_var, font=('Arial', 9, 'bold'))
            else:
                value_label = ttk.Label(frame, textvariable=var, font=('Arial', 9, 'bold'))
            value_label.grid(row=1, column=0, pady=(0,2))
            
            # Use dynamic scaling via expand=True
            slider = ttk.Scale(frame, from_=max_val, to=min_val,
                                  orient=tk.VERTICAL,
                                  style=style_name, # Apply custom style
                                  variable=var, command=command)
            # Fill vertically and expand to take all available space
            slider.grid(row=2, column=0, pady=2, sticky='nsew') 
            
            # CRITICAL FIX: Bind the wheel event to ALL possible event types for max compatibility
            # Bind <MouseWheel> (Windows/macOS)
            slider.bind('<MouseWheel>', wheel_handler)
            frame.bind('<MouseWheel>', wheel_handler)
            # Bind <Button-4> (Scroll Up, Linux/X11)
            slider.bind('<Button-4>', wheel_handler)
            frame.bind('<Button-4>', wheel_handler)
            # Bind <Button-5> (Scroll Down, Linux/X11)
            slider.bind('<Button-5>', wheel_handler)
            frame.bind('<Button-5>', wheel_handler)
            
            return slider, frame

        # --- 1. PWM Level (Red) ---
        self.level_slider, self.level_frame = create_control_group(
            controls_frame, 0, "Level", self.level_var, 0, 255, self.update_level_and_plots, self.on_mouse_wheel_level, 'Red.Vertical.TScale', label_var=None # Default tk.IntVar shows value
        )
        
        # --- 2. RC Filter Tau (Blue) ---
        self.tau_slider, self.tau_frame = create_control_group(
            controls_frame, 1, "Tau ($\tau$)", self.tau_var, 1.0, 100.0, self.update_tau_and_plots, self.on_mouse_wheel_tau, 'Blue.Vertical.TScale', label_var=self.tau_label_var
        )
        self.update_tau_and_plots(self.tau_var.get()) 

        # --- 3. Error Calc Length (Green) ---
        self.error_len_slider, self.error_len_frame = create_control_group(
            controls_frame, 2, "Err N ($2^N$)", self.error_calc_len_exp_var, 5, 13, self.update_error_calc_len_and_plots, self.on_mouse_wheel_error_len, 'Green.Vertical.TScale', label_var=self.error_len_label_var
        )
        self.update_error_calc_len_and_plots(self.error_calc_len_exp_var.get()) 

        # --- 4. FFT Length (Orange) ---
        self.seq_len_slider, self.seq_len_frame = create_control_group(
            controls_frame, 3, "FFT N ($2^N$)", self.seq_len_exp_var, 9, 13, self.update_seq_len, self.on_mouse_wheel_fft_len, 'Orange.Vertical.TScale', label_var=self.seq_len_label_var
        )
        self.update_seq_len()
        
        ttk.Separator(controls_frame, orient='horizontal').grid(row=4, column=0, sticky='ew', pady=(6,0))
        
        # --- 5. Steady-State Scaling Control (New) ---
        steady_state_control_frame = ttk.Frame(controls_frame, padding=(2, 4))
        steady_state_control_frame.grid(row=5, column=0, sticky='ew', pady=(4, 0))
        
        ttk.Label(steady_state_control_frame, text="Steady-State Scale:", font=('Arial', 8)).pack(side=tk.LEFT, padx=(0,4))
        
        check = ttk.Checkbutton(steady_state_control_frame, 
                                text="Dual Axis", 
                                variable=self.steady_state_scaled_var,
                                command=self.debounce_plot_update) # Plot update on toggle
        check.pack(side=tk.RIGHT)
        
        # Info bar (bottom)
        info_frame = ttk.Frame(self.master, padding=(6,4))
        info_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=6, pady=(2,6))

        ttk.Label(info_frame, text="Duty:", font=('Arial', 9)).pack(side=tk.LEFT, padx=(4,6))
        ttk.Label(info_frame, textvariable=self.lfsr_calc_ratio_var, foreground='green', font=('Arial', 9, 'bold')).pack(side=tk.LEFT)

        ttk.Label(info_frame, text=" | Err(%):", font=('Arial', 9)).pack(side=tk.LEFT, padx=(8,6))
        ttk.Label(info_frame, textvariable=self.lfsr_calc_error_var, foreground='green', font=('Arial', 9, 'bold')).pack(side=tk.LEFT)

        self.update_plots() # Initial plot without arguments

    def update_plots(self, *args):
        """
        The main plotting function. It now reads all state variables directly
        and is called only by the debouncer.
        """
        global RC_FILTER_ALPHA, ERROR_CALC_LENGTH
        
        level = self.level_var.get()
        sequence_len = 2**int(round(self.seq_len_exp_var.get()))
        use_twin_axis = self.steady_state_scaled_var.get()

        # Calculate Transient/Simulated data (using user-defined length)
        lfsr_signal, calc_ratio, calc_error = generate_lfsr_pwm_signal(level, sequence_len)

        if level < len(parsed_lfsr_config):
            period = parsed_lfsr_config[level]['period']
            ordinary_signal = generate_ordinary_pwm_signal(level, period, sequence_len)
        else:
            period = 1
            ordinary_signal = np.array([])

        # Use the dynamic global variable
        lfsr_filtered = simulate_rc_integrator(lfsr_signal, RC_FILTER_ALPHA)
        ord_filtered = simulate_rc_integrator(ordinary_signal, RC_FILTER_ALPHA)

        # Update info text (using the transient error)
        self.lfsr_calc_ratio_var.set(f"{calc_ratio:.6f}")
        self.lfsr_calc_error_var.set(f"{calc_error*100:.3f}")

        # --- Time Domain ---
        self.ax_sig.clear()
        segment_len = min(4 * period + 20, lfsr_signal.size, VISUAL_SEGMENT_CAP)
        self.ax_sig.plot(lfsr_signal[:segment_len], drawstyle='steps-post',
                              label='LFSR', linewidth=0.6, alpha=0.4)
        self.ax_sig.plot(lfsr_filtered[:segment_len],
                              label='LFSR filt', linewidth=1.0)
        self.ax_sig.plot(ord_filtered[:segment_len],
                              label='Ord filt', linestyle='--', linewidth=0.8)
        self.ax_sig.set_title(f'Filtered Output ($\\tau={self.tau_var.get():.1f}$) — Level {int(level)}', pad=6)
        self.ax_sig.set_xlabel('Time (steps)', labelpad=6)
        self.ax_sig.set_ylabel('Voltage', labelpad=6)
        self.ax_sig.set_ylim([-0.1, 1.1])
        self.ax_sig.set_xlim(0, max(1, segment_len))
        self.ax_sig.set_yticks(np.linspace(0, 1, 5))
        self.ax_sig.grid(axis='y', alpha=0.35, linewidth=0.4)
        self.ax_sig.legend(loc='upper right', fontsize=8, frameon=False, handlelength=1.2)

        # --- Frequency Domain ---
        self.ax_spec.clear()
        xf_lfsr, psd_lfsr = calculate_spectral_response(lfsr_signal)
        xf_ord, psd_ord = calculate_spectral_response(ordinary_signal)
        # Note: sequence_len is the FFT length. This is displayed via the slider.
        if xf_lfsr.size:
            self.ax_spec.plot(xf_lfsr, 10*np.log10(psd_lfsr + 1e-15), label='LFSR PSD', linewidth=0.8)
        if xf_ord.size:
            self.ax_spec.plot(xf_ord, 10*np.log10(psd_ord + 1e-15), label='Ord PSD', linewidth=0.8, alpha=0.8)
        self.ax_spec.set_title(f'Spectral Response (PSD) - FFT Len $2^{{{self.seq_len_exp_var.get()}}}$', pad=6)
        self.ax_spec.set_xlabel('Normalized f / f_s', labelpad=6)
        self.ax_spec.set_ylabel('Power (dB)', labelpad=6)
        self.ax_spec.grid(True, which="major", ls="-", alpha=0.35, linewidth=0.4)
        self.ax_spec.set_xlim(0, 0.5)
        self.ax_spec.legend(loc='upper right', fontsize=8, frameon=False)

        # --- Error plot ---
        self.ax_error.clear()
        self.ax_error_twin.clear() # Always clear twin axis
        
        all_levels = np.arange(256)
        
        # 1. Ordinary PWM Quantization Error (Always on Primary Axis)
        ord_quant_errors = np.array([
            (l / 255.0) - (round((l / 255.0) * entry['period']) / entry['period'])
            for l, entry in enumerate(parsed_lfsr_config)
        ])
        self.ax_error.plot(all_levels, ord_quant_errors * 100, 
                           label='Ord err', linewidth=0.7, alpha=0.7, color='grey')
        
        # 2. Transient/Simulated LFSR Error (Always on Primary Axis)
        all_lfsr_transient_errors = np.array([
            generate_lfsr_pwm_signal(l, ERROR_CALC_LENGTH)[2]
            for l in all_levels
        ])
        self.ax_error.plot(all_levels, all_lfsr_transient_errors * 100, 
                           label=f'Transient err ($2^{{{self.error_calc_len_exp_var.get()}}}$)', 
                           linewidth=1.0, color='red')
        
        # Marker for current level's transient error (Always on Primary Axis)
        current_error_for_marker = generate_lfsr_pwm_signal(level, ERROR_CALC_LENGTH)[2]
        self.ax_error.plot(level, current_error_for_marker * 100, 'o', markersize=6, color='green', label='Current')

        # 3. True Steady-State LFSR Error (Conditional Axis)
        if self.steady_state_errors.size == 256:
            ss_data = self.steady_state_errors * 100
            current_ss_error = self.steady_state_errors[level] * 100 # Calculate SS error for current level
            
            if use_twin_axis:
                # Plot on Secondary Axis (ax_error_twin)
                self.ax_error_twin.plot(all_levels, ss_data, 
                                        label='Steady-State err', 
                                        linewidth=1.5, color='purple', alpha=0.9)
                
                # Add marker for current steady-state error on the twin axis
                self.ax_error_twin.plot(level, current_ss_error, 'o', markersize=8, 
                                        color='gold', markeredgecolor='purple', markeredgewidth=1.5, 
                                        label='Current SS')

                # Set dynamic limits for the twin axis based on its data range
                ss_min = ss_data.min()
                ss_max = ss_data.max()
                if ss_min != ss_max:
                    margin = (ss_max - ss_min) * 0.1
                    self.ax_error_twin.set_ylim(ss_min - margin, ss_max + margin)
                else:
                    self.ax_error_twin.set_ylim(ss_min - 0.1, ss_max + 0.1) # Handle constant zero error
                    
                self.ax_error_twin.yaxis.set_visible(True)
                self.ax_error_twin.set_ylabel('Steady-State Error (%)', color='purple', labelpad=6)
                
            else:
                # Plot on Primary Axis (ax_error)
                self.ax_error.plot(all_levels, ss_data, 
                                   label='Steady-State err', 
                                   linewidth=1.0, color='purple', alpha=0.9)
                
                # Add marker for current steady-state error on the primary axis (same scale)
                self.ax_error.plot(level, current_ss_error, 'o', markersize=6, 
                                   color='gold', markeredgecolor='purple', markeredgewidth=1.5, 
                                   label='Current SS')
                
                self.ax_error_twin.yaxis.set_visible(False)
                self.ax_error_twin.set_ylabel('') # Clear label
        else:
            # Hide twin axis if steady state data isn't ready
            self.ax_error_twin.yaxis.set_visible(False)
            self.ax_error_twin.set_ylabel('') # Clear label
        
        
        # --- Legend Management ---
        # Get primary and secondary handlers and labels
        h1, l1 = self.ax_error.get_legend_handles_labels()
        h2, l2 = self.ax_error_twin.get_legend_handles_labels()
        
        # Clear existing legends
        if self.ax_error.legend_: self.ax_error.legend_.remove()
        if self.ax_error_twin.legend_: self.ax_error_twin.legend_.remove()

        # Set common axis properties
        self.ax_error.set_title(f'Quantization Error (%) - Calc Len $2^{{{self.error_calc_len_exp_var.get()}}}$ / Steady-State', pad=6)
        self.ax_error.set_xlabel('Level', labelpad=6)
        self.ax_error.set_ylabel('Transient Error (%)', labelpad=6)
        self.ax_error.grid(True, which="both", ls="-", alpha=0.28, linewidth=0.35)
        self.ax_error.set_xlim(0, 255)
        
        if use_twin_axis and h2:
            # Dual Axis Mode: Primary legend on left, Secondary legend on right
            self.ax_error.legend(h1, l1, loc='upper left', fontsize=8, frameon=False, bbox_to_anchor=(0.0, 1.0))
            self.ax_error_twin.legend(h2, l2, loc='upper right', fontsize=8, frameon=False, bbox_to_anchor=(1.0, 1.0))
            self.ax_error_twin.spines['right'].set_visible(True)
        else:
            # Single Axis Mode: Combined legend on lower left
            self.ax_error.legend(h1 + h2, l1 + l2, loc='lower left', fontsize=8, frameon=False)
            self.ax_error_twin.yaxis.set_visible(False)
            self.ax_error_twin.spines['right'].set_visible(False)

        # final tight adjust (keeps margins minimal)
        self.fig.subplots_adjust(left=0.02, right=0.98, top=0.975, bottom=0.03, hspace=0.18)
        self.canvas.draw()
        
if __name__ == '__main__':
    # Initialize the main application window
    root = tk.Tk()
    app = PWM_Analyzer_App(root)
    # Start the Tkinter event loop
    root.mainloop()
