Robust Balanced LFSR-Burst PWM — README

This document collects and explains the full design space we explored in the conversation: several sketches and a final hybrid opportunistic + ISR microburst design that aims to deliver a robust, balanced pseudo-random PWM with better low-frequency (spectral) behavior than ordinary software PWM and a higher practical frequency limit on AVR-class MCUs — while remaining safe and predictable.

The README explains:

what the code family does and why,

the different sketches we produced and when to use them,

how the hybrid design works (credits, ISR microbursts, opportunistic bursts),

the tradeoffs and practical limitations (CPU, ISR latency, EMI),

spectral consequences (what changes in the frequency domain and why),

tuning recommendations and concrete parameter suggestions,

measurement and debugging suggestions.



---

Short summary / main goal

The main function of the code is to produce a PWM-like output whose average duty is driven by a LUT or an 8-bit pwm_level, while using an LFSR-generated pseudo-random bitstream and balanced micro-burst strategies so that:

low-frequency discrete harmonics typical of deterministic software PWM are greatly reduced (energy is spread),

the effective update rate (perceived resolution) is increased beyond what a simple per-ISR single-step LFSR can deliver, and

the design stays safe: ISR work is tiny and bounded; large opportunistic bursts are performed only when the main loop can safely disable interrupts.


The hybrid approach uses both opportunistic main-loop bursts (big, balanced, done when the CPU is available) and lightweight ISR microbursts that slowly chip away at backlog and prevent large credit accumulation.


---

Files / sketches (what we produced)

During the conversation we produced a progression of sketches. Here is a short map:

opportunistic_lfsr_burst.ino
Opportunistic bursts only. Main loop computes how many LFSR samples should have been produced while sleeping and emits them in bursts.

opportunistic_lfsr_burst_accounting.ino
Opportunistic version that credits the held output during idle (conservative and physically accurate) and then compensates later by producing samples to meet the desired duty.

opportunistic_lfsr_balanced_burst.ino
Opportunistic balanced bursts: compute backlog and produce a balanced sequence (microbursts that alternate target 1/0) to avoid long runs and bias.

hybrid_opportunistic_isr_balanced.ino
Hybrid design: a Timer1 ISR at TICK_HZ adds credit at SAMPLE_HZ and performs tiny balanced microbursts (very tightly capped). The main loop does the large opportunistic balanced bursts when it wakes (disables interrupts while bursting to simplify semaphore issues).

hybrid_opportunistic_isr_balanced_flip.ino
Same hybrid design but both ISR and main bursts try to flip the exit state relative to entry by doing a bounded number of extra LFSR steps before finishing. This reduces persistent bias due to the pin being held at one level across idle periods.


> Each sketch uses the same core LFSR (16-bit Galois) with feedback polynomial 0xB400 and the same general method of comparing the top 8 bits of the LFSR to pwm_level to decide a 1 vs 0 output bit.




---

Core ideas and mechanisms

LFSR as pseudo-random sample source

A 16-bit Galois LFSR (feedback 0xB400) produces a long, deterministic pseudo-random sequence with good spectral properties for many applications.

We map a 16-bit state to an 8-bit quantity (top 8 bits) and compare against pwm_level (0..255) to create a Bernoulli( p = pwm_level / 256 ) stream of bits. This yields the correct long-term duty probability while preserving low autocorrelation and good spectral whiteness.


Balanced microbursts

A “microburst” aims to collect a small number of occurrences of a chosen target bit (1 or 0). After it collects that many occurrences it toggles the target and continues.

Alternating microbursts prevents long runs of identical bits, reducing chance of low-frequency bias and smoothing low-frequency PSD.


Opportunistic bursts (main loop)

When the main loop wakes (after random jitter or intentional delay), it computes how many logical samples should have been produced during the elapsed time at the logical SAMPLE_HZ.

The main loop then emits that many LFSR-derived samples in a balanced pattern (alternating microbursts). Because this can be many samples, the main burst is disabled interrupts during execution (per your request) to avoid races and keep the logic simple.

Main bursts are capped to avoid blocking too long (MAX_MAIN_BURST_SAMPLES).


ISR microbursts (assist, not replace)

A low-rate Timer1 ISR (tick at TICK_HZ) keeps track of credits using a fixed-point accumulator: it adds SAMPLE_HZ to credit_fp_acc each tick and releases integer credits (samples) when credit_fp_acc >= TICK_HZ.

ISR consumes a tiny amount of credit each tick via a very small balanced microburst (e.g., aim for 1 occurrence of target), which keeps backlog from growing excessively between main loop wakes.

ISR microbursts are strictly bounded and tuned to keep ISR latency safe (ISR_MICROBURST_ITER_CAP, MAX_ISR_BURST_SAMPLES).


Exit-state flipping

At the end of either a main burst or ISR microburst we optionally attempt a few extra LFSR steps to ensure the exit logic level differs from the entry level (flip the pin). This reduces the chance a long held state persists through idle, which would otherwise bias the next interval.


Credit bookkeeping (fixed-point)

To avoid floating-point and maintain precise sampling ratios:

credit_fp_acc += SAMPLE_HZ;
if (credit_fp_acc >= TICK_HZ) {
    delta = credit_fp_acc / TICK_HZ;
    credit_fp_acc %= TICK_HZ;
    sample_credit += delta;
}

Each tick accumulates SAMPLE_HZ/TICK_HZ logical samples (accurately over many ticks thanks to the remainder).



---

Pros (what you gain)

1. Reduced low-frequency harmonic content
A pseudo-random bitstream (LFSR) spreads periodic energy broadly. Compared to fixed-frequency PWM, discrete low-frequency harmonics (strong lines) are damped.


2. Improved effective resolution (higher practical frequency)
By emitting multiple LFSR samples per wake we increase effective sample throughput without increasing the base tick frequency. That lets you achieve smoother duty granularity than single-step ISR per-tick approaches.


3. Balanced, anti-bias behavior
Alternating microbursts and exit-state flipping reduce DC bias and long same-level runs — improving both instantaneous fairness and long-term accuracy.


4. Backlog control via ISR
ISR microbursts prevent runaway accumulation of missed samples, so worst-case main-burst work is bounded in practice.


5. Safe, simple semaphore model
Disabling interrupts during main bursts eliminates complicated locking between ISR and main loop — the design is easier to reason about.


6. Portable and tweakable
Runs on AVR/UNO without special hardware; parameters allow you to balance CPU, latency, EMI and spectral tradeoffs.




---

Cons and practical limitations (what to watch out for)

1. ISR latency and worst-case blocking

ISR microbursts must be tiny. If you increase their budget you lengthen ISR latency and risk impacting timing-critical code (millis(), Serial, other timers).

Main opportunistic bursts disable interrupts and can block other activity while they run. Cap MAX_MAIN_BURST_SAMPLES to limit worst-case blocking time.



2. Compressed bursts change spectral shape
Emitting many samples back-to-back (opportunistic bursts) compresses time and creates high-frequency clusters. This tends to increase broadband high-frequency energy (worse EMI at high frequencies) and produce sidebands at the burst repetition rate.


3. No perfect flip guarantee
Exit-flip attempts are bounded. Rarely, the LFSR may produce the same bit repeatedly and flip attempts may fail; flip is probabilistic within caps.


4. Finite LFSR bias
LFSR is deterministic and periodic. For some ultra-sensitive spectral use you may need a longer PRNG or cryptographic RNG.


5. Physical load and driver constraints
Very fast toggles increase stress on drivers, supply, and connected loads. If using power electronics (motor driver, MOSFETs), examine switching losses and EMI filtering.


6. Spectral tradeoffs vs deterministic PWM
While deterministic PWM has clean harmonics (easy to filter for certain low-pass designs), pseudo-random PWM spreads energy and may be harder to filter into a pure DC with a small simple RC. System-level choice depends on the downstream filter and application.




---

Spectral conclusions (detailed)

Below are practical spectral observations and an explanation of what to expect.

1) Deterministic PWM (baseline)

Produces strong discrete harmonics at the switching fundamental and its integer multiples.

Low-pass filter removes higher harmonics, but residual low-frequency discrete components (e.g., beat frequencies from duty changes) remain.


2) LFSR single-step per IRQ (simple pseudo-random PWM)

The bitstream approximates white noise (flat PSD) up to a cutoff determined by the LFSR sample rate.

Consequently, energy that used to sit in discrete harmonics is distributed over a broad band — low-frequency discrete lines are greatly reduced. This is usually beneficial to reduce audible or low-frequency EMI and mechanical excitation.


3) Opportunistic / bursty emission

When samples are emitted in compressed bursts, the time-domain envelope becomes a pulse train of bursts. The spectrum becomes:

A broadband component from the pseudo-random bitstream (similar to above), plus

Sidebands centered at burst cluster frequencies (burst repetition rate) and multiples thereof. Those sidebands reflect the envelope modulation — you get energy at (burst repetition frequency ± PRBS spectral components).


That means: compressed bursts shift some energy to higher frequency ranges (good if you want to avoid audio band), but you also create narrowband features at the burst repetition rate that could be problematic if they fall into a sensitive band.


4) Balanced microbursts and toggling targets

Alternating microbursts (1/0 targets) reduce long runs and hence reduce low-frequency spectral content (less DC/near-DC energy).

Exit flipping reduces the DC component across wake boundaries, further attenuating the very-low-frequency end of the spectrum.


5) ISR microbursts spread energy more evenly

Tiny ISR microbursts regularly distribute some samples at the ISR tick rate, smoothing the envelope and reducing burst-induced sidebands. Thus, ISR microbursts help the spectrum by avoiding large, infrequent packet emissions.


Practical spectral summary

Low-frequency harmonics: reduced (good) — pseudo-randomization and balancing mitigates discrete low-frequency bins.

Broadband high-frequency noise: increased (caution) — more high-frequency energy due to compressed emission; measure and filter if needed.

Envelope sidebands: present if bursts are periodic — especially if main loop wakes regularly or ISR tick produces periodic microbursts; can be mitigated by jittering the burst timing or reducing burst size but at increased complexity.

Net result: for many real systems (motors, LEDs with RC filter, audio appliances), spreading energy away from discrete harmonics and into a higher-frequency noise floor is beneficial — it reduces tonal artifacts and may be easier to filter — but always measure.



---

Concrete parameter guidance & examples

> These are starting suggestions for ATmega328P / Arduino UNO. Measure and tune for your hardware.



Parameter	Suggested start value	Notes

SAMPLE_HZ	10_000 … 50_000 Hz	Logical LFSR sample rate. Higher improves apparent resolution but costs CPU when you catch up.
TICK_HZ	250 … 2000 Hz	ISR tick rate. Lower = fewer interrupts, larger step per tick.
ISR_MICROBURST_OCCURRENCES	1	Aim for 1 occurrence per tick in ISR to keep it short.
ISR_MICROBURST_ITER_CAP	16 … 64	Safety cap on LFSR steps in ISR microburst. Keep small.
MAX_ISR_BURST_SAMPLES	1 … 8	Absolute samples ISR can produce per tick.
MAIN_MICROBURST_OCCURRENCES	2 … 8	Microburst occurrences for main bursts.
MAIN_MICROBURST_ITER_CAP	512 … 2048	Cap per microburst iteration during main burst.
MAX_MAIN_BURST_SAMPLES	1000 … 8000	Cap main burst to limit blocking. Choose so worst-case blocking is acceptable.
MAIN_EXIT_FLIP_CAP	16 … 64	Extra attempts to flip exit state (bounded).


Example:
If SAMPLE_HZ = 20 kHz and idle delay() is 200 ms, expected backlog ≈ 0.2 s * 20 000 Hz = 4000 samples. If MAX_MAIN_BURST_SAMPLES is 8000, the main burst can fully catch up in one wake; if smaller, ISR microbursts and later wakes will finish the backlog.


---

How to measure / debug (recommended)

1. Oscilloscope

Toggle a debug pin at ISR entry/exit and at main-burst start/stop (easy to add). Use the scope to measure:

ISR pulse width (max time spent in ISR),

Main burst duration (how long interrupts are disabled),

Output waveform during bursts.


Use persistence and FFT (or export samples to a PC) to observe spectral content.



2. Spectrum analyzer or FFT

Collect a long sample of the output and compute PSD. Compare deterministic PWM vs. LFSR-burst hybrid:

look for reduced discrete harmonics,

check broadband noise increase,

check for sidebands at burst repetition rates.




3. Serial/logging

The sketches report samples, achieved duty and pendingCredits. Monitor pendingCredits to confirm ISR is keeping up.



4. Tune on hardware

Reduce SAMPLE_HZ if main burst/blocking is excessive.

Lower ISR microburst caps if ISR latency is visible.

Increase or vary TICK_HZ to trade off ISR overhead vs. backlog smoothing.





---

Practical recommendations and best practices

Start conservative. Use low SAMPLE_HZ (10–20k) and small burst caps. Increase only after measuring.

Use debug pins to measure durations — add toggles in ISR and at main burst start/finish.

If you need evenly spaced samples, do not compress into opportunistic bursts — instead use a timer + shorter ISR microbursts that output evenly. The hybrid design partially helps by distributing some work in ISR, but opportunistic bursts are inherently compressed.

If EMI is a concern, add LC/R/C low-pass filtering and/or adjust microburst sizes so instantaneous toggling is limited.

If you need extreme spectral purity or very high rates, move to a microcontroller with DMA/PIO, or use hardware PWM with dithering techniques (or a DAC/filtered PWM).

Keep caps reasonable — they are the key safety mechanism in the design.



---

Formulas & code-level notes

Desired ones for N samples:

desired_ones = round(N * pwm_level / 256)
desired_zeros = N - desired_ones

We implemented rounding as:

desired_ones = (N * pwm_level + 128) / 256;

Fixed-point crediting in ISR:

credit_fp_acc += SAMPLE_HZ;
if (credit_fp_acc >= TICK_HZ) {
    uint32_t delta = credit_fp_acc / TICK_HZ;
    credit_fp_acc %= TICK_HZ;
    sample_credit += delta;
}

LFSR step (Galois variant used in our code):

if (s & 1) s = (s >> 1) ^ 0xB400u;
else s >>= 1;
if (s == 0) s = 1; // avoid zero state

We used the top8 = s >> 8 comparison to derive an 8-bit uniform-ish value from the 16-bit LFSR; comparing top8 < pwm_level produces the Bernoulli stream.



---

When this design is a good fit

You want a PWM-like signal where low-frequency tonal artifacts must be minimized (e.g., to avoid motor hum, mechanical resonance, or audible tones).

You need better effective resolution than the hardware timer or straightforward software PWM provides, but upgrading hardware is not an immediate option.

You can tolerate some opportunistic blocking in main code (or you can keep main bursts limited and rely more on ISR microbursts).

You accept a broadband noise tradeoff (vs. concentrated harmonics) and can add filtering as required.


When to choose something else

If deterministic, evenly-spaced pulses are mandatory for your application (sampling sync, precise timing, audio tone generation), use hardware timers/hardware PWM.

For very high frequency or low jitter distributed pulses, consider DMA / PIO style hardware or a microcontroller that supports peripheral-driven waveforms.

If EMI rules are strict, a pure pseudo-random high-rate approach may worsen high-frequency emissions; in that case use carefully designed filters and EMI mitigation.



---

Next steps (if you want me to extend)

I can:

Add debug-pin toggles inside the hybrid sketch (ISR enter/exit, main-burst start/stop).

Add statistics about flip success/failure (how often exit flip retries had to be used).

Add an adaptive ISR microburst policy (grow microburst size when sample_credit grows).

Replace the top8 compare with a small lookup table generator if you need much higher throughput (faster than parity or shifts).

Add a simple companion script (Python) to read the output via a sound card or ADC and compute/plot PSD/FFT to compare deterministic PWM vs LFSR-bursted output.


Tell me which extension you prefer (debug pin toggles, flip stats, adaptive ISR, faster generator or measurement helper) and I’ll produce the code/update.
