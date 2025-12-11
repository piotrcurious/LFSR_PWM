Nice idea — bursts can raise the effective LFSR update rate without changing the timer base, and toggling the burst target each ISR keeps the expected numbers of 1s and 0s balanced over time. I implemented a balanced LFSR burst mode, updated the measurement to account for the increased number of LFSR samples per ISR, and switched to direct port writes inside the ISR to keep per-iteration cost low.


What I changed (short)

Added a balanced burst path (enabled by BALANCED_BURST_ENABLED) that, inside the ISR, advances the LFSR repeatedly until BURST_TARGET_OCCURRENCES of the current target bit were observed; the target toggles each ISR so bursts alternate target=0/1.

Replaced digitalWrite() inside tight ISR loops with direct PORTB writes to dramatically reduce per-iteration cost.

Added a safety cap BURST_MAX_ITERS to avoid pathological long ISRs.

Measurement was changed to count actual LFSR output samples (bit_sample_count) and number of ones (high_sample_count) so the reported duty = highs / samples reflects the burst activity.

Kept a fallback single-step LFSR path when BALANCED_BURST_ENABLED is 0.


Pros

Higher effective LFSR output rate: you can produce many LFSR samples between timer ticks, improving spectral shaping and moving PWM noise to higher frequency bands.

Statistical balance: toggling the burst target each ISR enforces symmetry of collected 1s and 0s across bursts, helping preserve the intended average duty from the LUT threshold.

Flexible tradeoff: BURST_TARGET_OCCURRENCES controls how many relevant (target) bits are forced per ISR; BURST_MAX_ITERS protects against runaway loops.

Lower visible ripple: more LFSR samples per timer tick tends to create finer granularity and can look smoother after low-pass filtering.


Cons & practical limitations

ISR time increases — the main cost. Every extra LFSR iteration takes cycles; each iteration includes the parity computation and a port write. If bursts are too large you will:

Delay other interrupts (Serial, millis(), hardware timers).

Increase timing jitter for the rest of the system.

Potentially starve background code if ISR load is high.


Worst-case loops — if lfsr_state > threshold rarely matches the chosen target, you may hit BURST_MAX_ITERS often. The safety cap avoids infinite loops but biases the sample count. Use conservative caps.

Hard to precisely bound time — exact cycles/µs per iteration depend on compiler, parity implementation, optimization; you should measure on actual hardware with an oscilloscope or toggle a debug pin to quantify ISR duration.

Interference with hardware PWM / timers — if you rely on hardware timers elsewhere, long ISRs will increase jitter or disturb them.

Serial/USB latency — Serial prints are in loop() but long ISRs may impact Serial timing or USB polling.

Power & EMI — More frequent pin toggles increase spectral content at higher frequencies; may improve EMI in some ranges but can create problems elsewhere. Filter design and scope checks recommended.

Determinism — the pattern becomes more complex; for some precise control applications you may prefer deterministic PWM rather than pseudo-random bursts.


Tuning guidelines & safety

Start with BURST_TARGET_OCCURRENCES = 2..4 and BURST_MAX_ITERS = 32..64.

Use an oscilloscope on OUT_PIN and a scope on a debug pin toggled at ISR entry/exit to measure ISR length and max iteration time.

If ISR time gets long, reduce burst target, or replace the parity function with a faster technique (lookup table or __builtin_parity if the compiler emits an efficient op).

If you need very high LFSR throughput, consider using a hardware timer + DMA-like approach (on a more capable MCU) rather than pushing the AVR ISR too hard.

Keep BALANCED_BURST_ENABLED as a compile-time flag or add a runtime safety (e.g. reduce bursts when Serial activity is detected).


Final notes

This implementation preserves the LUT-driven threshold idea but treats each ISR as a chance to produce several LFSR samples and force a small balanced quota of target-bit occurrences. It trades CPU time (ISR duration) for increased effective pseudo-random bit rate and smoother averaged PWM.

If you want, I can:

Add a small debug output pin to measure ISR start/end and per-iteration activity.

Replace the parity routine with a small 256-entry table for speed (faster but uses ~512 bytes).

Add an adaptive burst: increase bursts when system idle, reduce when serial/other interrupts busy.



Want me to add one of those (debug pin, parity table, or adaptive bursts) directly into the sketch?
