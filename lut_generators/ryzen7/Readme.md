How these changes map to cryptanalysis methods

Fast sampling is analogous to using statistical / correlation tests to quickly eliminate masks that are unlikely to meet a target (cheap hypothesis testing).

Local hill-climb / simulated annealing mirrors cryptanalytic local-search / key-search improvements where you flip taps/tweak structure to improve correlation or linear complexity measures.

Keeping LC (linear complexity) as a tie-breaker is consistent with preferring richer LFSR polynomials that have more complex behavior.

Tuning recommendations

Start with conservative sampling: --sample-seeds 8 --sample-walk 128 --cull-factor 2.0. This will be fast and avoid many full decompositions.

If you need to be more aggressive at finding exotic masks, increase --iters and --topk and allow local refinement with --sa-iters 500.

If you have large n (≥20) keep sampling small or run fewer iterations because full decompositions are costly.

Would you like one of the following next?

I can add a mode that searches specifically for masks with a desired period (for example prefer primitive polynomials / maximal-length cycles) — that helps produce balanced sequences and fewer repeats.

I can add a reusable CSV debug output listing top-K candidates for each level (makes tuning thresholds easy).

I can tune the sampling/SA hyperparameters for your target hardware and n value if you tell me typical run time you want (rough budget) and typical n.

Pick one and I’ll produce the small change (or the CSV debug) right away.
