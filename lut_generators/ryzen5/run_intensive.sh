#./lfsr_pwm_algebraic --n 16 --scan 65535 --res 256 --out lfsr_pwm.c --arduino-out arduino_lut.h --iters 16 --threads 0
./lfsr_pwm_algebraic --n 16 --scan 65535 --res 256 --out lfsr_pwm.c --topk 32 --iters 16  --arduino-out arduino_lut.h
