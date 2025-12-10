#!/bin/bash
./lfsr_pwm_algebraic --n 16 --trunc-bits 16 --res 256 --topk 8 --threads 0 \
  --period-penalty 1e-3 --min-period 3 --reuse-limit 1 \
  --out lfsr_pwm.c --arduino-out arduino_lut.h
