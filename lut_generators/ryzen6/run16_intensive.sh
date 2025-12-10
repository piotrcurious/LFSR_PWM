#!/bin/bash
./lfsr_pwm_algebraic --iter 16 --n 16 --trunc-bits 16 --res 256 --topk 256 --threads 0 \
  --period-penalty 1e-2 --min-period 4 --reuse-limit 1 \
  --out lfsr_pwm.c --arduino-out arduino_lut.h
