#!/bin/bash
./lfsr_pwm_algebraic --iter 32 --n 16 --scan 65535 --trunc-bits 16 --res 256 --topk 512 --threads 0 \
  --period-penalty 1e-2 --min-period 2 --seed-variants 256 --reuse-limit 16 \
  --out lfsr_pwm.c --arduino-out arduino_lut.h
