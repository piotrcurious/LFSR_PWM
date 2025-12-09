#!/bin/bash
g++ -std=c++17 -O3 -march=znver4 -mtune=znver4 -flto -funroll-loops -fno-exceptions -fomit-frame-pointer -o lfsr_pwm_algebraic lfsr_pwm_algebraic.cpp
