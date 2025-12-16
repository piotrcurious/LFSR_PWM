#!/bin/bash
g++ -std=c++17 -O3 -march=native -flto -funroll-loops -pthread -o adv_vizout_spectral_01 adv_vizout_spectral_01.cpp
