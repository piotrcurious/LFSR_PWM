// Arduino LFSR PWM LUT: mask, seed (16-bit)
// PWM output: (state > seed) ? HIGH : LOW
// Seed doubles as both starting state AND threshold
#include <stdint.h>
#if defined(__AVR__)
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_word(addr) (*(addr))
#define pgm_read_dword(addr) (*(addr))
#endif

const struct { uint16_t mask; uint16_t seed; } lfsr_pwm_lut[] PROGMEM = {
  { 0x81f5u, 0x6db6u }, // 000: duty=0.6667 err=6.67e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 001: duty=0.6667 err=6.63e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 002: duty=0.6667 err=6.59e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 003: duty=0.6667 err=6.55e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 004: duty=0.6667 err=6.51e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 005: duty=0.6667 err=6.47e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 006: duty=0.6667 err=6.43e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 007: duty=0.6667 err=6.39e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 008: duty=0.6667 err=6.35e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 009: duty=0.6667 err=6.31e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 010: duty=0.6667 err=6.27e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 011: duty=0.6667 err=6.24e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 012: duty=0.6667 err=6.20e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 013: duty=0.6667 err=6.16e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 014: duty=0.6667 err=6.12e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 015: duty=0.6667 err=6.08e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 016: duty=0.6667 err=6.04e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 017: duty=0.6667 err=6.00e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 018: duty=0.6667 err=5.96e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 019: duty=0.6667 err=5.92e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 020: duty=0.6667 err=5.88e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 021: duty=0.6667 err=5.84e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 022: duty=0.6667 err=5.80e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 023: duty=0.6667 err=5.76e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 024: duty=0.6667 err=5.73e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 025: duty=0.6667 err=5.69e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 026: duty=0.6667 err=5.65e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 027: duty=0.6667 err=5.61e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 028: duty=0.6667 err=5.57e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 029: duty=0.6667 err=5.53e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 030: duty=0.6667 err=5.49e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 031: duty=0.6667 err=5.45e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 032: duty=0.6667 err=5.41e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 033: duty=0.6667 err=5.37e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 034: duty=0.6667 err=5.33e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 035: duty=0.6667 err=5.29e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 036: duty=0.6667 err=5.25e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 037: duty=0.6667 err=5.22e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 038: duty=0.6667 err=5.18e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 039: duty=0.6667 err=5.14e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 040: duty=0.6667 err=5.10e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 041: duty=0.6667 err=5.06e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 042: duty=0.6667 err=5.02e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 043: duty=0.6667 err=4.98e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 044: duty=0.6667 err=4.94e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 045: duty=0.6667 err=4.90e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 046: duty=0.6667 err=4.86e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 047: duty=0.6667 err=4.82e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 048: duty=0.6667 err=4.78e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 049: duty=0.6667 err=4.75e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 050: duty=0.6667 err=4.71e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 051: duty=0.6667 err=4.67e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 052: duty=0.6667 err=4.63e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 053: duty=0.6667 err=4.59e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 054: duty=0.6667 err=4.55e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 055: duty=0.6667 err=4.51e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 056: duty=0.6667 err=4.47e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 057: duty=0.6667 err=4.43e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 058: duty=0.6667 err=4.39e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 059: duty=0.6667 err=4.35e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 060: duty=0.6667 err=4.31e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 061: duty=0.6667 err=4.27e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 062: duty=0.6667 err=4.24e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 063: duty=0.6667 err=4.20e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 064: duty=0.6667 err=4.16e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 065: duty=0.6667 err=4.12e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 066: duty=0.6667 err=4.08e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 067: duty=0.6667 err=4.04e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 068: duty=0.6667 err=4.00e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 069: duty=0.6667 err=3.96e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 070: duty=0.6667 err=3.92e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 071: duty=0.6667 err=3.88e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 072: duty=0.6667 err=3.84e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 073: duty=0.6667 err=3.80e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 074: duty=0.6667 err=3.76e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 075: duty=0.6667 err=3.73e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 076: duty=0.6667 err=3.69e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 077: duty=0.6667 err=3.65e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 078: duty=0.6667 err=3.61e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 079: duty=0.6667 err=3.57e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 080: duty=0.6667 err=3.53e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 081: duty=0.6667 err=3.49e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 082: duty=0.6667 err=3.45e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 083: duty=0.6667 err=3.41e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 084: duty=0.6667 err=3.37e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 085: duty=0.6667 err=3.33e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 086: duty=0.6667 err=3.29e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 087: duty=0.6667 err=3.25e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 088: duty=0.6667 err=3.22e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 089: duty=0.6667 err=3.18e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 090: duty=0.6667 err=3.14e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 091: duty=0.6667 err=3.10e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 092: duty=0.6667 err=3.06e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 093: duty=0.6667 err=3.02e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 094: duty=0.6667 err=2.98e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 095: duty=0.6667 err=2.94e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 096: duty=0.6667 err=2.90e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 097: duty=0.6667 err=2.86e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 098: duty=0.6667 err=2.82e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 099: duty=0.6667 err=2.78e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 100: duty=0.6667 err=2.75e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 101: duty=0.6667 err=2.71e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 102: duty=0.6667 err=2.67e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 103: duty=0.6667 err=2.63e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 104: duty=0.6667 err=2.59e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 105: duty=0.6667 err=2.55e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 106: duty=0.6667 err=2.51e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 107: duty=0.6667 err=2.47e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 108: duty=0.6667 err=2.43e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 109: duty=0.6667 err=2.39e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 110: duty=0.6667 err=2.35e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 111: duty=0.6667 err=2.31e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 112: duty=0.6667 err=2.27e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 113: duty=0.6667 err=2.24e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 114: duty=0.6667 err=2.20e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 115: duty=0.6667 err=2.16e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 116: duty=0.6667 err=2.12e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 117: duty=0.6667 err=2.08e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 118: duty=0.6667 err=2.04e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 119: duty=0.6667 err=2.00e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 120: duty=0.6667 err=1.96e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 121: duty=0.6667 err=1.92e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 122: duty=0.6667 err=1.88e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 123: duty=0.6667 err=1.84e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 124: duty=0.6667 err=1.80e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 125: duty=0.6667 err=1.76e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 126: duty=0.6667 err=1.73e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 127: duty=0.6667 err=1.69e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 128: duty=0.6667 err=1.65e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 129: duty=0.6667 err=1.61e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 130: duty=0.6667 err=1.57e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 131: duty=0.6667 err=1.53e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 132: duty=0.6667 err=1.49e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 133: duty=0.6667 err=1.45e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 134: duty=0.6667 err=1.41e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 135: duty=0.6667 err=1.37e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 136: duty=0.6667 err=1.33e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 137: duty=0.6667 err=1.29e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 138: duty=0.6667 err=1.25e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 139: duty=0.6667 err=1.22e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 140: duty=0.6667 err=1.18e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 141: duty=0.6667 err=1.14e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 142: duty=0.6667 err=1.10e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 143: duty=0.6667 err=1.06e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 144: duty=0.6667 err=1.02e-01 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 145: duty=0.6667 err=9.80e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 146: duty=0.6667 err=9.41e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 147: duty=0.6667 err=9.02e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 148: duty=0.6667 err=8.63e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 149: duty=0.6667 err=8.24e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 150: duty=0.6667 err=7.84e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 151: duty=0.6667 err=7.45e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 152: duty=0.6667 err=7.06e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 153: duty=0.6667 err=6.67e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 154: duty=0.6667 err=6.27e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 155: duty=0.6667 err=5.88e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 156: duty=0.6667 err=5.49e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 157: duty=0.6667 err=5.10e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 158: duty=0.6667 err=4.71e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 159: duty=0.6667 err=4.31e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 160: duty=0.6667 err=3.92e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 161: duty=0.6667 err=3.53e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 162: duty=0.6667 err=3.14e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 163: duty=0.6667 err=2.75e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 164: duty=0.6667 err=2.35e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 165: duty=0.6667 err=1.96e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 166: duty=0.6667 err=1.57e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 167: duty=0.6667 err=1.18e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 168: duty=0.6667 err=7.84e-03 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 169: duty=0.6667 err=3.92e-03 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 170: duty=0.6667 err=0.00e+00 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 171: duty=0.6667 err=3.92e-03 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 172: duty=0.6667 err=7.84e-03 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 173: duty=0.6667 err=1.18e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 174: duty=0.6667 err=1.57e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 175: duty=0.6667 err=1.96e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 176: duty=0.6667 err=2.35e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 177: duty=0.6667 err=2.75e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 178: duty=0.6667 err=3.14e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 179: duty=0.6667 err=3.53e-02 P=3 LC=16
  { 0x81f5u, 0x6db6u }, // 180: duty=0.6667 err=3.92e-02 P=3 LC=16
  { 0xfb93u, 0x3333u }, // 181: duty=0.7500 err=4.02e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 182: duty=0.7500 err=3.63e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 183: duty=0.7500 err=3.24e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 184: duty=0.7500 err=2.84e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 185: duty=0.7500 err=2.45e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 186: duty=0.7500 err=2.06e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 187: duty=0.7500 err=1.67e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 188: duty=0.7500 err=1.27e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 189: duty=0.7500 err=8.82e-03 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 190: duty=0.7500 err=4.90e-03 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 191: duty=0.7500 err=9.80e-04 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 192: duty=0.7500 err=2.94e-03 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 193: duty=0.7500 err=6.86e-03 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 194: duty=0.7500 err=1.08e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 195: duty=0.7500 err=1.47e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 196: duty=0.7500 err=1.86e-02 P=4 LC=16
  { 0xfb93u, 0x3333u }, // 197: duty=0.7500 err=2.25e-02 P=4 LC=16
  { 0xbc51u, 0x18c6u }, // 198: duty=0.8000 err=2.35e-02 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 199: duty=0.8000 err=1.96e-02 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 200: duty=0.8000 err=1.57e-02 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 201: duty=0.8000 err=1.18e-02 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 202: duty=0.8000 err=7.84e-03 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 203: duty=0.8000 err=3.92e-03 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 204: duty=0.8000 err=0.00e+00 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 205: duty=0.8000 err=3.92e-03 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 206: duty=0.8000 err=7.84e-03 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 207: duty=0.8000 err=1.18e-02 P=5 LC=16
  { 0xbc51u, 0x18c6u }, // 208: duty=0.8000 err=1.57e-02 P=5 LC=16
  { 0x81f5u, 0x1451u }, // 209: duty=0.8333 err=1.37e-02 P=6 LC=16
  { 0x81f5u, 0x1451u }, // 210: duty=0.8333 err=9.80e-03 P=6 LC=16
  { 0x81f5u, 0x1451u }, // 211: duty=0.8333 err=5.88e-03 P=6 LC=16
  { 0x81f5u, 0x1451u }, // 212: duty=0.8333 err=1.96e-03 P=6 LC=16
  { 0x81f5u, 0x1451u }, // 213: duty=0.8333 err=1.96e-03 P=6 LC=16
  { 0x81f5u, 0x1451u }, // 214: duty=0.8333 err=5.88e-03 P=6 LC=16
  { 0x81f5u, 0x1451u }, // 215: duty=0.8333 err=9.80e-03 P=6 LC=16
  { 0xd799u, 0x3a74u }, // 216: duty=0.8571 err=1.01e-02 P=7 LC=16
  { 0xd799u, 0x3a74u }, // 217: duty=0.8571 err=6.16e-03 P=7 LC=16
  { 0xd799u, 0x3a74u }, // 218: duty=0.8571 err=2.24e-03 P=7 LC=16
  { 0xd799u, 0x3a74u }, // 219: duty=0.8571 err=1.68e-03 P=7 LC=16
  { 0xd799u, 0x3a74u }, // 220: duty=0.8571 err=5.60e-03 P=7 LC=16
  { 0x25bdu, 0x0f0fu }, // 221: duty=0.8750 err=8.33e-03 P=8 LC=16
  { 0x25bdu, 0x0f0fu }, // 222: duty=0.8750 err=4.41e-03 P=8 LC=16
  { 0x25bdu, 0x0f0fu }, // 223: duty=0.8750 err=4.90e-04 P=8 LC=16
  { 0x25bdu, 0x0f0fu }, // 224: duty=0.8750 err=3.43e-03 P=8 LC=16
  { 0xdb5bu, 0x0381u }, // 225: duty=0.8889 err=6.54e-03 P=9 LC=16
  { 0xdb5bu, 0x0381u }, // 226: duty=0.8889 err=2.61e-03 P=9 LC=16
  { 0xdb5bu, 0x0381u }, // 227: duty=0.8889 err=1.31e-03 P=9 LC=16
  { 0xdb5bu, 0x0381u }, // 228: duty=0.8889 err=5.23e-03 P=9 LC=16
  { 0xc8d7u, 0x07c1u }, // 229: duty=0.9000 err=1.96e-03 P=10 LC=16
  { 0xc8d7u, 0x07c1u }, // 230: duty=0.9000 err=1.96e-03 P=10 LC=16
  { 0x77d1u, 0x0060u }, // 231: duty=0.9091 err=3.21e-03 P=11 LC=16
  { 0x77d1u, 0x0060u }, // 232: duty=0.9091 err=7.13e-04 P=11 LC=16
  { 0xb0cdu, 0x0770u }, // 233: duty=0.9167 err=2.94e-03 P=12 LC=16
  { 0xb0cdu, 0x0770u }, // 234: duty=0.9167 err=9.80e-04 P=12 LC=16
  { 0x600bu, 0x0008u }, // 235: duty=0.9231 err=1.51e-03 P=13 LC=16
  { 0x600bu, 0x0008u }, // 236: duty=0.9231 err=2.41e-03 P=13 LC=16
  { 0xc949u, 0x03ccu }, // 237: duty=0.9286 err=8.40e-04 P=14 LC=16
  { 0xba49u, 0x1eb2u }, // 238: duty=0.9333 err=0.00e+00 P=15 LC=16
  { 0xb8b9u, 0x00ffu }, // 239: duty=0.9375 err=2.45e-04 P=16 LC=16
  { 0xeb6bu, 0x009cu }, // 240: duty=0.9412 err=0.00e+00 P=17 LC=16
  { 0xf2f9u, 0x007fu }, // 241: duty=0.9444 err=6.54e-04 P=18 LC=16
  { 0xc8d7u, 0x02bfu }, // 242: duty=0.9500 err=9.80e-04 P=20 LC=16
  { 0x21a5u, 0x08cau }, // 243: duty=0.9524 err=5.60e-04 P=21 LC=16
  { 0xd33fu, 0x0015u }, // 244: duty=0.9565 err=3.41e-04 P=23 LC=16
  { 0x2009u, 0x0007u }, // 245: duty=0.9615 err=7.54e-04 P=26 LC=16
  { 0x377du, 0x075bu }, // 246: duty=0.9643 err=4.20e-04 P=28 LC=16
  { 0xa56du, 0x0722u }, // 247: duty=0.9677 err=8.86e-04 P=31 LC=16
  { 0xc2e1u, 0x0019u }, // 248: duty=0.9722 err=3.27e-04 P=36 LC=16
  { 0xcaa7u, 0x0002u }, // 249: duty=0.9767 err=2.74e-04 P=43 LC=16
  { 0x52c9u, 0x00a2u }, // 250: duty=0.9804 err=0.00e+00 P=51 LC=16
  { 0xfb93u, 0x028eu }, // 251: duty=0.9841 err=1.87e-04 P=63 LC=16
  { 0xba49u, 0x0169u }, // 252: duty=0.9882 err=0.00e+00 P=85 LC=16
  { 0xfb93u, 0x0148u }, // 253: duty=0.9921 err=3.09e-05 P=127 LC=16
  { 0xba49u, 0x001au }, // 254: duty=0.9961 err=0.00e+00 P=255 LC=16
  { 0x6e9bu, 0x0001u }, // 255: duty=1.0000 err=1.53e-05 P=65535 LC=16
};

/* Example Arduino usage:

uint16_t lfsr_state;
uint16_t lfsr_mask;
uint16_t lfsr_threshold;

// Portable parity function for AVR
inline uint8_t parity16(uint16_t x) {
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return x & 1;
}

void setup_pwm(uint8_t level) {
  lfsr_mask = pgm_read_word(&lfsr_pwm_lut[level].mask);
  lfsr_threshold = pgm_read_word(&lfsr_pwm_lut[level].seed);
  lfsr_state = lfsr_threshold;  // Start at threshold value
}

void pwm_update() {
  // Output based on comparison (Fibonacci LFSR)
  digitalWrite(PWM_PIN, (lfsr_state > lfsr_threshold) ? HIGH : LOW);
  
  // Advance LFSR state: feedback = parity(state & mask)
  uint16_t feedback = parity16(lfsr_state & lfsr_mask);
  lfsr_state = (lfsr_state >> 1) | (feedback << 15);
}
*/
