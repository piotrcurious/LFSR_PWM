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
  { 0x03cfu, 0xffa7u }, // 000: duty=0.0012 err=1.22e-03 P=13107 LC=256
  { 0x57b7u, 0xfee8u }, // 001: duty=0.0042 err=2.97e-04 P=3556 LC=256
  { 0x2ce3u, 0xfdf7u }, // 002: duty=0.0079 err=9.16e-05 P=65535 LC=255
  { 0xf60bu, 0xfd18u }, // 003: duty=0.0113 err=4.73e-04 P=32767 LC=256
  { 0xd753u, 0xfbe3u }, // 004: duty=0.0160 err=3.16e-04 P=19685 LC=256
  { 0xebbdu, 0xfab5u }, // 005: duty=0.0207 err=1.07e-03 P=64897 LC=256
  { 0x6006u, 0xf9f9u }, // 006: duty=0.0235 err=1.72e-05 P=30705 LC=256
  { 0x53aau, 0xf8c3u }, // 007: duty=0.0279 err=4.68e-04 P=7665 LC=256
  { 0xd8c1u, 0xf7bau }, // 008: duty=0.0323 err=9.51e-04 P=12282 LC=256
  { 0x97ecu, 0xf695u }, // 009: duty=0.0362 err=8.89e-04 P=5334 LC=256
  { 0x3b92u, 0xf5a8u }, // 010: duty=0.0398 err=5.76e-04 P=4599 LC=256
  { 0x8dc2u, 0xf4fcu }, // 011: duty=0.0430 err=9.26e-05 P=3810 LC=256
  { 0xba07u, 0xf380u }, // 012: duty=0.0483 err=1.24e-03 P=21845 LC=256
  { 0xe2d0u, 0xf062u }, // 013: duty=0.0591 err=8.07e-03 P=1016 LC=253
  { 0x0487u, 0xfbdeu }, // 014: duty=0.0548 err=6.33e-05 P=310 LC=251
  { 0xe2d0u, 0xf062u }, // 015: duty=0.0591 err=2.32e-04 P=1016 LC=253
  { 0xe019u, 0xf136u }, // 016: duty=0.0582 err=4.58e-03 P=16383 LC=249
  { 0x4f68u, 0xeb85u }, // 017: duty=0.0799 err=1.33e-02 P=7905 LC=240
  { 0x4f68u, 0xeb85u }, // 018: duty=0.0799 err=9.36e-03 P=7905 LC=240
  { 0x4f68u, 0xeb85u }, // 019: duty=0.0799 err=5.44e-03 P=7905 LC=240
  { 0x4f68u, 0xeb85u }, // 020: duty=0.0799 err=1.52e-03 P=7905 LC=240
  { 0x4f68u, 0xeb85u }, // 021: duty=0.0799 err=2.40e-03 P=7905 LC=240
  { 0x4f68u, 0xeb85u }, // 022: duty=0.0799 err=6.33e-03 P=7905 LC=240
  { 0x4f68u, 0xeb85u }, // 023: duty=0.0799 err=1.02e-02 P=7905 LC=240
  { 0x43ddu, 0xe6ddu }, // 024: duty=0.0944 err=3.07e-04 P=4681 LC=183
  { 0x7c14u, 0xe6c1u }, // 025: duty=0.0985 err=4.20e-04 P=7140 LC=206
  { 0x7c14u, 0xe6c1u }, // 026: duty=0.0985 err=3.50e-03 P=7140 LC=206
  { 0x7c14u, 0xe6c1u }, // 027: duty=0.0985 err=7.42e-03 P=7140 LC=206
  { 0x34cfu, 0xe523u }, // 028: duty=0.1049 err=4.88e-03 P=32766 LC=176
  { 0x34cfu, 0xe523u }, // 029: duty=0.1049 err=8.80e-03 P=32766 LC=176
  { 0x2e91u, 0xe40bu }, // 030: duty=0.1111 err=6.54e-03 P=2142 LC=167
  { 0x2e91u, 0xe40bu }, // 031: duty=0.1111 err=1.05e-02 P=2142 LC=167
  { 0x2450u, 0xc120u }, // 032: duty=0.1460 err=2.05e-02 P=315 LC=178
  { 0xe280u, 0xde6fu }, // 033: duty=0.1299 err=5.09e-04 P=254 LC=155
  { 0x0455u, 0x8aa0u }, // 034: duty=0.1260 err=7.35e-03 P=254 LC=181
  { 0x2450u, 0xc120u }, // 035: duty=0.1460 err=8.78e-03 P=315 LC=178
  { 0x2450u, 0xc120u }, // 036: duty=0.1460 err=4.86e-03 P=315 LC=178
  { 0x2450u, 0xc120u }, // 037: duty=0.1460 err=9.34e-04 P=315 LC=178
  { 0x2450u, 0xc120u }, // 038: duty=0.1460 err=2.99e-03 P=315 LC=178
  { 0x2450u, 0xc120u }, // 039: duty=0.1460 err=6.91e-03 P=315 LC=178
  { 0x2450u, 0xc120u }, // 040: duty=0.1460 err=1.08e-02 P=315 LC=178
  { 0x2450u, 0xc120u }, // 041: duty=0.1460 err=1.48e-02 P=315 LC=178
  { 0x400fu, 0xd554u }, // 042: duty=0.1667 err=1.96e-03 P=23622 LC=155
  { 0x400fu, 0xd554u }, // 043: duty=0.1667 err=1.96e-03 P=23622 LC=155
  { 0x400fu, 0xd554u }, // 044: duty=0.1667 err=5.88e-03 P=23622 LC=155
  { 0x400fu, 0xd554u }, // 045: duty=0.1667 err=9.80e-03 P=23622 LC=155
  { 0xe9d8u, 0xc049u }, // 046: duty=0.1922 err=1.18e-02 P=255 LC=162
  { 0xe9d8u, 0xc049u }, // 047: duty=0.1922 err=7.84e-03 P=255 LC=162
  { 0xe9d8u, 0xc049u }, // 048: duty=0.1922 err=3.92e-03 P=255 LC=162
  { 0xe9d8u, 0xc049u }, // 049: duty=0.1922 err=0.00e+00 P=255 LC=162
  { 0xdbe0u, 0xc0abu }, // 050: duty=0.2118 err=1.57e-02 P=255 LC=189
  { 0xdbe0u, 0xc0abu }, // 051: duty=0.2118 err=1.18e-02 P=255 LC=189
  { 0xdbe0u, 0xc0abu }, // 052: duty=0.2118 err=7.84e-03 P=255 LC=189
  { 0xdbe0u, 0xc0abu }, // 053: duty=0.2118 err=3.92e-03 P=255 LC=189
  { 0xdbe0u, 0xc0abu }, // 054: duty=0.2118 err=0.00e+00 P=255 LC=189
  { 0x0015u, 0x8008u }, // 055: duty=0.2402 err=2.45e-02 P=254 LC=226
  { 0x0015u, 0x8008u }, // 056: duty=0.2402 err=2.05e-02 P=254 LC=226
  { 0x5445u, 0x800au }, // 057: duty=0.2396 err=1.61e-02 P=434 LC=238
  { 0x5445u, 0x800au }, // 058: duty=0.2396 err=1.22e-02 P=434 LC=238
  { 0x5445u, 0x800au }, // 059: duty=0.2396 err=8.26e-03 P=434 LC=238
  { 0x5445u, 0x800au }, // 060: duty=0.2396 err=4.34e-03 P=434 LC=238
  { 0x5445u, 0x800au }, // 061: duty=0.2396 err=4.16e-04 P=434 LC=238
  { 0x0014u, 0x8008u }, // 062: duty=0.2441 err=9.57e-04 P=254 LC=228
  { 0x5511u, 0x8002u }, // 063: duty=0.2471 err=0.00e+00 P=510 LC=228
  { 0x6f40u, 0xbe80u }, // 064: duty=0.2524 err=1.47e-03 P=511 LC=207
  { 0x6f40u, 0xbe80u }, // 065: duty=0.2524 err=2.46e-03 P=511 LC=207
  { 0x5445u, 0x5540u }, // 066: duty=0.2627 err=3.85e-03 P=434 LC=205
  { 0x5445u, 0x5540u }, // 067: duty=0.2627 err=7.23e-05 P=434 LC=205
  { 0x1411u, 0x5515u }, // 068: duty=0.2667 err=0.00e+00 P=510 LC=200
  { 0x6f40u, 0xbe80u }, // 069: duty=0.2524 err=1.81e-02 P=511 LC=207
  { 0x4111u, 0x5500u }, // 070: duty=0.2788 err=4.29e-03 P=434 LC=182
  { 0x0003u, 0x8001u }, // 071: duty=0.3059 err=2.75e-02 P=255 LC=224
  { 0x0003u, 0x8001u }, // 072: duty=0.3059 err=2.35e-02 P=255 LC=224
  { 0x0003u, 0x8001u }, // 073: duty=0.3059 err=1.96e-02 P=255 LC=224
  { 0x0003u, 0x8001u }, // 074: duty=0.3059 err=1.57e-02 P=255 LC=224
  { 0x0003u, 0x8001u }, // 075: duty=0.3059 err=1.18e-02 P=255 LC=224
  { 0x0003u, 0x8001u }, // 076: duty=0.3059 err=7.84e-03 P=255 LC=224
  { 0x0003u, 0x8001u }, // 077: duty=0.3059 err=3.92e-03 P=255 LC=224
  { 0x0003u, 0x8001u }, // 078: duty=0.3059 err=0.00e+00 P=255 LC=224
  { 0x1120u, 0xc088u }, // 079: duty=0.2903 err=1.95e-02 P=341 LC=186
  { 0x1120u, 0xc088u }, // 080: duty=0.2903 err=2.34e-02 P=341 LC=186
  { 0x0715u, 0x7600u }, // 081: duty=0.3176 err=0.00e+00 P=255 LC=196
  { 0x28ddu, 0x8005u }, // 082: duty=0.3226 err=1.01e-03 P=279 LC=206
  { 0x0941u, 0x8092u }, // 083: duty=0.3241 err=1.37e-03 P=651 LC=146
  { 0xd45cu, 0x80c8u }, // 084: duty=0.3281 err=1.33e-03 P=381 LC=205
  { 0x0521u, 0x6d80u }, // 085: duty=0.3349 err=1.54e-03 P=651 LC=231
  { 0x1304u, 0x6d30u }, // 086: duty=0.3386 err=1.33e-03 P=381 LC=212
  { 0xd45cu, 0x6d12u }, // 087: duty=0.3438 err=2.66e-03 P=381 LC=196
  { 0x3149u, 0x8047u }, // 088: duty=0.3714 err=2.63e-02 P=315 LC=214
  { 0x3149u, 0x8047u }, // 089: duty=0.3714 err=2.24e-02 P=315 LC=214
  { 0x3149u, 0x8047u }, // 090: duty=0.3714 err=1.85e-02 P=315 LC=214
  { 0x3149u, 0x8047u }, // 091: duty=0.3714 err=1.46e-02 P=315 LC=214
  { 0x3149u, 0x8047u }, // 092: duty=0.3714 err=1.06e-02 P=315 LC=214
  { 0x3149u, 0x8047u }, // 093: duty=0.3714 err=6.72e-03 P=315 LC=214
  { 0x1547u, 0x8094u }, // 094: duty=0.3714 err=2.80e-03 P=315 LC=230
  { 0x1547u, 0x8094u }, // 095: duty=0.3714 err=1.12e-03 P=315 LC=230
  { 0xb6dau, 0x8004u }, // 096: duty=0.3846 err=8.14e-03 P=273 LC=226
  { 0xb6dau, 0x8004u }, // 097: duty=0.3846 err=4.22e-03 P=273 LC=226
  { 0xb6dau, 0x8004u }, // 098: duty=0.3846 err=3.02e-04 P=273 LC=226
  { 0xbc02u, 0x8051u }, // 099: duty=0.3900 err=1.79e-03 P=341 LC=236
  { 0xcccau, 0x8013u }, // 100: duty=0.3930 err=8.05e-04 P=341 LC=228
  { 0x0170u, 0x80b4u }, // 101: duty=0.3968 err=7.47e-04 P=315 LC=230
  { 0x6921u, 0x809bu }, // 102: duty=0.4000 err=0.00e+00 P=255 LC=220
  { 0x72e4u, 0x800bu }, // 103: duty=0.4066 err=2.67e-03 P=273 LC=227
  { 0x72e4u, 0x800bu }, // 104: duty=0.4066 err=1.25e-03 P=273 LC=227
  { 0x0203u, 0x810eu }, // 105: duty=0.4206 err=8.87e-03 P=504 LC=233
  { 0xa212u, 0x804au }, // 106: duty=0.4178 err=2.12e-03 P=438 LC=232
  { 0x0203u, 0x810eu }, // 107: duty=0.4206 err=1.03e-03 P=504 LC=233
  { 0x135du, 0x8075u }, // 108: duty=0.4222 err=1.31e-03 P=315 LC=228
  { 0xe9d8u, 0x8092u }, // 109: duty=0.4275 err=0.00e+00 P=255 LC=224
  { 0x5cfbu, 0x8087u }, // 110: duty=0.4311 err=2.88e-04 P=341 LC=228
  { 0x5df6u, 0x8066u }, // 111: duty=0.4366 err=1.35e-03 P=584 LC=237
  { 0x01d3u, 0x8080u }, // 112: duty=0.4392 err=0.00e+00 P=255 LC=231
  { 0xf4ceu, 0x8011u }, // 113: duty=0.4428 err=3.22e-04 P=1023 LC=239
  { 0x727au, 0x7bf5u }, // 114: duty=0.4471 err=0.00e+00 P=255 LC=236
  { 0x7f88u, 0x80b1u }, // 115: duty=0.4540 err=3.03e-03 P=511 LC=241
  { 0x7f88u, 0x80b1u }, // 116: duty=0.4540 err=8.90e-04 P=511 LC=241
  { 0x46c3u, 0x80acu }, // 117: duty=0.4593 err=5.17e-04 P=1365 LC=239
  { 0xc3c0u, 0x8099u }, // 118: duty=0.4604 err=2.33e-03 P=341 LC=245
  { 0xeb19u, 0x826au }, // 119: duty=0.4663 err=3.91e-04 P=1023 LC=239
  { 0x4184u, 0x8090u }, // 120: duty=0.4706 err=0.00e+00 P=765 LC=239
  { 0x8b78u, 0x7c00u }, // 121: duty=0.4762 err=1.68e-03 P=315 LC=243
  { 0x2995u, 0x7e3eu }, // 122: duty=0.4780 err=4.26e-04 P=1023 LC=240
  { 0x4fc2u, 0x7f0au }, // 123: duty=0.4835 err=1.16e-03 P=4095 LC=241
  { 0xdeb4u, 0x8101u }, // 124: duty=0.4860 err=2.83e-04 P=5461 LC=241
  { 0xb558u, 0x826au }, // 125: duty=0.4904 err=2.38e-04 P=7161 LC=242
  { 0xa424u, 0x8183u }, // 126: duty=0.4941 err=5.37e-05 P=7665 LC=241
  { 0x6e0eu, 0x7f52u }, // 127: duty=0.4982 err=1.45e-04 P=4681 LC=241
  { 0x3820u, 0x7f32u }, // 128: duty=0.5028 err=8.55e-04 P=1953 LC=245
  { 0xfaaeu, 0x7e27u }, // 129: duty=0.5060 err=7.00e-05 P=840 LC=243
  { 0x6cbcu, 0x7d83u }, // 130: duty=0.5092 err=6.06e-04 P=2555 LC=242
  { 0xd668u, 0x7c6bu }, // 131: duty=0.5140 err=2.53e-04 P=8191 LC=242
  { 0x1a6du, 0x7c04u }, // 132: duty=0.5175 err=1.87e-04 P=1260 LC=240
  { 0x30cfu, 0x7cf2u }, // 133: duty=0.5211 err=4.45e-04 P=4095 LC=240
  { 0x896cu, 0x7749u }, // 134: duty=0.5275 err=1.98e-03 P=910 LC=242
  { 0xe2d0u, 0x7831u }, // 135: duty=0.5285 err=8.68e-04 P=1016 LC=241
  { 0x599au, 0x7f7cu }, // 136: duty=0.5348 err=1.47e-03 P=819 LC=241
  { 0x4108u, 0x7ec2u }, // 137: duty=0.5385 err=1.21e-03 P=819 LC=241
  { 0xa8f1u, 0x7fa9u }, // 138: duty=0.5402 err=1.01e-03 P=585 LC=239
  { 0x88f9u, 0x7ebeu }, // 139: duty=0.5436 err=1.51e-03 P=585 LC=239
  { 0x6dbdu, 0x7ff4u }, // 140: duty=0.5337 err=1.53e-02 P=1023 LC=231
  { 0x6dbdu, 0x7ff4u }, // 141: duty=0.5337 err=1.92e-02 P=1023 LC=231
  { 0x3045u, 0x7f3au }, // 142: duty=0.5562 err=6.56e-04 P=1023 LC=235
  { 0xa74eu, 0x8333u }, // 143: duty=0.5751 err=1.43e-02 P=273 LC=221
  { 0xa74eu, 0x8333u }, // 144: duty=0.5751 err=1.04e-02 P=273 LC=221
  { 0xa74eu, 0x8333u }, // 145: duty=0.5751 err=6.46e-03 P=273 LC=221
  { 0x851eu, 0x7f87u }, // 146: duty=0.5725 err=0.00e+00 P=255 LC=223
  { 0xa74eu, 0x8333u }, // 147: duty=0.5751 err=1.38e-03 P=273 LC=221
  { 0x6dbdu, 0x7ff4u }, // 148: duty=0.5337 err=4.67e-02 P=1023 LC=231
  { 0x6dbdu, 0x7ff4u }, // 149: duty=0.5337 err=5.06e-02 P=1023 LC=231
  { 0xcb96u, 0x7ff4u }, // 150: duty=0.5897 err=1.51e-03 P=273 LC=227
  { 0xcb96u, 0x7ff4u }, // 151: duty=0.5897 err=2.41e-03 P=273 LC=227
  { 0xcb96u, 0x7ff4u }, // 152: duty=0.5897 err=6.33e-03 P=273 LC=227
  { 0xcb96u, 0x7ff4u }, // 153: duty=0.5897 err=1.03e-02 P=273 LC=227
  { 0xcb96u, 0x7ff4u }, // 154: duty=0.5897 err=1.42e-02 P=273 LC=227
  { 0xcb96u, 0x7ff4u }, // 155: duty=0.5897 err=1.81e-02 P=273 LC=227
  { 0x6db7u, 0x7ffbu }, // 156: duty=0.6117 err=4.31e-05 P=273 LC=226
  { 0x6db7u, 0x7ffbu }, // 157: duty=0.6117 err=3.96e-03 P=273 LC=226
  { 0x6db7u, 0x7ffbu }, // 158: duty=0.6117 err=7.89e-03 P=273 LC=226
  { 0x6db7u, 0x7ffbu }, // 159: duty=0.6117 err=1.18e-02 P=273 LC=226
  { 0x6db7u, 0x7ffbu }, // 160: duty=0.6117 err=1.57e-02 P=273 LC=226
  { 0x6db7u, 0x7ffbu }, // 161: duty=0.6117 err=1.97e-02 P=273 LC=226
  { 0x5544u, 0x1551u }, // 162: duty=0.6339 err=1.44e-03 P=254 LC=187
  { 0x5544u, 0x1551u }, // 163: duty=0.6339 err=5.36e-03 P=254 LC=187
  { 0x5544u, 0x1551u }, // 164: duty=0.6339 err=9.28e-03 P=254 LC=187
  { 0x5544u, 0x1551u }, // 165: duty=0.6339 err=1.32e-02 P=254 LC=187
  { 0x5544u, 0x1551u }, // 166: duty=0.6339 err=1.71e-02 P=254 LC=187
  { 0x5544u, 0x1551u }, // 167: duty=0.6339 err=2.10e-02 P=254 LC=187
  { 0x5544u, 0x1551u }, // 168: duty=0.6339 err=2.50e-02 P=254 LC=187
  { 0x82d6u, 0x560fu }, // 169: duty=0.6627 err=7.39e-05 P=4088 LC=133
  { 0x7040u, 0x4680u }, // 170: duty=0.6686 err=1.96e-03 P=341 LC=138
  { 0x7040u, 0x4680u }, // 171: duty=0.6686 err=1.97e-03 P=341 LC=138
  { 0x7040u, 0x45c8u }, // 172: duty=0.6745 err=2.30e-05 P=341 LC=138
  { 0x31d8u, 0x405bu }, // 173: duty=0.6941 err=1.57e-02 P=255 LC=161
  { 0x31d8u, 0x405bu }, // 174: duty=0.6941 err=1.18e-02 P=255 LC=161
  { 0x31d8u, 0x405bu }, // 175: duty=0.6941 err=7.84e-03 P=255 LC=161
  { 0x31d8u, 0x405bu }, // 176: duty=0.6941 err=3.92e-03 P=255 LC=161
  { 0x31d8u, 0x405bu }, // 177: duty=0.6941 err=0.00e+00 P=255 LC=161
  { 0x31d8u, 0x405bu }, // 178: duty=0.6941 err=3.92e-03 P=255 LC=161
  { 0x02b8u, 0x4080u }, // 179: duty=0.7024 err=4.20e-04 P=252 LC=158
  { 0x7858u, 0x3f75u }, // 180: duty=0.7059 err=0.00e+00 P=255 LC=173
  { 0x6c40u, 0x43f4u }, // 181: duty=0.7333 err=2.35e-02 P=420 LC=203
  { 0x5e40u, 0x41b7u }, // 182: duty=0.7143 err=5.60e-04 P=315 LC=179
  { 0x6c40u, 0x43f4u }, // 183: duty=0.7333 err=1.57e-02 P=420 LC=203
  { 0x6c40u, 0x43f4u }, // 184: duty=0.7333 err=1.18e-02 P=420 LC=203
  { 0x6c40u, 0x43f4u }, // 185: duty=0.7333 err=7.84e-03 P=420 LC=203
  { 0x6c40u, 0x43f4u }, // 186: duty=0.7333 err=3.92e-03 P=420 LC=203
  { 0x6c40u, 0x43f4u }, // 187: duty=0.7333 err=0.00e+00 P=420 LC=203
  { 0x1780u, 0x4168u }, // 188: duty=0.7421 err=4.81e-03 P=252 LC=200
  { 0x1780u, 0x4168u }, // 189: duty=0.7421 err=8.87e-04 P=252 LC=200
  { 0x1780u, 0x4168u }, // 190: duty=0.7421 err=3.03e-03 P=252 LC=200
  { 0x8cc0u, 0x3e00u }, // 191: duty=0.7554 err=6.36e-03 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 192: duty=0.7554 err=2.44e-03 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 193: duty=0.7554 err=1.48e-03 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 194: duty=0.7554 err=5.40e-03 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 195: duty=0.7554 err=9.32e-03 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 196: duty=0.7554 err=1.32e-02 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 197: duty=0.7554 err=1.72e-02 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 198: duty=0.7554 err=2.11e-02 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 199: duty=0.7554 err=2.50e-02 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 200: duty=0.7554 err=2.89e-02 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 201: duty=0.7554 err=3.29e-02 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 202: duty=0.7554 err=3.68e-02 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 203: duty=0.7554 err=4.07e-02 P=511 LC=208
  { 0x8cc0u, 0x3e00u }, // 204: duty=0.7554 err=4.46e-02 P=511 LC=208
  { 0x09b9u, 0x32b2u }, // 205: duty=0.8045 err=5.60e-04 P=3570 LC=133
  { 0x701cu, 0x312au }, // 206: duty=0.8080 err=1.29e-04 P=16383 LC=134
  { 0xe33bu, 0x3000u }, // 207: duty=0.8118 err=6.33e-05 P=3906 LC=134
  { 0x1f5du, 0x2e10u }, // 208: duty=0.8151 err=5.60e-04 P=5355 LC=135
  { 0x11e7u, 0x2e37u }, // 209: duty=0.8195 err=1.31e-04 P=47523 LC=133
  { 0x401eu, 0x2aa8u }, // 210: duty=0.8333 err=9.80e-03 P=15810 LC=169
  { 0x401eu, 0x2aa8u }, // 211: duty=0.8333 err=5.88e-03 P=15810 LC=169
  { 0x401eu, 0x2aa8u }, // 212: duty=0.8333 err=1.96e-03 P=15810 LC=169
  { 0x401eu, 0x2aa8u }, // 213: duty=0.8333 err=1.96e-03 P=15810 LC=169
  { 0x401eu, 0x2aa8u }, // 214: duty=0.8333 err=5.88e-03 P=15810 LC=169
  { 0x401eu, 0x2aa8u }, // 215: duty=0.8333 err=9.80e-03 P=15810 LC=169
  { 0x401eu, 0x2aa8u }, // 216: duty=0.8333 err=1.37e-02 P=15810 LC=169
  { 0x401eu, 0x2aa8u }, // 217: duty=0.8333 err=1.76e-02 P=15810 LC=169
  { 0x401eu, 0x2aa8u }, // 218: duty=0.8333 err=2.16e-02 P=15810 LC=169
  { 0xe201u, 0x1b30u }, // 219: duty=0.8573 err=1.54e-03 P=1023 LC=155
  { 0xc241u, 0x180bu }, // 220: duty=0.8645 err=1.77e-03 P=310 LC=175
  { 0xc241u, 0x16c0u }, // 221: duty=0.8677 err=1.08e-03 P=310 LC=175
  { 0xc241u, 0x16c0u }, // 222: duty=0.8677 err=2.85e-03 P=310 LC=175
  { 0xc241u, 0x16c0u }, // 223: duty=0.8677 err=6.77e-03 P=310 LC=175
  { 0xddecu, 0x1b58u }, // 224: duty=0.8931 err=1.46e-02 P=8191 LC=209
  { 0xddecu, 0x1b58u }, // 225: duty=0.8931 err=1.07e-02 P=8191 LC=209
  { 0xddecu, 0x1b58u }, // 226: duty=0.8931 err=6.78e-03 P=8191 LC=209
  { 0xddecu, 0x1b58u }, // 227: duty=0.8931 err=2.86e-03 P=8191 LC=209
  { 0x82fbu, 0x166fu }, // 228: duty=0.9123 err=1.82e-02 P=30660 LC=239
  { 0xddecu, 0x1b58u }, // 229: duty=0.8931 err=4.99e-03 P=8191 LC=209
  { 0x82fbu, 0x18bau }, // 230: duty=0.9034 err=1.43e-03 P=30660 LC=222
  { 0x82fbu, 0x166fu }, // 231: duty=0.9123 err=6.45e-03 P=30660 LC=239
  { 0x82fbu, 0x166fu }, // 232: duty=0.9123 err=2.52e-03 P=30660 LC=239
  { 0x82fbu, 0x166fu }, // 233: duty=0.9123 err=1.40e-03 P=30660 LC=239
  { 0x82fbu, 0x166fu }, // 234: duty=0.9123 err=5.32e-03 P=30660 LC=239
  { 0x82fbu, 0x166fu }, // 235: duty=0.9123 err=9.24e-03 P=30660 LC=239
  { 0x8ba5u, 0x1206u }, // 236: duty=0.9296 err=4.10e-03 P=55335 LC=236
  { 0x82fbu, 0x166fu }, // 237: duty=0.9123 err=1.71e-02 P=30660 LC=239
  { 0x2223u, 0x0facu }, // 238: duty=0.9333 err=0.00e+00 P=240 LC=208
  { 0x006fu, 0x1000u }, // 239: duty=0.9255 err=1.18e-02 P=255 LC=233
  { 0x0586u, 0x0800u }, // 240: duty=0.9498 err=8.60e-03 P=438 LC=242
  { 0x0586u, 0x0800u }, // 241: duty=0.9498 err=4.67e-03 P=438 LC=242
  { 0x8885u, 0x0c86u }, // 242: duty=0.9504 err=1.37e-03 P=3810 LC=247
  { 0x9a5eu, 0x0820u }, // 243: duty=0.9527 err=2.53e-04 P=465 LC=250
  { 0xe276u, 0x0af0u }, // 244: duty=0.9572 err=3.49e-04 P=16383 LC=253
  { 0x025cu, 0x0943u }, // 245: duty=0.9618 err=1.06e-03 P=1022 LC=251
  { 0xfaeeu, 0x0896u }, // 246: duty=0.9648 err=1.04e-04 P=341 LC=253
  { 0x3656u, 0x07ecu }, // 247: duty=0.9690 err=4.04e-04 P=24573 LC=253
  { 0xd716u, 0x04f2u }, // 248: duty=0.9727 err=1.87e-04 P=5355 LC=252
  { 0xc66eu, 0x0620u }, // 249: duty=0.9760 err=4.78e-04 P=14329 LC=254
  { 0x1d8bu, 0x04fbu }, // 250: duty=0.9805 err=1.27e-04 P=57337 LC=253
  { 0xdf18u, 0x03f6u }, // 251: duty=0.9844 err=5.36e-05 P=4094 LC=254
  { 0x84a6u, 0x0386u }, // 252: duty=0.9883 err=1.51e-05 P=4681 LC=254
  { 0x23e4u, 0x01cbu }, // 253: duty=0.9922 err=1.53e-05 P=1022 LC=254
  { 0x8e0du, 0x00dau }, // 254: duty=0.9958 err=3.18e-04 P=3066 LC=253
  { 0xd638u, 0x0061u }, // 255: duty=0.9988 err=1.17e-03 P=2555 LC=253
};

/* Example Arduino usage:

uint16_t lfsr_state;
uint16_t lfsr_mask;
uint16_t lfsr_threshold;

void setup_pwm(uint8_t level) {
  lfsr_mask = pgm_read_word(&lfsr_pwm_lut[level].mask);
  lfsr_threshold = pgm_read_word(&lfsr_pwm_lut[level].seed);
  lfsr_state = lfsr_threshold;  // Start at threshold value
}

void pwm_update() {
  // Output based on comparison
  digitalWrite(PWM_PIN, (lfsr_state > lfsr_threshold) ? HIGH : LOW);
  
  // Advance LFSR state
  uint16_t feedback = __builtin_parity(lfsr_state & lfsr_mask) & 1;
  lfsr_state = (lfsr_state >> 1) | (feedback << 15);
}
*/
