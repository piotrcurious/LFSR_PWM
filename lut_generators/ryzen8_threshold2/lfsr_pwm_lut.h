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
  { 0x502bu, 0xfff6u }, // 000: duty=0.0000 err=4.58e-05 P=21845 LC=16381
  { 0x994eu, 0xfefeu }, // 001: duty=0.0039 err=1.52e-05 P=32767 LC=8200
  { 0x4217u, 0xfdfcu }, // 002: duty=0.0078 err=5.20e-06 P=57337 LC=8200
  { 0xf40du, 0xfcfbu }, // 003: duty=0.0118 err=7.80e-06 P=57337 LC=8199
  { 0x867cu, 0xfbfbu }, // 004: duty=0.0157 err=7.18e-07 P=16383 LC=9901
  { 0xebadu, 0xfaf9u }, // 005: duty=0.0196 err=1.50e-05 P=32767 LC=8199
  { 0x4b91u, 0xf9f6u }, // 006: duty=0.0235 err=3.59e-07 P=32767 LC=8199
  { 0xf027u, 0xf802u }, // 007: duty=0.0275 err=1.65e-05 P=16383 LC=10020
  { 0x6cc2u, 0xf7f5u }, // 008: duty=0.0314 err=3.35e-06 P=16382 LC=12540
  { 0xc783u, 0xf6f6u }, // 009: duty=0.0353 err=5.95e-06 P=57337 LC=8199
  { 0xa667u, 0xf5f5u }, // 010: duty=0.0392 err=1.44e-05 P=24573 LC=8199
  { 0x03a3u, 0xf4f4u }, // 011: duty=0.0431 err=0.00e+00 P=65535 LC=8198
  { 0x238cu, 0xf3f3u }, // 012: duty=0.0471 err=2.15e-06 P=16383 LC=9428
  { 0x4685u, 0xf2f2u }, // 013: duty=0.0510 err=1.09e-06 P=57337 LC=8199
  { 0x4129u, 0xf1f0u }, // 014: duty=0.0549 err=1.36e-05 P=28644 LC=8199
  { 0xcfebu, 0xf006u }, // 015: duty=0.0588 err=1.37e-05 P=21483 LC=12724
  { 0x0435u, 0xefecu }, // 016: duty=0.0627 err=6.97e-06 P=27559 LC=14460
  { 0xe6dfu, 0xeeefu }, // 017: duty=0.0667 err=7.35e-06 P=63457 LC=8199
  { 0x8f01u, 0xededu }, // 018: duty=0.0706 err=5.01e-06 P=63457 LC=8198
  { 0x4dc5u, 0xececu }, // 019: duty=0.0745 err=7.13e-06 P=64897 LC=8199
  { 0x0ba6u, 0xebecu }, // 020: duty=0.0784 err=2.72e-05 P=15841 LC=8279
  { 0x2e17u, 0xeaa6u }, // 021: duty=0.0824 err=0.00e+00 P=21845 LC=8200
  { 0xff97u, 0xe9e8u }, // 022: duty=0.0863 err=1.01e-05 P=24564 LC=8199
  { 0x69edu, 0xe809u }, // 023: duty=0.0902 err=1.94e-05 P=16383 LC=8706
  { 0x906bu, 0xe7e7u }, // 024: duty=0.0941 err=4.31e-06 P=32766 LC=10825
  { 0x29e9u, 0xe80au }, // 025: duty=0.0980 err=1.08e-05 P=16383 LC=9901
  { 0x538bu, 0xe5e4u }, // 026: duty=0.1020 err=1.98e-06 P=63457 LC=8200
  { 0xcf96u, 0xe4e4u }, // 027: duty=0.1059 err=1.36e-05 P=32767 LC=8199
  { 0x7386u, 0xe3e2u }, // 028: duty=0.1098 err=1.68e-06 P=32767 LC=8198
  { 0x223au, 0xe2e3u }, // 029: duty=0.1137 err=1.35e-05 P=32767 LC=8201
  { 0x2fdfu, 0xe1a6u }, // 030: duty=0.1176 err=0.00e+00 P=21845 LC=8201
  { 0xd075u, 0xdfe2u }, // 031: duty=0.1216 err=1.59e-05 P=21483 LC=9425
  { 0xf572u, 0xdfdeu }, // 032: duty=0.1255 err=1.34e-05 P=24573 LC=15450
  { 0x74b1u, 0xdfeeu }, // 033: duty=0.1294 err=0.00e+00 P=21845 LC=9718
  { 0xcb07u, 0xdddcu }, // 034: duty=0.1333 err=1.16e-06 P=57337 LC=8198
  { 0x3479u, 0xdcdbu }, // 035: duty=0.1373 err=3.40e-06 P=63457 LC=8199
  { 0xb09fu, 0xdbd9u }, // 036: duty=0.1412 err=2.15e-06 P=32767 LC=8200
  { 0xe1b1u, 0xdad9u }, // 037: duty=0.1451 err=6.59e-06 P=64897 LC=8199
  { 0xf6cfu, 0xd9dau }, // 038: duty=0.1490 err=5.31e-06 P=63457 LC=8199
  { 0x6f27u, 0xd8d8u }, // 039: duty=0.1529 err=6.53e-06 P=64897 LC=8200
  { 0xc219u, 0xd7fdu }, // 040: duty=0.1568 err=1.81e-05 P=18445 LC=10060
  { 0xfd37u, 0xd7fcu }, // 041: duty=0.1608 err=7.90e-06 P=16383 LC=9919
  { 0x723au, 0xd5d5u }, // 042: duty=0.1647 err=2.51e-06 P=32767 LC=8199
  { 0x01e1u, 0xd4d4u }, // 043: duty=0.1686 err=6.71e-06 P=24564 LC=8200
  { 0x1ceau, 0xd3d2u }, // 044: duty=0.1726 err=2.63e-06 P=32767 LC=8199
  { 0xb5b9u, 0xd2d2u }, // 045: duty=0.1765 err=0.00e+00 P=65535 LC=8198
  { 0x8d49u, 0xd1d0u }, // 046: duty=0.1804 err=9.10e-06 P=24573 LC=8200
  { 0x282du, 0xd00eu }, // 047: duty=0.1843 err=1.53e-05 P=21845 LC=12107
  { 0xdbbbu, 0xcffdu }, // 048: duty=0.1882 err=0.00e+00 P=18445 LC=13996
  { 0xc45bu, 0xcfedu }, // 049: duty=0.1922 err=4.93e-06 P=21483 LC=13068
  { 0x82fau, 0xcdccu }, // 050: duty=0.1961 err=2.99e-06 P=32767 LC=8198
  { 0xcd76u, 0xcccdu }, // 051: duty=0.2000 err=1.22e-05 P=32767 LC=8199
  { 0xbb43u, 0xcbc9u }, // 052: duty=0.2039 err=3.11e-06 P=32767 LC=8200
  { 0x4eb1u, 0xcacau }, // 053: duty=0.2078 err=0.00e+00 P=65535 LC=8199
  { 0x3cafu, 0xc9cau }, // 054: duty=0.2118 err=6.44e-06 P=47523 LC=8201
  { 0x9d15u, 0xc7f2u }, // 055: duty=0.2157 err=2.51e-05 P=16383 LC=9668
  { 0x8b69u, 0xc7fcu }, // 056: duty=0.2196 err=1.01e-05 P=16383 LC=9903
  { 0xec1fu, 0xc7f5u }, // 057: duty=0.2235 err=5.03e-06 P=16383 LC=9975
  { 0xee8au, 0xc5c3u }, // 058: duty=0.2274 err=2.01e-05 P=16383 LC=8200
  { 0xf02du, 0xc4c4u }, // 059: duty=0.2314 err=0.00e+00 P=65535 LC=8197
  { 0x0d29u, 0xc3c2u }, // 060: duty=0.2353 err=1.81e-06 P=64897 LC=8199
  { 0x8b7du, 0xc2c1u }, // 061: duty=0.2392 err=1.42e-06 P=63457 LC=8198
  { 0xda0du, 0xc010u }, // 062: duty=0.2431 err=1.94e-05 P=16383 LC=13000
  { 0x6c7bu, 0xc012u }, // 063: duty=0.2471 err=2.66e-05 P=16383 LC=15501
  { 0xb537u, 0xc006u }, // 064: duty=0.2510 err=1.15e-05 P=16383 LC=13513
  { 0xb4cfu, 0xc012u }, // 065: duty=0.2549 err=3.59e-06 P=16383 LC=13824
  { 0xfe1du, 0xbdbeu }, // 066: duty=0.2588 err=0.00e+00 P=32385 LC=8199
  { 0xaf81u, 0xbcbcu }, // 067: duty=0.2627 err=2.74e-07 P=57337 LC=8200
  { 0x84afu, 0xbbbbu }, // 068: duty=0.2667 err=0.00e+00 P=65535 LC=8200
  { 0x7c37u, 0xbabau }, // 069: duty=0.2706 err=0.00e+00 P=65535 LC=8198
  { 0xa44du, 0xb9b7u }, // 070: duty=0.2745 err=4.19e-06 P=32767 LC=8200
  { 0x21edu, 0xb7fcu }, // 071: duty=0.2785 err=2.80e-05 P=16383 LC=9634
  { 0x6b43u, 0xb7fdu }, // 072: duty=0.2824 err=1.29e-05 P=16383 LC=10065
  { 0x1133u, 0xb7eau }, // 073: duty=0.2863 err=1.64e-06 P=21483 LC=9435
  { 0xbf91u, 0xb5b4u }, // 074: duty=0.2902 err=4.79e-07 P=57337 LC=8199
  { 0x05ddu, 0xb4b4u }, // 075: duty=0.2941 err=0.00e+00 P=65535 LC=8198
  { 0x6e43u, 0xb3b1u }, // 076: duty=0.2980 err=5.68e-06 P=57337 LC=8201
  { 0x4b09u, 0xb2b5u }, // 077: duty=0.3020 err=9.03e-06 P=42987 LC=8198
  { 0x873fu, 0xafe2u }, // 078: duty=0.3059 err=1.26e-05 P=21483 LC=11067
  { 0xd4c9u, 0xaff1u }, // 079: duty=0.3098 err=2.94e-05 P=16383 LC=11487
  { 0x190du, 0xaff5u }, // 080: duty=0.3137 err=1.44e-05 P=16383 LC=11081
  { 0x4121u, 0xaffau }, // 081: duty=0.3176 err=7.18e-07 P=16383 LC=11105
  { 0xf875u, 0xadacu }, // 082: duty=0.3216 err=3.46e-06 P=63457 LC=8199
  { 0xd549u, 0xacacu }, // 083: duty=0.3255 err=0.00e+00 P=65535 LC=8198
  { 0xa79fu, 0xabacu }, // 084: duty=0.3294 err=1.54e-05 P=23622 LC=8199
  { 0xbe6bu, 0xaaaau }, // 085: duty=0.3333 err=0.00e+00 P=24573 LC=8199
  { 0xf213u, 0xa9a9u }, // 086: duty=0.3373 err=2.90e-06 P=63457 LC=8202
  { 0xa3e1u, 0xa80cu }, // 087: duty=0.3412 err=0.00e+00 P=21845 LC=9295
  { 0x4b7du, 0xa7a7u }, // 088: duty=0.3451 err=0.00e+00 P=65535 LC=8198
  { 0x8cfdu, 0xa6a5u }, // 089: duty=0.3490 err=4.58e-06 P=57337 LC=8199
  { 0xf4adu, 0xa5a1u }, // 090: duty=0.3529 err=1.10e-05 P=21483 LC=8199
  { 0xd989u, 0xa4a4u }, // 091: duty=0.3569 err=6.92e-06 P=63457 LC=8199
  { 0x2c6bu, 0xa3a3u }, // 092: duty=0.3608 err=0.00e+00 P=65535 LC=8201
  { 0x12c1u, 0xa2a1u }, // 093: duty=0.3647 err=1.48e-05 P=24564 LC=8199
  { 0x901du, 0xa012u }, // 094: duty=0.3686 err=1.04e-05 P=21483 LC=8851
  { 0x2e8bu, 0xa008u }, // 095: duty=0.3725 err=2.87e-05 P=16383 LC=15272
  { 0x45cdu, 0xa002u }, // 096: duty=0.3765 err=1.72e-05 P=16383 LC=13910
  { 0x72ffu, 0xa006u }, // 097: duty=0.3804 err=2.15e-06 P=16383 LC=15538
  { 0x9e93u, 0x9d9au }, // 098: duty=0.3843 err=5.86e-06 P=32767 LC=8202
  { 0x8496u, 0x9c98u }, // 099: duty=0.3882 err=4.31e-06 P=24573 LC=8202
  { 0x0cddu, 0x9a9bu }, // 100: duty=0.3922 err=1.53e-05 P=21845 LC=8200
  { 0xd2b9u, 0x9a99u }, // 101: duty=0.3961 err=9.22e-06 P=32767 LC=8199
  { 0xd661u, 0x9997u }, // 102: duty=0.4000 err=3.15e-06 P=63457 LC=8200
  { 0x849fu, 0x989eu }, // 103: duty=0.4039 err=1.84e-05 P=23622 LC=8203
  { 0x9edfu, 0x9797u }, // 104: duty=0.4078 err=0.00e+00 P=65535 LC=8198
  { 0x3a95u, 0x97fbu }, // 105: duty=0.4118 err=0.00e+00 P=21845 LC=8894
  { 0x4659u, 0x9594u }, // 106: duty=0.4157 err=1.92e-06 P=30660 LC=8199
  { 0x047au, 0x9495u }, // 107: duty=0.4196 err=0.00e+00 P=32385 LC=8199
  { 0x2acbu, 0x9391u }, // 108: duty=0.4235 err=1.64e-06 P=57337 LC=8199
  { 0xe48fu, 0x9290u }, // 109: duty=0.4274 err=8.74e-06 P=32767 LC=8200
  { 0x3215u, 0x8ffcu }, // 110: duty=0.4314 err=1.08e-05 P=16383 LC=13912
  { 0x9377u, 0x8ffeu }, // 111: duty=0.4353 err=2.59e-05 P=16383 LC=12824
  { 0xea6bu, 0x8ff8u }, // 112: duty=0.4392 err=2.01e-05 P=16383 LC=13723
  { 0xc525u, 0x8e8eu }, // 113: duty=0.4431 err=9.10e-06 P=24564 LC=8199
  { 0xe4d5u, 0x8d8bu }, // 114: duty=0.4471 err=6.82e-06 P=32767 LC=8202
  { 0x3159u, 0x8c8bu }, // 115: duty=0.4510 err=2.39e-06 P=32764 LC=8199
  { 0x9581u, 0x8b89u }, // 116: duty=0.4549 err=3.01e-06 P=42987 LC=8198
  { 0xbdb5u, 0x8a89u }, // 117: duty=0.4588 err=8.26e-06 P=32767 LC=8200
  { 0x40ddu, 0x8989u }, // 118: duty=0.4627 err=9.34e-06 P=32766 LC=8202
  { 0x904bu, 0x87f8u }, // 119: duty=0.4667 err=1.53e-05 P=21845 LC=9342
  { 0x0674u, 0x8787u }, // 120: duty=0.4706 err=2.15e-05 P=16383 LC=8202
  { 0x6e43u, 0x8684u }, // 121: duty=0.4745 err=5.47e-07 P=57337 LC=8198
  { 0x8611u, 0x8585u }, // 122: duty=0.4784 err=9.93e-06 P=32764 LC=8199
  { 0xb469u, 0x8486u }, // 123: duty=0.4824 err=2.97e-06 P=47523 LC=8200
  { 0x729bu, 0x8382u }, // 124: duty=0.4863 err=1.92e-06 P=30705 LC=8199
  { 0x7a81u, 0x8282u }, // 125: duty=0.4902 err=1.24e-05 P=31682 LC=8199
  { 0x0017u, 0x8011u }, // 126: duty=0.4941 err=7.90e-06 P=16383 LC=12128
  { 0x5bcbu, 0x801bu }, // 127: duty=0.4980 err=2.30e-05 P=16383 LC=10946
  { 0xebc9u, 0x801au }, // 128: duty=0.5020 err=2.30e-05 P=16383 LC=12299
  { 0xb63du, 0x7e7eu }, // 129: duty=0.5059 err=0.00e+00 P=65535 LC=8198
  { 0x19eau, 0x7d7cu }, // 130: duty=0.5098 err=1.68e-05 P=24573 LC=8200
  { 0x1e89u, 0x7c7cu }, // 131: duty=0.5137 err=0.00e+00 P=31620 LC=8199
  { 0x6917u, 0x7b7bu }, // 132: duty=0.5176 err=0.00e+00 P=65535 LC=8199
  { 0x6737u, 0x7a7au }, // 133: duty=0.5216 err=3.69e-06 P=64897 LC=8198
  { 0x6341u, 0x7978u }, // 134: duty=0.5255 err=4.94e-07 P=63457 LC=8199
  { 0x526bu, 0x780au }, // 135: duty=0.5294 err=0.00e+00 P=21845 LC=9093
  { 0x9cfdu, 0x7779u }, // 136: duty=0.5333 err=0.00e+00 P=30705 LC=8200
  { 0xe2adu, 0x7675u }, // 137: duty=0.5373 err=9.34e-06 P=32766 LC=8198
  { 0x8b71u, 0x7575u }, // 138: duty=0.5412 err=6.86e-06 P=63457 LC=8199
  { 0xb4f5u, 0x7473u }, // 139: duty=0.5451 err=4.99e-06 P=57337 LC=8198
  { 0x69a6u, 0x736eu }, // 140: duty=0.5490 err=0.00e+00 P=32385 LC=8201
  { 0xb5b3u, 0x7272u }, // 141: duty=0.5529 err=0.00e+00 P=65535 LC=8198
  { 0xcfebu, 0x6feau }, // 142: duty=0.5569 err=3.83e-06 P=21483 LC=12322
  { 0xfec7u, 0x6ffbu }, // 143: duty=0.5608 err=2.01e-05 P=16383 LC=13631
  { 0x19c7u, 0x6ffbu }, // 144: duty=0.5647 err=2.59e-05 P=16383 LC=9884
  { 0x9d3du, 0x6ff3u }, // 145: duty=0.5686 err=1.53e-05 P=21845 LC=10472
  { 0xafafu, 0x6d6du }, // 146: duty=0.5725 err=0.00e+00 P=65535 LC=8199
  { 0x77c7u, 0x6c6au }, // 147: duty=0.5765 err=1.11e-05 P=32766 LC=8199
  { 0x5606u, 0x6b6bu }, // 148: duty=0.5804 err=8.86e-06 P=32767 LC=8199
  { 0x300fu, 0x680eu }, // 149: duty=0.5843 err=1.15e-05 P=16383 LC=8592
  { 0x9941u, 0x6969u }, // 150: duty=0.5882 err=0.00e+00 P=65535 LC=8198
  { 0xd6c1u, 0x6803u }, // 151: duty=0.5921 err=1.87e-05 P=16383 LC=9743
  { 0x9cb3u, 0x6765u }, // 152: duty=0.5961 err=9.10e-06 P=32767 LC=8199
  { 0x8985u, 0x657au }, // 153: duty=0.6000 err=0.00e+00 P=21845 LC=8201
  { 0xccedu, 0x6565u }, // 154: duty=0.6039 err=0.00e+00 P=65535 LC=8199
  { 0xf7c3u, 0x6464u }, // 155: duty=0.6078 err=5.75e-06 P=30705 LC=8198
  { 0x075bu, 0x6362u }, // 156: duty=0.6118 err=4.31e-06 P=57337 LC=8199
  { 0x5e57u, 0x6262u }, // 157: duty=0.6157 err=0.00e+00 P=65535 LC=8198
  { 0x32fdu, 0x6025u }, // 158: duty=0.6196 err=1.64e-06 P=21483 LC=9776
  { 0x69d1u, 0x6022u }, // 159: duty=0.6235 err=0.00e+00 P=21845 LC=10966
  { 0x74f5u, 0x5fd1u }, // 160: duty=0.6275 err=2.87e-05 P=16383 LC=8478
  { 0xe4a9u, 0x5e5du }, // 161: duty=0.6314 err=2.84e-06 P=64897 LC=8199
  { 0xdf25u, 0x5d5du }, // 162: duty=0.6353 err=4.98e-06 P=59055 LC=8203
  { 0x756du, 0x5c5bu }, // 163: duty=0.6392 err=2.78e-06 P=64897 LC=8198
  { 0xb4afu, 0x5b5au }, // 164: duty=0.6431 err=4.96e-06 P=64897 LC=8198
  { 0x90c5u, 0x5a5au }, // 165: duty=0.6471 err=7.18e-06 P=57337 LC=8201
  { 0x6e61u, 0x57f9u }, // 166: duty=0.6510 err=7.18e-07 P=16383 LC=9670
  { 0x5fddu, 0x57fau }, // 167: duty=0.6549 err=1.53e-05 P=21845 LC=9394
  { 0xb37du, 0x575bu }, // 168: duty=0.6588 err=1.75e-05 P=27559 LC=8199
  { 0xbd12u, 0x5656u }, // 169: duty=0.6627 err=5.15e-06 P=32767 LC=8199
  { 0x44dbu, 0x5553u }, // 170: duty=0.6667 err=0.00e+00 P=47523 LC=8200
  { 0x5e95u, 0x5454u }, // 171: duty=0.6706 err=0.00e+00 P=32385 LC=8200
  { 0xd9f9u, 0x5350u }, // 172: duty=0.6745 err=1.03e-05 P=32767 LC=8199
  { 0x540bu, 0x5006u }, // 173: duty=0.6784 err=1.58e-05 P=16383 LC=11768
  { 0xeb11u, 0x500du }, // 174: duty=0.6824 err=7.18e-07 P=16383 LC=11907
  { 0xa3a5u, 0x4ff2u }, // 175: duty=0.6863 err=1.44e-05 P=16383 LC=10850
  { 0x4d53u, 0x4fecu }, // 176: duty=0.6902 err=1.53e-05 P=21845 LC=10486
  { 0x9ecbu, 0x500eu }, // 177: duty=0.6941 err=0.00e+00 P=21845 LC=9552
  { 0xf864u, 0x4d4eu }, // 178: duty=0.6980 err=1.44e-06 P=16383 LC=8199
  { 0x5eb2u, 0x4c4au }, // 179: duty=0.7019 err=1.36e-05 P=16383 LC=8198
  { 0x37a1u, 0x4b49u }, // 180: duty=0.7059 err=5.44e-06 P=64897 LC=8199
  { 0x540bu, 0x4808u }, // 181: duty=0.7098 err=1.72e-05 P=16383 LC=10066
  { 0x48cfu, 0x4806u }, // 182: duty=0.7137 err=2.15e-06 P=16383 LC=9696
  { 0x04cdu, 0x4805u }, // 183: duty=0.7176 err=1.29e-05 P=16383 LC=9933
  { 0x43d5u, 0x4804u }, // 184: duty=0.7215 err=2.80e-05 P=16383 LC=9705
  { 0xa9cfu, 0x4644u }, // 185: duty=0.7255 err=7.52e-06 P=57337 LC=8198
  { 0xd5c9u, 0x4541u }, // 186: duty=0.7294 err=5.12e-06 P=27559 LC=8199
  { 0x4421u, 0x4443u }, // 187: duty=0.7333 err=2.33e-06 P=57337 LC=8198
  { 0xaf91u, 0x4340u }, // 188: duty=0.7373 err=0.00e+00 P=32385 LC=8199
  { 0xc93fu, 0x402fu }, // 189: duty=0.7412 err=1.87e-05 P=16383 LC=8695
  { 0x7eb7u, 0x4145u }, // 190: duty=0.7451 err=1.29e-05 P=19685 LC=8202
  { 0x0a87u, 0x403eu }, // 191: duty=0.7490 err=8.07e-06 P=57337 LC=12338
  { 0x0113u, 0x4034u }, // 192: duty=0.7529 err=2.66e-05 P=16383 LC=8319
  { 0x4ffdu, 0x3fb2u }, // 193: duty=0.7569 err=1.53e-05 P=21845 LC=8345
  { 0x3c8eu, 0x3d3cu }, // 194: duty=0.7608 err=1.16e-05 P=32767 LC=8199
  { 0xe06fu, 0x3c3bu }, // 195: duty=0.7647 err=3.59e-06 P=32767 LC=8198
  { 0xca81u, 0x3b3au }, // 196: duty=0.7686 err=3.28e-06 P=63457 LC=8198
  { 0x2bdbu, 0x3a3au }, // 197: duty=0.7725 err=0.00e+00 P=65535 LC=8198
  { 0x1711u, 0x37f8u }, // 198: duty=0.7765 err=5.03e-06 P=16383 LC=8890
  { 0xd6c1u, 0x37f6u }, // 199: duty=0.7804 err=1.01e-05 P=16383 LC=9182
  { 0xb581u, 0x3735u }, // 200: duty=0.7843 err=1.20e-05 P=32767 LC=8200
  { 0x236du, 0x3636u }, // 201: duty=0.7882 err=0.00e+00 P=65535 LC=8198
  { 0xdf29u, 0x3604u }, // 202: duty=0.7922 err=1.53e-05 P=21845 LC=8200
  { 0x786bu, 0x3432u }, // 203: duty=0.7961 err=3.11e-06 P=32767 LC=8199
  { 0x801fu, 0x33e8u }, // 204: duty=0.8000 err=2.44e-05 P=16383 LC=8199
  { 0x7ce3u, 0x2ff1u }, // 205: duty=0.8039 err=2.15e-05 P=16383 LC=11755
  { 0x6065u, 0x3015u }, // 206: duty=0.8078 err=3.05e-05 P=13107 LC=9486
  { 0x20e3u, 0x2fedu }, // 207: duty=0.8118 err=0.00e+00 P=21845 LC=12433
  { 0xed53u, 0x2ff0u }, // 208: duty=0.8157 err=2.37e-05 P=16383 LC=12362
  { 0x9b0eu, 0x2e2eu }, // 209: duty=0.8196 err=2.75e-06 P=32767 LC=8200
  { 0x942cu, 0x2d2cu }, // 210: duty=0.8235 err=7.18e-06 P=16383 LC=8199
  { 0xbe55u, 0x2c2au }, // 211: duty=0.8274 err=1.33e-06 P=64897 LC=8201
  { 0xb7e1u, 0x2b2bu }, // 212: duty=0.8314 err=0.00e+00 P=65535 LC=8199
  { 0x9d15u, 0x2804u }, // 213: duty=0.8353 err=2.30e-05 P=16383 LC=9744
  { 0x58abu, 0x2804u }, // 214: duty=0.8392 err=7.90e-06 P=16383 LC=9869
  { 0x383du, 0x2809u }, // 215: duty=0.8431 err=7.18e-06 P=16383 LC=9781
  { 0x57b3u, 0x2804u }, // 216: duty=0.8470 err=2.23e-05 P=16383 LC=9430
  { 0xedafu, 0x2625u }, // 217: duty=0.8510 err=2.97e-06 P=47523 LC=8199
  { 0x6d83u, 0x2525u }, // 218: duty=0.8549 err=1.04e-05 P=47523 LC=8199
  { 0xcbabu, 0x2424u }, // 219: duty=0.8588 err=0.00e+00 P=65535 LC=8199
  { 0x1c0fu, 0x2320u }, // 220: duty=0.8628 err=1.32e-05 P=32767 LC=8199
  { 0x540bu, 0x201bu }, // 221: duty=0.8667 err=2.44e-05 P=16383 LC=10461
  { 0xb7e7u, 0x201cu }, // 222: duty=0.8706 err=9.34e-06 P=16383 LC=8379
  { 0x9eb3u, 0x201bu }, // 223: duty=0.8745 err=4.38e-06 P=21483 LC=15570
  { 0x2cd9u, 0x2019u }, // 224: duty=0.8784 err=2.08e-05 P=16383 LC=11831
  { 0x662eu, 0x1e1fu }, // 225: duty=0.8824 err=1.80e-06 P=32767 LC=8200
  { 0x5f55u, 0x1d1du }, // 226: duty=0.8863 err=0.00e+00 P=65535 LC=8198
  { 0x33a9u, 0x1c1cu }, // 227: duty=0.8902 err=0.00e+00 P=65535 LC=8199
  { 0x1975u, 0x1b1bu }, // 228: duty=0.8941 err=0.00e+00 P=65535 LC=8199
  { 0x1925u, 0x1a1au }, // 229: duty=0.8980 err=0.00e+00 P=65535 LC=8200
  { 0x3dcdu, 0x1919u }, // 230: duty=0.9020 err=0.00e+00 P=65535 LC=8198
  { 0xe68du, 0x1817u }, // 231: duty=0.9059 err=1.01e-05 P=24573 LC=11482
  { 0xf29bu, 0x17f5u }, // 232: duty=0.9098 err=1.94e-05 P=16383 LC=9632
  { 0xfb23u, 0x1615u }, // 233: duty=0.9137 err=4.39e-06 P=63457 LC=8198
  { 0x6475u, 0x1513u }, // 234: duty=0.9177 err=7.07e-06 P=64897 LC=8198
  { 0xf2c4u, 0x140eu }, // 235: duty=0.9216 err=2.72e-05 P=15841 LC=8546
  { 0xd495u, 0x1311u }, // 236: duty=0.9255 err=7.13e-06 P=64897 LC=8201
  { 0x3c3fu, 0x100fu }, // 237: duty=0.9294 err=2.73e-05 P=16383 LC=12410
  { 0xa35fu, 0x1013u }, // 238: duty=0.9333 err=1.53e-05 P=21845 LC=10387
  { 0x3716u, 0x1011u }, // 239: duty=0.9373 err=9.57e-07 P=32767 LC=14432
  { 0x472bu, 0x1008u }, // 240: duty=0.9412 err=1.80e-05 P=16383 LC=10282
  { 0x14afu, 0x0e0eu }, // 241: duty=0.9451 err=0.00e+00 P=65535 LC=8198
  { 0x0ccdu, 0x0d08u }, // 242: duty=0.9490 err=1.15e-05 P=30705 LC=8199
  { 0x674eu, 0x0c08u }, // 243: duty=0.9529 err=5.03e-06 P=16382 LC=9792
  { 0x939du, 0x0b08u }, // 244: duty=0.9569 err=1.07e-05 P=32764 LC=8200
  { 0xa18au, 0x0a0au }, // 245: duty=0.9608 err=5.98e-07 P=32767 LC=8199
  { 0xa1c1u, 0x0809u }, // 246: duty=0.9647 err=1.36e-05 P=16383 LC=9501
  { 0xe8f2u, 0x0804u }, // 247: duty=0.9686 err=3.35e-06 P=16382 LC=12914
  { 0x45cdu, 0x07f2u }, // 248: duty=0.9725 err=1.65e-05 P=16383 LC=8317
  { 0xb195u, 0x0604u }, // 249: duty=0.9765 err=1.81e-07 P=64897 LC=8198
  { 0x092bu, 0x0502u }, // 250: duty=0.9804 err=7.55e-06 P=64897 LC=8201
  { 0x7174u, 0x0404u }, // 251: duty=0.9843 err=7.18e-07 P=16383 LC=9914
  { 0xb01au, 0x0302u }, // 252: duty=0.9883 err=1.51e-05 P=32767 LC=8199
  { 0x6aedu, 0x0200u }, // 253: duty=0.9922 err=6.04e-08 P=64897 LC=8199
  { 0x0a22u, 0x00ffu }, // 254: duty=0.9961 err=0.00e+00 P=32385 LC=8199
  { 0xc00du, 0x0008u }, // 255: duty=0.9999 err=1.22e-04 P=16376 LC=16362
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
