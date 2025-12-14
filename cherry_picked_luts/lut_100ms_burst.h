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
  { 0x502bu, 0xfff6u }, // 000: duty=0.0000 err=4.88e-04 P=21845 LC=16381
  { 0xf257u, 0xfea8u }, // 0xee9du, 0xfefau 001: duty=0.0040 err=1.97e-03 P=15841 LC=7922
  { 0xbf96u, 0xfdf7u }, // 002: duty=0.0078 err=1.98e-03 P=8191 LC=4096
  { 0x4402u, 0xfc4au }, // 003: duty=0.0117 err=2.00e-03 P=5461 LC=2731
  { 0x6149u, 0xfbe7u }, // 004: duty=0.0159 err=1.89e-03 P=4095 LC=2050
  { 0xf0c1u, 0xfb01u }, // 005: duty=0.0198 err=1.88e-03 P=4095 LC=2048
  { 0x5efdu, 0xf98du }, // 006: duty=0.0240 err=1.86e-03 P=2170 LC=1089
  { 0x5efdu, 0xf8bau }, // 007: duty=0.0276 err=1.85e-03 P=2170 LC=1086
  { 0xdf18u, 0xf7d8u }, // 008: duty=0.0313 err=3.66e-04 P=2047 LC=1027
  { 0xcb4du, 0xf6f2u }, // 009: duty=0.0352 err=8.39e-04 P=2046 LC=1029
  { 0x9427u, 0xf5dbu }, // 010: duty=0.0391 err=3.35e-04 P=2047 LC=1029
  { 0x6ed2u, 0xf554u }, // 011: duty=0.0430 err=8.08e-04 P=2046 LC=1027
  { 0xd22au, 0xf483u }, // 012: duty=0.0469 err=7.93e-04 P=2046 LC=1029
  { 0xc20du, 0xf2fau }, // 013: duty=0.0508 err=2.89e-04 P=2047 LC=1029
  { 0xbfacu, 0xf1cfu }, // 014: duty=0.0547 err=2.74e-04 P=2047 LC=1029
  { 0x5bd8u, 0xf0dcu }, // 015: duty=0.0586 err=2.59e-04 P=2047 LC=1027
  { 0x9e0fu, 0xefdfu }, // 016: duty=0.0625 err=2.45e-04 P=2047 LC=1479
  { 0x45aau, 0xeedfu }, // 017: duty=0.0664 err=2.60e-04 P=2047 LC=1027
  { 0xf525u, 0xedc1u }, // 018: duty=0.0703 err=2.76e-04 P=2047 LC=1028
  { 0xe02du, 0xeca9u }, // 019: duty=0.0747 err=6.86e-04 P=2047 LC=1028
  { 0x7441u, 0xebc1u }, // 020: duty=0.0787 err=6.70e-04 P=2047 LC=1028
  { 0x2c45u, 0xead1u }, // 021: duty=0.0826 err=6.55e-04 P=2047 LC=1028
  { 0x561cu, 0xe9aau }, // 022: duty=0.0865 err=6.40e-04 P=2047 LC=1028
  { 0x98ffu, 0xe8a8u }, // 023: duty=0.0904 err=6.24e-04 P=2047 LC=1029
  { 0x2d1du, 0xe7cfu }, // 024: duty=0.0943 err=6.09e-04 P=2047 LC=1029
  { 0x9e4eu, 0xe6a4u }, // 025: duty=0.0982 err=5.94e-04 P=2047 LC=1027
  { 0x6ea3u, 0xe5bfu }, // 026: duty=0.1021 err=5.78e-04 P=2047 LC=1028
  { 0xcd01u, 0xe4dau }, // 027: duty=0.1060 err=5.63e-04 P=2047 LC=1028
  { 0x3d27u, 0xdfccu }, // 028: duty=0.1100 err=1.04e-03 P=2046 LC=1427
  { 0x15ecu, 0xdfbfu }, // 029: duty=0.1139 err=1.02e-03 P=2046 LC=1635
  { 0xf521u, 0xdf45u }, // 030: duty=0.1178 err=1.01e-03 P=2046 LC=1616
  { 0x1f12u, 0xdfc6u }, // 031: duty=0.1217 err=9.90e-04 P=2046 LC=1550
  { 0x9c55u, 0xdfdfu }, // 032: duty=0.1255 err=4.86e-04 P=2047 LC=1809
  { 0x2ad3u, 0xdfdfu }, // 033: duty=0.1295 err=9.59e-04 P=2046 LC=1639
  { 0x6ac2u, 0xdfeau }, // 034: duty=0.1334 err=9.44e-04 P=2046 LC=1518
  { 0xbeb4u, 0xdfbfu }, // 035: duty=0.1373 err=9.29e-04 P=2046 LC=1580
  { 0x0649u, 0xdfcfu }, // 036: duty=0.1413 err=9.13e-04 P=2046 LC=1400
  { 0x9e4eu, 0xdaa2u }, // 037: duty=0.1451 err=4.10e-04 P=2047 LC=1028
  { 0xb764u, 0xd9b1u }, // 038: duty=0.1490 err=3.94e-04 P=2047 LC=1027
  { 0x1d52u, 0xd8adu }, // 039: duty=0.1529 err=3.79e-04 P=2047 LC=1028
  { 0x6c04u, 0xd7a8u }, // 040: duty=0.1568 err=3.64e-04 P=2047 LC=1028
  { 0xf7d5u, 0xd635u }, // 041: duty=0.1608 err=8.37e-04 P=2046 LC=1028
  { 0xe072u, 0xd5abu }, // 042: duty=0.1646 err=3.33e-04 P=2047 LC=1027
  { 0x07cfu, 0xd1d5u }, // 043: duty=0.1686 err=8.06e-04 P=2046 LC=1029
  { 0x6212u, 0xd3b3u }, // 044: duty=0.1724 err=3.03e-04 P=2047 LC=1028
  { 0xccbbu, 0xd2fcu }, // 045: duty=0.1764 err=7.76e-04 P=2046 LC=1028
  { 0x8c20u, 0xd1c7u }, // 046: duty=0.1803 err=2.72e-04 P=2047 LC=1028
  { 0x0750u, 0xd0beu }, // 047: duty=0.1842 err=2.57e-04 P=2047 LC=1045
  { 0xedfau, 0xcfd4u }, // 048: duty=0.1881 err=2.47e-04 P=2047 LC=1466
  { 0x8041u, 0xcebau }, // 049: duty=0.1920 err=2.62e-04 P=2047 LC=1028
  { 0x0837u, 0xcdc9u }, // 050: duty=0.1959 err=2.78e-04 P=2047 LC=1030
  { 0x9427u, 0xcca5u }, // 051: duty=0.1998 err=2.93e-04 P=2047 LC=1029
  { 0xe072u, 0xcbbdu }, // 052: duty=0.2037 err=3.08e-04 P=2047 LC=1027
  { 0xc20du, 0xcac5u }, // 053: duty=0.2076 err=3.24e-04 P=2047 LC=1028
  { 0x193cu, 0xc9bdu }, // 054: duty=0.2115 err=3.39e-04 P=2047 LC=1028
  { 0x0750u, 0xc882u }, // 055: duty=0.2159 err=6.22e-04 P=2047 LC=1030
  { 0x7bb7u, 0xc7afu }, // 056: duty=0.2198 err=6.07e-04 P=2047 LC=1027
  { 0xa0d6u, 0xc697u }, // 057: duty=0.2237 err=5.92e-04 P=2047 LC=1028
  { 0x9683u, 0xc020u }, // 058: duty=0.2273 err=5.76e-04 P=2046 LC=1170
  { 0x70c1u, 0xc675u }, // 059: duty=0.2312 err=5.61e-04 P=2046 LC=1028
  { 0x23e2u, 0xc0afu }, // 060: duty=0.2351 err=5.46e-04 P=2046 LC=1671
  { 0x28a1u, 0xbfb6u }, // 061: duty=0.2392 err=1.51e-03 P=2044 LC=1852
  { 0xd215u, 0xbff1u }, // 062: duty=0.2429 err=5.15e-04 P=2046 LC=1874
  { 0x5cd5u, 0xbf83u }, // 063: duty=0.2471 err=1.48e-03 P=2044 LC=1845
  { 0x90c4u, 0xbfdfu }, // 064: duty=0.2505 err=1.95e-03 P=2040 LC=1949
  { 0x6e2bu, 0xbfecu }, // 065: duty=0.2549 err=1.45e-03 P=2044 LC=1853
  { 0xb9e9u, 0xbfc2u }, // 066: duty=0.2590 err=9.42e-04 P=2046 LC=1715
  { 0x1349u, 0xc0acu }, // 067: duty=0.2630 err=9.27e-04 P=2046 LC=1761
  { 0x6959u, 0xbf89u }, // 068: duty=0.2669 err=9.11e-04 P=2046 LC=1256
  { 0x8bc1u, 0xba99u }, // 069: duty=0.2706 err=4.08e-04 P=2047 LC=1028
  { 0x1aa6u, 0xb985u }, // 070: duty=0.2745 err=3.93e-04 P=2047 LC=1029
  { 0x3f06u, 0xb8bbu }, // 071: duty=0.2785 err=3.77e-04 P=2047 LC=1029
  { 0x31e8u, 0xb790u }, // 072: duty=0.2824 err=3.62e-04 P=2047 LC=1028
  { 0x264bu, 0xb600u }, // 073: duty=0.2864 err=8.35e-04 P=2046 LC=1028
  { 0x4f05u, 0xb599u }, // 074: duty=0.2902 err=3.31e-04 P=2047 LC=1027
  { 0x8a8fu, 0xb4a7u }, // 075: duty=0.2941 err=3.16e-04 P=2047 LC=1027
  { 0x1294u, 0xb38bu }, // 076: duty=0.2980 err=3.01e-04 P=2047 LC=1029
  { 0xc354u, 0xb289u }, // 077: duty=0.3019 err=2.85e-04 P=2047 LC=1028
  { 0xb224u, 0xb1afu }, // 078: duty=0.3058 err=2.70e-04 P=2047 LC=1028
  { 0x4246u, 0xb080u }, // 079: duty=0.3097 err=2.55e-04 P=2047 LC=1102
  { 0x6b48u, 0xafdeu }, // 080: duty=0.3136 err=1.22e-03 P=2044 LC=1362
  { 0x2ff0u, 0xae85u }, // 081: duty=0.3175 err=2.64e-04 P=2047 LC=1027
  { 0xe66au, 0xad98u }, // 082: duty=0.3214 err=2.80e-04 P=2047 LC=1027
  { 0xc351u, 0xab6au }, // 083: duty=0.3255 err=6.82e-04 P=2046 LC=1029
  { 0x4a21u, 0xab9fu }, // 084: duty=0.3293 err=3.10e-04 P=2047 LC=1028
  { 0xd906u, 0xa971u }, // 085: duty=0.3333 err=6.51e-04 P=2046 LC=1028
  { 0x6d3fu, 0xa991u }, // 086: duty=0.3371 err=3.41e-04 P=2047 LC=1028
  { 0xb2aeu, 0xa5d6u }, // 087: duty=0.3412 err=6.20e-04 P=2046 LC=1030
  { 0x7426u, 0xa786u }, // 088: duty=0.3449 err=3.71e-04 P=2047 LC=1027
  { 0x4a7du, 0xa12cu }, // 089: duty=0.3490 err=5.90e-04 P=2046 LC=1035
  { 0x9532u, 0xa07bu }, // 090: duty=0.3529 err=5.74e-04 P=2046 LC=1122
  { 0x92d7u, 0xa0a0u }, // 091: duty=0.3567 err=1.05e-03 P=2044 LC=1224
  { 0x2f6fu, 0x9fabu }, // 092: duty=0.3607 err=5.44e-04 P=2046 LC=1490
  { 0x90d5u, 0x9facu }, // 093: duty=0.3645 err=1.02e-03 P=2044 LC=1497
  { 0x4b76u, 0xa03au }, // 094: duty=0.3684 err=1.00e-03 P=2044 LC=1573
  { 0xa576u, 0xa05du }, // 095: duty=0.3723 err=9.86e-04 P=2044 LC=1609
  { 0x2c45u, 0x9f9fu }, // 096: duty=0.3766 err=4.83e-04 P=2047 LC=1681
  { 0xebf1u, 0x9fc8u }, // 097: duty=0.3803 err=5.09e-04 P=2046 LC=1462
  { 0xeecdu, 0xa02eu }, // 098: duty=0.3845 err=1.43e-03 P=2044 LC=1435
  { 0x1b02u, 0xa03fu }, // 099: duty=0.3881 err=5.40e-04 P=2046 LC=1604
  { 0xe73bu, 0x9fa7u }, // 100: duty=0.3924 err=1.40e-03 P=2044 LC=1530
  { 0x4fe0u, 0x9a94u }, // 101: duty=0.3962 err=4.06e-04 P=2047 LC=1028
  { 0x7e77u, 0x998au }, // 102: duty=0.4001 err=3.91e-04 P=2047 LC=1027
  { 0x63e6u, 0x9b74u }, // 103: duty=0.4037 err=6.01e-04 P=2046 LC=1031
  { 0x4246u, 0x9766u }, // 104: duty=0.4079 err=3.60e-04 P=2047 LC=1027
  { 0x4056u, 0x9674u }, // 105: duty=0.4118 err=3.45e-04 P=2047 LC=1028
  { 0xb286u, 0x9560u }, // 106: duty=0.4157 err=3.29e-04 P=2047 LC=1027
  { 0xad72u, 0x946bu }, // 107: duty=0.4196 err=3.14e-04 P=2047 LC=1028
  { 0x88eeu, 0x938au }, // 108: duty=0.4235 err=2.99e-04 P=2047 LC=1029
  { 0xe951u, 0x9416u }, // 109: duty=0.4277 err=7.72e-04 P=2046 LC=1028
  { 0x9f0au, 0x9196u }, // 110: duty=0.4314 err=2.68e-04 P=2047 LC=1028
  { 0x4d2au, 0x907au }, // 111: duty=0.4355 err=7.41e-04 P=2046 LC=1328
  { 0x7f6du, 0x8f92u }, // 112: duty=0.4392 err=2.51e-04 P=2047 LC=1270
  { 0xff9du, 0x8e61u }, // 113: duty=0.4431 err=2.66e-04 P=2047 LC=1028
  { 0x112du, 0x8d86u }, // 114: duty=0.4470 err=2.81e-04 P=2047 LC=1028
  { 0x719eu, 0x8c67u }, // 115: duty=0.4509 err=2.97e-04 P=2047 LC=1029
  { 0xae49u, 0x88e3u }, // 116: duty=0.4550 err=6.64e-04 P=2046 LC=1030
  { 0x61dbu, 0x8645u }, // 117: duty=0.4589 err=1.14e-03 P=2044 LC=1028
  { 0x730du, 0x899fu }, // 118: duty=0.4626 err=3.43e-04 P=2047 LC=1029
  { 0x3595u, 0x8882u }, // 119: duty=0.4665 err=3.58e-04 P=2047 LC=1028
  { 0xa0d6u, 0x877cu }, // 120: duty=0.4704 err=3.73e-04 P=2047 LC=1028
  { 0x36e1u, 0x8683u }, // 121: duty=0.4744 err=3.89e-04 P=2047 LC=1027
  { 0x91c5u, 0x802bu }, // 122: duty=0.4784 err=1.87e-03 P=2040 LC=1678
  { 0x7b3bu, 0x803eu }, // 123: duty=0.4824 err=5.57e-04 P=2046 LC=1589
  { 0xdbadu, 0x802du }, // 124: duty=0.4863 err=1.03e-03 P=2044 LC=1536
  { 0x24ebu, 0x7fb1u }, // 125: duty=0.4902 err=5.27e-04 P=2046 LC=1849
  { 0x17dbu, 0x8037u }, // 126: duty=0.4936 err=9.54e-04 P=2046 LC=2017
  { 0xe816u, 0x8058u }, // 127: duty=0.4980 err=1.96e-03 P=2040 LC=1944
  { 0x748au, 0x7fafu }, // 128: duty=0.5020 err=1.96e-03 P=2040 LC=1812
  { 0x5924u, 0x7fc1u }, // 129: duty=0.5059 err=5.11e-04 P=2046 LC=2004
  { 0x1349u, 0x800cu }, // 130: duty=0.5093 err=1.01e-03 P=2046 LC=1970
  { 0x8e07u, 0x7fcdu }, // 131: duty=0.5137 err=1.03e-03 P=2044 LC=1537
  { 0x0649u, 0x7fa3u }, // 132: duty=0.5176 err=5.57e-04 P=2046 LC=1048
  { 0x943fu, 0x7f8eu }, // 133: duty=0.5221 err=1.87e-03 P=2040 LC=1738
  { 0xf98fu, 0x798bu }, // 134: duty=0.5254 err=5.88e-04 P=2046 LC=1028
  { 0x1b68u, 0x7845u }, // 135: duty=0.5296 err=3.73e-04 P=2047 LC=1030
  { 0x85a4u, 0x74f9u }, // 136: duty=0.5332 err=6.18e-04 P=2046 LC=1029
  { 0xeb86u, 0x7656u }, // 137: duty=0.5374 err=3.43e-04 P=2047 LC=1028
  { 0xbff4u, 0x7578u }, // 138: duty=0.5413 err=3.27e-04 P=2047 LC=1030
  { 0xc754u, 0x730au }, // 139: duty=0.5450 err=6.64e-04 P=2046 LC=1027
  { 0x217fu, 0x7346u }, // 140: duty=0.5491 err=2.97e-04 P=2047 LC=1028
  { 0x26f7u, 0x726au }, // 141: duty=0.5530 err=2.81e-04 P=2047 LC=1027
  { 0xba67u, 0x6fcau }, // 142: duty=0.5567 err=7.10e-04 P=2046 LC=1028
  { 0xe3e8u, 0x7041u }, // 143: duty=0.5608 err=2.51e-04 P=2047 LC=1347
  { 0x712eu, 0x6f7fu }, // 144: duty=0.5647 err=2.53e-04 P=2047 LC=1296
  { 0x8bc1u, 0x6e4au }, // 145: duty=0.5686 err=2.68e-04 P=2047 LC=1028
  { 0xe66au, 0x6d5au }, // 146: duty=0.5725 err=2.83e-04 P=2047 LC=1029
  { 0x32a6u, 0x6833u }, // 147: duty=0.5762 err=7.87e-04 P=2046 LC=1028
  { 0x54d4u, 0x6b5eu }, // 148: duty=0.5804 err=3.14e-04 P=2047 LC=1029
  { 0x6ea3u, 0x6a5cu }, // 149: duty=0.5843 err=3.29e-04 P=2047 LC=1027
  { 0x5fabu, 0x65d6u }, // 150: duty=0.5885 err=6.32e-04 P=2046 LC=1028
  { 0x550cu, 0x6867u }, // 151: duty=0.5921 err=3.60e-04 P=2047 LC=1027
  { 0x6541u, 0x677fu }, // 152: duty=0.5960 err=3.75e-04 P=2047 LC=1029
  { 0xe2b3u, 0x6648u }, // 153: duty=0.6002 err=5.86e-04 P=2046 LC=1029
  { 0xb6beu, 0x60bfu }, // 154: duty=0.6041 err=5.71e-04 P=2046 LC=1297
  { 0x07cfu, 0x6040u }, // 155: duty=0.6080 err=5.55e-04 P=2046 LC=1560
  { 0x30dfu, 0x604cu }, // 156: duty=0.6115 err=1.41e-03 P=2044 LC=1518
  { 0xca2au, 0x5fceu }, // 157: duty=0.6155 err=1.43e-03 P=2044 LC=1476
  { 0xdf2eu, 0x5fb2u }, // 158: duty=0.6194 err=1.44e-03 P=2044 LC=1521
  { 0x953eu, 0x6010u }, // 159: duty=0.6233 err=1.46e-03 P=2044 LC=1726
  { 0x71d7u, 0x5f7eu }, // 160: duty=0.6273 err=4.98e-04 P=2047 LC=1551
  { 0x9f1au, 0x6028u }, // 161: duty=0.6315 err=5.13e-04 P=2046 LC=1352
  { 0xa2c4u, 0x608fu }, // 162: duty=0.6354 err=5.28e-04 P=2046 LC=1275
  { 0xb9e9u, 0x5fb6u }, // 163: duty=0.6393 err=5.44e-04 P=2046 LC=1492
  { 0xd63au, 0x5f65u }, // 164: duty=0.6432 err=5.59e-04 P=2046 LC=1332
  { 0x4f78u, 0x5a36u }, // 165: duty=0.6473 err=4.02e-04 P=2047 LC=1028
  { 0x62c5u, 0x5923u }, // 166: duty=0.6512 err=3.87e-04 P=2047 LC=1028
  { 0x28adu, 0x5820u }, // 167: duty=0.6551 err=3.71e-04 P=2047 LC=1027
  { 0x03f2u, 0x564au }, // 168: duty=0.6588 err=6.20e-04 P=2046 LC=1029
  { 0x3b50u, 0x5638u }, // 169: duty=0.6629 err=3.41e-04 P=2047 LC=1027
  { 0x4056u, 0x5538u }, // 170: duty=0.6668 err=3.26e-04 P=2047 LC=1030
  { 0x2690u, 0x5439u }, // 171: duty=0.6707 err=3.10e-04 P=2047 LC=1027
  { 0xae49u, 0x4f56u }, // 172: duty=0.6745 err=6.82e-04 P=2046 LC=1028
  { 0x4cb8u, 0x5231u }, // 173: duty=0.6786 err=2.80e-04 P=2047 LC=1029
  { 0xe939u, 0x513cu }, // 174: duty=0.6825 err=2.64e-04 P=2047 LC=1028
  { 0xdf18u, 0x5028u }, // 175: duty=0.6864 err=2.49e-04 P=2047 LC=1476
  { 0xc20du, 0x4f41u }, // 176: duty=0.6903 err=2.55e-04 P=2047 LC=1027
  { 0x83e0u, 0x4e45u }, // 177: duty=0.6942 err=2.70e-04 P=2047 LC=1028
  { 0xbc86u, 0x4d54u }, // 178: duty=0.6981 err=2.85e-04 P=2047 LC=1028
  { 0x4e45u, 0x4c24u }, // 179: duty=0.7020 err=3.01e-04 P=2047 LC=1027
  { 0x41e4u, 0x4b55u }, // 180: duty=0.7059 err=3.16e-04 P=2047 LC=1028
  { 0x42dfu, 0x4b5fu }, // 181: duty=0.7097 err=8.20e-04 P=2046 LC=1028
  { 0xad67u, 0x494bu }, // 182: duty=0.7137 err=3.47e-04 P=2047 LC=1028
  { 0x616eu, 0x4840u }, // 183: duty=0.7176 err=3.62e-04 P=2047 LC=1028
  { 0xdab8u, 0x473cu }, // 184: duty=0.7215 err=3.77e-04 P=2047 LC=1027
  { 0x943fu, 0x4092u }, // 185: duty=0.7260 err=1.86e-03 P=2040 LC=1178
  { 0xf853u, 0x3fc1u }, // 186: duty=0.7299 err=1.87e-03 P=2040 LC=1818
  { 0xabf5u, 0x403fu }, // 187: duty=0.7331 err=9.11e-04 P=2046 LC=1798
  { 0x0e4bu, 0x4097u }, // 188: duty=0.7373 err=1.42e-03 P=2044 LC=1827
  { 0xe8c2u, 0x406bu }, // 189: duty=0.7410 err=9.42e-04 P=2046 LC=1843
  { 0x17dbu, 0x4062u }, // 190: duty=0.7449 err=9.57e-04 P=2046 LC=1368
  { 0x608au, 0x4020u }, // 191: duty=0.7488 err=9.73e-04 P=2046 LC=1971
  { 0x5924u, 0x3fe0u }, // 192: duty=0.7532 err=5.00e-04 P=2046 LC=1919
  { 0xca4au, 0x40dbu }, // 193: duty=0.7571 err=5.15e-04 P=2046 LC=1893
  { 0xe176u, 0x3fd4u }, // 194: duty=0.7610 err=5.30e-04 P=2046 LC=1943
  { 0x30c5u, 0x3f3cu }, // 195: duty=0.7649 err=5.46e-04 P=2046 LC=1439
  { 0x788au, 0x3e9au }, // 196: duty=0.7688 err=5.61e-04 P=2046 LC=1034
  { 0x9909u, 0x3a5fu }, // 197: duty=0.7723 err=5.76e-04 P=2047 LC=1028
  { 0xe939u, 0x393du }, // 198: duty=0.7763 err=5.92e-04 P=2047 LC=1027
  { 0xb197u, 0x3836u }, // 199: duty=0.7802 err=6.07e-04 P=2047 LC=1028
  { 0xa0d6u, 0x3739u }, // 200: duty=0.7841 err=6.22e-04 P=2047 LC=1028
  { 0xab15u, 0x363bu }, // 201: duty=0.7885 err=3.39e-04 P=2047 LC=1030
  { 0x9c47u, 0x352fu }, // 202: duty=0.7924 err=3.24e-04 P=2047 LC=1029
  { 0xb286u, 0x3409u }, // 203: duty=0.7963 err=3.08e-04 P=2047 LC=1028
  { 0x730du, 0x3338u }, // 204: duty=0.8002 err=2.93e-04 P=2047 LC=1028
  { 0x2ff0u, 0x3205u }, // 205: duty=0.8041 err=2.78e-04 P=2047 LC=1028
  { 0x8942u, 0x2ef3u }, // 206: duty=0.8079 err=7.14e-04 P=2046 LC=1027
  { 0x5220u, 0x303du }, // 207: duty=0.8119 err=2.47e-04 P=2047 LC=1465
  { 0xfa60u, 0x2f3au }, // 208: duty=0.8158 err=2.57e-04 P=2047 LC=1046
  { 0x8bc1u, 0x2e0cu }, // 209: duty=0.8197 err=2.72e-04 P=2047 LC=1027
  { 0x2334u, 0x2daeu }, // 210: duty=0.8236 err=7.76e-04 P=2046 LC=1029
  { 0x0635u, 0x2c3au }, // 211: duty=0.8276 err=3.03e-04 P=2047 LC=1029
  { 0x616eu, 0x2b2eu }, // 212: duty=0.8315 err=3.18e-04 P=2047 LC=1028
  { 0x6bf9u, 0x2700u }, // 213: duty=0.8353 err=8.21e-04 P=2046 LC=1028
  { 0x29cbu, 0x2937u }, // 214: duty=0.8393 err=3.48e-04 P=2047 LC=1029
  { 0xc77eu, 0x2675u }, // 215: duty=0.8431 err=8.52e-04 P=2046 LC=1027
  { 0x7426u, 0x2703u }, // 216: duty=0.8471 err=3.79e-04 P=2047 LC=1027
  { 0x6be7u, 0x2608u }, // 217: duty=0.8509 err=8.83e-04 P=2046 LC=1029
  { 0xb004u, 0x2534u }, // 218: duty=0.8549 err=4.10e-04 P=2047 LC=1028
  { 0x6b1du, 0x1fbdu }, // 219: duty=0.8586 err=1.89e-03 P=2044 LC=1115
  { 0x90d5u, 0x2010u }, // 220: duty=0.8625 err=1.91e-03 P=2044 LC=1521
  { 0x8942u, 0x1fc6u }, // 221: duty=0.8666 err=9.44e-04 P=2046 LC=1442
  { 0x9f1au, 0x1fcdu }, // 222: duty=0.8705 err=9.59e-04 P=2046 LC=1622
  { 0xbd99u, 0x2020u }, // 223: duty=0.8745 err=4.86e-04 P=2047 LC=1808
  { 0x923cu, 0x2086u }, // 224: duty=0.8783 err=9.90e-04 P=2046 LC=1332
  { 0x54bbu, 0x1f17u }, // 225: duty=0.8826 err=1.49e-03 P=2044 LC=1194
  { 0x5e12u, 0x1f92u }, // 226: duty=0.8861 err=1.02e-03 P=2046 LC=1427
  { 0x6a47u, 0x1c2eu }, // 227: duty=0.8901 err=5.48e-04 P=2047 LC=1028
  { 0xf20bu, 0x1b25u }, // 228: duty=0.8940 err=5.63e-04 P=2047 LC=1027
  { 0x00a7u, 0x1a1du }, // 229: duty=0.8979 err=5.78e-04 P=2047 LC=1029
  { 0xf764u, 0x1936u }, // 230: duty=0.9018 err=5.94e-04 P=2047 LC=1028
  { 0x9c55u, 0x183au }, // 231: duty=0.9057 err=6.09e-04 P=2047 LC=1026
  { 0xcd90u, 0x1704u }, // 232: duty=0.9096 err=6.24e-04 P=2047 LC=1028
  { 0x4056u, 0x160eu }, // 233: duty=0.9135 err=6.40e-04 P=2047 LC=1028
  { 0xe353u, 0x183fu }, // 234: duty=0.9179 err=6.55e-04 P=2046 LC=1029
  { 0x65f5u, 0x1425u }, // 235: duty=0.9213 err=6.70e-04 P=2047 LC=1029
  { 0x460du, 0x133fu }, // 236: duty=0.9253 err=6.86e-04 P=2047 LC=1029
  { 0x21aeu, 0x11f3u }, // 237: duty=0.9297 err=2.76e-04 P=2047 LC=1028
  { 0x6842u, 0x10fau }, // 238: duty=0.9336 err=2.60e-04 P=2047 LC=1029
  { 0x0d5au, 0x11cdu }, // 239: duty=0.9374 err=7.31e-04 P=2046 LC=1029
  { 0x1d52u, 0x0ef3u }, // 240: duty=0.9414 err=2.59e-04 P=2047 LC=1027
  { 0x6e1fu, 0x0e11u }, // 241: duty=0.9453 err=2.74e-04 P=2047 LC=1031
  { 0xe1a4u, 0x0cfeu }, // 242: duty=0.9492 err=7.77e-04 P=2046 LC=1029
  { 0x5fd7u, 0x0c02u }, // 243: duty=0.9531 err=3.04e-04 P=2047 LC=1028
  { 0x431eu, 0x0af7u }, // 244: duty=0.9570 err=3.20e-04 P=2047 LC=1027
  { 0x9d53u, 0x09e0u }, // 245: duty=0.9609 err=3.35e-04 P=2047 LC=1028
  { 0x103cu, 0x08e5u }, // 246: duty=0.9648 err=1.33e-03 P=2044 LC=1029
  { 0x4d2fu, 0x0b13u }, // 247: duty=0.9683 err=1.83e-03 P=2142 LC=1071
  { 0x0292u, 0x06c8u }, // 248: duty=0.9722 err=1.85e-03 P=2159 LC=1081
  { 0x3072u, 0x05cdu }, // 249: duty=0.9760 err=1.86e-03 P=2170 LC=1085
  { 0xd55du, 0x07dfu }, // 250: duty=0.9802 err=1.88e-03 P=4095 LC=2049
  { 0xdaafu, 0x0542u }, // 251: duty=0.9841 err=1.89e-03 P=4095 LC=2049
  { 0xdee8u, 0x02fbu }, // 252: duty=0.9883 err=2.00e-03 P=6141 LC=3070
  { 0xbf96u, 0x0205u }, // 253: duty=0.9922 err=1.98e-03 P=8191 LC=4096
  { 0x1fcdu, 0x0168u }, // 254: duty=0.9961 err=1.94e-03 P=16383 LC=8194
  { 0xc00du, 0x0008u }, // 255: duty=0.9999 err=9.77e-04 P=16376 LC=16362
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
