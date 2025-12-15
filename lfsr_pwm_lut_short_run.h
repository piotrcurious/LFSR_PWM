// Arduino LFSR PWM LUT: mask, seed (16-bit)
// PWM output: (state > seed) ? HIGH : LOW
// Seed doubles as both starting state AND threshold
// Duty cycle calculated over 65535 cycles
#include <stdint.h>
#if defined(__AVR__)
#include <avr/pgmspace.h>
#else
#define PROGMEM
#define pgm_read_word(addr) (*(addr))
#define pgm_read_dword(addr) (*(addr))
#endif

const struct { uint16_t mask; uint16_t seed; } lfsr_pwm_lut[] PROGMEM = {
  { 0x502bu, 0xfff6u }, // 000: duty=0.00005 err=3.91e-03 P=21845 LC=16381
  { 0x9a17u, 0xfdd7u }, // 001: duty=0.00383 err=7.80e-03 P=3906 LC=1956
  { 0xcf5bu, 0xfac5u }, // 002: duty=0.00717 err=7.84e-03 P=1533 LC=768
  { 0xcd6fu, 0xfac2u }, // 003: duty=0.01100 err=7.86e-03 P=1365 LC=683
  { 0x6e42u, 0xf752u }, // 004: duty=0.01465 err=7.87e-03 P=1023 LC=514
  { 0x27d3u, 0xf67au }, // 005: duty=0.01936 err=7.89e-03 P=930 LC=468
  { 0x26bcu, 0xf647u }, // 006: duty=0.02365 err=7.90e-03 P=930 LC=466
  { 0xd2c5u, 0xd22du }, // 007: duty=0.02921 err=7.92e-03 P=651 LC=330
  { 0x279fu, 0xf5f3u }, // 008: duty=0.03175 err=7.94e-03 P=630 LC=316
  { 0x7e9du, 0xecf5u }, // 009: duty=0.03662 err=4.04e-03 P=273 LC=260
  { 0x8bdfu, 0xf46fu }, // 010: duty=0.03929 err=4.06e-03 P=280 LC=259
  { 0x6db7u, 0xedf3u }, // 011: duty=0.04396 err=4.07e-03 P=273 LC=257
  { 0x37bbu, 0xeecfu }, // 012: duty=0.04762 err=4.09e-03 P=273 LC=258
  { 0xd88bu, 0xca01u }, // 013: duty=0.05068 err=8.01e-03 P=651 LC=325
  { 0x8443u, 0xf9beu }, // 014: duty=0.05814 err=4.12e-03 P=258 LC=257
  { 0xc575u, 0xc865u }, // 015: duty=0.05774 err=8.04e-03 P=381 LC=270
  { 0x021cu, 0xc269u }, // 016: duty=0.06299 err=8.06e-03 P=381 LC=265
  { 0x3559u, 0xebf4u }, // 017: duty=0.06615 err=4.17e-03 P=257 LC=257
  { 0xbc7bu, 0xeca0u }, // 018: duty=0.07366 err=4.18e-03 P=258 LC=257
  { 0x1931u, 0xe9d2u }, // 019: duty=0.07393 err=4.20e-03 P=257 LC=257
  { 0xa0e1u, 0xc00du }, // 020: duty=0.07874 err=8.12e-03 P=381 LC=294
  { 0x021cu, 0xc101u }, // 021: duty=0.08136 err=8.13e-03 P=381 LC=297
  { 0x1931u, 0xe897u }, // 022: duty=0.08560 err=4.24e-03 P=257 LC=256
  { 0x2109u, 0xe6fbu }, // 023: duty=0.08949 err=4.26e-03 P=257 LC=257
  { 0x1931u, 0xe64fu }, // 024: duty=0.09339 err=4.27e-03 P=257 LC=256
  { 0x1931u, 0xe5f5u }, // 025: duty=0.09728 err=4.29e-03 P=257 LC=257
  { 0x1931u, 0xe4cfu }, // 026: duty=0.10117 err=4.30e-03 P=257 LC=256
  { 0x2109u, 0xe4c4u }, // 027: duty=0.10506 err=4.32e-03 P=257 LC=257
  { 0x1931u, 0xe42bu }, // 028: duty=0.10895 err=4.34e-03 P=257 LC=256
  { 0x2109u, 0xe2adu }, // 029: duty=0.11284 err=4.35e-03 P=257 LC=257
  { 0x1931u, 0xe271u }, // 030: duty=0.11673 err=4.37e-03 P=257 LC=256
  { 0xc107u, 0xe99eu }, // 031: duty=0.12062 err=4.38e-03 P=257 LC=257
  { 0x2109u, 0xe107u }, // 032: duty=0.12451 err=4.40e-03 P=257 LC=256
  { 0x1931u, 0xde64u }, // 033: duty=0.12840 err=4.41e-03 P=257 LC=257
  { 0x1931u, 0xdcd1u }, // 034: duty=0.13230 err=4.43e-03 P=257 LC=256
  { 0x1931u, 0xdc85u }, // 035: duty=0.13619 err=4.44e-03 P=257 LC=257
  { 0x1931u, 0xd980u }, // 036: duty=0.14008 err=4.46e-03 P=257 LC=256
  { 0x1931u, 0xd862u }, // 037: duty=0.14397 err=4.47e-03 P=257 LC=257
  { 0x1931u, 0xd427u }, // 038: duty=0.14786 err=4.49e-03 P=257 LC=256
  { 0x1931u, 0xd3a4u }, // 039: duty=0.15175 err=4.50e-03 P=257 LC=257
  { 0x1931u, 0xd324u }, // 040: duty=0.15564 err=4.52e-03 P=257 LC=256
  { 0xeb0du, 0xd5beu }, // 041: duty=0.16078 err=3.28e-03 P=255 LC=255
  { 0xb61au, 0xd569u }, // 042: duty=0.16471 err=3.26e-03 P=255 LC=254
  { 0xb61au, 0xd4f6u }, // 043: duty=0.16863 err=3.25e-03 P=255 LC=255
  { 0xeb0du, 0xd40fu }, // 044: duty=0.17255 err=3.23e-03 P=255 LC=254
  { 0xb61au, 0xd28bu }, // 045: duty=0.17647 err=3.22e-03 P=255 LC=255
  { 0xb61au, 0xd1b5u }, // 046: duty=0.18039 err=3.20e-03 P=255 LC=254
  { 0xb61au, 0xd02au }, // 047: duty=0.18431 err=3.19e-03 P=255 LC=255
  { 0x4609u, 0xce3cu }, // 048: duty=0.18824 err=3.17e-03 P=255 LC=254
  { 0x4609u, 0xcd56u }, // 049: duty=0.19216 err=3.16e-03 P=255 LC=255
  { 0x34e6u, 0xc44fu }, // 050: duty=0.19608 err=3.14e-03 P=255 LC=254
  { 0xb61au, 0xcc9du }, // 051: duty=0.20000 err=3.12e-03 P=255 LC=255
  { 0x4609u, 0xca37u }, // 052: duty=0.20392 err=3.11e-03 P=255 LC=254
  { 0xeb0du, 0xce3du }, // 053: duty=0.20784 err=3.09e-03 P=255 LC=255
  { 0x4609u, 0xc8e8u }, // 054: duty=0.21176 err=3.08e-03 P=255 LC=254
  { 0x34e6u, 0xc055u }, // 055: duty=0.21569 err=3.06e-03 P=255 LC=255
  { 0xafc7u, 0xc672u }, // 056: duty=0.21961 err=3.05e-03 P=255 LC=254
  { 0xb61au, 0xc61au }, // 057: duty=0.22353 err=3.03e-03 P=255 LC=255
  { 0xeb0du, 0xc8b1u }, // 058: duty=0.22745 err=3.02e-03 P=255 LC=254
  { 0xeb0du, 0xc7a3u }, // 059: duty=0.23137 err=3.00e-03 P=255 LC=255
  { 0x34e6u, 0xb9f0u }, // 060: duty=0.23529 err=2.99e-03 P=255 LC=254
  { 0xb61au, 0xc2c6u }, // 061: duty=0.23922 err=2.97e-03 P=255 LC=255
  { 0xeb0du, 0xc167u }, // 062: duty=0.24314 err=2.96e-03 P=255 LC=254
  { 0xb61au, 0xc067u }, // 063: duty=0.24706 err=2.94e-03 P=255 LC=255
  { 0x1931u, 0xbe9du }, // 064: duty=0.25292 err=2.93e-03 P=257 LC=257
  { 0x1931u, 0xbd98u }, // 065: duty=0.25681 err=2.91e-03 P=257 LC=256
  { 0x1931u, 0xbd32u }, // 066: duty=0.26070 err=2.90e-03 P=257 LC=257
  { 0x1931u, 0xbcc9u }, // 067: duty=0.26459 err=2.88e-03 P=257 LC=256
  { 0x1931u, 0xbc4eu }, // 068: duty=0.26848 err=2.86e-03 P=257 LC=257
  { 0x1931u, 0xb9a3u }, // 069: duty=0.27237 err=2.85e-03 P=257 LC=256
  { 0x1931u, 0xb97du }, // 070: duty=0.27626 err=2.83e-03 P=257 LC=257
  { 0x1931u, 0xb90au }, // 071: duty=0.28016 err=2.82e-03 P=257 LC=256
  { 0x1931u, 0xb3b0u }, // 072: duty=0.28405 err=2.80e-03 P=257 LC=257
  { 0x1931u, 0xb301u }, // 073: duty=0.28794 err=2.79e-03 P=257 LC=256
  { 0x2109u, 0xad78u }, // 074: duty=0.29183 err=2.77e-03 P=257 LC=257
  { 0x1931u, 0xafa7u }, // 075: duty=0.29572 err=2.76e-03 P=257 LC=256
  { 0x1931u, 0xa84eu }, // 076: duty=0.29961 err=2.74e-03 P=257 LC=257
  { 0x1931u, 0xa748u }, // 077: duty=0.30350 err=2.73e-03 P=257 LC=256
  { 0x1931u, 0xa649u }, // 078: duty=0.30739 err=2.71e-03 P=257 LC=257
  { 0x1931u, 0xa45eu }, // 079: duty=0.31128 err=2.70e-03 P=257 LC=256
  { 0x1931u, 0xa391u }, // 080: duty=0.31518 err=2.68e-03 P=257 LC=257
  { 0x1931u, 0xa25cu }, // 081: duty=0.31907 err=2.67e-03 P=257 LC=256
  { 0x2109u, 0xa3d6u }, // 082: duty=0.32296 err=2.65e-03 P=257 LC=257
  { 0x2109u, 0xa113u }, // 083: duty=0.32685 err=2.63e-03 P=257 LC=256
  { 0x1931u, 0x9ee4u }, // 084: duty=0.33074 err=2.62e-03 P=257 LC=257
  { 0x1931u, 0x9de6u }, // 085: duty=0.33463 err=2.60e-03 P=257 LC=256
  { 0x1931u, 0x9d86u }, // 086: duty=0.33852 err=2.59e-03 P=257 LC=257
  { 0x1931u, 0x9d22u }, // 087: duty=0.34241 err=2.57e-03 P=257 LC=256
  { 0x1931u, 0x9c59u }, // 088: duty=0.34630 err=2.56e-03 P=257 LC=257
  { 0x1931u, 0x9bd3u }, // 089: duty=0.35019 err=2.54e-03 P=257 LC=256
  { 0x1931u, 0x9a39u }, // 090: duty=0.35409 err=2.53e-03 P=257 LC=257
  { 0x1931u, 0x99eeu }, // 091: duty=0.35798 err=2.51e-03 P=257 LC=256
  { 0x1931u, 0x997bu }, // 092: duty=0.36187 err=2.50e-03 P=257 LC=257
  { 0x1931u, 0x993eu }, // 093: duty=0.36576 err=2.48e-03 P=257 LC=256
  { 0xf93fu, 0xb1ddu }, // 094: duty=0.36965 err=2.47e-03 P=257 LC=257
  { 0x1931u, 0x980fu }, // 095: duty=0.37354 err=2.45e-03 P=257 LC=256
  { 0x2109u, 0x9491u }, // 096: duty=0.37743 err=2.44e-03 P=257 LC=257
  { 0x1931u, 0x97b3u }, // 097: duty=0.38132 err=2.42e-03 P=257 LC=256
  { 0x1931u, 0x972fu }, // 098: duty=0.38521 err=2.41e-03 P=257 LC=257
  { 0x1931u, 0x93e0u }, // 099: duty=0.38911 err=2.39e-03 P=257 LC=256
  { 0xf93fu, 0xa7a2u }, // 100: duty=0.39300 err=2.37e-03 P=257 LC=257
  { 0x1931u, 0x932fu }, // 101: duty=0.39689 err=2.36e-03 P=257 LC=256
  { 0x2109u, 0x9200u }, // 102: duty=0.40078 err=2.34e-03 P=257 LC=257
  { 0x1931u, 0x91e8u }, // 103: duty=0.40467 err=2.33e-03 P=257 LC=256
  { 0x2109u, 0x910bu }, // 104: duty=0.40856 err=2.31e-03 P=257 LC=257
  { 0x1931u, 0x90afu }, // 105: duty=0.41245 err=2.30e-03 P=257 LC=256
  { 0x1931u, 0x8f44u }, // 106: duty=0.41634 err=2.28e-03 P=257 LC=257
  { 0x1931u, 0x8e47u }, // 107: duty=0.42023 err=2.27e-03 P=257 LC=256
  { 0x2109u, 0x8ab7u }, // 108: duty=0.42412 err=2.25e-03 P=257 LC=257
  { 0x1931u, 0x8bc4u }, // 109: duty=0.42802 err=2.24e-03 P=257 LC=256
  { 0xf93fu, 0x9e8bu }, // 110: duty=0.43191 err=2.22e-03 P=257 LC=257
  { 0x1931u, 0x89c5u }, // 111: duty=0.43580 err=2.21e-03 P=257 LC=256
  { 0x1931u, 0x8972u }, // 112: duty=0.43969 err=2.19e-03 P=257 LC=257
  { 0x1931u, 0x8800u }, // 113: duty=0.44358 err=2.18e-03 P=257 LC=256
  { 0x1931u, 0x86e6u }, // 114: duty=0.44747 err=2.16e-03 P=257 LC=257
  { 0x1931u, 0x8620u }, // 115: duty=0.45136 err=2.14e-03 P=257 LC=256
  { 0xf93fu, 0x973fu }, // 116: duty=0.45525 err=2.13e-03 P=257 LC=257
  { 0x1931u, 0x84efu }, // 117: duty=0.45914 err=2.11e-03 P=257 LC=256
  { 0x1931u, 0x80f9u }, // 118: duty=0.46304 err=2.10e-03 P=257 LC=257
  { 0x1931u, 0x80cdu }, // 119: duty=0.46693 err=2.08e-03 P=257 LC=256
  { 0xc107u, 0x917eu }, // 120: duty=0.47082 err=2.07e-03 P=257 LC=257
  { 0x1931u, 0x7372u }, // 121: duty=0.47471 err=2.05e-03 P=257 LC=256
  { 0xc107u, 0x8e5cu }, // 122: duty=0.47860 err=2.04e-03 P=257 LC=257
  { 0x1931u, 0x7c99u }, // 123: duty=0.48249 err=2.02e-03 P=257 LC=256
  { 0xf93fu, 0x8f13u }, // 124: duty=0.48638 err=2.01e-03 P=257 LC=257
  { 0x1931u, 0x7b90u }, // 125: duty=0.49027 err=1.99e-03 P=257 LC=256
  { 0xa38bu, 0x899du }, // 126: duty=0.49416 err=1.98e-03 P=257 LC=257
  { 0x1405u, 0x8091u }, // 127: duty=0.50001 err=5.87e-03 P=510 LC=494
  { 0x1405u, 0x8091u }, // 128: duty=0.50001 err=5.87e-03 P=510 LC=494
  { 0x1501u, 0x7f83u }, // 129: duty=0.50390 err=5.88e-03 P=510 LC=298
  { 0x2109u, 0x7910u }, // 130: duty=0.50973 err=1.99e-03 P=257 LC=257
  { 0x1931u, 0x7799u }, // 131: duty=0.51362 err=2.01e-03 P=257 LC=256
  { 0xf93fu, 0x89b9u }, // 132: duty=0.51751 err=2.02e-03 P=257 LC=257
  { 0x1931u, 0x748bu }, // 133: duty=0.52140 err=2.04e-03 P=257 LC=256
  { 0x2109u, 0x7292u }, // 134: duty=0.52529 err=2.05e-03 P=257 LC=257
  { 0x1931u, 0x72fau }, // 135: duty=0.52918 err=2.07e-03 P=257 LC=256
  { 0x1931u, 0x723du }, // 136: duty=0.53307 err=2.08e-03 P=257 LC=257
  { 0x1931u, 0x7215u }, // 137: duty=0.53696 err=2.10e-03 P=257 LC=256
  { 0xa38bu, 0x785cu }, // 138: duty=0.54086 err=2.11e-03 P=257 LC=257
  { 0x1931u, 0x6f4cu }, // 139: duty=0.54475 err=2.13e-03 P=257 LC=256
  { 0xa38bu, 0x7789u }, // 140: duty=0.54864 err=2.14e-03 P=257 LC=257
  { 0x1931u, 0x68e4u }, // 141: duty=0.55253 err=2.16e-03 P=257 LC=256
  { 0xa38bu, 0x7632u }, // 142: duty=0.55642 err=2.18e-03 P=257 LC=257
  { 0x1931u, 0x6761u }, // 143: duty=0.56031 err=2.19e-03 P=257 LC=256
  { 0x1931u, 0x66f4u }, // 144: duty=0.56420 err=2.21e-03 P=257 LC=257
  { 0x1931u, 0x6603u }, // 145: duty=0.56809 err=2.22e-03 P=257 LC=256
  { 0xf93fu, 0x77e7u }, // 146: duty=0.57198 err=2.24e-03 P=257 LC=257
  { 0x1931u, 0x64f8u }, // 147: duty=0.57588 err=2.25e-03 P=257 LC=256
  { 0x1931u, 0x6499u }, // 148: duty=0.57977 err=2.27e-03 P=257 LC=257
  { 0x1931u, 0x6200u }, // 149: duty=0.58366 err=2.28e-03 P=257 LC=256
  { 0xf93fu, 0x7647u }, // 150: duty=0.58755 err=2.30e-03 P=257 LC=257
  { 0x1931u, 0x6188u }, // 151: duty=0.59144 err=2.31e-03 P=257 LC=256
  { 0x1931u, 0x603eu }, // 152: duty=0.59533 err=2.33e-03 P=257 LC=257
  { 0x2109u, 0x59e5u }, // 153: duty=0.59922 err=2.34e-03 P=257 LC=256
  { 0x1931u, 0x5f4eu }, // 154: duty=0.60311 err=2.36e-03 P=257 LC=257
  { 0x1931u, 0x5eccu }, // 155: duty=0.60700 err=2.37e-03 P=257 LC=256
  { 0x2109u, 0x547au }, // 156: duty=0.61089 err=2.39e-03 P=257 LC=257
  { 0x1931u, 0x5cbeu }, // 157: duty=0.61479 err=2.41e-03 P=257 LC=256
  { 0x2109u, 0x5246u }, // 158: duty=0.61868 err=2.42e-03 P=257 LC=257
  { 0x1931u, 0x57d3u }, // 159: duty=0.62257 err=2.44e-03 P=257 LC=256
  { 0x1931u, 0x509du }, // 160: duty=0.62646 err=2.45e-03 P=257 LC=257
  { 0x1931u, 0x4f80u }, // 161: duty=0.63035 err=2.47e-03 P=257 LC=256
  { 0x2109u, 0x4f08u }, // 162: duty=0.63424 err=2.48e-03 P=257 LC=257
  { 0x1931u, 0x4e91u }, // 163: duty=0.63813 err=2.50e-03 P=257 LC=256
  { 0x1931u, 0x4e2cu }, // 164: duty=0.64202 err=2.51e-03 P=257 LC=257
  { 0x1931u, 0x4cf7u }, // 165: duty=0.64591 err=2.53e-03 P=257 LC=256
  { 0xd557u, 0x5089u }, // 166: duty=0.64981 err=2.54e-03 P=257 LC=257
  { 0x1931u, 0x4c93u }, // 167: duty=0.65370 err=2.56e-03 P=257 LC=256
  { 0x2109u, 0x494eu }, // 168: duty=0.65759 err=2.57e-03 P=257 LC=257
  { 0x1931u, 0x4997u }, // 169: duty=0.66148 err=2.59e-03 P=257 LC=256
  { 0xc107u, 0x5c9fu }, // 170: duty=0.66537 err=2.60e-03 P=257 LC=257
  { 0x1931u, 0x47a2u }, // 171: duty=0.66926 err=2.62e-03 P=257 LC=256
  { 0xf93fu, 0x61dau }, // 172: duty=0.67315 err=2.63e-03 P=257 LC=257
  { 0x1931u, 0x461bu }, // 173: duty=0.67704 err=2.65e-03 P=257 LC=256
  { 0x2109u, 0x464fu }, // 174: duty=0.68093 err=2.67e-03 P=257 LC=257
  { 0x1931u, 0x44b9u }, // 175: duty=0.68482 err=2.68e-03 P=257 LC=256
  { 0xc107u, 0x5440u }, // 176: duty=0.68872 err=2.70e-03 P=257 LC=257
  { 0x1931u, 0x4277u }, // 177: duty=0.69261 err=2.71e-03 P=257 LC=256
  { 0x2109u, 0x42dfu }, // 178: duty=0.69650 err=2.73e-03 P=257 LC=257
  { 0x2109u, 0x4227u }, // 179: duty=0.70039 err=2.74e-03 P=257 LC=256
  { 0xa38bu, 0x53fdu }, // 180: duty=0.70428 err=2.76e-03 P=257 LC=257
  { 0x1931u, 0x3dc8u }, // 181: duty=0.70817 err=2.77e-03 P=257 LC=256
  { 0x1931u, 0x3d12u }, // 182: duty=0.71206 err=2.79e-03 P=257 LC=257
  { 0x1931u, 0x3bccu }, // 183: duty=0.71595 err=2.80e-03 P=257 LC=256
  { 0xc107u, 0x4b3fu }, // 184: duty=0.71984 err=2.82e-03 P=257 LC=257
  { 0x1931u, 0x3a45u }, // 185: duty=0.72374 err=2.83e-03 P=257 LC=256
  { 0x1931u, 0x391eu }, // 186: duty=0.72763 err=2.85e-03 P=257 LC=257
  { 0x1931u, 0x38b3u }, // 187: duty=0.73152 err=2.86e-03 P=257 LC=256
  { 0x1931u, 0x37a6u }, // 188: duty=0.73541 err=2.88e-03 P=257 LC=257
  { 0x1931u, 0x3734u }, // 189: duty=0.73930 err=2.90e-03 P=257 LC=256
  { 0x1931u, 0x3472u }, // 190: duty=0.74319 err=2.91e-03 P=257 LC=257
  { 0x1931u, 0x33dcu }, // 191: duty=0.74708 err=2.93e-03 P=257 LC=256
  { 0x4609u, 0x3e27u }, // 192: duty=0.75294 err=2.94e-03 P=255 LC=254
  { 0xb61au, 0x3e8du }, // 193: duty=0.75686 err=2.96e-03 P=255 LC=255
  { 0xb61au, 0x3db3u }, // 194: duty=0.76078 err=2.97e-03 P=255 LC=254
  { 0xb61au, 0x3c2cu }, // 195: duty=0.76471 err=2.99e-03 P=255 LC=255
  { 0xeb0du, 0x48ddu }, // 196: duty=0.76863 err=3.00e-03 P=255 LC=254
  { 0xb61au, 0x3a51u }, // 197: duty=0.77255 err=3.02e-03 P=255 LC=255
  { 0xeb0du, 0x46a6u }, // 198: duty=0.77647 err=3.03e-03 P=255 LC=254
  { 0xeb0du, 0x458cu }, // 199: duty=0.78039 err=3.05e-03 P=255 LC=255
  { 0xeb0du, 0x4300u }, // 200: duty=0.78431 err=3.06e-03 P=255 LC=254
  { 0xb61au, 0x36abu }, // 201: duty=0.78824 err=3.08e-03 P=255 LC=255
  { 0xb61au, 0x3595u }, // 202: duty=0.79216 err=3.09e-03 P=255 LC=254
  { 0xeb0du, 0x3d52u }, // 203: duty=0.79608 err=3.11e-03 P=255 LC=255
  { 0x34e6u, 0x281bu }, // 204: duty=0.80000 err=3.13e-03 P=255 LC=254
  { 0x4609u, 0x3150u }, // 205: duty=0.80392 err=3.14e-03 P=255 LC=255
  { 0xb61au, 0x3149u }, // 206: duty=0.80784 err=3.16e-03 P=255 LC=254
  { 0xeb0du, 0x3ab7u }, // 207: duty=0.81176 err=3.17e-03 P=255 LC=255
  { 0xeb0du, 0x39d5u }, // 208: duty=0.81569 err=3.19e-03 P=255 LC=254
  { 0x8eccu, 0x2ebdu }, // 209: duty=0.81961 err=3.20e-03 P=255 LC=255
  { 0xb61au, 0x2dfeu }, // 210: duty=0.82353 err=3.22e-03 P=255 LC=254
  { 0xdb05u, 0x2c93u }, // 211: duty=0.82745 err=3.23e-03 P=255 LC=255
  { 0xeb0du, 0x35edu }, // 212: duty=0.83137 err=3.25e-03 P=255 LC=254
  { 0x34e6u, 0x2191u }, // 213: duty=0.83529 err=3.26e-03 P=255 LC=255
  { 0xb61au, 0x2922u }, // 214: duty=0.83922 err=3.28e-03 P=255 LC=254
  { 0xf93fu, 0x2f2cu }, // 215: duty=0.84436 err=4.52e-03 P=257 LC=257
  { 0x2109u, 0x20e1u }, // 216: duty=0.84825 err=4.50e-03 P=257 LC=256
  { 0xa38bu, 0x2ac3u }, // 217: duty=0.85214 err=4.49e-03 P=257 LC=257
  { 0x1931u, 0x1c8fu }, // 218: duty=0.85603 err=4.47e-03 P=257 LC=256
  { 0xc107u, 0x23d7u }, // 219: duty=0.85992 err=4.46e-03 P=257 LC=257
  { 0x1931u, 0x19bdu }, // 220: duty=0.86381 err=4.44e-03 P=257 LC=256
  { 0x2109u, 0x1c3fu }, // 221: duty=0.86770 err=4.43e-03 P=257 LC=257
  { 0x1931u, 0x186eu }, // 222: duty=0.87160 err=4.41e-03 P=257 LC=256
  { 0xa38bu, 0x2637u }, // 223: duty=0.87549 err=4.40e-03 P=257 LC=257
  { 0x1931u, 0x1676u }, // 224: duty=0.87938 err=4.38e-03 P=257 LC=256
  { 0x2109u, 0x156fu }, // 225: duty=0.88327 err=4.37e-03 P=257 LC=257
  { 0x1931u, 0x13bcu }, // 226: duty=0.88716 err=4.35e-03 P=257 LC=256
  { 0x1931u, 0x138bu }, // 227: duty=0.89105 err=4.34e-03 P=257 LC=257
  { 0x1931u, 0x12e5u }, // 228: duty=0.89494 err=4.32e-03 P=257 LC=256
  { 0x2109u, 0x113cu }, // 229: duty=0.89883 err=4.30e-03 P=257 LC=257
  { 0x1931u, 0x1000u }, // 230: duty=0.90272 err=4.29e-03 P=257 LC=256
  { 0xf93fu, 0x1da5u }, // 231: duty=0.90661 err=4.27e-03 P=257 LC=257
  { 0x1931u, 0x0dcdu }, // 232: duty=0.91051 err=4.26e-03 P=257 LC=256
  { 0x8312u, 0x496du }, // 233: duty=0.91337 err=8.15e-03 P=381 LC=296
  { 0x8312u, 0x3ef3u }, // 234: duty=0.91862 err=8.13e-03 P=381 LC=285
  { 0xa38bu, 0x1700u }, // 235: duty=0.92218 err=4.21e-03 P=257 LC=257
  { 0x1931u, 0x09deu }, // 236: duty=0.92607 err=4.20e-03 P=257 LC=256
  { 0x2109u, 0x0b7du }, // 237: duty=0.92996 err=4.18e-03 P=257 LC=257
  { 0x1931u, 0x07c9u }, // 238: duty=0.93385 err=4.17e-03 P=257 LC=256
  { 0xd557u, 0x0d98u }, // 239: duty=0.93774 err=4.15e-03 P=257 LC=257
  { 0x8443u, 0x0641u }, // 240: duty=0.93797 err=4.14e-03 P=258 LC=256
  { 0x57c5u, 0x37b7u }, // 241: duty=0.94487 err=8.03e-03 P=381 LC=264
  { 0x1931u, 0x0337u }, // 242: duty=0.94942 err=4.11e-03 P=257 LC=256
  { 0x2109u, 0x0496u }, // 243: duty=0.95331 err=4.09e-03 P=257 LC=257
  { 0x3bb9u, 0x13dcu }, // 244: duty=0.95698 err=4.07e-03 P=279 LC=257
  { 0xbc7bu, 0x0537u }, // 245: duty=0.95735 err=4.06e-03 P=258 LC=258
  { 0x37bbu, 0x0cb1u }, // 246: duty=0.96336 err=4.04e-03 P=273 LC=257
  { 0x7e26u, 0x14beu }, // 247: duty=0.96825 err=4.03e-03 P=315 LC=258
  { 0x8782u, 0x0f79u }, // 248: duty=0.97142 err=4.01e-03 P=315 LC=258
  { 0x2e5fu, 0x088fu }, // 249: duty=0.97665 err=7.90e-03 P=771 LC=385
  { 0xf7f5u, 0x0943u }, // 250: duty=0.98167 err=7.89e-03 P=819 LC=409
  { 0x9dedu, 0x08a9u }, // 251: duty=0.98338 err=7.75e-03 P=1023 LC=513
  { 0xcd6fu, 0x04e3u }, // 252: duty=0.98900 err=7.86e-03 P=1365 LC=683
  { 0xa613u, 0x01e9u }, // 253: duty=0.99248 err=7.84e-03 P=1860 LC=930
  { 0x9ed9u, 0x00ddu }, // 254: duty=0.99660 err=7.80e-03 P=2667 LC=1334
  { 0xd02du, 0x0004u }, // 255: duty=0.99992 err=3.91e-03 P=32767 LC=16359
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
