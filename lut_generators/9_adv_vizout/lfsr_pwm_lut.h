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
  { 0x43f3u, 0xfff5u }, // 000: duty=0.0001 err=3.91e-03 P=16383 LC=10909
  { 0x58d7u, 0xfefdu }, // 001: duty=0.0039 err=3.92e-03 P=32767 LC=8192
  { 0x186bu, 0xfc91u }, // 002: duty=0.0076 err=7.84e-03 P=4095 LC=2048
  { 0x91d1u, 0xfc2eu }, // 003: duty=0.0113 err=7.86e-03 P=1953 LC=977
  { 0x011fu, 0xf74eu }, // 004: duty=0.0159 err=7.87e-03 P=1953 LC=976
  { 0x01edu, 0xf9b4u }, // 005: duty=0.0194 err=7.89e-03 P=1395 LC=698
  { 0x01edu, 0xf952u }, // 006: duty=0.0229 err=7.90e-03 P=1395 LC=697
  { 0xa9c6u, 0xf436u }, // 007: duty=0.0274 err=7.92e-03 P=1023 LC=511
  { 0x6249u, 0xf1d3u }, // 008: duty=0.0311 err=7.94e-03 P=771 LC=386
  { 0xde4au, 0xf559u }, // 009: duty=0.0366 err=7.95e-03 P=930 LC=465
  { 0x5524u, 0xf4e6u }, // 010: duty=0.0385 err=7.97e-03 P=623 LC=311
  { 0x1b3bu, 0xf468u }, // 011: duty=0.0411 err=7.98e-03 P=657 LC=328
  { 0xaf71u, 0xf129u }, // 012: duty=0.0462 err=8.00e-03 P=585 LC=293
  { 0xd88bu, 0xca01u }, // 013: duty=0.0507 err=8.01e-03 P=651 LC=326
  { 0xc568u, 0xea66u }, // 014: duty=0.0548 err=8.03e-03 P=511 LC=256
  { 0x0847u, 0xe970u }, // 015: duty=0.0587 err=8.04e-03 P=511 LC=256
  { 0xb3e1u, 0xf17cu }, // 016: duty=0.0626 err=8.06e-03 P=511 LC=257
  { 0x4fb4u, 0xea60u }, // 017: duty=0.0665 err=8.07e-03 P=511 LC=256
  { 0x4fb4u, 0xe95du }, // 018: duty=0.0705 err=8.09e-03 P=511 LC=256
  { 0xfd02u, 0xe300u }, // 019: duty=0.0744 err=8.10e-03 P=511 LC=256
  { 0xa0e1u, 0xc00du }, // 020: duty=0.0787 err=8.12e-03 P=381 LC=192
  { 0x61beu, 0xe01bu }, // 021: duty=0.0814 err=8.13e-03 P=381 LC=189
  { 0x43f5u, 0xecdcu }, // 022: duty=0.0850 err=8.15e-03 P=341 LC=171
  { 0xf61au, 0xddcfu }, // 023: duty=0.0892 err=8.16e-03 P=381 LC=191
  { 0xd85au, 0xde50u }, // 024: duty=0.0938 err=8.18e-03 P=341 LC=171
  { 0xd85au, 0xdd85u }, // 025: duty=0.0968 err=8.20e-03 P=341 LC=171
  { 0xc42du, 0xe6f6u }, // 026: duty=0.0997 err=8.21e-03 P=341 LC=170
  { 0x739du, 0xc705u }, // 027: duty=0.1048 err=8.23e-03 P=315 LC=158
  { 0xf1d9u, 0xe35au }, // 028: duty=0.1111 err=8.24e-03 P=315 LC=158
  { 0xf1d9u, 0xe319u }, // 029: duty=0.1143 err=8.26e-03 P=315 LC=157
  { 0xa6cdu, 0xd675u }, // 030: duty=0.1173 err=8.27e-03 P=341 LC=171
  { 0xa6cdu, 0xd620u }, // 031: duty=0.1202 err=8.29e-03 P=341 LC=171
  { 0xf1d9u, 0xe00du }, // 032: duty=0.1238 err=8.30e-03 P=315 LC=158
  { 0x1555u, 0x8a8au }, // 033: duty=0.1299 err=3.40e-03 P=254 LC=135
  { 0x3bb9u, 0xd725u }, // 034: duty=0.1326 err=8.33e-03 P=279 LC=140
  { 0x81feu, 0x8a80u }, // 035: duty=0.1378 err=3.37e-03 P=254 LC=133
  { 0x1555u, 0x8a20u }, // 036: duty=0.1417 err=3.35e-03 P=254 LC=132
  { 0x7dedu, 0xd239u }, // 037: duty=0.1429 err=8.38e-03 P=315 LC=158
  { 0x5fe7u, 0xde51u }, // 038: duty=0.1490 err=3.32e-03 P=255 LC=131
  { 0xe666u, 0x8a00u }, // 039: duty=0.1535 err=3.31e-03 P=254 LC=131
  { 0x5330u, 0xd1aau }, // 040: duty=0.1569 err=3.29e-03 P=255 LC=131
  { 0x5045u, 0x8888u }, // 041: duty=0.1614 err=3.28e-03 P=254 LC=131
  { 0x74dfu, 0xde7au }, // 042: duty=0.1647 err=3.26e-03 P=255 LC=131
  { 0x7c01u, 0xd87cu }, // 043: duty=0.1686 err=3.25e-03 P=255 LC=131
  { 0xe19eu, 0x8828u }, // 044: duty=0.1732 err=3.23e-03 P=254 LC=132
  { 0x659cu, 0xcf01u }, // 045: duty=0.1765 err=3.22e-03 P=255 LC=132
  { 0x8fe3u, 0xcbc8u }, // 046: duty=0.1792 err=8.52e-03 P=279 LC=140
  { 0xd4a9u, 0xc000u }, // 047: duty=0.1843 err=3.19e-03 P=255 LC=132
  { 0x8fe3u, 0xc910u }, // 048: duty=0.1864 err=8.55e-03 P=279 LC=140
  { 0x5cc2u, 0xbe54u }, // 049: duty=0.1922 err=3.16e-03 P=255 LC=131
  { 0xbb19u, 0xc003u }, // 050: duty=0.1961 err=3.14e-03 P=255 LC=132
  { 0x0c5fu, 0xc071u }, // 051: duty=0.2000 err=3.12e-03 P=255 LC=131
  { 0x29c3u, 0xc254u }, // 052: duty=0.2039 err=3.11e-03 P=255 LC=132
  { 0x1e7eu, 0xc7deu }, // 053: duty=0.2078 err=3.09e-03 P=255 LC=131
  { 0x02dcu, 0xbe57u }, // 054: duty=0.2118 err=3.08e-03 P=255 LC=132
  { 0x9bcdu, 0xc000u }, // 055: duty=0.2157 err=3.06e-03 P=255 LC=131
  { 0xc707u, 0x80aau }, // 056: duty=0.2205 err=3.05e-03 P=254 LC=132
  { 0xf9e2u, 0xc006u }, // 057: duty=0.2235 err=3.03e-03 P=255 LC=131
  { 0x54adu, 0xc001u }, // 058: duty=0.2275 err=3.02e-03 P=255 LC=133
  { 0xdc6bu, 0x8082u }, // 059: duty=0.2323 err=3.00e-03 P=254 LC=156
  { 0xdc6bu, 0x802au }, // 060: duty=0.2362 err=2.99e-03 P=254 LC=157
  { 0x5544u, 0x8020u }, // 061: duty=0.2402 err=2.97e-03 P=254 LC=157
  { 0x5544u, 0x800au }, // 062: duty=0.2441 err=2.96e-03 P=254 LC=157
  { 0x1504u, 0x8002u }, // 063: duty=0.2480 err=2.94e-03 P=254 LC=157
  { 0x1504u, 0x8002u }, // 064: duty=0.2480 err=4.89e-03 P=254 LC=157
  { 0x4101u, 0x5541u }, // 065: duty=0.2559 err=2.91e-03 P=254 LC=151
  { 0x0544u, 0x5545u }, // 066: duty=0.2598 err=6.80e-03 P=254 LC=149
  { 0x1504u, 0x5540u }, // 067: duty=0.2638 err=6.79e-03 P=254 LC=146
  { 0xc707u, 0x5514u }, // 068: duty=0.2677 err=6.77e-03 P=254 LC=151
  { 0x1005u, 0x5501u }, // 069: duty=0.2717 err=6.76e-03 P=254 LC=155
  { 0x5441u, 0x5455u }, // 070: duty=0.2756 err=6.74e-03 P=254 LC=147
  { 0x1504u, 0x5501u }, // 071: duty=0.2795 err=6.72e-03 P=254 LC=144
  { 0x1504u, 0x5455u }, // 072: duty=0.2835 err=6.71e-03 P=254 LC=144
  { 0x0003u, 0x8033u }, // 073: duty=0.2863 err=2.79e-03 P=255 LC=149
  { 0x8001u, 0x8019u }, // 074: duty=0.2902 err=2.77e-03 P=255 LC=155
  { 0x8001u, 0x8008u }, // 075: duty=0.2941 err=2.76e-03 P=255 LC=155
  { 0x8001u, 0x8007u }, // 076: duty=0.2980 err=2.74e-03 P=255 LC=155
  { 0x8001u, 0x8002u }, // 077: duty=0.3020 err=2.73e-03 P=255 LC=155
  { 0x8001u, 0x8001u }, // 078: duty=0.3059 err=2.71e-03 P=255 LC=155
  { 0x0003u, 0x8000u }, // 079: duty=0.3098 err=2.70e-03 P=255 LC=155
  { 0xe354u, 0xa1adu }, // 080: duty=0.3137 err=2.68e-03 P=255 LC=130
  { 0xd44du, 0xaea6u }, // 081: duty=0.3176 err=2.67e-03 P=255 LC=130
  { 0x9f68u, 0xbc5au }, // 082: duty=0.3216 err=2.65e-03 P=255 LC=131
  { 0x51c1u, 0x7068u }, // 083: duty=0.3255 err=2.63e-03 P=255 LC=143
  { 0xdfd9u, 0x9eb0u }, // 084: duty=0.3294 err=2.62e-03 P=255 LC=131
  { 0x9bcdu, 0xa792u }, // 085: duty=0.3333 err=2.60e-03 P=255 LC=131
  { 0x5c35u, 0x8083u }, // 086: duty=0.3373 err=2.59e-03 P=255 LC=155
  { 0x5875u, 0x8000u }, // 087: duty=0.3412 err=2.57e-03 P=255 LC=155
  { 0xff46u, 0xb1ffu }, // 088: duty=0.3451 err=2.56e-03 P=255 LC=130
  { 0x5c35u, 0x6d86u }, // 089: duty=0.3490 err=2.54e-03 P=255 LC=154
  { 0xcff5u, 0xa7bau }, // 090: duty=0.3529 err=2.53e-03 P=255 LC=131
  { 0x8032u, 0x9ae5u }, // 091: duty=0.3569 err=2.51e-03 P=255 LC=131
  { 0x5c35u, 0x6c32u }, // 092: duty=0.3608 err=2.50e-03 P=255 LC=133
  { 0xb46bu, 0xa16bu }, // 093: duty=0.3647 err=2.48e-03 P=255 LC=132
  { 0x5911u, 0x809cu }, // 094: duty=0.3686 err=2.47e-03 P=255 LC=155
  { 0x5911u, 0x8000u }, // 095: duty=0.3725 err=2.45e-03 P=255 LC=155
  { 0xed6fu, 0x9800u }, // 096: duty=0.3774 err=2.44e-03 P=257 LC=131
  { 0x5911u, 0x7ea9u }, // 097: duty=0.3804 err=2.42e-03 P=255 LC=155
  { 0xaf4du, 0x80f5u }, // 098: duty=0.3843 err=2.41e-03 P=255 LC=157
  { 0xaf4du, 0x80e0u }, // 099: duty=0.3882 err=2.39e-03 P=255 LC=157
  { 0x51c7u, 0x8084u }, // 100: duty=0.3922 err=2.37e-03 P=255 LC=157
  { 0x51c7u, 0x8002u }, // 101: duty=0.3961 err=2.36e-03 P=255 LC=157
  { 0x092du, 0x80d5u }, // 102: duty=0.4000 err=2.34e-03 P=255 LC=155
  { 0x62f7u, 0x80a1u }, // 103: duty=0.4039 err=2.33e-03 P=255 LC=157
  { 0x62f7u, 0x801fu }, // 104: duty=0.4078 err=2.31e-03 P=255 LC=157
  { 0xde8du, 0x8003u }, // 105: duty=0.4118 err=2.30e-03 P=255 LC=157
  { 0x2836u, 0x8137u }, // 106: duty=0.4157 err=2.28e-03 P=255 LC=159
  { 0xe6d3u, 0x800au }, // 107: duty=0.4196 err=2.27e-03 P=255 LC=159
  { 0x0d1du, 0x8139u }, // 108: duty=0.4235 err=2.25e-03 P=255 LC=158
  { 0x0d1du, 0x804cu }, // 109: duty=0.4275 err=2.24e-03 P=255 LC=158
  { 0xfbe8u, 0x8005u }, // 110: duty=0.4314 err=2.22e-03 P=255 LC=158
  { 0x793au, 0x80ddu }, // 111: duty=0.4353 err=2.21e-03 P=255 LC=156
  { 0x0ceau, 0x8001u }, // 112: duty=0.4392 err=2.19e-03 P=255 LC=156
  { 0x2468u, 0x8093u }, // 113: duty=0.4431 err=2.18e-03 P=255 LC=158
  { 0x2468u, 0x8005u }, // 114: duty=0.4471 err=2.16e-03 P=255 LC=158
  { 0x1411u, 0x8c25u }, // 115: duty=0.4529 err=9.96e-03 P=510 LC=255
  { 0x1411u, 0x8b48u }, // 116: duty=0.4549 err=6.04e-03 P=510 LC=254
  { 0x5055u, 0x8ad7u }, // 117: duty=0.4608 err=9.93e-03 P=510 LC=256
  { 0x1411u, 0x89ecu }, // 118: duty=0.4627 err=6.00e-03 P=510 LC=256
  { 0x1411u, 0x889eu }, // 119: duty=0.4667 err=5.99e-03 P=510 LC=255
  { 0x0451u, 0x8749u }, // 120: duty=0.4706 err=5.97e-03 P=510 LC=256
  { 0x1051u, 0x8626u }, // 121: duty=0.4745 err=5.96e-03 P=510 LC=256
  { 0x1411u, 0x85f0u }, // 122: duty=0.4784 err=5.94e-03 P=510 LC=256
  { 0x0451u, 0x84c9u }, // 123: duty=0.4824 err=5.93e-03 P=510 LC=256
  { 0x0151u, 0x83fcu }, // 124: duty=0.4863 err=5.91e-03 P=510 LC=257
  { 0x5415u, 0x8247u }, // 125: duty=0.4902 err=5.90e-03 P=510 LC=256
  { 0x0451u, 0x81a7u }, // 126: duty=0.4941 err=5.88e-03 P=510 LC=256
  { 0x1411u, 0x80b9u }, // 127: duty=0.4980 err=1.96e-03 P=510 LC=256
  { 0x0151u, 0x80a4u }, // 128: duty=0.5000 err=5.87e-03 P=510 LC=187
  { 0x1441u, 0x7e99u }, // 129: duty=0.5059 err=5.88e-03 P=510 LC=274
  { 0x1441u, 0x7ddeu }, // 130: duty=0.5098 err=5.90e-03 P=510 LC=257
  { 0x1411u, 0x7c44u }, // 131: duty=0.5137 err=5.91e-03 P=510 LC=255
  { 0x0445u, 0x7b89u }, // 132: duty=0.5176 err=5.93e-03 P=510 LC=255
  { 0x1501u, 0x7accu }, // 133: duty=0.5216 err=5.94e-03 P=510 LC=255
  { 0x1501u, 0x79b2u }, // 134: duty=0.5255 err=5.96e-03 P=510 LC=256
  { 0x5005u, 0x7903u }, // 135: duty=0.5294 err=5.97e-03 P=510 LC=255
  { 0x0445u, 0x77ddu }, // 136: duty=0.5333 err=9.64e-03 P=510 LC=255
  { 0x4015u, 0x7657u }, // 137: duty=0.5373 err=9.62e-03 P=510 LC=256
  { 0x0451u, 0x7567u }, // 138: duty=0.5412 err=9.93e-03 P=510 LC=256
  { 0x4015u, 0x757eu }, // 139: duty=0.5451 err=9.59e-03 P=510 LC=258
  { 0x78b7u, 0x7c0fu }, // 140: duty=0.5490 err=2.14e-03 P=255 LC=151
  { 0x0451u, 0x72eau }, // 141: duty=0.5529 err=9.97e-03 P=510 LC=255
  { 0x755du, 0x713cu }, // 142: duty=0.5564 err=2.18e-03 P=257 LC=130
  { 0x4791u, 0x83d4u }, // 143: duty=0.5608 err=2.19e-03 P=255 LC=156
  { 0x13c5u, 0x8001u }, // 144: duty=0.5647 err=2.21e-03 P=255 LC=156
  { 0x100au, 0x748au }, // 145: duty=0.5686 err=2.22e-03 P=255 LC=131
  { 0x4791u, 0x7f87u }, // 146: duty=0.5725 err=2.24e-03 P=255 LC=138
  { 0x4791u, 0x7f59u }, // 147: duty=0.5765 err=2.25e-03 P=255 LC=135
  { 0x0445u, 0x6bbeu }, // 148: duty=0.5824 err=9.45e-03 P=510 LC=255
  { 0x3cc5u, 0x53bau }, // 149: duty=0.5843 err=2.28e-03 P=255 LC=131
  { 0x8103u, 0x5610u }, // 150: duty=0.5875 err=2.30e-03 P=257 LC=130
  { 0x181fu, 0x671eu }, // 151: duty=0.5922 err=2.31e-03 P=255 LC=130
  { 0x9f68u, 0x73c0u }, // 152: duty=0.5961 err=2.33e-03 P=255 LC=131
  { 0xe591u, 0x4f88u }, // 153: duty=0.6000 err=2.34e-03 P=255 LC=131
  { 0x8103u, 0x5243u }, // 154: duty=0.6031 err=2.36e-03 P=257 LC=130
  { 0x7233u, 0x4bc9u }, // 155: duty=0.6078 err=2.37e-03 P=255 LC=131
  { 0x1931u, 0x5e27u }, // 156: duty=0.6109 err=2.39e-03 P=257 LC=130
  { 0x8324u, 0x6198u }, // 157: duty=0.6157 err=2.41e-03 P=255 LC=130
  { 0x1876u, 0x60a1u }, // 158: duty=0.6196 err=2.42e-03 P=255 LC=131
  { 0x1931u, 0x57d3u }, // 159: duty=0.6226 err=2.44e-03 P=257 LC=130
  { 0xb55bu, 0x5c6eu }, // 160: duty=0.6275 err=2.45e-03 P=255 LC=131
  { 0xb771u, 0x3f8eu }, // 161: duty=0.6314 err=2.47e-03 P=255 LC=132
  { 0xe2a9u, 0x5128u }, // 162: duty=0.6353 err=2.48e-03 P=255 LC=132
  { 0x49abu, 0x4d43u }, // 163: duty=0.6392 err=2.50e-03 P=255 LC=131
  { 0x1ab3u, 0x53fau }, // 164: duty=0.6431 err=2.51e-03 P=255 LC=131
  { 0x2417u, 0x538fu }, // 165: duty=0.6471 err=2.53e-03 P=255 LC=131
  { 0xe9b3u, 0x4cefu }, // 166: duty=0.6510 err=2.54e-03 P=255 LC=131
  { 0x6fedu, 0x440cu }, // 167: duty=0.6537 err=2.56e-03 P=257 LC=131
  { 0xd324u, 0x560eu }, // 168: duty=0.6588 err=2.57e-03 P=255 LC=133
  { 0x6995u, 0x4977u }, // 169: duty=0.6627 err=2.59e-03 P=255 LC=131
  { 0x1cb2u, 0x54c6u }, // 170: duty=0.6667 err=2.60e-03 P=255 LC=132
  { 0x6d97u, 0x4000u }, // 171: duty=0.6706 err=2.62e-03 P=255 LC=131
  { 0x1dc3u, 0x3780u }, // 172: duty=0.6745 err=2.63e-03 P=255 LC=131
  { 0xe6d3u, 0x3c12u }, // 173: duty=0.6784 err=2.65e-03 P=255 LC=131
  { 0x4353u, 0x4012u }, // 174: duty=0.6824 err=2.67e-03 P=255 LC=132
  { 0x6fedu, 0x3f2au }, // 175: duty=0.6848 err=2.68e-03 P=257 LC=131
  { 0xad6bu, 0x41e4u }, // 176: duty=0.6887 err=2.70e-03 P=257 LC=131
  { 0xb738u, 0x4008u }, // 177: duty=0.6941 err=2.71e-03 P=255 LC=133
  { 0x4daau, 0x4136u }, // 178: duty=0.6980 err=2.73e-03 P=255 LC=131
  { 0x9170u, 0x40b2u }, // 179: duty=0.7020 err=2.74e-03 P=255 LC=132
  { 0xfdb2u, 0x3c45u }, // 180: duty=0.7059 err=2.76e-03 P=255 LC=131
  { 0x2d21u, 0x40a4u }, // 181: duty=0.7098 err=2.77e-03 P=255 LC=133
  { 0xd69du, 0x3128u }, // 182: duty=0.7137 err=2.79e-03 P=255 LC=131
  { 0xc905u, 0x4493u }, // 183: duty=0.7176 err=2.80e-03 P=255 LC=132
  { 0x0ebfu, 0x3cf4u }, // 184: duty=0.7216 err=2.82e-03 P=255 LC=132
  { 0xb46bu, 0x45f3u }, // 185: duty=0.7255 err=2.83e-03 P=255 LC=134
  { 0xaed6u, 0x416du }, // 186: duty=0.7294 err=2.85e-03 P=255 LC=132
  { 0xab11u, 0x4192u }, // 187: duty=0.7333 err=2.86e-03 P=255 LC=131
  { 0x49c8u, 0x4396u }, // 188: duty=0.7373 err=2.88e-03 P=255 LC=135
  { 0x65e4u, 0x427du }, // 189: duty=0.7412 err=2.90e-03 P=255 LC=135
  { 0x48c5u, 0x410eu }, // 190: duty=0.7451 err=2.91e-03 P=255 LC=135
  { 0x48c5u, 0x4056u }, // 191: duty=0.7490 err=2.93e-03 P=255 LC=135
  { 0x7461u, 0x3e75u }, // 192: duty=0.7529 err=2.94e-03 P=255 LC=132
  { 0x179fu, 0x3e02u }, // 193: duty=0.7569 err=2.96e-03 P=255 LC=134
  { 0x5084u, 0x3d46u }, // 194: duty=0.7608 err=2.97e-03 P=255 LC=132
  { 0xa009u, 0x41f0u }, // 195: duty=0.7647 err=2.99e-03 P=255 LC=131
  { 0x920au, 0x3dffu }, // 196: duty=0.7686 err=3.00e-03 P=255 LC=131
  { 0xc393u, 0x3134u }, // 197: duty=0.7725 err=3.02e-03 P=255 LC=132
  { 0x467cu, 0x3ed2u }, // 198: duty=0.7765 err=3.03e-03 P=255 LC=131
  { 0x2d95u, 0x2d87u }, // 199: duty=0.7804 err=3.05e-03 P=255 LC=131
  { 0xf605u, 0x3208u }, // 200: duty=0.7843 err=3.06e-03 P=255 LC=131
  { 0xda3du, 0x3e7cu }, // 201: duty=0.7882 err=3.08e-03 P=255 LC=132
  { 0xbdb7u, 0x3e5au }, // 202: duty=0.7922 err=3.09e-03 P=255 LC=131
  { 0x9ed5u, 0x2095u }, // 203: duty=0.7961 err=3.11e-03 P=255 LC=131
  { 0x0276u, 0x339eu }, // 204: duty=0.8000 err=3.13e-03 P=255 LC=130
  { 0x2277u, 0x329cu }, // 205: duty=0.8039 err=3.14e-03 P=255 LC=131
  { 0xa5b4u, 0x31c6u }, // 206: duty=0.8078 err=3.16e-03 P=255 LC=131
  { 0x4807u, 0x2fd5u }, // 207: duty=0.8118 err=3.17e-03 P=255 LC=130
  { 0x3f4au, 0x2696u }, // 208: duty=0.8157 err=3.19e-03 P=255 LC=133
  { 0x15f0u, 0x286du }, // 209: duty=0.8196 err=3.20e-03 P=255 LC=131
  { 0x260au, 0x382cu }, // 210: duty=0.8235 err=3.22e-03 P=255 LC=130
  { 0xdc89u, 0x2cd7u }, // 211: duty=0.8275 err=3.23e-03 P=255 LC=131
  { 0x65e4u, 0x2bbfu }, // 212: duty=0.8314 err=3.25e-03 P=255 LC=130
  { 0x5c35u, 0x0d14u }, // 213: duty=0.8353 err=3.26e-03 P=255 LC=131
  { 0x8bcdu, 0x1d07u }, // 214: duty=0.8392 err=3.28e-03 P=255 LC=131
  { 0x8fe3u, 0x2b63u }, // 215: duty=0.8423 err=8.43e-03 P=279 LC=140
  { 0x371fu, 0x2de3u }, // 216: duty=0.8476 err=8.41e-03 P=315 LC=158
  { 0xbc2du, 0x1da5u }, // 217: duty=0.8510 err=3.32e-03 P=255 LC=131
  { 0x0003u, 0x0100u }, // 218: duty=0.8549 err=3.34e-03 P=255 LC=142
  { 0x0003u, 0x00ffu }, // 219: duty=0.8588 err=3.35e-03 P=255 LC=142
  { 0x949du, 0x3f1bu }, // 220: duty=0.8635 err=8.35e-03 P=315 LC=157
  { 0x8001u, 0x00ccu }, // 221: duty=0.8667 err=3.39e-03 P=255 LC=155
  { 0x0003u, 0x00c0u }, // 222: duty=0.8706 err=3.40e-03 P=255 LC=139
  { 0x8001u, 0x00aau }, // 223: duty=0.8745 err=3.42e-03 P=255 LC=157
  { 0xa6cdu, 0x298au }, // 224: duty=0.8798 err=8.29e-03 P=341 LC=170
  { 0xa6cdu, 0x28b8u }, // 225: duty=0.8827 err=8.27e-03 P=341 LC=170
  { 0x8001u, 0x0080u }, // 226: duty=0.8863 err=3.46e-03 P=255 LC=158
  { 0x0003u, 0x007fu }, // 227: duty=0.8902 err=3.48e-03 P=255 LC=158
  { 0x8001u, 0x0066u }, // 228: duty=0.8941 err=3.49e-03 P=255 LC=155
  { 0x7836u, 0x185bu }, // 229: duty=0.9003 err=8.21e-03 P=341 LC=171
  { 0xd5fbu, 0x1e1au }, // 230: duty=0.9029 err=8.20e-03 P=381 LC=191
  { 0xd5fbu, 0x1ad0u }, // 231: duty=0.9055 err=8.18e-03 P=381 LC=191
  { 0xc4f7u, 0x18acu }, // 232: duty=0.9108 err=8.16e-03 P=381 LC=191
  { 0xc4f7u, 0x1869u }, // 233: duty=0.9134 err=8.15e-03 P=381 LC=191
  { 0xc4f7u, 0x16f1u }, // 234: duty=0.9186 err=8.13e-03 P=381 LC=191
  { 0xd091u, 0x150du }, // 235: duty=0.9213 err=8.12e-03 P=381 LC=191
  { 0x0383u, 0x1cf1u }, // 236: duty=0.9256 err=8.10e-03 P=511 LC=256
  { 0xdf39u, 0x1854u }, // 237: duty=0.9295 err=8.09e-03 P=511 LC=257
  { 0x4fb4u, 0x1570u }, // 238: duty=0.9335 err=8.07e-03 P=511 LC=256
  { 0xd756u, 0x15deu }, // 239: duty=0.9374 err=8.06e-03 P=511 LC=256
  { 0x0847u, 0x149eu }, // 240: duty=0.9413 err=8.04e-03 P=511 LC=257
  { 0xc568u, 0x145du }, // 241: duty=0.9452 err=8.03e-03 P=511 LC=256
  { 0x1c31u, 0x0eb5u }, // 242: duty=0.9491 err=8.01e-03 P=511 LC=256
  { 0x12deu, 0x131bu }, // 243: duty=0.9530 err=8.00e-03 P=511 LC=256
  { 0x6352u, 0x0cd1u }, // 244: duty=0.9573 err=7.98e-03 P=585 LC=292
  { 0xdc7cu, 0x0bc6u }, // 245: duty=0.9608 err=7.97e-03 P=765 LC=383
  { 0x25a7u, 0x0b93u }, // 246: duty=0.9647 err=7.95e-03 P=651 LC=326
  { 0xdc7cu, 0x0939u }, // 247: duty=0.9686 err=7.94e-03 P=765 LC=382
  { 0x7d25u, 0x0b83u }, // 248: duty=0.9726 err=7.92e-03 P=1023 LC=512
  { 0x4f32u, 0x0a28u }, // 249: duty=0.9763 err=7.90e-03 P=1057 LC=528
  { 0x6a99u, 0x04d6u }, // 250: duty=0.9805 err=7.89e-03 P=1285 LC=644
  { 0xa3c8u, 0x03c5u }, // 251: duty=0.9853 err=7.87e-03 P=1365 LC=684
  { 0xb0deu, 0x02f0u }, // 252: duty=0.9882 err=7.86e-03 P=1953 LC=977
  { 0x2ae9u, 0x031du }, // 253: duty=0.9924 err=7.84e-03 P=4369 LC=2185
  { 0x909au, 0x0100u }, // 254: duty=0.9961 err=3.92e-03 P=32767 LC=8191
  { 0x93afu, 0x0002u }, // 255: duty=1.0000 err=3.91e-03 P=47523 LC=15790
};

/* Example Arduino usage:

uint16_t lfsr_state;
uint16_t lfsr_mask;
uint16_t lfsr_threshold;

void setup_pwm(uint8_t level) {
  lfsr_mask = pgm_read_word(&lfsr_pwm_lut[level].mask);
  lfsr_threshold = pgm_read_word(&lfsr_pwm_lut[level].seed);
  lfsr_state = lfsr_threshold;  // Start at threshold value (keeps behavior consistent with search)
}

void pwm_update() {
  // Output based on comparison
  digitalWrite(PWM_PIN, (lfsr_state > lfsr_threshold) ? HIGH : LOW);
  
  // Advance LFSR state
  uint16_t feedback = __builtin_parity(lfsr_state & lfsr_mask) & 1;
  lfsr_state = (lfsr_state >> 1) | (feedback << 15);
}
*/
