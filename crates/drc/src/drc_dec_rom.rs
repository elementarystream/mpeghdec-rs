// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libDRCdec/src/drcDec_rom.cpp, drcDec_rom.h.

//! Constant tables of the DRC decoder (generated with `scripts/codegen/dump_table.py`).

use mpeghdec_fdk::{FixpDbl, FixpSgl};

/// `FILTER_BANK_PARAMETERS`
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FilterBankParameters {
    pub f_cross_norm: FixpDbl,
    pub gamma: FixpDbl,
    pub delta: FixpDbl,
}

const fn fbp(f_cross_norm: i32, gamma: i32, delta: i32) -> FilterBankParameters {
    FilterBankParameters { f_cross_norm, gamma, delta }
}

/// `deltaGain_codingProfile_0_1_huffman`
pub static DELTA_GAIN_CODING_PROFILE_0_1_HUFFMAN: [[i8; 2]; 24] = [
    [1, 2],
    [3, 4],
    [-63, -65],
    [5, -66],
    [-64, 6],
    [-80, 7],
    [8, 9],
    [-68, 10],
    [11, 12],
    [-56, -67],
    [-61, 13],
    [-62, -69],
    [14, 15],
    [16, -72],
    [-71, 17],
    [-70, -60],
    [18, -59],
    [19, 20],
    [21, -79],
    [-57, -73],
    [22, -58],
    [-76, 23],
    [-75, -74],
    [-78, -77],
];

/// `deltaGain_codingProfile_2_huffman`
pub static DELTA_GAIN_CODING_PROFILE_2_HUFFMAN: [[i8; 2]; 48] = [
    [1, 2],
    [3, 4],
    [5, 6],
    [7, 8],
    [9, 10],
    [11, 12],
    [13, -65],
    [14, -64],
    [15, -66],
    [16, -67],
    [17, 18],
    [19, -68],
    [20, -63],
    [-69, 21],
    [-59, 22],
    [-61, -62],
    [-60, 23],
    [24, -58],
    [-70, -57],
    [-56, -71],
    [25, 26],
    [27, -55],
    [-72, 28],
    [-54, 29],
    [-53, 30],
    [-73, -52],
    [31, -74],
    [32, 33],
    [-75, 34],
    [-76, 35],
    [-51, 36],
    [-78, 37],
    [-77, 38],
    [-96, 39],
    [-48, 40],
    [-50, -79],
    [41, 42],
    [-80, -81],
    [-82, 43],
    [44, -49],
    [45, -84],
    [-83, -89],
    [-86, 46],
    [-90, -85],
    [-91, -93],
    [-92, 47],
    [-88, -87],
    [-95, -94],
];

/// `slopeSteepness` (e = 2)
pub static SLOPE_STEEPNESS: [FixpSgl; 15] =
    [-25000, -10000, -4000, -1600, -640, -256, -41, 0, 41, 256, 640, 1600, 4000, 10000, 25000];

/// `slopeSteepness_huffman`
pub static SLOPE_STEEPNESS_HUFFMAN: [[i8; 2]; 14] = [
    [1, -57],
    [-58, 2],
    [3, 4],
    [5, 6],
    [7, -56],
    [8, -60],
    [-61, -55],
    [9, -59],
    [10, -54],
    [-64, 11],
    [-51, 12],
    [-62, -50],
    [-63, 13],
    [-52, -53],
];

/// `downmixCoeff` (e = 2)
pub static DOWNMIX_COEFF: [FixpDbl; 16] = [
    0x20000000, 0x1E35BF28, 0x1C8520B0, 0x1AECB581, 0x196B230C, 0x17FF2224, 0x16A77DEB, 0x156312C9,
    0x1430CD75, 0x130FAA0E, 0x11FEB33C, 0x10FD0160, 0x1009B9CF, 0x0D7E89AA, 0x0B5AA19C, 0x00000000,
];

/// `downmixCoeffV1` (e = 2)
pub static DOWNMIX_COEFF_V1: [FixpDbl; 32] = [
    0x653160EB, 0x3FD9305B, 0x35B8C334, 0x2D3381F0, 0x260835D3, 0x20000000, 0x1E35BF28, 0x1C8520B0,
    0x1AECB581, 0x196B230C, 0x17FF2224, 0x16A77DEB, 0x156312C9, 0x1430CD75, 0x130FAA0E, 0x11FEB33C,
    0x10FD0160, 0x1009B9CF, 0x0F240E1C, 0x0E4B3B64, 0x0D7E89AA, 0x0CBD4B40, 0x0B5AA19C, 0x0A1E89B1,
    0x0904D1BD, 0x0809BCC4, 0x05B0C439, 0x03333333, 0x01CCAB86, 0x01030DC5, 0x0051EB85, 0x00000000,
];

/// `filterBankParameters`
pub static FILTER_BANK_PARAMETERS: [FilterBankParameters; 16] = [
    fbp(0x00400000, 0x0001391B, 0x7EE4E280),
    fbp(0x00600000, 0x0002BD76, 0x7E583F00),
    fbp(0x00800000, 0x0004D9B3, 0x7DCC3780),
    fbp(0x00A00000, 0x00078BDD, 0x7D40CB00),
    fbp(0x00C00000, 0x000AD208, 0x7CB5F900),
    fbp(0x01000000, 0x001312CF, 0x7BA22100),
    fbp(0x01800000, 0x002A32B7, 0x79818F00),
    fbp(0x02000000, 0x0049C7A5, 0x776A5200),
    fbp(0x03000000, 0x00A0A9A3, 0x73571C00),
    fbp(0x04000000, 0x0114A670, 0x6F66FF80),
    fbp(0x06000000, 0x02495754, 0x67EA1A80),
    fbp(0x08000000, 0x03D58D40, 0x60E7E100),
    fbp(0x0C000000, 0x07BE6A40, 0x54277E00),
    fbp(0x10000000, 0x0C7F2CD0, 0x48D62880),
    fbp(0x18000000, 0x17E599A0, 0x35866080),
    fbp(0x20000000, 0x257D8680, 0x257D8680),
];
