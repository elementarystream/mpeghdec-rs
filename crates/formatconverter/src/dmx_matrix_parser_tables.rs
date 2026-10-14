// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// Tables of libFormatConverter/src/FDK_dmxMatrixParser.cpp, generated with
// scripts/codegen/dump_table.py.

/// `eqMinRanges` (exp = 5)
pub static EQ_MIN_RANGES: [[i32; 4]; 2] = [
    [0xE0000000u32 as i32, 0xE0000000u32 as i32, 0xE0000000u32 as i32, 0xE6666666u32 as i32],
    [0xC0000000u32 as i32, 0xC0000000u32 as i32, 0xC0000000u32 as i32, 0xCCCCCCCDu32 as i32],
];

/// `compactTemplate_CICP13_to_CICP6`: 22_2 to 5_1
pub static COMPACT_TEMPLATE_CICP13_TO_CICP6: [i8; 60] = [
    1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0,
    0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0,
];

/// `compactTemplate_CICP14_to_CICP6`: 5_2_1 to 5_1
pub static COMPACT_TEMPLATE_CICP14_TO_CICP6: [i8; 20] = [
    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 0,
];

/// `compactTemplate_CICP12_to_CICP6`: 7_1 to 5_1
pub static COMPACT_TEMPLATE_CICP12_TO_CICP6: [i8; 20] = [
    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 1,
];

/// `compactTemplate_CICP7_to_CICP6`: 7_1_ALT to 5_1
pub static COMPACT_TEMPLATE_CICP7_TO_CICP6: [i8; 20] = [
    1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 0, 1,
];

/// `compactTemplate_CICP13_to_CICP14`: 22_2 to 5_2_1
pub static COMPACT_TEMPLATE_CICP13_TO_CICP14: [i8; 75] = [
    1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1,
    0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
];

/// `compactTemplate_CICP13_to_CICP12`: 22_2 to 7_1
pub static COMPACT_TEMPLATE_CICP13_TO_CICP12: [i8; 75] = [
    1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1,
    1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0,
    0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
];

/// `compactTemplate_CICP13_to_CICP7`: 22_2 to 7_1_ALT
pub static COMPACT_TEMPLATE_CICP13_TO_CICP7: [i8; 75] = [
    0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0,
    1, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 0, 1, 0,
    0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0,
];

/// `compactTemplate_CICP13_to_CICP2`: 22_2 to 2_0
pub static COMPACT_TEMPLATE_CICP13_TO_CICP2: [i8; 15] = [
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
];

/// `compactTemplates_InputIndex`
pub static COMPACT_TEMPLATES_INPUT_INDEX: [i16; 8] = [
    13, 13, 13, 13, 14, 12, 7, 13,
];

/// `compactTemplates_OutputIndex`
pub static COMPACT_TEMPLATES_OUTPUT_INDEX: [i16; 8] = [
    6, 14, 12, 7, 6, 6, 6, 2,
];
