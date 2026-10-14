// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Interface of libMpeghDec to libFormatConverter (the IIS_FormatConverter_*() calls of
// aacdecoder.cpp and aacdecoder_lib.cpp).

//! Interface of the core decoder to the format converter ([`mpeghdec_formatconverter`]).
//!
//! All calls of the frame level into the format converter go through this module:
//! `IIS_FormatConverter_*()`, `FormatConverterFrame()` and `DownmixMatrixSet()`.
//! `applyDownmixMatrixPerSignalGroup()`, which works on the format converter internals, is in
//! [`crate::aacdecoder`].

use mpeghdec_drc::DrcDecoder;
use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::cicp2geometry::Cicp2GeometryChannelGeometry;
use mpeghdec_fdk::common_fix::FixpDbl;
use mpeghdec_fdk::thread_pool::ThreadPool;
use mpeghdec_formatconverter::*;

pub use mpeghdec_formatconverter::{
    FdkDownmixGroupsMatrixSet, IisFormatConverter, IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT,
};

/// `IIS_FORMATCONVERTER` (`IIS_FORMATCONVERTER_HANDLE` is `Box<FormatConverter>`)
pub type FormatConverter = IisFormatConverter;

/// `FDK_DOWNMIX_GROUPS_MATRIX_SET`
pub type DownmixGroupsMatrixSet = FdkDownmixGroupsMatrixSet;

const DMX_MEM_BYTES: usize =
    FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID * FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE;
const DMX_SIZE_BYTES: usize = FDK_DOWNMIX_CONFIG_MAX_DMX_MATRICES_PER_ID * 4;

/// `sizeof(FDK_DOWNMIX_GROUPS_MATRIX_SET)` in `FIXP_DBL` words: `downmixMatrixMemory[9][256]`,
/// `downmixMatrixSize[9]` (`INT`), `downmixMatrix[FDK_MPEGHAUDIO_DEC_MAX_SIGNAL_GROUPS]`.
pub const DOWNMIX_GROUPS_MATRIX_SET_WORDS: usize =
    (DMX_MEM_BYTES + DMX_SIZE_BYTES + FDK_MPEGHAUDIO_DEC_MAX_SIGNAL_GROUPS).div_ceil(4);

/// Byte `k` of the `FIXP_DBL` buffer `mem` in memory order (little endian).
fn mem_byte(mem: &[FixpDbl], k: usize) -> u8 {
    mem[k / 4].to_le_bytes()[k % 4]
}

fn set_mem_byte(mem: &mut [FixpDbl], k: usize, v: u8) {
    let mut b = mem[k / 4].to_le_bytes();
    b[k % 4] = v;
    mem[k / 4] = FixpDbl::from_le_bytes(b);
}

/// Stores the downmix matrix set in the memory the C++ decoder borrows for it (`pTimeData2`),
/// with the layout of `FDK_DOWNMIX_GROUPS_MATRIX_SET`.
pub fn store_downmix_matrix_set(set: &DownmixGroupsMatrixSet, mem: &mut [FixpDbl]) {
    for (m, row) in set.downmix_matrix_memory.iter().enumerate() {
        for (i, &b) in row.iter().enumerate() {
            set_mem_byte(mem, m * FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE + i, b);
        }
    }
    for (m, &v) in set.downmix_matrix_size.iter().enumerate() {
        mem[(DMX_MEM_BYTES + 4 * m) / 4] = v;
    }
    for (g, &b) in set.downmix_matrix.iter().enumerate() {
        set_mem_byte(mem, DMX_MEM_BYTES + DMX_SIZE_BYTES + g, b);
    }
}

/// Reads the downmix matrix set from the memory the C++ decoder borrows for it (`pTimeData2`).
pub fn load_downmix_matrix_set(mem: &[FixpDbl]) -> DownmixGroupsMatrixSet {
    let mut set = DownmixGroupsMatrixSet::default();
    for (m, row) in set.downmix_matrix_memory.iter_mut().enumerate() {
        for (i, b) in row.iter_mut().enumerate() {
            *b = mem_byte(mem, m * FDK_DOWNMIX_CONFIG_MAX_DMX_MATRIX_SIZE + i);
        }
    }
    for (m, v) in set.downmix_matrix_size.iter_mut().enumerate() {
        *v = mem[(DMX_MEM_BYTES + 4 * m) / 4];
    }
    for (g, b) in set.downmix_matrix.iter_mut().enumerate() {
        *b = mem_byte(mem, DMX_MEM_BYTES + DMX_SIZE_BYTES + g);
    }
    set
}

/// `FORMAT_CONVERTER_MAX_CHANNELS`
pub const FORMAT_CONVERTER_MAX_CHANNELS: usize = 28;
/// `FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS`
pub const FDK_FORMAT_CONVERTER_MAX_INPUT_CHANNELS: u32 = 28;
/// `FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS`
pub const FDK_FORMAT_CONVERTER_MAX_OUTPUT_CHANNELS: i32 = 24;

/// `IIS_FormatConverter_Create(self, mode, outGeo, numOutChannels, sampleRate, frameSize)`:
/// returns the error code (0 = success) and the instance.
pub fn fc_create(
    mode: i32,
    out_geo: &[Cicp2GeometryChannelGeometry],
    num_out_channels: u32,
    sample_rate: u32,
    frame_size: u32,
) -> (i32, Option<Box<FormatConverter>>) {
    let mut fc = None;
    let err = iis_format_converter_create(
        &mut fc,
        mode,
        out_geo,
        num_out_channels,
        sample_rate,
        frame_size,
    );
    (err, fc)
}

/// `IIS_FormatConverter_Config_SetImmersiveDownmixFlag(self, immersiveFlag)`
pub fn fc_set_immersive_downmix_flag(fc: &mut FormatConverter, immersive_flag: u32) -> i32 {
    iis_format_converter_config_set_immersive_downmix_flag(fc, immersive_flag)
}

/// `IIS_FormatConverter_IsOpen(self)`
pub fn fc_is_open(fc: &FormatConverter) -> bool {
    iis_format_converter_is_open(fc) != 0
}

/// `IIS_FormatConverter_Config_AddInputSetup(self, geo, numChannels, 0, 0)`
pub fn fc_add_input_setup(
    fc: &mut FormatConverter,
    geo: &[Cicp2GeometryChannelGeometry],
    num_channels: u32,
) -> i32 {
    iis_format_converter_config_add_input_setup(fc, geo, num_channels, 0, None)
}

/// `IIS_FormatConverter_Config_SetAES(self, aes)`
pub fn fc_set_aes(fc: &mut FormatConverter, aes: u32) {
    iis_format_converter_config_set_aes(fc, aes);
}

/// `IIS_FormatConverter_Config_SetPAS(self, pas)`
pub fn fc_set_pas(fc: &mut FormatConverter, pas: u32) {
    iis_format_converter_config_set_pas(fc, pas);
}

/// `IIS_FormatConverter_Open(self, (INT*)workBufferCore2, sizeof(INT) * (24) * (1024 * 3))`
pub fn fc_open(fc: &mut FormatConverter) -> i32 {
    iis_format_converter_open(fc, (core::mem::size_of::<i32>() * 24 * (1024 * 3)) as u32)
}

/// `IIS_FormatConverter_Process(self, hDrcDec, pFormatConvIn, outBuffer,
/// inputBufferChannelOffset)`: `input_offsets[i]` is the offset of input channel `i` in
/// `time_data` (the C++ pointer array `pFormatConvIn[]`; the remaining entries are `NULL`).
/// `bypass`: fast mode (not in C, not bit-exact), see [`iis_format_converter_process_fast`].
#[allow(clippy::too_many_arguments)]
pub fn fc_process(
    fc: &mut FormatConverter,
    pool: Option<&ThreadPool>,
    drc: Option<&mut DrcDecoder>,
    input_offsets: &[usize],
    time_data: &mut [FixpDbl],
    out_buffer: &mut [FixpDbl],
    input_buffer_channel_offset: i32,
    bypass: bool,
) -> i32 {
    /* split the time buffer into the (increasing, non-overlapping) input channels */
    let mut dein: Vec<Option<&mut [FixpDbl]>> = Vec::with_capacity(FORMAT_CONVERTER_MAX_CHANNELS);
    let mut rest: &mut [FixpDbl] = time_data;
    let mut base = 0usize; /* position of rest[0] in time_data */
    for (i, &off) in input_offsets.iter().enumerate() {
        let next =
            input_offsets.get(i + 1).copied().unwrap_or(off + input_buffer_channel_offset as usize);
        let tail = &mut core::mem::take(&mut rest)[off - base..];
        let len = (next - off).min(tail.len());
        let (ch, r) = tail.split_at_mut(len);
        dein.push(Some(ch));
        rest = r;
        base = off + len;
    }
    while dein.len() < FORMAT_CONVERTER_MAX_CHANNELS {
        dein.push(None);
    }
    iis_format_converter_process_fast(
        fc,
        pool,
        drc,
        &mut dein,
        out_buffer,
        input_buffer_channel_offset,
        bypass,
    )
}

/// `FormatConverterFrame(FormatConverter, hBitStr)`
pub fn fc_frame(fc: &mut FormatConverter, bs: &mut FdkBitStream) -> i32 {
    format_converter_frame(fc, bs)
}

/// `DownmixMatrixSet(hBs, groupsDownmixMatrixSet, targetLayout, downmixConfigType, downmixId,
/// hUniDrcDec)`: parses the downmix matrix set of `downmixConfig()`. Returns 0 on success.
pub fn downmix_matrix_set(
    bs: &mut FdkBitStream,
    set: &mut DownmixGroupsMatrixSet,
    target_layout: i32,
    downmix_config_type: i32,
    downmix_id: &mut i32,
    drc: &mut DrcDecoder,
) -> i32 {
    mpeghdec_formatconverter::downmix_matrix_set(
        bs,
        set,
        target_layout,
        downmix_config_type,
        downmix_id,
        Some(drc),
    )
}
