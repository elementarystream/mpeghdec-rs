// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Port of `libFormatConverter`: the channel format converter of MPEG-H 3D Audio.
//!
//! The converter maps the channel signal groups of a stream (input geometries) to the target
//! loudspeaker layout (output geometry). The downmix matrix is generated from downmix rules
//! (amplitude panning, static equalizers, spatial elevation rendering of the immersive mode,
//! [`mod@format_converter_init`]), falls back to static VBAP gains for unknown loudspeaker
//! positions (`mpeghdec-gvbap`) or is taken from the bitstream (`DownmixMatrix()` of
//! `mpegh3daConfig`, [`dmx_matrix_parser`]). The processing is either a passive downmix in the time domain or an
//! active (energy preserving, adaptive equalization) downmix in the STFT domain
//! ([`format_converter_active_dmx_stft`]).
//!
//! # Call pattern of the core decoder (`libMpeghDec`)
//!
//! The C++ handle `IIS_FORMATCONVERTER_HANDLE` is `Box<IisFormatConverter>`; its `member` is the
//! internal instance ([`IisFormatConverterInternal`], `IIS_FORMATCONVERTER_INTERNAL_HANDLE`).
//!
//! 1. Configuration parsing (`aacDecoder_ParseDmxMatrixCallback()` in `aacdecoder_lib.cpp`,
//!    transport decoder callback `parse_dmx_matrix`): clear a [`FdkDownmixGroupsMatrixSet`]
//!    (C borrows `pTimeData2` for it; it has to live until the renderer initialization) and call
//!    [`downmix_matrix_set`] (`DownmixMatrixSet(hBs, set, usc->targetLayout,
//!    usc->downmixConfigType, &downmixId, hUniDrcDecoder)`). On success the decoder sets
//!    `asi->activeDmxId = downmixId & 0xFF` and marks every channel signal group `i` with
//!    `set.downmix_matrix_size[set.downmix_matrix[i]] > 0` as `bUseCustomDownmixMatrix` (all
//!    channel groups or none, otherwise parse error).
//! 2. Renderer initialization (`CAacDecoder_InitRenderer()` in `aacdecoder.cpp`, only if
//!    `targetLayout_config != -1`):
//!    - substream 0: [`iis_format_converter_create`] with
//!      `IIS_FORMATCONVERTER_MODE_CUSTOM_FREQ_DOMAIN_STFT`, the geometry of the target layout
//!      (`cicp2geometry_get_geometry_from_cicp()`), its channel count (at most 24), the sampling
//!      rate and the frame length (only 1024 is accepted for the STFT modes);
//!    - aes/pas: `aes = 0, pas = 0` if `passiveDownmixFlag`, else `aes = 7, pas =
//!      phaseAlignStrength`; [`iis_format_converter_config_set_immersive_downmix_flag`]`(fc, 1)`
//!      if `immersiveDownmixFlag` (substream 0);
//!    - for each active channel signal group (`type == 0`, on/off flag set): return
//!      `AAC_DEC_PARSE_ERROR` if [`iis_format_converter_is_open`], add `count` to
//!      `num_signals_total` (error if > 28), and call
//!      [`iis_format_converter_config_add_input_setup`] with the speaker geometry of the group
//!      (`cicp_loudspeaker_index = -1`, `Az`, `El`, `LFE` of the speakers);
//!    - substream 0: [`iis_format_converter_config_set_aes`] / `_set_pas`;
//!    - last substream: [`iis_format_converter_open`] (`buf_size = sizeof(INT) * 24 * 1024 *
//!      3`), then `applyDownmixMatrixPerSignalGroup()`: for each active signal group with
//!      `bUseCustomDownmixMatrix`, build the [`SpeakerInformation`] lists of the group and of
//!      the target layout, set up a bitstream reader on
//!      `set.downmix_matrix_memory[set.downmix_matrix[grp]]` (256 bytes, `downmix_matrix_size`
//!      valid bits), call [`decode_downmix_matrix`]`(group Layout, count, inputs, targetLayout,
//!      numOutputCh + numOutLfes, outputs, bs, &mut
//!      params.dmx_mtx_sorted[offset..], &mut eq_config)`, then
//!      [`format_converter_dmx_matrix_exponent`]`(fc)` and [`format_converter_set_eqs`]`(offset,
//!      eq_config.num_eqs, Some(&eq_config.eq_params), &eq_config.eq_map, grp, fc)` and finally
//!      `fc.amount_of_added_dmx_matrices_and_equalizers += fc.num_input_channels[grp]`.
//!      `offset` starts at 0 and advances by `count * numOutputChannels` for every active group
//!      (with or without custom matrix). Errors map to `AAC_DEC_UNSUPPORTED_CHANNELCONFIG` /
//!      `AAC_DEC_UNSUPPORTED_FORMAT`.
//! 3. Per frame, while parsing the extension elements: [`format_converter_frame`] for
//!    `ID_EXT_ELE_FMT_CNVRTR` (substream 0).
//! 4. Per frame, rendering: [`iis_format_converter_process`] once, at the first active channel
//!    signal group, with the DRC decoder if `multibandDrcPresent`, one input slice per input
//!    channel (the time signals of the active channel groups, `aacSamplesPerFrame + 256` samples
//!    apart in C; they are scaled in place) and the output buffer (`numOutputChannels` blocks of
//!    `frameSize` samples, `workBufferCore2`) to which the result is added. The STFT modes delay
//!    the signal by [`iis_format_converter_get_delay`] (256) samples.
//!    With a thread pool, [`iis_format_converter_process_pool`] runs the STFT analysis of the
//!    input channels and the synthesis of the output channels (all time slots of the frame) in
//!    parallel; the result is identical.
//! 5. [`iis_format_converter_close`] (`CAacDecoder_DeInitRenderer()`), or dropping the
//!    instance.
//!
//! # Differences to the C++ API
//!
//! - Scratch buffers (`p_buffer`) are not passed: [`iis_format_converter_open`] only keeps the
//!   buffer size check, [`decode_downmix_matrix`] and [`format_converter_set_eqs`] allocate their
//!   temporary data. [`format_converter_set_eqs`] takes the offset into
//!   `fcParams->eqIndexVec_sorted` instead of a pointer into it.
//! - Pointers become options, indices or selectors: `dmxMatrixL_FDK` / `dmxMatrixH_FDK` /
//!   `eqIndex_FDK` are [`DmxMtxSel`] / [`EqIndexSel`], `SpeakerInformation::symmetricPair` is an
//!   index, the STFT buffers are owned by the internal instance (not duplicated in the active
//!   downmix state).
//!
//! Where C has undefined behaviour the port avoids panics (only possible for configurations the
//! decoder does not create, or for invalid bitstreams): an output setup of more than 24 channels
//! is rejected by [`iis_format_converter_create`]; entries of the temporary conversion lists that
//! C leaves uninitialized are empty; the unchecked second pass of `converter_init()` fails
//! instead of writing out of bounds; run length decoded compact matrix entries beyond the matrix
//! are dropped and compact template reads beyond the template yield 0 (C reads neighbouring
//! memory); in the immersive 2D rendering with transmitted downmix matrices, equalizer indices
//! without equalizer (uninitialized pointers in C) use the neutral equalizer. Only the modes
//! `PASSIVE_TIME_DOMAIN`, `ACTIVE_FREQ_DOMAIN_STFT` and `CUSTOM_FREQ_DOMAIN_STFT` are
//! functional (C reads uninitialized scratch memory for the others).

pub mod dmx_matrix_parser;
pub mod format_converter;
pub mod format_converter_active_dmx_stft;
pub mod format_converter_constants;
pub mod format_converter_constants_stft;
pub mod format_converter_data;
pub mod format_converter_fast;
pub mod format_converter_init;
pub mod format_converter_intern;
pub mod format_converter_lib;
pub mod format_converter_process;

pub use dmx_matrix_parser::*;
pub use format_converter::*;
pub use format_converter_data::{
    format_converter_dmx_matrix_control, format_converter_dmx_matrix_exponent,
};
pub use format_converter_fast::{format_converter_bypass_map, BypassMap};
pub use format_converter_intern::*;
pub use format_converter_lib::*;
