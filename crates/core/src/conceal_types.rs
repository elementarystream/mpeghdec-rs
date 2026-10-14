// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libMpeghDec/src/conceal_types.h.

//! Error concealment structs and types.
//!
//! `FIXP_CNCL` is `FIXP_DBL` in the reference configuration.

use mpeghdec_fdk::digest::StateDigest;

use crate::aac_rom::PcmDec;
use mpeghdec_fdk::common_fix::{FixpDbl, FixpSgl};
use mpeghdec_fdk::mdct::{BlockType, BLOCK_LONG};

/// `CONCEAL_MAX_NUM_FADE_FACTORS`
pub const CONCEAL_MAX_NUM_FADE_FACTORS: usize = 32;

/// `FIXP_CNCL`
pub type FixpCncl = FixpDbl;

/// `CConcealmentMethod`. Warning: Do not ever change these values.
pub type CConcealmentMethod = i32;
pub const ConcealMethodNone: CConcealmentMethod = -1;
pub const ConcealMethodMute: CConcealmentMethod = 0;
pub const ConcealMethodNoise: CConcealmentMethod = 1;
pub const ConcealMethodInter: CConcealmentMethod = 2;
pub const ConcealMethodTonal: CConcealmentMethod = 3;

/// `CConcealmentState`
pub type CConcealmentState = i32;
pub const ConcealState_Ok: CConcealmentState = 0;
pub const ConcealState_Single: CConcealmentState = 1;
pub const ConcealState_FadeIn: CConcealmentState = 2;
pub const ConcealState_Mute: CConcealmentState = 3;
pub const ConcealState_FadeOut: CConcealmentState = 4;

/// `CConcealParams`: concealment parameters common to all channels
/// (`AAC_DECODER_INSTANCE::concealCommonData`).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CConcealParams {
    /// `fadeOutFactor`
    pub fade_out_factor: [FixpSgl; CONCEAL_MAX_NUM_FADE_FACTORS],
    /// `fadeInFactor`
    pub fade_in_factor: [FixpSgl; CONCEAL_MAX_NUM_FADE_FACTORS],
    /// `method`
    pub method: CConcealmentMethod,
    /// `numFadeOutFrames`
    pub num_fade_out_frames: i32,
    /// `numFadeInFrames`
    pub num_fade_in_frames: i32,
    /// `numMuteReleaseFrames`
    pub num_mute_release_frames: i32,
    /// `comfortNoiseLevel`
    pub comfort_noise_level: FixpDbl,
}

/// `TDfadingType`
pub type TDfadingType = i32;
pub const FADE_TIMEDOMAIN_TOSPECTRALMUTE: TDfadingType = 1;
pub const FADE_TIMEDOMAIN_FROMSPECTRALMUTE: TDfadingType = 2;
pub const FADE_TIMEDOMAIN: TDfadingType = 3;

/// `CConcealmentInfo`: concealment state of one channel
/// (`CAacDecoderStaticChannelInfo::concealmentInfo`).
///
/// State digest: the fields that are only read while a frame is concealed (the concealment state
/// is not `ConcealState_Ok` or the frame is not ok) are not part of the digest; the concealment of
/// a frame signals a [hazard](mpeghdec_fdk::digest::hazard) (`CConcealment_Apply()`,
/// `CConcealment_TDFading()` and the mute release checks of the frame decoding).
///
/// Differences to the C++ structure:
/// - `pConcealParams` always points to the decoder's `concealCommonData`; it is not stored, the
///   functions that use it take the [`CConcealParams`] as an argument.
/// - `spectralCoefficient` points to the concealment spectrum buffer of the channel
///   (`GetConcealmentSpecBuffer(ch)`, 1024 values), here owned by the structure. It is also the
///   previous frame spectrum of complex stereo prediction and IGF stereo filling
///   (`CJointStereoPersistentData::spectralCoeffs[ch]`; `specScale[ch]` is
///   [`CConcealmentInfo::spec_scale`]).
#[derive(StateDigest, Clone, Debug, PartialEq, Eq)]
pub struct CConcealmentInfo {
    /// `spectralCoefficient` (1024 values)
    pub spectral_coefficient: Vec<FixpCncl>,
    /// `specScale`
    pub spec_scale: [i16; 8],

    /// `iRandomPhase`
    pub i_random_phase: i32,
    /// `prevFrameOk`
    pub prev_frame_ok: [i32; 2],
    /// `cntValidFrames` (read only while concealing: not part of the state digest)
    #[digest(skip)]
    pub cnt_valid_frames: i32,
    /// `cntFadeFrames`: state for signal fade-in/out (read only while concealing: not part of the
    /// state digest)
    #[digest(skip)]
    pub cnt_fade_frames: i32,
    /// `winGrpOffset`: state for signal fade-out of frames with more than one window/subframe
    /// (`[0]` used by the update-cntFadeFrames mode of `CConcealment_ApplyFadeOut`, `[1]` by the
    /// fade-out mode; read only while concealing: not part of the state digest)
    #[digest(skip)]
    pub win_grp_offset: [i32; 2],
    /// `attGrpOffset`: state for faster signal fade-out of frames with transient signal parts
    /// (read only while concealing: not part of the state digest)
    #[digest(skip)]
    pub att_grp_offset: [i32; 2],

    /// `lastRenderMode`
    pub last_render_mode: i8,

    /// `windowShape`
    pub window_shape: u8,
    /// `windowSequence`
    pub window_sequence: BlockType,
    /// `lastWinGrpLen`
    pub last_win_grp_len: u8,

    /// `concealState`
    pub conceal_state: CConcealmentState,
    /// `concealState_old`
    pub conceal_state_old: CConcealmentState,
    /// `fade_old`: last fading factor
    pub fade_old: FixpDbl,
    /// `lastFadingType`: last fading type
    pub last_fading_type: TDfadingType,

    /// `lastWindowGroups` (written, never read: not part of the state digest)
    #[digest(skip)]
    pub last_window_groups: u8,
    /// `lastWindowGroupLength` (written, never read: not part of the state digest)
    #[digest(skip)]
    pub last_window_group_length: [u8; 8],

    /// `TDNoiseSeed`
    ///
    /// Not part of the state digest: the seed advances in every frame (it depends on the number
    /// of frames decoded so far) and influences the output only through the concealment noise.
    #[digest(skip)]
    pub td_noise_seed: u32,
    /// `TDNoiseStates` (only read for the concealment noise: not part of the state digest)
    #[digest(skip)]
    pub td_noise_states: [PcmDec; 3],
    /// `TDNoiseCoef`
    pub td_noise_coef: [FixpSgl; 3],
    /// `TDNoiseAtt`
    pub td_noise_att: FixpSgl,
}

impl Default for CConcealmentInfo {
    /// Zero-initialized like the C++ allocation (`FDKcalloc`); the spectrum buffer has 1024 values.
    fn default() -> Self {
        CConcealmentInfo {
            spectral_coefficient: vec![0; 1024],
            spec_scale: [0; 8],
            i_random_phase: 0,
            prev_frame_ok: [0; 2],
            cnt_valid_frames: 0,
            cnt_fade_frames: 0,
            win_grp_offset: [0; 2],
            att_grp_offset: [0; 2],
            last_render_mode: 0,
            window_shape: 0,
            window_sequence: BLOCK_LONG,
            last_win_grp_len: 0,
            conceal_state: ConcealState_Ok,
            conceal_state_old: ConcealState_Ok,
            fade_old: 0,
            last_fading_type: 0,
            last_window_groups: 0,
            last_window_group_length: [0; 8],
            td_noise_seed: 0,
            td_noise_states: [0; 3],
            td_noise_coef: [0; 3],
            td_noise_att: 0,
        }
    }
}
