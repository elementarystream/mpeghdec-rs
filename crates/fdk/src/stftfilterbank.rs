// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/FDK_stftfilterbank_api.h, libFDK/src/FDK_stftfilterbank.h,
// libFDK/src/FDK_stftfilterbank.cpp, libFDK/src/FDK_stftfilterbank_data.h,
// libFDK/src/FDK_stftfilterbank_data.cpp, libFDK/src/FDK_stftfilterbank_process.h,
// libFDK/src/FDK_stftfilterbank_process.cpp and libFDK/src/arm/FDK_stftfilterbank_process_arm.cpp
// (the AArch64 build compiles the generic C variants of the ARM file).
//
// StftFilterbank_GetConfig(), StftFilterbank_DisplayConfig() and StftFilterbank_ErrorCodeToStr()
// are declared in the C++ API header but not implemented; they are not ported.

//! STFT filterbank (sine windowed, 50% overlap, real FFT based analysis and synthesis).
//!
//! Supported FFT sizes are 256 and 512 (the frame size is half the FFT size).

use crate::digest::StateDigest;

use crate::common_fix::*;
use crate::fixpoint_math::f_add_saturate;
use crate::rfft::{irfft, rfft};
use crate::scale::{get_scalefactor, scale_values};
use crate::tools_rom::fdk_get_window_slope;

/// `STFT_FILTERBANK_MODE`
pub type StftFilterbankMode = i32;
/// invalid stft filterbank mode
pub const STFT_FILTERBANK_MODE_INVALID: StftFilterbankMode = 0;
/// time to frequency transform, analysis
pub const STFT_FILTERBANK_MODE_TIME_TO_FREQ: StftFilterbankMode = 1;
/// frequency to time transform, synthesis
pub const STFT_FILTERBANK_MODE_FREQ_TO_TIME: StftFilterbankMode = 2;

/// `STFT_FILTERBANK_CONFIG`: STFT filterbank configuration.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StftFilterbankConfig {
    /// STFT filterbank mode
    pub stft_filterbank_mode: StftFilterbankMode,
    /// Frame size, number of samples, per channel, per processing call.
    pub frame_size: u32,
    /// FFT size, at least 2 * frameSize, if greater, zero padding is performed
    pub fft_size: u32,
}

/// `STFT_FILTERBANK_ERRORS`: internal STFT filterbank error codes.
pub type StftFilterbankErrors = i32;
pub const STFT_ERR_INVALID: StftFilterbankErrors = -1;
pub const STFT_ERR_NO_ERROR: StftFilterbankErrors = 0;
pub const STFT_ERR_GENERAL_ERROR: StftFilterbankErrors = 1;
pub const STFT_ERR_NULL_PTR_ERROR: StftFilterbankErrors = 2;
pub const STFT_ERR_BAD_PARAM_WARNING: StftFilterbankErrors = 3;
pub const STFT_ERR_BAD_PARAM_FATAL: StftFilterbankErrors = 4;
pub const STFT_ERR_MEM_ERROR: StftFilterbankErrors = 5;

/// `STFT_FILTERBANK` (`struct T_STFT_FILTERBANK`): STFT filterbank instance.
#[derive(StateDigest, Clone, Debug, Default)]
pub struct StftFilterbank {
    /// STFT filterbank configuration.
    pub stft_filterbank_config: StftFilterbankConfig,
    /// Number of zeros padded: fftSize - 2 * frameSize, zeros pre- and appended.
    pub num_zeros_padded_two_sided: i32,
    /// Time sample buffer of previous time samples (`frameSize` values).
    pub prev_audio_input: Vec<FixpDbl>,
    /// The sine window slope of length `fftSize / 2` as the factors of the rising half
    /// (`window_rise[k]`, `k < fftSize / 2`) and of the falling half (`window_fall[k]` for the
    /// sample `fftSize / 2 + k`) of the window (not in C).
    pub window_rise: Vec<FixpSgl>,
    pub window_fall: Vec<FixpSgl>,
}

/// `HANDLE_STFT_FILTERBANK`
pub type HandleStftFilterbank = Box<StftFilterbank>;

/// `allocateStftFilterbankData(stft_filterbank)`: allocate internal buffers.
pub fn allocate_stft_filterbank_data(stft_filterbank: &mut StftFilterbank) -> i32 {
    stft_filterbank.prev_audio_input =
        vec![0; stft_filterbank.stft_filterbank_config.frame_size as usize];
    0
}

/// `freeStftFilterbankData(stft_filterbank)`: free internal buffers.
pub fn free_stft_filterbank_data(stft_filterbank: &mut StftFilterbank) -> i32 {
    stft_filterbank.prev_audio_input = Vec::new();
    0
}

/// `StftFilterbank_Open(stftFilterbankConfig, ph_stft_filterbank)`: open an STFT filterbank
/// instance. Returns 0 on success, otherwise an error code.
pub fn stft_filterbank_open(
    stft_filterbank_config: &StftFilterbankConfig,
    ph_stft_filterbank: &mut Option<HandleStftFilterbank>,
) -> i32 {
    let mut config_local = StftFilterbankConfig::default();

    // check configuration
    match stft_filterbank_config.stft_filterbank_mode {
        STFT_FILTERBANK_MODE_TIME_TO_FREQ | STFT_FILTERBANK_MODE_FREQ_TO_TIME => {
            config_local.stft_filterbank_mode = stft_filterbank_config.stft_filterbank_mode;
        }
        _ => return STFT_ERR_BAD_PARAM_WARNING,
    }

    config_local.frame_size = stft_filterbank_config.frame_size;

    let num_zeros_padded_two_sided =
        stft_filterbank_config.fft_size.wrapping_sub(2 * stft_filterbank_config.frame_size) as i32;
    // must be even and positive
    debug_assert!(num_zeros_padded_two_sided >= 0 && (num_zeros_padded_two_sided & 1) == 0);
    // fftSize must be multiple of 4
    debug_assert!(((stft_filterbank_config.fft_size / 2) & 1) == 0);

    config_local.fft_size = stft_filterbank_config.fft_size;

    // no errors, allocate struct
    let (window_rise, window_fall) = window_halves(config_local.fft_size as usize);
    let mut stft_filterbank = Box::new(StftFilterbank {
        stft_filterbank_config: config_local,
        num_zeros_padded_two_sided,
        prev_audio_input: Vec::new(),
        window_rise,
        window_fall,
    });

    if allocate_stft_filterbank_data(&mut stft_filterbank) != 0 {
        return STFT_ERR_MEM_ERROR;
    }
    // return allocated struct
    *ph_stft_filterbank = Some(stft_filterbank);

    STFT_ERR_NO_ERROR
}

/// `StftFilterbank_Process(inputBuffer, outputBuffer, h_stft_filterbank, STFT_headroom)`:
/// transform of a time signal to a frequency signal, or vice versa, depending on the filterbank
/// mode.
///
/// Analysis: `input_buffer` holds `frameSize` time samples (scaled in place by the headroom and
/// restored only for the internal state), `output_buffer` receives `fftSize` spectral values.
/// Synthesis: `input_buffer` holds `fftSize` spectral values (overwritten), the scaled output
/// signal (`frameSize` samples) is accumulated on `output_buffer`.
pub fn stft_filterbank_process(
    input_buffer: &mut [FixpDbl],
    output_buffer: &mut [FixpDbl],
    h_stft_filterbank: &mut StftFilterbank,
    stft_headroom: i32,
) {
    let mut stft_headroom = stft_headroom;
    let frame_size = h_stft_filterbank.stft_filterbank_config.frame_size as usize;
    let fft_size = h_stft_filterbank.stft_filterbank_config.fft_size as usize;

    if h_stft_filterbank.stft_filterbank_config.stft_filterbank_mode
        == STFT_FILTERBANK_MODE_TIME_TO_FREQ
    {
        if stft_headroom != 0 {
            scale_values(&mut input_buffer[..frame_size], stft_headroom);
            scale_values(&mut h_stft_filterbank.prev_audio_input[..frame_size], stft_headroom);
        }
        process_analysis_stft(input_buffer, output_buffer, h_stft_filterbank);
        if stft_headroom != 0 {
            scale_values(&mut h_stft_filterbank.prev_audio_input[..frame_size], -stft_headroom);
        }
    } else {
        // make sure we have one bit headroom for FFT
        if get_scalefactor(&input_buffer[..fft_size]) == 0 {
            scale_values(&mut input_buffer[..fft_size], -1);
            stft_headroom -= 1;
        }
        if stft_headroom != 0 {
            scale_values(&mut h_stft_filterbank.prev_audio_input[..frame_size], stft_headroom);
            scale_values(&mut output_buffer[..frame_size], stft_headroom);
        }
        process_synthesis_stft(input_buffer, output_buffer, h_stft_filterbank);
        if stft_headroom != 0 {
            scale_values(&mut output_buffer[..frame_size], -stft_headroom);
            scale_values(&mut h_stft_filterbank.prev_audio_input[..frame_size], -stft_headroom);
        }
    }
}

/// `StftFilterbank_Close(ph_stft_filterbank)`: close an STFT filterbank instance.
pub fn stft_filterbank_close(ph_stft_filterbank: &mut Option<HandleStftFilterbank>) -> i32 {
    if let Some(mut h) = ph_stft_filterbank.take() {
        free_stft_filterbank_data(&mut h);
    }
    0
}

/// `processAnalysisSTFT(inputBuffer, outputBuffer, h_stft_filterbank)`: FFT analysis function.
pub fn process_analysis_stft(
    input_buffer: &[FixpDbl],
    output_buffer: &mut [FixpDbl],
    h_stft_filterbank: &mut StftFilterbank,
) -> i32 {
    let fft_size = h_stft_filterbank.stft_filterbank_config.fft_size as i32;
    let frame_size = h_stft_filterbank.stft_filterbank_config.frame_size as i32;
    filterbank_analysis_stft(
        input_buffer,
        &mut h_stft_filterbank.prev_audio_input,
        output_buffer,
        fft_size,
        frame_size,
        &h_stft_filterbank.window_rise,
        &h_stft_filterbank.window_fall,
    );
    0
}

/// `processSynthesisSTFT(inputBuffer, outputBuffer, h_stft_filterbank)`: FFT synthesis function.
pub fn process_synthesis_stft(
    input_buffer: &mut [FixpDbl],
    output_buffer: &mut [FixpDbl],
    h_stft_filterbank: &mut StftFilterbank,
) -> i32 {
    let fft_size = h_stft_filterbank.stft_filterbank_config.fft_size;
    let frame_size = h_stft_filterbank.stft_filterbank_config.frame_size;
    filterbank_synthesis_stft(input_buffer, fft_size);
    filterbank_overlap_add_audio_frames(
        input_buffer,
        &mut h_stft_filterbank.prev_audio_input,
        output_buffer,
        fft_size,
        frame_size,
        &h_stft_filterbank.window_rise,
        &h_stft_filterbank.window_fall,
    );
    0
}

/// The window factors of `filterbankSineWindowingSTFT()` and `filterbankOverlapAddAudioFrames()`
/// in natural order: the C++ loops use `w[j].im` for the sample `j` and `w[j].re` for the sample
/// `fftSize / 2 - 1 - j` of the rising half, and `w[j].re` for `fftSize / 2 + j` and `w[j].im` for
/// `fftSize - 1 - j` of the falling half, with the window slope
/// `w = FDKgetWindowSlope(fftSize / 2, 0)` and `j < fftSize / 4`.
fn window_halves(fft_size: usize) -> (Vec<FixpSgl>, Vec<FixpSgl>) {
    let half = fft_size / 2;
    let sinetab = fdk_get_window_slope(half as i32, 0);
    let rise = (0..half)
        .map(|k| if k < half / 2 { sinetab[k].im } else { sinetab[half - 1 - k].re })
        .collect();
    let fall = (0..half)
        .map(|k| if k < half / 2 { sinetab[k].re } else { sinetab[half - 1 - k].im })
        .collect();
    (rise, fall)
}

crate::avx2_clone! {
/// `filterbankSineWindowingSTFT()`: windows the previous and the current input into
/// `audio_output_freq` and stores the current input as the previous one.
fn filterbank_sine_windowing_stft(
    audio_input_time: &[FixpDbl],
    audio_input_time_prev: &mut [FixpDbl],
    audio_output_freq: &mut [FixpDbl],
    fft_size: usize,
    window_rise: &[FixpSgl],
    window_fall: &[FixpSgl],
) {
    let half = fft_size / 2;
    let (out_rise, out_fall) = audio_output_freq[..fft_size].split_at_mut(half);
    let time = &audio_input_time[..half];
    let prev = &mut audio_input_time_prev[..half];
    for ((o, &x), &w) in out_rise.iter_mut().zip(prev.iter()).zip(&window_rise[..half]) {
        *o = f_mult(x, w);
    }
    for ((o, &x), &w) in out_fall.iter_mut().zip(time).zip(&window_fall[..half]) {
        *o = f_mult(x, w);
    }
    prev.copy_from_slice(time);
}
}

fn filterbank_analysis_stft(
    audio_input_time: &[FixpDbl],
    audio_input_time_prev: &mut [FixpDbl],
    audio_output_freq: &mut [FixpDbl],
    fft_size: i32,
    _frame_size: i32,
    window_rise: &[FixpSgl],
    window_fall: &[FixpSgl],
) {
    let mut dummy_scale = 0;
    filterbank_sine_windowing_stft(
        audio_input_time,
        audio_input_time_prev,
        audio_output_freq,
        fft_size as usize,
        window_rise,
        window_fall,
    );
    rfft(fft_size as u32, audio_output_freq, fft_size as u32, &mut dummy_scale);
}

/// `filterbankSynthesisSTFT()`
fn filterbank_synthesis_stft(audio_input_output: &mut [FixpDbl], fft_length: u32) {
    let mut dummy_scale = 0;
    irfft(fft_length, audio_input_output, fft_length, &mut dummy_scale);
}

crate::avx2_clone! {
/// `filterbankOverlapAddAudioFrames()`: adds the windowed first half of the inverse transform
/// and the previous second half to the output and stores the windowed second half.
fn filterbank_overlap_add_audio_frames(
    audio_input: &[FixpDbl],
    prev_audio_input: &mut [FixpDbl],
    audio_output: &mut [FixpDbl],
    fft_size: u32,
    _frame_size: u32,
    window_rise: &[FixpSgl],
    window_fall: &[FixpSgl],
) {
    let fft_size = fft_size as usize;
    let half = fft_size / 2;
    let (in_rise, in_fall) = audio_input[..fft_size].split_at(half);
    let prev = &mut prev_audio_input[..half];
    let out = &mut audio_output[..half];
    for (((o, p), &x), &w) in out.iter_mut().zip(prev.iter()).zip(in_rise).zip(&window_rise[..half])
    {
        *o = o.wrapping_add(f_add_saturate(f_mult(x << 8, w), *p));
    }
    for ((p, &x), &w) in prev.iter_mut().zip(in_fall).zip(&window_fall[..half]) {
        *p = f_mult(x << 8, w);
    }
}
}
