// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: the opt-in fast mode.

//! Fast mode (not in C): opt-in shortcuts whose output is **not bit-exact** with the reference
//! decoder.
//!
//! By default the decoder output is identical to the one of the reference decoder. The options
//! of [`FastOptions`] trade this for speed where the difference is the reference's own rounding
//! noise (a few LSB of 16 bit PCM). They are set with [`AacDecoderInstance::set_fast_options`]
//! (or [`MpeghDecoderContext::set_fast_mode`](crate::mpeghdecoder::MpeghDecoderContext::set_fast_mode),
//! `mpeghdec::Decoder::set_fast_mode()`) and apply from the next access unit on; they can be
//! changed at any time.

use crate::aacdecoder::AacDecoderInstance;
use mpeghdec_fdk::digest::StateDigest;

/// Relaxed exactness options (not in C). The default is [`FastOptions::EXACT`]: the output is
/// bit-exact with the reference decoder.
///
/// The struct is `#[non_exhaustive]` so that options can be added: start from
/// [`FastOptions::EXACT`] or [`FastOptions::ALL`] and change single fields.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct FastOptions {
    /// Replace the STFT format converter by a delayed copy of the channels when it is
    /// equivalent to one, i.e. when the channel signals are rendered to the same loudspeaker
    /// layout (e.g. stereo to stereo, 5.1 to 5.1, 22.2 to 22.2; exact conditions in
    /// `mpeghdec_formatconverter::format_converter_fast`). The reference decoder runs the STFT
    /// analysis, the active downmix and the STFT synthesis in that case as well; the bypass removes
    /// that work from the rendering (with frame pipelining the rendering runs next to the core
    /// decoding, so the gain in throughput is small).
    ///
    /// The output then differs from the reference decoder by the reference's own rounding noise
    /// of the active downmix (equalizer gains that fluctuate around 1 by about 1e-5): over all
    /// test material (`scripts/fast_accuracy.py`) at most 842 LSB and 52 LSB rms of 24 bit per
    /// channel, i.e. at most 3.3 LSB of 16 bit. The timing (256 samples delay of the STFT) and
    /// the number of output samples are unchanged. The bypass also applies when channels are
    /// only copied into a larger layout (e.g. stereo to 5.1, the other output channels stay
    /// silent). Real downmixes, object-only streams, multiband (STFT domain) DRC and the
    /// immersive downmix are decoded bit-exactly (as long as no frame of the stream has been
    /// bypassed before).
    pub format_converter_bypass: bool,
}

impl FastOptions {
    /// No relaxation: the output is bit-exact with the reference decoder (the default).
    pub const EXACT: FastOptions = FastOptions { format_converter_bypass: false };
    /// All relaxations.
    pub const ALL: FastOptions = FastOptions { format_converter_bypass: true };

    /// Whether no relaxation is enabled (the output is bit-exact).
    pub fn is_exact(&self) -> bool {
        *self == FastOptions::EXACT
    }
}

impl AacDecoderInstance {
    /// Sets the fast mode options (not in C, see [`FastOptions`]). They apply to the access
    /// units decoded afterwards (with frame pipelining: to the rendering of those access units).
    pub fn set_fast_options(&mut self, options: FastOptions) {
        self.fast_options = options;
    }

    /// The fast mode options (see [`AacDecoderInstance::set_fast_options`]).
    pub fn fast_options(&self) -> FastOptions {
        self.fast_options
    }

    /// Number of frames whose format conversion used the time domain bypass of the fast mode
    /// ([`FastOptions::format_converter_bypass`]) since the decoder was opened. Waits for a
    /// running rendering (frame pipelining).
    pub fn fast_bypass_frames(&mut self) -> u64 {
        self.join_render();
        let rs = self.render.as_deref().expect("render state");
        rs.fast_bypass_frames
            + rs.p_format_converter.as_deref().map_or(0, |fc| fc.member.fast_bypass_frames)
    }
}
