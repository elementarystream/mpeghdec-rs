// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Port of `libgVBAPRenderer`: the object renderer (generalized Vector Base Amplitude Panning).
//!
//! The renderer triangulates the output loudspeaker setup (quickhull with imaginary speakers,
//! [`quick_hull`]), precomputes the inverse speaker matrices ([`vbap_core`]) and pans each object
//! according to its object metadata (OAM): VBAP gains, spread (legacy MDAP spread or the enhanced
//! spread with virtual spread objects), downmix of the imaginary speakers and linear gain
//! interpolation over the frame ([`gvbap_renderer`]).
//!
//! # Call pattern
//!
//! In `libMpeghDec` (`aacdecoder.cpp`, `aacdecoder_lib.cpp`) there is one renderer per active
//! object signal group:
//!
//! 1. [`gvbap_renderer_open`] (`gVBAPRenderer_Open`) at configuration time with the number of
//!    objects of the group, the core frame length, the OAM frame length, the geometry of the
//!    target layout (`cicp2geometry_get_geometry_from_cicp(targetLayout_config)`), the number of
//!    output channels including LFEs, the target CICP index, `hasUniformSpread` of the OAM config
//!    and `GVBAP_ENHANCED` if the stream contains production metadata (`GVBAP_LEGACY`
//!    otherwise).
//! 2. Per frame, while parsing the extension elements:
//!    [`object_metadata_frame`] (`objectMetadataFrame`) for `ID_EXT_ELE_OBJ_METADATA` (with
//!    `num_objects = renderer.num_objects`) and [`prod_metadata_frame_group`]
//!    (`prodMetadataFrameGroup`) for each object group of `ID_EXT_ELE_PROD_METADATA` (with
//!    `None` for inactive groups, whose data is only skipped).
//! 3. The user interface (`ui.cpp`) may modify `oam_samples[k][o].sph.azi/ele` (position
//!    offsets) of all OAM frames `k < num_oam_frames`.
//! 4. [`gvbap_renderer_render_frame_time`] (`gVBAPRenderer_RenderFrame_Time`) with the object
//!    signals of the group (`startSamplePosition = 256`, `inputBufferChannelOffset = frameSize +
//!    256`); the result is added to the output buffer (one block of `frame_length` samples per
//!    output channel). If no valid metadata was parsed for the frame, the last valid metadata is
//!    used (concealment).
//! 5. [`gvbap_renderer_close`] (`gVBAPRenderer_Close`), or simply dropping the renderer.
//!
//! `libFormatConverter` (`gVBAPRenderer_GetStaticGains()`) opens a renderer with frame length
//! and OAM frame length 1, CICP index -1 and `GVBAP_ENHANCED`, and calls [`calculate_vbap`]
//! (`calculateVbap`) with `no_downmix = 0` for each input channel position (converted with
//! [`spherical_to_cartesian`], radius 1/16, gain `MAXVAL_DBL`) to obtain static downmix gains,
//! mapped to the output channels with `speaker_setup.mapping`.

pub mod cartesian_math;
pub mod gvbap_renderer;
pub mod qsort;
pub mod quick_hull;
pub mod tables;
pub mod vbap_core;

pub use cartesian_math::*;
pub use gvbap_renderer::*;
pub use vbap_core::*;
