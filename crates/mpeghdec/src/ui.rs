// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Not in C: the safe Rust API of the UI manager (`include/mpeghUIManager.h`).

//! [`UiManager`]: user interactivity for MPEG-H streams.

use mpeghdec_uimanager::mpegh_ui_manager as c;
use mpeghdec_uimanager::mpegh_ui_manager::MpeghUiManager;

/// The errors of the UI manager (`MPEGH_UI_ERROR`).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub enum UiError {
    /// Memory could not be allocated (`MPEGH_UI_OUT_OF_MEMORY`).
    OutOfMemory,
    /// A buffer is too small (`MPEGH_UI_BUFFER_TOO_SMALL`); with an MHAS buffer, UI events are
    /// delayed until the next random access point.
    BufferTooSmall,
    /// An invalid parameter (`MPEGH_UI_INVALID_PARAM`).
    InvalidParam,
    /// The action is not allowed (`MPEGH_UI_NOT_ALLOWED`), e.g. the stream already contains UI
    /// packets.
    NotAllowed,
    /// The UI manager is not in a state for the call (`MPEGH_UI_INVALID_STATE`).
    InvalidState,
    /// The input could not be parsed (`MPEGH_UI_PARSE_ERROR`).
    ParseError,
}

impl UiError {
    fn from_code(code: c::MpeghUiError) -> UiError {
        match code {
            c::MPEGH_UI_OUT_OF_MEMORY => UiError::OutOfMemory,
            c::MPEGH_UI_BUFFER_TOO_SMALL => UiError::BufferTooSmall,
            c::MPEGH_UI_NOT_ALLOWED => UiError::NotAllowed,
            c::MPEGH_UI_INVALID_STATE => UiError::InvalidState,
            c::MPEGH_UI_PARSE_ERROR => UiError::ParseError,
            _ => UiError::InvalidParam,
        }
    }

    /// The `MPEGH_UI_ERROR` value of the C API.
    pub fn code(self) -> i32 {
        match self {
            UiError::OutOfMemory => c::MPEGH_UI_OUT_OF_MEMORY,
            UiError::BufferTooSmall => c::MPEGH_UI_BUFFER_TOO_SMALL,
            UiError::InvalidParam => c::MPEGH_UI_INVALID_PARAM,
            UiError::NotAllowed => c::MPEGH_UI_NOT_ALLOWED,
            UiError::InvalidState => c::MPEGH_UI_INVALID_STATE,
            UiError::ParseError => c::MPEGH_UI_PARSE_ERROR,
        }
    }
}

impl core::fmt::Display for UiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            UiError::OutOfMemory => "out of memory",
            UiError::BufferTooSmall => "buffer too small",
            UiError::InvalidParam => "invalid parameter",
            UiError::NotAllowed => "not allowed",
            UiError::InvalidState => "invalid state",
            UiError::ParseError => "parse error",
        })
    }
}

impl std::error::Error for UiError {}

/// `MPEGH_UI_OK` and `MPEGH_UI_OK_BUT_NO_VALID_DATA` are success.
fn result(code: c::MpeghUiError) -> Result<(), UiError> {
    match code {
        c::MPEGH_UI_OK | c::MPEGH_UI_OK_BUT_NO_VALID_DATA => Ok(()),
        e => Err(UiError::from_code(e)),
    }
}

/// The result of [`UiManager::xml_scene_state`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
#[non_exhaustive]
pub struct XmlSceneState {
    /// The length of the XML text written to the start of the output buffer (0 if nothing
    /// changed).
    pub len: usize,
    /// Nothing has changed since the previous call; no XML was written (`MPEGH_UI_NO_CHANGE`).
    pub unchanged: bool,
    /// The XML continues the incomplete output of the previous call (`MPEGH_UI_CONTINUES_XML`).
    pub continues: bool,
    /// The XML is incomplete; call again for the rest (`MPEGH_UI_INCOMPLETE_XML`).
    pub incomplete: bool,
}

/// The MPEG-H UI manager: applies user interactivity (XML actions such as preset selection,
/// prominence or position changes) to an MHAS stream before it is decoded, by inserting UI
/// packets into the access units, and describes the audio scene as XML.
///
/// For every access unit: [`UiManager::feed_mhas`], then optionally
/// [`UiManager::xml_scene_state`] and [`UiManager::apply_xml_action`], then
/// [`UiManager::update_mhas`] with the same access unit, and pass the updated access unit to the
/// decoder.
pub struct UiManager {
    inner: Box<MpeghUiManager>,
}

impl core::fmt::Debug for UiManager {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("UiManager").finish_non_exhaustive()
    }
}

impl Default for UiManager {
    fn default() -> UiManager {
        UiManager::new()
    }
}

impl UiManager {
    /// A UI manager without persistence memory.
    pub fn new() -> UiManager {
        UiManager { inner: MpeghUiManager::open() }
    }

    /// Feeds the MHAS packets of exactly one access unit.
    pub fn feed_mhas(&mut self, access_unit: &[u8]) -> Result<(), UiError> {
        result(self.inner.feed_mhas(access_unit))
    }

    /// Inserts the UI and DRC packets into the access unit passed to the previous
    /// [`UiManager::feed_mhas`]: `buffer[..len]` holds it, the rest of `buffer` is room for the
    /// inserted packets (256 bytes are recommended). Returns the new length.
    pub fn update_mhas(&mut self, buffer: &mut [u8], len: usize) -> Result<usize, UiError> {
        let mut n = len.min(buffer.len()) as u32;
        result(self.inner.update_mhas(buffer, &mut n))?;
        Ok(n as usize)
    }

    /// Applies an XML action (a UI command).
    pub fn apply_xml_action(&mut self, xml: impl AsRef<[u8]>) -> Result<(), UiError> {
        let mut flags = 0;
        result(self.inner.apply_xml_action(xml.as_ref(), &mut flags))
    }

    /// Writes the XML description of the audio scene and of the user interactivity to `out`
    /// (at least 1024 bytes are recommended). Without `force_update` nothing is written if
    /// nothing has changed since the previous call; `force_restart` restarts an incomplete
    /// output from the beginning.
    pub fn xml_scene_state(
        &mut self,
        out: &mut [u8],
        force_update: bool,
        force_restart: bool,
    ) -> Result<XmlSceneState, UiError> {
        let flags_in = if force_update { c::MPEGH_UI_FORCE_UPDATE } else { 0 }
            | if force_restart { c::MPEGH_UI_FORCE_RESTART_XML } else { 0 };
        let mut flags = 0;
        result(self.inner.get_xml_scene_state(out, flags_in, &mut flags))?;
        let unchanged = flags & c::MPEGH_UI_NO_CHANGE != 0;
        let len = if unchanged { 0 } else { out.iter().position(|&b| b == 0).unwrap_or(out.len()) };
        Ok(XmlSceneState {
            len,
            unchanged,
            continues: flags & c::MPEGH_UI_CONTINUES_XML != 0,
            incomplete: flags & c::MPEGH_UI_INCOMPLETE_XML != 0,
        })
    }

    /// Activates the persistence of the user settings in `block` (at least 512 bytes are
    /// recommended; empty, or holding the data of a previous session), or deactivates it with
    /// `None`. Returns whether `block` held valid persistence data.
    pub fn set_persistence_memory(&mut self, block: Option<Vec<u8>>) -> Result<bool, UiError> {
        match self.inner.set_persistence_memory(block) {
            c::MPEGH_UI_OK => Ok(true),
            c::MPEGH_UI_OK_BUT_NO_VALID_DATA => Ok(false),
            e => Err(UiError::from_code(e)),
        }
    }

    /// The persistence memory with the current settings, to be saved for a later session
    /// (`None` if persistence is not active).
    pub fn persistence_memory(&mut self) -> Option<&[u8]> {
        let (_, mem, size) = self.inner.get_persistence_memory();
        mem.map(|m| &m[..(size as usize).min(m.len())])
    }
}
