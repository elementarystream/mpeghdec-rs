// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

//! Port of `libUIManager` (user interactivity manager, embedded by the core decoder) and
//! `libMpeghUIMan` (the `mpegh_UI_*` MHAS UI manager API, [`mpegh_ui_manager`]).
//!
//! The UI manager ([`ui_manager::UiManager`]) owns the audio scene information that the
//! transport decoder fills (`UI_Manager_GetAsiPointer()`), the user interactivity state, the XML
//! writer state and the persistence manager.

pub mod mpegh_ui_manager;
pub mod ui_manager;
pub mod ui_persistence;
pub mod ui_xml;

pub use mpegh_ui_manager::*;
pub use ui_manager::*;
