// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from demo/mpeghUiManager/mpeghUiManagerProcessor.h, mpeghUiManagerProcessor.cpp
// (the per sample processing used by the decoder demo) and interactivityScriptParser.h.

//! MPEG-H UI manager processing of the decoder demo: applies an XML user interactivity script to
//! the MHAS access units before they are decoded, writes the XML scene state and keeps the UI
//! persistence data in a file.

use mpeghdec::{UiError, UiManager};
use std::io::Write;

/// `MAX_MPEGH_FRAME_SIZE`
const MAX_MPEGH_FRAME_SIZE: usize = 65536;
/// `PERSISTENCE_BUFFER_SIZE`
const PERSISTENCE_BUFFER_SIZE: usize = 2048;
/// `XML_BUFFER_SIZE`
const XML_BUFFER_SIZE: usize = 1024;

/// `SInteractivityEvent`
struct InteractivityEvent {
    sample_pos: i32,
    action: String,
}

/// `CInteractivityScriptParser`: a list of `<ActionEvent .../>` lines, delayed by
/// `<Sleep Frame="n" />` lines.
#[derive(Default)]
pub struct InteractivityScriptParser {
    events: Vec<InteractivityEvent>,
}

impl InteractivityScriptParser {
    pub fn open(script_filename: &str) -> Result<Self, String> {
        let text = std::fs::read(script_filename)
            .map_err(|_| "ERROR: Cannot open script file!".to_string())?;
        let text = String::from_utf8_lossy(&text);
        let sleep_frame = "<Sleep Frame=\"";
        let action_event = "<ActionEvent ";
        let mut events = Vec::new();
        let mut event_pos: i32 = -1;
        let mut found_valid_line = false;
        for line in text.split('\n') {
            if line.is_empty() {
                continue;
            } else if let Some(rest) = line.strip_prefix(sleep_frame) {
                found_valid_line = false;
                if let Some(n) = rest.find(|c: char| c.is_ascii_digit()) {
                    let digits: String =
                        rest[n..].chars().take_while(|c| c.is_ascii_digit()).collect();
                    /* std::stoi() throws on overflow */
                    event_pos = digits.parse().map_err(|_| "stoi".to_string())?;
                    found_valid_line = true;
                }
            } else if line.starts_with(action_event) && (found_valid_line || event_pos <= 0) {
                events.push(InteractivityEvent { sample_pos: event_pos, action: line.to_string() });
            }
        }
        Ok(InteractivityScriptParser { events })
    }

    /// `getCommands(position)`
    pub fn get_commands(&self, position: i32) -> Vec<&str> {
        self.events.iter().filter(|e| e.sample_pos == position).map(|e| e.action.as_str()).collect()
    }
}

/// `CUIManagerProcessor` (without the MP4 reader/writer of the stand-alone UI manager demo).
pub struct UiManagerProcessor {
    persistence_filename: String,
    ui_manager: UiManager,
    /// `m_sceneStateFile`: `None` if not opened. A default constructed (unopened)
    /// `std::ofstream` is usable in a boolean context until the first write fails, so the XML
    /// scene state is still requested until there is output
    /// ([`UiManagerProcessor::scene_state_ok`]).
    scene_state_file: Option<std::io::BufWriter<std::fs::File>>,
    /// `(bool)m_sceneStateFile`: no write to the (unopened) stream has failed yet
    scene_state_ok: bool,
    parser: InteractivityScriptParser,
    read_only_mode: bool,
}

impl UiManagerProcessor {
    pub fn new(
        script_filename: &str,
        persist_filename: &str,
        xml_scene_state_filename: &str,
    ) -> Result<Self, String> {
        // Open XML command script file (optional)
        let parser = if script_filename.is_empty() {
            InteractivityScriptParser::default()
        } else {
            InteractivityScriptParser::open(script_filename)?
        };

        // Open XML scene state output file (optional)
        let scene_state_file = if xml_scene_state_filename.is_empty() {
            None
        } else {
            Some(std::io::BufWriter::new(
                std::fs::File::create(xml_scene_state_filename)
                    .map_err(|_| "Warning: Failed to open XML output file!".to_string())?,
            ))
        };

        let mut persistence_memory = vec![0u8; PERSISTENCE_BUFFER_SIZE];
        // Load persistence data from file (optional)
        if !persist_filename.is_empty() {
            match std::fs::read(persist_filename) {
                Ok(d) => {
                    if d.len() < PERSISTENCE_BUFFER_SIZE {
                        println!("Warning: Unable to read enough data from persistence file!");
                    } else {
                        persistence_memory.copy_from_slice(&d[..PERSISTENCE_BUFFER_SIZE]);
                    }
                }
                Err(_) => {
                    println!(
                        "Warning: Persistence input file not found! Creating a new one after processing."
                    );
                }
            }
        }

        // Open an UI manager instance
        let mut ui_manager = UiManager::new();

        // Set persistence data memory
        let _ = ui_manager.set_persistence_memory(Some(persistence_memory));

        Ok(UiManagerProcessor {
            persistence_filename: persist_filename.to_string(),
            ui_manager,
            scene_state_file,
            scene_state_ok: true,
            parser,
            read_only_mode: false,
        })
    }

    /// `processSingleSample(sample, sampleCounter)`: feeds the access unit to the UI manager,
    /// writes the XML scene state, applies the script actions of this sample and updates the
    /// access unit (inserting the UI packets).
    pub fn process_single_sample(&mut self, sample: &mut Vec<u8>, sample_counter: u64) {
        // Feed data to the UI manager
        let mhas_length = sample.len();
        let fed = self.ui_manager.feed_mhas(sample).is_ok();
        if !fed {
            println!("Warning: Unable to feed MHAS for ISOBMFF/MP4 Sample {sample_counter}");
        }

        // Write XML scene state
        if self.scene_state_ok {
            let mut buf = vec![0u8; XML_BUFFER_SIZE];
            loop {
                let Ok(state) = self.ui_manager.xml_scene_state(&mut buf, false, false) else {
                    println!(
                        "Warning: Failed to get XML scene state for ISOBMFF/MP4 Sample {sample_counter}"
                    );
                    break;
                };
                if state.unchanged {
                    break;
                }
                match self.scene_state_file.as_mut() {
                    Some(f) => {
                        if !state.continues {
                            let _ = writeln!(f, "[{sample_counter}]");
                        }
                        let _ = f.write_all(&buf[..state.len]);
                    }
                    /* writing to the unopened stream fails */
                    None => self.scene_state_ok = false,
                }
                if !state.incomplete {
                    break;
                }
            }
        }

        // process XML actions
        let position = sample_counter.wrapping_sub(1) as i32;
        let commands: Vec<String> =
            self.parser.get_commands(position).iter().map(|s| s.to_string()).collect();
        for c in &commands {
            let res = self.ui_manager.apply_xml_action(c);
            if res == Err(UiError::NotAllowed) {
                if !self.read_only_mode {
                    println!(
                        "Warning: 1Input MHAS already contains UI manager packets, ignoring interactivity script!"
                    );
                }
                self.read_only_mode = true;
            } else if res.is_err() {
                println!(
                    "Warning: Failed to apply XML action for ISOBMFF/MP4 Sample {sample_counter} with command:"
                );
                println!("{c}");
            }
        }

        if fed {
            sample.resize(MAX_MPEGH_FRAME_SIZE, 0);
            // Update the MPEG-H frame
            let res = self.ui_manager.update_mhas(sample, mhas_length);
            if res == Err(UiError::NotAllowed) {
                sample.truncate(mhas_length);
                if !self.read_only_mode {
                    println!(
                        "Warning: Input MHAS already contains UI manager packets, ignoring interactivity script!"
                    );
                }
                self.read_only_mode = true;
            } else if let Ok(new_mhas_length) = res {
                sample.truncate(new_mhas_length);
            } else {
                sample.truncate(mhas_length);
                println!("Warning: Failed to update MHAS for ISOBMFF/MP4 Sample {sample_counter}");
            }
        }
    }

    /// `~CUIManagerProcessor()`: saves the persistence data to the file (optional).
    pub fn finish(mut self) {
        if let Some(mut f) = self.scene_state_file.take() {
            let _ = f.flush();
        }
        if self.persistence_filename.is_empty() {
            return;
        }
        match self.ui_manager.persistence_memory() {
            Some(m) if !m.is_empty() => {
                if std::fs::write(&self.persistence_filename, m).is_err() {
                    println!("Warning: Failed to open/create persistence data file!");
                }
            }
            _ => println!("Warning: Obtaining persistence memory failed!"),
        }
    }
}
