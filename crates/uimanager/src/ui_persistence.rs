// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libUIManager/src/uiPersistence.h and libUIManager/src/uiPersistence.cpp.

//! MPEG-H UI persistence handling: stores the UI commands per content (UUID) in a memory block
//! owned by the application.
//!
//! The memory block layout is identical to the C++ implementation on a little-endian machine
//! (the C++ code accesses the block as native `USHORT`s):
//!
//! | offset | content |
//! | --- | --- |
//! | 0 | version ID (`u16`) |
//! | 2 | first key index (`u16`) |
//! | 4 | accessibility setting, preferred language (3 bytes) |
//! | 8 | `maxKeys` keys (`UI_PERSISTENCE_KEY`, 20 bytes each) |
//! | 8 + 20 * `maxKeys` | `maxCommands` commands (`UI_PERSISTENCE_COMMAND`, 8 bytes each) |
//! | `memSize` - 2 | CRC (`u16`) |

use mpeghdec_fdk::bitstream::FdkBitStream;
use mpeghdec_fdk::crc::FdkCrcInfo;
use mpeghdec_fdk::digest::StateDigest;

use crate::ui_manager::*;

/// average number of commands per key
const AVG_CMDS_PER_KEY: usize = 3;

const INVALID_COMMAND: u8 = 0xFF;
const INVALID_INDEX: u16 = 0xFFFF;

const VERSION_ID: u16 = 0x0101;

const FLAG_BEFORE_LANGUAGE_CHANGE: u8 = 0x01;
const FLAG_BEFORE_ACCESSIBILITY_CHANGE: u8 = 0x02;

/// `sizeof(UI_PERSISTENCE_KEY)`: `UCHAR uuid[16]; USHORT firstCmdIdx; USHORT nextKeyIdx;`
const KEY_SIZE: usize = 20;
/// `sizeof(UI_PERSISTENCE_COMMAND)`: `UCHAR type; UCHAR id; SHORT param; USHORT nextCmdIdx;
/// USHORT flags;`
const CMD_SIZE: usize = 8;
/// byte offset of the keys
const KEYS_OFFSET: usize = 8;

/// `UI_PERSISTENCE_COMMAND`: UI command
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
struct UiPersistenceCommand {
    type_: u8,
    id: u8,
    param: i16,
    next_cmd_idx: u16,
    flags: u16,
}

/// `UI_PERSISTENCE_MANAGER`: persistence manager. The keys and commands live in the memory
/// block (`memory`), accessed through byte offsets.
#[derive(StateDigest, Clone, Debug, Default)]
pub struct UiPersistenceManager {
    pub memory: Option<Vec<u8>>,
    pub mem_size: u16,

    pub max_keys: u16,

    pub max_commands: u16,
    /// byte offset of the commands (`commands` pointer)
    pub commands_offset: usize,

    pub first_key_idx: u16,
    pub next_get_cmd_idx: u16,
    pub next_get_cmd_flags: u8,

    pub accessibility_setting: u8,
    pub preferred_language: [i8; 3],
}

/// calc memory block CRC over the first `size` bytes of `mem`
fn calc_crc(mem: &[u8], size: u16) -> u16 {
    // The bitstream reader may access bytes behind the CRC region; give it a zero padded copy
    // of the block.
    let buf_size = (mem.len() + 8).next_power_of_two();
    let mut buf = vec![0u8; buf_size];
    buf[..mem.len()].copy_from_slice(mem);
    let mut bs = FdkBitStream::reader(&buf, (size as u32) << 3);
    let mut crc_info = FdkCrcInfo::default();
    crc_info.init(0x8021, 0xFFFF, 16);
    let reg = crc_info.start_reg(&mut bs, 0);
    bs.push_for((size as u32) << 3);
    crc_info.end_reg(&mut bs, reg);

    crc_info.get_crc()
}

impl UiPersistenceManager {
    // ---- memory access (out of range accesses, which only a corrupted memory block with a
    // valid CRC can cause, read 0xFF and do not write) ----

    fn rd8(&self, off: usize) -> u8 {
        self.memory.as_ref().and_then(|m| m.get(off).copied()).unwrap_or(0xFF)
    }

    fn wr8(&mut self, off: usize, v: u8) {
        if let Some(b) = self.memory.as_mut().and_then(|m| m.get_mut(off)) {
            *b = v;
        }
    }

    fn rd16(&self, off: usize) -> u16 {
        u16::from_le_bytes([self.rd8(off), self.rd8(off + 1)])
    }

    fn wr16(&mut self, off: usize, v: u16) {
        let b = v.to_le_bytes();
        self.wr8(off, b[0]);
        self.wr8(off + 1, b[1]);
    }

    fn key_off(k: u16) -> usize {
        KEYS_OFFSET + k as usize * KEY_SIZE
    }

    fn key_uuid(&self, k: u16) -> [u8; 16] {
        let o = Self::key_off(k);
        core::array::from_fn(|i| self.rd8(o + i))
    }

    fn set_key_uuid(&mut self, k: u16, uuid: &[u8; 16]) {
        let o = Self::key_off(k);
        for (i, &b) in uuid.iter().enumerate() {
            self.wr8(o + i, b);
        }
    }

    fn key_first_cmd(&self, k: u16) -> u16 {
        self.rd16(Self::key_off(k) + 16)
    }

    fn set_key_first_cmd(&mut self, k: u16, v: u16) {
        self.wr16(Self::key_off(k) + 16, v)
    }

    fn key_next(&self, k: u16) -> u16 {
        self.rd16(Self::key_off(k) + 18)
    }

    fn set_key_next(&mut self, k: u16, v: u16) {
        self.wr16(Self::key_off(k) + 18, v)
    }

    fn cmd_off(&self, c: u16) -> usize {
        self.commands_offset + c as usize * CMD_SIZE
    }

    fn cmd(&self, c: u16) -> UiPersistenceCommand {
        let o = self.cmd_off(c);
        UiPersistenceCommand {
            type_: self.rd8(o),
            id: self.rd8(o + 1),
            param: self.rd16(o + 2) as i16,
            next_cmd_idx: self.rd16(o + 4),
            flags: self.rd16(o + 6),
        }
    }

    fn set_cmd(&mut self, c: u16, cmd: &UiPersistenceCommand) {
        let o = self.cmd_off(c);
        self.wr8(o, cmd.type_);
        self.wr8(o + 1, cmd.id);
        self.wr16(o + 2, cmd.param as u16);
        self.wr16(o + 4, cmd.next_cmd_idx);
        self.wr16(o + 6, cmd.flags);
    }

    fn set_cmd_type(&mut self, c: u16, t: u8) {
        let o = self.cmd_off(c);
        self.wr8(o, t);
    }

    fn set_cmd_next(&mut self, c: u16, v: u16) {
        let o = self.cmd_off(c);
        self.wr16(o + 4, v);
    }

    fn set_cmd_flags(&mut self, c: u16, v: u16) {
        let o = self.cmd_off(c);
        self.wr16(o + 6, v);
    }

    // ---- uiPersistence.cpp ----

    /// delete commands
    fn delete_commands(&mut self, first_cmd_idx: u16) {
        let mut cmd_idx = first_cmd_idx;

        while cmd_idx != INVALID_INDEX {
            self.set_cmd_type(cmd_idx, INVALID_COMMAND);
            cmd_idx = self.cmd(cmd_idx).next_cmd_idx;
        }
    }

    /// add flag for all existing commands
    fn add_command_flag(&mut self, flag: u8) {
        for i in 0..self.max_commands {
            let f = self.cmd(i).flags | flag as u16;
            self.set_cmd_flags(i, f);
        }
    }

    /// get key (for reading find existing, for writing find empty or delete oldest)
    fn get_key(&mut self, uuid: &[u8; 16], for_writing: bool) -> u16 {
        let mut key_idx = self.first_key_idx;
        let mut prev_key_idx = INVALID_INDEX;

        // loop over key entries
        while key_idx != INVALID_INDEX {
            // stop on empty key
            if self.key_first_cmd(key_idx) == INVALID_INDEX {
                if for_writing {
                    break;
                } else {
                    return INVALID_INDEX;
                }
            }

            // stop if existing key found
            if self.key_uuid(key_idx) == *uuid {
                break;
            }

            // if this is the last key, delete it
            if for_writing && self.key_next(key_idx) == INVALID_INDEX {
                // delete commands
                self.delete_commands(self.key_first_cmd(key_idx));
                self.set_key_first_cmd(key_idx, INVALID_INDEX);

                break;
            }

            // go to next key
            prev_key_idx = key_idx;
            key_idx = self.key_next(key_idx);
        }

        // move key to front
        if key_idx != INVALID_INDEX && prev_key_idx != INVALID_INDEX {
            self.set_key_next(prev_key_idx, self.key_next(key_idx));
            self.set_key_next(key_idx, self.first_key_idx);
            self.first_key_idx = key_idx;
        }

        key_idx
    }

    /// delete key
    fn delete_key(&mut self, uuid: &[u8; 16]) {
        let mut key_idx = self.first_key_idx;
        let mut prev_key_idx = INVALID_INDEX;

        // find key
        while key_idx != INVALID_INDEX {
            // stop on empty key
            if self.key_first_cmd(key_idx) == INVALID_INDEX {
                return;
            }

            // stop if key found
            if self.key_uuid(key_idx) == *uuid {
                break;
            }

            // go to next key
            prev_key_idx = key_idx;
            key_idx = self.key_next(key_idx);
        }

        // delete key
        if key_idx != INVALID_INDEX {
            self.delete_commands(self.key_first_cmd(key_idx));
            self.set_key_first_cmd(key_idx, INVALID_INDEX);

            // move key to end
            if self.key_next(key_idx) != INVALID_INDEX {
                if prev_key_idx == INVALID_INDEX {
                    self.first_key_idx = self.key_next(key_idx);
                } else {
                    self.set_key_next(prev_key_idx, self.key_next(key_idx));
                }

                prev_key_idx = self.key_next(key_idx);
                while self.key_next(prev_key_idx) != INVALID_INDEX {
                    prev_key_idx = self.key_next(prev_key_idx);
                }
                self.set_key_next(prev_key_idx, key_idx);

                self.set_key_next(key_idx, INVALID_INDEX);
            }
        }
    }

    /// delete oldest key
    fn delete_oldest_key(&mut self) -> u16 {
        let mut key_idx = self.first_key_idx;

        // find last key
        while key_idx != INVALID_INDEX {
            // stop on last valid key
            let next = self.key_next(key_idx);
            if next == INVALID_INDEX {
                break;
            }
            if self.key_first_cmd(next) == INVALID_INDEX {
                break;
            }

            // go to next key
            key_idx = next;
        }

        // delete commands
        self.delete_commands(self.key_first_cmd(key_idx));
        self.set_key_first_cmd(key_idx, INVALID_INDEX);

        key_idx
    }

    /// get free command entry
    fn get_free_cmd(&mut self) -> u16 {
        loop {
            // find free command entry
            for i in 0..self.max_commands {
                if self.cmd(i).type_ == INVALID_COMMAND {
                    return i;
                }
            }

            // if no free entry found, delete old key and try again
            self.delete_oldest_key();
        }
    }

    /// update memory
    fn update_memory(&mut self) {
        if self.memory.is_none() {
            return;
        }
        let mem_size = self.mem_size as usize;

        // 16 bits for version ID, 16 bits for first key index, preferred settings
        self.wr16(0, VERSION_ID);
        self.wr16(2, self.first_key_idx);
        self.wr8(4, self.accessibility_setting);
        self.wr8(5, self.preferred_language[0] as u8);
        self.wr8(6, self.preferred_language[1] as u8);
        self.wr8(7, self.preferred_language[2] as u8);

        // 16 bits at end for CRC
        let crc = calc_crc(&self.memory.as_ref().unwrap()[..mem_size], self.mem_size - 2);
        self.wr16(mem_size - 2, crc);
    }

    /// `persistenceManagerCreate()`: create instance
    pub fn create() -> Box<UiPersistenceManager> {
        Box::default()
    }

    /// `persistenceManagerSetMemory(hPersistence, persistenceMemory, persistenceMemorySize)`:
    /// set memory. The block (`None` for `NULL`) is owned by the persistence manager; its size
    /// is `min(len, 65535)`. Returns -1 if the block is too small, 0 if it did not contain valid
    /// data (or persistence is disabled), 1 if valid data was restored.
    ///
    /// Differences to C++: the 16 bit alignment check (-2) does not apply, blocks smaller than
    /// 10 bytes are rejected with -1 (C++ has undefined behaviour) and passing the same block
    /// again is not possible.
    pub fn set_memory(&mut self, persistence_memory: Option<Vec<u8>>) -> i32 {
        // finalize current persistence memory block
        self.update_memory();

        // if no memory passed, disable persistency
        let Some(mut mem) = persistence_memory else {
            self.memory = None;
            self.mem_size = 0;
            return 0;
        };
        let mut size = mem.len().min(0xFFFF) as u16;
        if size & 1 != 0 {
            size -= 1;
        }
        if size == 0 {
            self.memory = None;
            self.mem_size = 0;
            return 0;
        }
        if size < 10 {
            /* undefined behaviour in C++ */
            mpeghdec_fdk::deviation::hit();
            self.memory = None;
            return -1;
        }

        // 16 bits at end for CRC, 16 bits for version ID, 16 bits for first key index, 32 bits
        // for preferred settings
        let crc_off = size as usize - 2;
        let mut left = size as usize - 2 - 2 - 2 - 4;

        // assign memory to keys
        self.max_keys = (left / (KEY_SIZE + AVG_CMDS_PER_KEY * CMD_SIZE)) as u16;
        left -= self.max_keys as usize * KEY_SIZE;

        // assign remaining memory to commands
        self.max_commands = (left / CMD_SIZE) as u16;
        self.commands_offset = KEYS_OFFSET + self.max_keys as usize * KEY_SIZE;

        // check memory is not too small
        if self.max_keys < 10 || self.max_commands < 30 {
            self.memory = None;
            return -1;
        }

        let version = u16::from_le_bytes([mem[0], mem[1]]);
        let stored_crc = u16::from_le_bytes([mem[crc_off], mem[crc_off + 1]]);
        // init
        let ret =
            if version != VERSION_ID || stored_crc != calc_crc(&mem[..size as usize], size - 2) {
                // clear memory
                mem[..size as usize].fill(0xFF);
                self.memory = Some(mem);

                // init keys
                self.first_key_idx = 0;

                for i in 0..self.max_keys {
                    self.set_key_next(i, i + 1);
                }
                self.set_key_next(self.max_keys - 1, INVALID_INDEX);

                0
            } else {
                // restore first key index
                self.first_key_idx = u16::from_le_bytes([mem[2], mem[3]]);

                // restore preferred settings
                self.accessibility_setting = mem[4];
                self.preferred_language = [mem[5] as i8, mem[6] as i8, mem[7] as i8];
                self.memory = Some(mem);

                1
            };

        self.mem_size = size;
        self.next_get_cmd_idx = INVALID_INDEX;

        ret
    }

    /// `persistenceManagerGetMemory(hPersistence, persistenceMemory, persistenceMemorySize)`:
    /// get memory (updates the block content).
    pub fn get_memory(&mut self) -> (Option<&[u8]>, u16) {
        self.update_memory();

        (self.memory.as_deref(), self.mem_size)
    }

    /// `persistenceManagerSaveCommand(hPersistence, uiAction)`: save command
    pub fn save_command(&mut self, ui_action: &UiManagerAction) {
        // check if active
        if self.memory.is_none() {
            return;
        }

        // init command data
        let mut command = UiPersistenceCommand {
            type_: ui_action.action_type,
            id: 0,
            param: 0,
            next_cmd_idx: INVALID_INDEX,
            flags: 0,
        };

        // handle UI actions
        match ui_action.action_type {
            UI_MANAGER_COMMAND_RESET => {
                self.delete_key(&ui_action.uuid);
                return;
            }

            UI_MANAGER_COMMAND_PRESET_SELECTED => {
                self.delete_key(&ui_action.uuid);
                command.id = ui_action.param_int as u8;
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_MUTING_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_MUTING_CHANGED => {
                command.id = ui_action.param_int as u8;
                command.param = ui_action.param_bool as i16;
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_BALANCE_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_AZIMUTH_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_ELEVATION_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_BALANCE_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_AZIMUTH_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_ELEVATION_CHANGED => {
                command.id = ui_action.param_int as u8;
                command.param = (ui_action.param_float >> 15) as i16;
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_SELECTED => {
                command.id = ui_action.param_int as u8;
                command.param = (ui_action.param_float >> 16) as i16;
            }

            UI_MANAGER_COMMAND_ACCESSIBILITY_PREFERENCE => {
                if ui_action.param_int != self.accessibility_setting as i32 {
                    self.accessibility_setting = ui_action.param_int as u8;
                    self.add_command_flag(FLAG_BEFORE_ACCESSIBILITY_CHANGE);
                }
                return;
            }

            UI_MANAGER_COMMAND_AUDIO_LANGUAGE_SELECTED => {
                if ui_action.param_text[..3] != self.preferred_language {
                    if ui_action.param_int == 0 {
                        self.preferred_language.copy_from_slice(&ui_action.param_text[..3]);
                    }
                    self.add_command_flag(FLAG_BEFORE_LANGUAGE_CHANGE);
                }
                return;
            }

            _ => return,
        }

        // get key
        let key_idx = self.get_key(&ui_action.uuid, true);

        self.set_key_uuid(key_idx, &ui_action.uuid);

        // get command index
        let mut cmd_idx = self.key_first_cmd(key_idx);
        let mut last_cmd_idx = cmd_idx;

        // check for existing command to replace
        while cmd_idx != INVALID_INDEX {
            let cmd2 = self.cmd(cmd_idx);

            // if command of same type with same ID exists, replace it
            if cmd2.type_ == command.type_ && cmd2.id == command.id {
                command.next_cmd_idx = cmd2.next_cmd_idx;
                break;
            }

            last_cmd_idx = cmd_idx;
            cmd_idx = cmd2.next_cmd_idx;
        }

        // if no existing command to replace, add new command
        if cmd_idx == INVALID_INDEX {
            cmd_idx = self.get_free_cmd();

            if last_cmd_idx == INVALID_INDEX || self.key_first_cmd(key_idx) == INVALID_INDEX {
                // this is the first command for this key
                self.set_key_first_cmd(key_idx, cmd_idx);
            } else {
                // set next index for last command of this key
                self.set_cmd_next(last_cmd_idx, cmd_idx);
            }
        }

        // write command
        self.set_cmd(cmd_idx, &command);
    }

    /// `persistenceManagerGetCommand(hPersistence, uuid, uiAction)`: get command (if UUID passed
    /// get first command, if `None` passed get next command). Returns 1 if a command was
    /// returned.
    pub fn get_command(&mut self, uuid: Option<&[u8; 16]>, ui_action: &mut UiManagerAction) -> u8 {
        // check if active
        if self.memory.is_none() {
            return 0;
        }

        let key_idx;
        let mut cmd_idx;
        let mut flags: u8;

        // get command index
        if let Some(uuid) = uuid {
            // get key index
            key_idx = self.get_key(uuid, false);
            if key_idx == INVALID_INDEX {
                self.next_get_cmd_idx = INVALID_INDEX;
                return 0;
            }

            // first command index
            cmd_idx = self.key_first_cmd(key_idx);
            flags = FLAG_BEFORE_ACCESSIBILITY_CHANGE | FLAG_BEFORE_LANGUAGE_CHANGE;
        } else {
            // next command index
            key_idx = self.first_key_idx;
            cmd_idx = self.next_get_cmd_idx;
            flags = self.next_get_cmd_flags;
        }

        // find next command
        let mut flags_diff = flags;
        if cmd_idx == INVALID_INDEX {
            if flags == 0 {
                return 0;
            }

            flags -= 1;
            cmd_idx = self.key_first_cmd(key_idx);
        }
        while self.cmd(cmd_idx).flags != flags as u16 {
            cmd_idx = self.cmd(cmd_idx).next_cmd_idx;
            if cmd_idx == INVALID_INDEX {
                if flags == 0 {
                    break;
                }

                flags -= 1;
                cmd_idx = self.key_first_cmd(key_idx);
            }
        }
        flags_diff ^= flags;

        // send commands to apply automatic accessibility/language selection
        if uuid.is_none() {
            if flags_diff & FLAG_BEFORE_ACCESSIBILITY_CHANGE != 0 {
                ui_action.action_type = UI_MANAGER_COMMAND_ACCESSIBILITY_PREFERENCE;
                ui_action.present_flags = 0;

                self.next_get_cmd_idx = cmd_idx;
                self.next_get_cmd_flags = flags;

                return 1;
            }
            if flags_diff & FLAG_BEFORE_LANGUAGE_CHANGE != 0 {
                ui_action.action_type = UI_MANAGER_COMMAND_AUDIO_LANGUAGE_SELECTED;
                ui_action.present_flags = 0;

                self.next_get_cmd_idx = cmd_idx;
                self.next_get_cmd_flags = flags;

                return 1;
            }
        }

        // check if no more commands
        if cmd_idx == INVALID_INDEX {
            self.next_get_cmd_idx = INVALID_INDEX;
            self.next_get_cmd_flags = 0;
            return 0;
        }

        // restore UI action
        let command = self.cmd(cmd_idx);

        ui_action.uuid = self.key_uuid(key_idx);
        ui_action.action_type = command.type_;
        ui_action.present_flags = 0;

        match command.type_ {
            UI_MANAGER_COMMAND_PRESET_SELECTED => {
                ui_action.param_int = command.id as i32;
                ui_action.present_flags |= FLAG_XML_PARAM_INT;
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_MUTING_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_MUTING_CHANGED => {
                ui_action.param_int = command.id as i32;
                ui_action.param_bool = command.param as u8;
                ui_action.present_flags |= FLAG_XML_PARAM_INT | FLAG_XML_PARAM_BOOL;
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_BALANCE_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_AZIMUTH_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_ELEVATION_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_BALANCE_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_AZIMUTH_CHANGED
            | UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_ELEVATION_CHANGED => {
                ui_action.param_int = command.id as i32;
                ui_action.param_float = (command.param as i32) << 15;
                ui_action.present_flags |= FLAG_XML_PARAM_INT | FLAG_XML_PARAM_FLOAT;
            }

            UI_MANAGER_COMMAND_AUDIO_ELEMENT_SWITCH_SELECTED => {
                ui_action.param_int = command.id as i32;
                ui_action.param_float = (command.param as i32) << 16;
                ui_action.present_flags |= FLAG_XML_PARAM_INT | FLAG_XML_PARAM_FLOAT;
            }

            _ => {}
        }

        // save next command index
        self.next_get_cmd_idx = command.next_cmd_idx;
        self.next_get_cmd_flags = flags;

        1
    }
}
