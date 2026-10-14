// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from include/mpeghUIManager.h (the C API).

//! UI manager part of the C ABI: `mpegh_UI_*()` with the signatures of `mpeghUIManager.h`.
//!
//! `HANDLE_MPEGH_UI_MANAGER` points to a [`MPEGH_UI_MANAGER`] (a `Box` released by
//! `mpegh_UI_Manager_Close()`), which holds the straight port
//! ([`mpeghdec_uimanager::mpegh_ui_manager`]).
//!
//! **Persistence memory.** C keeps the caller's memory block and reads and writes it in place;
//! the port keeps its own copy. The C ABI copies the block into the port in
//! `mpegh_UI_SetPersistenceMemory()` and copies the port's content back into the caller's block
//! at the end of every call, so the block holds after each call what C writes into it. The
//! pointer-based rules of C are reproduced: the current block is finalized before a new one is
//! set, an odd address is rejected (`MPEGH_UI_INVALID_PARAM`), and setting the current block
//! again changes nothing.
//!
//! **Deviations from C.** A NULL `self`, a NULL `mhasBuffer` of `mpegh_UI_UpdateMHAS()` or a NULL
//! `mhasLength` return `MPEGH_UI_INVALID_PARAM` (C dereferences them); a NULL `mhasBuffer` of
//! `mpegh_UI_FeedMHAS()` is an empty buffer. After a panic (no input is known to cause one) the
//! handle is marked failed and its functions return `MPEGH_UI_INVALID_STATE`.

use core::ffi::{c_char, c_uint, c_ushort, c_void};
use mpeghdec_uimanager::mpegh_ui_manager as u;
use std::panic::{catch_unwind, AssertUnwindSafe};

/// `MPEGH_UI_ERROR`
pub type MPEGH_UI_ERROR = core::ffi::c_int;

/// `struct MPEGH_UI_MANAGER`: the UI manager behind a handle.
pub struct MPEGH_UI_MANAGER {
    ui: Box<u::MpeghUiManager>,
    /// The caller's persistence memory block and its size as C keeps them (`memory`,
    /// `memSize`).
    block: Option<(*mut u8, u16)>,
    /// A call on this handle panicked.
    failed: bool,
}

/// `HANDLE_MPEGH_UI_MANAGER`
pub type HANDLE_MPEGH_UI_MANAGER = *mut MPEGH_UI_MANAGER;

impl MPEGH_UI_MANAGER {
    /// Copies the port's persistence memory into the caller's block.
    fn write_back(&mut self) {
        let Some((ptr, size)) = self.block else { return };
        let Some(mem) = self.ui.h_ui_manager.h_persistence.memory.as_deref() else { return };
        let n = mem.len().min(size as usize);
        // SAFETY: `ptr` is valid for `size` bytes while it is set (contract of
        // `mpegh_UI_SetPersistenceMemory()`), and the port's copy is separate memory.
        unsafe { core::ptr::copy_nonoverlapping(mem.as_ptr(), ptr, n) };
    }
}

/// Runs `f` on the UI manager of `h` and copies the persistence memory back:
/// `MPEGH_UI_INVALID_PARAM` for a NULL handle, `MPEGH_UI_INVALID_STATE` after a panic.
///
/// # Safety
/// `h` NULL or an open handle.
unsafe fn with_ui(
    h: HANDLE_MPEGH_UI_MANAGER,
    f: impl FnOnce(&mut MPEGH_UI_MANAGER) -> MPEGH_UI_ERROR,
) -> MPEGH_UI_ERROR {
    // SAFETY: the caller's guarantee.
    let Some(h) = (unsafe { h.as_mut() }) else { return u::MPEGH_UI_INVALID_PARAM };
    if h.failed {
        return u::MPEGH_UI_INVALID_STATE;
    }
    match catch_unwind(AssertUnwindSafe(|| {
        let r = f(h);
        h.write_back();
        r
    })) {
        Ok(r) => r,
        Err(_) => {
            h.failed = true;
            u::MPEGH_UI_INVALID_STATE
        }
    }
}

/// `HANDLE_MPEGH_UI_MANAGER mpegh_UI_Manager_Open(void)`
#[no_mangle]
pub extern "C" fn mpegh_UI_Manager_Open() -> HANDLE_MPEGH_UI_MANAGER {
    match catch_unwind(u::MpeghUiManager::open) {
        Ok(ui) => Box::into_raw(Box::new(MPEGH_UI_MANAGER { ui, block: None, failed: false })),
        Err(_) => core::ptr::null_mut(),
    }
}

/// `void mpegh_UI_Manager_Close(HANDLE_MPEGH_UI_MANAGER self)`
///
/// # Safety
/// `self_` NULL or an open handle, which is invalid afterwards.
#[no_mangle]
pub unsafe extern "C" fn mpegh_UI_Manager_Close(self_: HANDLE_MPEGH_UI_MANAGER) {
    if !self_.is_null() {
        // SAFETY: an open handle is a `Box` of `mpegh_UI_Manager_Open()`.
        let h = unsafe { Box::from_raw(self_) };
        let _ = catch_unwind(AssertUnwindSafe(|| drop(h)));
    }
}

/// `MPEGH_UI_ERROR mpegh_UI_GetXmlSceneState(HANDLE_MPEGH_UI_MANAGER self, char* xmlOut,
/// unsigned int xmlOutSize, unsigned int flagsIn, unsigned int* flagsOut)`
///
/// # Safety
/// `self_` NULL or an open handle; `xmlOut` NULL or valid for writes of `xmlOutSize` bytes;
/// `flagsOut` NULL or valid for writes.
#[no_mangle]
pub unsafe extern "C" fn mpegh_UI_GetXmlSceneState(
    self_: HANDLE_MPEGH_UI_MANAGER,
    xmlOut: *mut c_char,
    xmlOutSize: c_uint,
    flagsIn: c_uint,
    flagsOut: *mut c_uint,
) -> MPEGH_UI_ERROR {
    if xmlOut.is_null() || flagsOut.is_null() {
        return u::MPEGH_UI_INVALID_PARAM;
    }
    // SAFETY: the caller's guarantee.
    let (out, flags) = unsafe {
        (core::slice::from_raw_parts_mut(xmlOut.cast::<u8>(), xmlOutSize as usize), &mut *flagsOut)
    };
    // SAFETY: the caller's guarantee.
    unsafe { with_ui(self_, |h| h.ui.get_xml_scene_state(out, flagsIn, flags)) }
}

/// `MPEGH_UI_ERROR mpegh_UI_ApplyXmlAction(HANDLE_MPEGH_UI_MANAGER self, const char* xmlIn,
/// unsigned int xmlInSize, unsigned int* flagsOut)`
///
/// # Safety
/// `self_` NULL or an open handle; `xmlIn` NULL or valid for reads of `xmlInSize` bytes;
/// `flagsOut` NULL or valid for writes.
#[no_mangle]
pub unsafe extern "C" fn mpegh_UI_ApplyXmlAction(
    self_: HANDLE_MPEGH_UI_MANAGER,
    xmlIn: *const c_char,
    xmlInSize: c_uint,
    flagsOut: *mut c_uint,
) -> MPEGH_UI_ERROR {
    if xmlIn.is_null() || flagsOut.is_null() {
        return u::MPEGH_UI_INVALID_PARAM;
    }
    // SAFETY: the caller's guarantee.
    let (xml, flags) = unsafe {
        (core::slice::from_raw_parts(xmlIn.cast::<u8>(), xmlInSize as usize), &mut *flagsOut)
    };
    // SAFETY: the caller's guarantee.
    unsafe { with_ui(self_, |h| h.ui.apply_xml_action(xml, flags)) }
}

/// `MPEGH_UI_ERROR mpegh_UI_FeedMHAS(HANDLE_MPEGH_UI_MANAGER self, unsigned char* mhasBuffer,
/// unsigned int mhasLength)`
///
/// # Safety
/// `self_` NULL or an open handle; `mhasBuffer` NULL or valid for reads of `mhasLength` bytes.
#[no_mangle]
pub unsafe extern "C" fn mpegh_UI_FeedMHAS(
    self_: HANDLE_MPEGH_UI_MANAGER,
    mhasBuffer: *mut u8,
    mhasLength: c_uint,
) -> MPEGH_UI_ERROR {
    let data: &[u8] = if mhasBuffer.is_null() || mhasLength == 0 {
        &[]
    } else {
        // SAFETY: the caller's guarantee.
        unsafe { core::slice::from_raw_parts(mhasBuffer, mhasLength as usize) }
    };
    // SAFETY: the caller's guarantee.
    unsafe { with_ui(self_, |h| h.ui.feed_mhas(data)) }
}

/// `MPEGH_UI_ERROR mpegh_UI_UpdateMHAS(HANDLE_MPEGH_UI_MANAGER self, unsigned char* mhasBuffer,
/// unsigned int mhasBufferLength, unsigned int* mhasLength)`
///
/// # Safety
/// `self_` NULL or an open handle; `mhasBuffer` NULL or valid for reads and writes of
/// `mhasBufferLength` bytes; `mhasLength` NULL or valid for reads and writes.
#[no_mangle]
pub unsafe extern "C" fn mpegh_UI_UpdateMHAS(
    self_: HANDLE_MPEGH_UI_MANAGER,
    mhasBuffer: *mut u8,
    mhasBufferLength: c_uint,
    mhasLength: *mut c_uint,
) -> MPEGH_UI_ERROR {
    if mhasBuffer.is_null() || mhasLength.is_null() {
        return u::MPEGH_UI_INVALID_PARAM;
    }
    // SAFETY: the caller's guarantee.
    let (buf, len) = unsafe {
        (core::slice::from_raw_parts_mut(mhasBuffer, mhasBufferLength as usize), &mut *mhasLength)
    };
    // SAFETY: the caller's guarantee.
    unsafe { with_ui(self_, |h| h.ui.update_mhas(buf, len)) }
}

/// `MPEGH_UI_ERROR mpegh_UI_SetPersistenceMemory(HANDLE_MPEGH_UI_MANAGER self, void*
/// persistenceMemoryBlock, unsigned short persistenceMemorySize)`
///
/// # Safety
/// `self_` NULL or an open handle; `persistenceMemoryBlock` NULL or valid for reads and writes
/// of `persistenceMemorySize` bytes until the handle is closed or another block is set.
#[no_mangle]
pub unsafe extern "C" fn mpegh_UI_SetPersistenceMemory(
    self_: HANDLE_MPEGH_UI_MANAGER,
    persistenceMemoryBlock: *mut c_void,
    persistenceMemorySize: c_ushort,
) -> MPEGH_UI_ERROR {
    let ptr = persistenceMemoryBlock.cast::<u8>();
    let set = |h: &mut MPEGH_UI_MANAGER| {
        /* finalize the current block (C: updateMemory()) */
        let _ = h.ui.get_persistence_memory();
        h.write_back();
        /* check required 16 bit alignment */
        if ptr as usize & 1 != 0 {
            return u::MPEGH_UI_INVALID_PARAM;
        }
        let size = persistenceMemorySize & !1;
        if ptr.is_null() || size == 0 {
            h.block = None;
            return h.ui.set_persistence_memory(None);
        }
        /* if the same memory block is passed again, nothing changes */
        if h.block == Some((ptr, size)) {
            return u::MPEGH_UI_OK;
        }
        // SAFETY: the caller's guarantee.
        let content = unsafe { core::slice::from_raw_parts(ptr, size as usize) }.to_vec();
        let err = h.ui.set_persistence_memory(Some(content));
        h.block = h.ui.h_ui_manager.h_persistence.memory.is_some().then_some((ptr, size));
        err
    };
    // SAFETY: the caller's guarantee.
    unsafe { with_ui(self_, set) }
}

/// `MPEGH_UI_ERROR mpegh_UI_GetPersistenceMemory(HANDLE_MPEGH_UI_MANAGER self, void**
/// persistenceMemoryBlock, unsigned short* persistenceMemorySize)`
///
/// # Safety
/// `self_` NULL or an open handle; the output pointers NULL or valid for writes.
#[no_mangle]
pub unsafe extern "C" fn mpegh_UI_GetPersistenceMemory(
    self_: HANDLE_MPEGH_UI_MANAGER,
    persistenceMemoryBlock: *mut *mut c_void,
    persistenceMemorySize: *mut c_ushort,
) -> MPEGH_UI_ERROR {
    // SAFETY: the caller's guarantee.
    unsafe {
        with_ui(self_, |h| {
            let (_, _, size) = h.ui.get_persistence_memory();
            let ptr = h.block.map_or(core::ptr::null_mut(), |(p, _)| p.cast::<c_void>());
            if !persistenceMemoryBlock.is_null() {
                *persistenceMemoryBlock = ptr;
            }
            if !persistenceMemorySize.is_null() {
                *persistenceMemorySize = size;
            }
            u::MPEGH_UI_OK
        })
    }
}
