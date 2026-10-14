// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/FDK_lpc.h, libFDK/src/FDK_lpc.cpp and libFDK/src/arm/FDK_lpc_arm.cpp
// (the ARM file only contains NEON code, which is inactive in the reference configuration).

//! LPC related functions.
//!
//! The C++ overloads for `FIXP_LPC_TNS` (= `FIXP_DBL`) and `FIXP_LPC` (= `FIXP_SGL`)
//! coefficients are separate functions here; the `FIXP_SGL` variants carry the `_sgl` suffix.
//! The signal length (`signal_size` in C++) is the length of the `signal` slice.

use crate::common_fix::*;
use crate::fixpoint_math::{f_add_norm, f_div_norm_signed, f_div_norm_signed_noexp, schur_div};
use crate::scale::{
    saturate_left_shift, saturate_left_shift_alt, scale_value, scale_value_saturate,
};

pub const LPC_MAX_ORDER: usize = 24;

/// `FIXP_LPC_TNS`: LPC coefficient type of the TNS IIR synthesis variant.
pub type FixpLpcTns = FixpDbl;
/// `FIXP_LPC`: LPC coefficient type.
pub type FixpLpc = FixpSgl;

/// `FX_DBL2FX_LPC_TNS(x)`
#[inline(always)]
pub const fn fx_dbl2fx_lpc_tns(x: FixpDbl) -> FixpLpcTns {
    x
}

/// `FX_LPC_TNS2FX_DBL(x)`
#[inline(always)]
pub const fn fx_lpc_tns2fx_dbl(x: FixpLpcTns) -> FixpDbl {
    x
}

/// `FL2FXCONST_LPC_TNS(val)`
#[inline(always)]
pub const fn fl2fxconst_lpc_tns(val: f64) -> FixpLpcTns {
    fl2fxconst_dbl(val)
}

pub const MAXVAL_LPC_TNS: i32 = MAXVAL_DBL;

/// `FX_DBL2FX_LPC(x)`
#[inline(always)]
pub const fn fx_dbl2fx_lpc(x: FixpDbl) -> FixpLpc {
    fx_dbl2fx_sgl(x)
}

/// `FX_DBL2FXCONST_LPC(x)`
#[inline(always)]
pub const fn fx_dbl2fxconst_lpc(x: FixpDbl) -> FixpLpc {
    fx_dbl2fxconst_sgl(x)
}

/// `FX_LPC2FX_DBL(x)`
#[inline(always)]
pub const fn fx_lpc2fx_dbl(x: FixpLpc) -> FixpDbl {
    fx_sgl2fx_dbl(x)
}

/// `FL2FXCONST_LPC(val)`
#[inline(always)]
pub const fn fl2fxconst_lpc(val: f64) -> FixpLpc {
    fl2fxconst_sgl(val)
}

pub const MAXVAL_LPC: i32 = MAXVAL_SGL;

/// Internal scaling of LPC synthesis to avoid overflow of filter states. This depends on the LPC
/// order, because the LPC order defines the amount of MAC operations.
/// Assume that Synthesis filter output does not clip and filter accu does change no more than 1.0
/// for each iteration. ceil(0.5*log((1:24))/log(2))
static ORDER_LD: [i8; 24] =
    [0, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3];

/// Start index and direction of a signal traversal with increment `inc` (1 or -1).
#[inline(always)]
fn signal_start(signal_size: usize, inc: i32) -> isize {
    if inc == -1 {
        signal_size as isize - 1
    } else {
        0
    }
}

/// `CLpc_SynthesisLattice(signal, signal_size, signal_e, signal_e_out, inc, coeff, order, state)`
/// with `FIXP_SGL` ParCor coefficients.
///
/// Synthesize the signal from the residual through LPC synthesis, using ParCor coefficients, in
/// place. The algorithm assumes a filter gain of max 1.0. If the filter gain is higher, this must
/// be accounted into the values of `signal_e` and/or `signal_e_out` to avoid overflows. `state`
/// holds at least `order` values.
pub fn clpc_synthesis_lattice_sgl(
    signal: &mut [FixpDbl],
    signal_e: i32,
    signal_e_out: i32,
    inc: i32,
    coeff: &[FixpSgl],
    order: i32,
    state: &mut [FixpDbl],
) {
    let signal_size = signal.len();
    let mut p_signal = signal_start(signal_size, inc);
    let order = order as isize;

    let shift = -(ORDER_LD[(order - 1) as usize] as i32);

    for _ in 0..signal_size {
        let mut p_state = order - 1;
        let mut p_coeff = order - 1;

        // The filter recursion wraps around like the C++ code on overflow.
        let mut tmp = scale_value(signal[p_signal as usize], shift + signal_e)
            .wrapping_sub(f_mult_div2(coeff[p_coeff as usize], state[p_state as usize]));
        p_coeff -= 1;
        p_state -= 1;
        for _ in 1..order {
            tmp = f_mult_sub_div2(tmp, coeff[p_coeff as usize], state[p_state as usize]);
            state[(p_state + 1) as usize] = state[p_state as usize]
                .wrapping_add(f_mult_div2(coeff[p_coeff as usize], tmp) << 2);
            p_coeff -= 1;
            p_state -= 1;
        }

        signal[p_signal as usize] = scale_value_saturate(tmp, -shift - signal_e_out);

        // exponent of state[] is -1
        state[(p_state + 1) as usize] = tmp << 1;
        p_signal += inc as isize;
    }
}

/// `CLpc_SynthesisLattice(signal, signal_size, signal_e, signal_e_out, inc, coeff, order, state)`
/// with `FIXP_DBL` ParCor coefficients. See [`clpc_synthesis_lattice_sgl`].
pub fn clpc_synthesis_lattice(
    signal: &mut [FixpDbl],
    signal_e: i32,
    signal_e_out: i32,
    inc: i32,
    coeff: &[FixpDbl],
    order: i32,
    state: &mut [FixpDbl],
) {
    let signal_size = signal.len();
    let mut p_signal = signal_start(signal_size, inc);
    let order = order as isize;

    for _ in 0..signal_size {
        let mut p_state = order - 1;
        let mut p_coeff = order - 1;

        let accu = f_mult_sub_div2(
            scale_value(signal[p_signal as usize], signal_e - 1),
            coeff[p_coeff as usize],
            state[p_state as usize],
        );
        p_coeff -= 1;
        p_state -= 1;
        let mut tmp = saturate_left_shift_alt(accu, 1, DFRACT_BITS);

        for _ in 1..order {
            let accu = f_mult_sub_div2(tmp >> 1, coeff[p_coeff as usize], state[p_state as usize]);
            tmp = saturate_left_shift_alt(accu, 1, DFRACT_BITS);

            let accu = f_mult_add_div2(state[p_state as usize] >> 1, coeff[p_coeff as usize], tmp);
            p_coeff -= 1;
            state[(p_state + 1) as usize] = saturate_left_shift_alt(accu, 1, DFRACT_BITS);

            p_state -= 1;
        }

        signal[p_signal as usize] = scale_value(tmp, -signal_e_out);

        // exponent of state[] is 0
        state[(p_state + 1) as usize] = tmp;
        p_signal += inc as isize;
    }
}

/// `CLpc_Synthesis(signal, signal_size, inc, lpcCoeff_m, lpcCoeff_e, order, state, pStateIndex)`
/// with `FIXP_LPC_TNS` (`FIXP_DBL`) coefficients.
///
/// Synthesize the signal from the residual through LPC synthesis, using LP coefficients, in
/// place. `state` is a state buffer of at least `order` values, `p_state_index` the state index
/// storage.
pub fn clpc_synthesis(
    signal: &mut [FixpDbl],
    inc: i32,
    lpc_coeff_m: &[FixpLpcTns],
    lpc_coeff_e: i32,
    order: i32,
    state: &mut [FixpDbl],
    p_state_index: &mut i32,
) {
    let signal_size = signal.len();
    let mut state_index = *p_state_index;
    let lpc_coeff_shift = lpc_coeff_e + 1;
    let ord = order as usize;

    if (1..=LPC_MAX_ORDER).contains(&ord) && (0..ord as i32).contains(&state_index) {
        if inc == -1 {
            clpc_synthesis_hist::<true>(
                signal,
                lpc_coeff_m,
                lpc_coeff_shift,
                ord,
                state,
                p_state_index,
            );
        } else {
            clpc_synthesis_hist::<false>(
                signal,
                lpc_coeff_m,
                lpc_coeff_shift,
                ord,
                state,
                p_state_index,
            );
        }
        return;
    }

    let mut coeff = [0 as FixpLpcTns; 2 * LPC_MAX_ORDER];
    coeff[..ord].copy_from_slice(&lpc_coeff_m[..ord]);
    coeff[ord..2 * ord].copy_from_slice(&lpc_coeff_m[..ord]);

    let mut p_signal = signal_start(signal_size, inc);

    for _ in 0..signal_size {
        let p_coeff = &coeff[(order - state_index) as usize..];

        let mut x = signal[p_signal as usize].wrapping_shr(lpc_coeff_shift as u32);
        // y(n) = x(n) - lpc[1]*y(n-1) - ... - lpc[order]*y(n-order)
        for j in 0..ord {
            x = f_mult_sub_div2(x, state[j], p_coeff[j]);
        }
        x = saturate_left_shift(x, lpc_coeff_shift, DFRACT_BITS);

        // Update states
        state_index = if state_index - 1 < 0 { order - 1 } else { state_index - 1 };
        state[state_index as usize] = x;

        signal[p_signal as usize] = x;
        p_signal += inc as isize;
    }

    *p_state_index = state_index;
}

/// [`clpc_synthesis`] for `1 <= order <= LPC_MAX_ORDER` with the past outputs in a linear
/// history buffer instead of the state ring (`REV`: `inc == -1`).
///
/// The ring holds `y(n-1-k)` in `state[(stateIndex + k) % order]`, which is filtered with
/// `lpc[k]`. The products `fMultDiv2(y(n-1-k), lpc[k])` are subtracted with wrapping
/// arithmetic, so their order does not matter: the older outputs are subtracted first and the
/// newest one (kept in a register) last, which shortens the recursion. The ring and the state
/// index are updated at the end exactly as the per sample updates of the C++ code leave them.
fn clpc_synthesis_hist<const REV: bool>(
    signal: &mut [FixpDbl],
    lpc_coeff_m: &[FixpLpcTns],
    lpc_coeff_shift: i32,
    ord: usize,
    state: &mut [FixpDbl],
    p_state_index: &mut i32,
) {
    const W: usize = 256;
    let lpc = &lpc_coeff_m[..ord];
    let state = &mut state[..ord];
    let si = *p_state_index as usize;
    let n = signal.len();

    // h[pos - 1 - k] = y(n-1-k)
    let mut h = [0 as FixpDbl; W + LPC_MAX_ORDER];
    for k in 0..ord {
        h[ord - 1 - k] = state[(si + k) % ord];
    }
    let mut pos = ord;
    let mut newest = h[ord - 1];
    let c0 = lpc[0];
    let older = &lpc[1..];

    for i in 0..n {
        let idx = if REV { n - 1 - i } else { i };
        let mut x = signal[idx].wrapping_shr(lpc_coeff_shift as u32);
        // y(n) = x(n) - lpc[1]*y(n-1) - ... - lpc[order]*y(n-order)
        let hist = &h[pos - ord..pos - 1];
        for (&y, &c) in hist.iter().zip(older.iter().rev()) {
            x = f_mult_sub_div2(x, y, c);
        }
        x = f_mult_sub_div2(x, newest, c0);
        x = saturate_left_shift(x, lpc_coeff_shift, DFRACT_BITS);

        signal[idx] = x;
        newest = x;
        h[pos] = x;
        pos += 1;
        if pos == W + ord {
            h.copy_within(W..W + ord, 0);
            pos = ord;
        }
    }

    // The C++ code decrements the state index (modulo order) per sample and stores y(n) there.
    let si_end = (si + ord - n % ord) % ord;
    for k in 0..ord {
        state[(si_end + k) % ord] = h[pos - 1 - k];
    }
    *p_state_index = si_end as i32;
}

/// `CLpc_Synthesis(signal, signal_size, inc, coeff_m, coeff_e, order, filtState, pStateIndex)`
/// with `FIXP_LPC` (`FIXP_SGL`) coefficients. See [`clpc_synthesis`].
pub fn clpc_synthesis_sgl(
    signal: &mut [FixpDbl],
    inc: i32,
    lpc_coeff_m: &[FixpLpc],
    lpc_coeff_e: i32,
    order: i32,
    state: &mut [FixpDbl],
    p_state_index: &mut i32,
) {
    let signal_size = signal.len();
    let mut state_index = *p_state_index;
    let lpc_coeff_shift = lpc_coeff_e + 1;
    let ord = order as usize;

    let mut coeff = [0 as FixpLpc; 2 * LPC_MAX_ORDER];
    coeff[..ord].copy_from_slice(&lpc_coeff_m[..ord]);
    coeff[ord..2 * ord].copy_from_slice(&lpc_coeff_m[..ord]);

    let mut p_signal = signal_start(signal_size, inc);

    for _ in 0..signal_size {
        let p_coeff = &coeff[(order - state_index) as usize..];

        let mut x = signal[p_signal as usize].wrapping_shr(lpc_coeff_shift as u32);
        // y(n) = x(n) - lpc[1]*y(n-1) - ... - lpc[order]*y(n-order)
        for j in 0..ord {
            x = f_mult_sub_div2(x, state[j], p_coeff[j]);
        }
        x = saturate_left_shift(x, lpc_coeff_shift, DFRACT_BITS);

        // Update states
        state_index = if state_index - 1 < 0 { order - 1 } else { state_index - 1 };
        state[state_index as usize] = x;

        signal[p_signal as usize] = x;
        p_signal += inc as isize;
    }

    *p_state_index = state_index;
}

/// `CLpc_AutoCorr(signal, signal_e, acorr, pAcorr_e, signal_size, order)`: calculate the
/// autocorrelation of any order and size.
///
/// `signal` holds `signal_size` values, `acorr` receives `order` values; `p_acorr_e` receives the
/// exponent of `acorr`. Returns the energy (`acorr[0]`).
pub fn clpc_auto_corr(
    signal_m: &[FixpDbl],
    signal_e: i32,
    acorr: &mut [FixpDbl],
    p_acorr_e: &mut i32,
    order: i32,
) -> FixpDbl {
    let signal_size = signal_m.len();
    let mut norm_m: FixpDbl = 0;

    // Determine energy and scale
    let mut scale = 0;
    for i in 0..signal_size {
        if norm_m > fl2fxconst_dbl(0.5)
            || (signal_m[i] >> scale) > fl2fxconst_dbl(0.70710678118654752440084436210485)
        {
            scale += 1;
            norm_m >>= 1;
        }
        // Note: only positive samples are checked, so large negative samples can make the sum
        // wrap around like in C++.
        norm_m = norm_m.wrapping_add(f_pow2(signal_m[i]) >> scale);
    }
    let nrg = norm_m;

    acorr[0] = nrg;

    for l in 1..order as usize {
        let mut result: FixpDbl = 0;
        for i in 0..signal_size.saturating_sub(l) {
            result = result.wrapping_add(f_mult(signal_m[i], signal_m[i + l]) >> scale);
        }
        acorr[l] = result;
    }

    *p_acorr_e = signal_e * 2 + scale;

    nrg
}

/// `CLpc_ParcorToLpc(reflCoeff, LpcCoeff, numOfCoeff, workBuffer)` with `FIXP_LPC_TNS`
/// (`FIXP_DBL`) coefficients: convert ParCor coefficients to LPC coefficients.
///
/// `work_buffer` holds at least `num_of_coeff` values. Returns the exponent of the LPC
/// coefficients.
pub fn clpc_parcor_to_lpc(
    refl_coeff: &[FixpLpcTns],
    lpc_coeff: &mut [FixpLpcTns],
    num_of_coeff: i32,
    work_buffer: &mut [FixpQdl],
) -> i32 {
    let n = num_of_coeff as usize;
    let mut shiftval = 0;
    let mut max_val: FixpQdl = 0;

    work_buffer[0] = refl_coeff[0] as FixpQdl;
    for i in 1..n {
        let mut j = 0;
        while j < i / 2 {
            let tmp1 = work_buffer[j];
            let tmp2 = work_buffer[i - 1 - j];
            work_buffer[j] += f_mult(refl_coeff[i], tmp2);
            work_buffer[i - 1 - j] += f_mult(refl_coeff[i], tmp1);
            j += 1;
        }
        if i & 1 != 0 {
            work_buffer[j] += f_mult(refl_coeff[i], work_buffer[j]);
        }

        work_buffer[i] = refl_coeff[i] as FixpQdl;
    }

    // calculate exponent
    for i in 0..n {
        max_val = f_max(max_val, f_abs(work_buffer[i]));
    }

    if max_val > MAXVAL_DBL as FixpQdl {
        shiftval = (DFRACT_BITS - 1) - f_norm((max_val >> (DFRACT_BITS - 1)) as FixpDbl);
    }

    for i in 0..n {
        lpc_coeff[i] = (work_buffer[i] >> shiftval) as FixpDbl;
    }

    shiftval
}

/// `CLpc_ParcorToLpc(reflCoeff, LpcCoeff, numOfCoeff, workBuffer)` with `FIXP_LPC` (`FIXP_SGL`)
/// coefficients. See [`clpc_parcor_to_lpc`].
pub fn clpc_parcor_to_lpc_sgl(
    refl_coeff: &[FixpLpc],
    lpc_coeff: &mut [FixpLpc],
    num_of_coeff: i32,
    work_buffer: &mut [FixpQdl],
) -> i32 {
    let n = num_of_coeff as usize;
    let mut shiftval = 0;
    let mut max_val: FixpQdl = 0;

    work_buffer[0] = fx_lpc2fx_dbl(refl_coeff[0]) as FixpQdl;
    for i in 1..n {
        let r = fx_lpc2fx_dbl(refl_coeff[i]);
        let mut j = 0;
        while j < i / 2 {
            let tmp1 = work_buffer[j];
            let tmp2 = work_buffer[i - 1 - j];
            work_buffer[j] += f_mult(r, tmp2);
            work_buffer[i - 1 - j] += f_mult(r, tmp1);
            j += 1;
        }
        if i & 1 != 0 {
            work_buffer[j] += f_mult(r, work_buffer[j]);
        }

        work_buffer[i] = r as FixpQdl;
    }

    // calculate exponent
    for i in 0..n {
        max_val = f_max(max_val, f_abs(work_buffer[i]));
    }

    if max_val > MAXVAL_DBL as FixpQdl {
        shiftval = (DFRACT_BITS - 1) - f_norm((max_val >> (DFRACT_BITS - 1)) as FixpDbl);
    }

    for i in 0..n {
        lpc_coeff[i] = fx_dbl2fx_lpc((work_buffer[i] >> shiftval) as FixpDbl);
    }

    shiftval
}

/// `CLpc_AutoToParcor(acorr, acorr_e, reflCoeff, order, pPredictionGain_m, pPredictionGain_e)`:
/// calculate ParCor (Partial autoCorrelation, reflection) coefficients from autocorrelation
/// coefficients using the Schur algorithm (instead of Levinson Durbin).
///
/// `acorr` holds `order + 1` autocorrelation coefficients (modified). `refl_coeff` receives the
/// `order` reflection coefficients; the first coefficient which is always 1.0 is omitted.
/// `p_prediction_gain` receives the prediction gain mantissa and exponent if given.
pub fn clpc_auto_to_parcor(
    acorr: &mut [FixpDbl],
    _acorr_e: i32,
    refl_coeff: &mut [FixpLpc],
    num_of_coeff: i32,
    p_prediction_gain: Option<(&mut FixpDbl, &mut i32)>,
) {
    let n = num_of_coeff as usize;
    let mut parcor_work_buffer = [0 as FixpDbl; LPC_MAX_ORDER];

    let auto_corr_0 = acorr[0];

    refl_coeff[..n].fill(0);

    if auto_corr_0 == fl2fxconst_dbl(0.0) {
        if let Some((gain_m, gain_e)) = p_prediction_gain {
            *gain_m = fl2fxconst_dbl(0.5);
            *gain_e = 1;
        }
        return;
    }

    parcor_work_buffer[..n].copy_from_slice(&acorr[1..n + 1]);
    // workBuffer points into parcorWorkBuffer at offset i
    for i in 0..n {
        let sign = parcor_work_buffer[i] >> (DFRACT_BITS - 1);
        let tmp = parcor_work_buffer[i] ^ sign;

        if acorr[0] < tmp {
            break;
        }

        // Check preconditions for division function: num<=denum. For 1st iteration acorr[0]
        // cannot be 0, it is checked before loop. Due to exor operation with "sign", num(=tmp)
        // is greater/equal 0.
        let tmp = schur_div(tmp, acorr[0], FRACT_BITS) ^ !sign;
        refl_coeff[i] = fx_dbl2fx_lpc(tmp);

        for j in (0..n - i).rev() {
            let accu1 = f_mult(tmp, acorr[j]);
            let accu2 = f_mult(tmp, parcor_work_buffer[i + j]);
            parcor_work_buffer[i + j] += accu1;
            acorr[j] += accu2;
        }
        if acorr[0] == 0 {
            break;
        }
    }

    if let Some((gain_m, gain_e)) = p_prediction_gain {
        if acorr[0] > 0 {
            // prediction gain = signal power / error (residual) power
            let mut scale = 0;
            *gain_m = f_div_norm_signed(auto_corr_0, acorr[0], &mut scale);
            *gain_e = scale;
        } else {
            *gain_m = 0;
            *gain_e = 0;
        }
    }
}

/// `CLpc_AutoToLpcIGF(lpcCoeff_m, lpcCoeff_e, R, R_e, m, rc)`: Levinson-Durbin algorithm to
/// compute the `FIXP_DBL` LPC coefficients from the given `m + 1` autocorrelation values.
///
/// `rc` (reflection coefficients) is not implemented, like in C++.
pub fn clpc_auto_to_lpc_igf(
    lpc_coeff_m: &mut [FixpDbl],
    lpc_coeff_e: &mut i32,
    r: &[FixpDbl],
    r_e: i32,
    m: i32,
    _rc: Option<&mut [FixpDbl]>,
) {
    let mut a_m = [0 as FixpDbl; LPC_MAX_ORDER];
    let mut a_e = [0i32; LPC_MAX_ORDER];
    let mut a_e_max = 0;
    let mut err_e = 0;
    let p = m as usize;

    // First iteration
    a_m[0] = f_div_norm_signed_noexp(r[1], r[0]).wrapping_neg();
    a_e[0] = 0;
    let mut err_m = f_add_norm(r[0], r_e, f_mult(r[1], a_m[0]), r_e + a_e[0], &mut err_e);

    for t in 1..p {
        let mut s_m = r[t + 1];
        let mut s_e = r_e;

        for i in 0..t {
            s_m = f_add_norm(s_m, s_e, f_mult(a_m[i], r[t - i]), a_e[i] + r_e, &mut s_e);
        }

        let mut g_e = 0;
        let mut g_m = f_div_norm_signed(s_m, err_m, &mut g_e);
        g_m = g_m.wrapping_neg(); // negate after division to circumvent possible overflow of -s_m
        g_e += s_e - err_e;

        // a = [ a + g * fliplr(a), g ];
        let mut i = 0;
        while i < t / 2 {
            let a0_m = a_m[i];
            let a1_m = a_m[t - 1 - i];
            let a0_e = a_e[i];
            let a1_e = a_e[t - 1 - i];
            a_m[i] = f_add_norm(a0_m, a0_e, f_mult(a1_m, g_m), a1_e + g_e, &mut a_e[i]);
            a_m[t - 1 - i] =
                f_add_norm(f_mult(a0_m, g_m), a0_e + g_e, a1_m, a1_e, &mut a_e[t - 1 - i]);
            i += 1;
        }
        if t & 1 != 0 {
            a_m[i] = f_add_norm(a_m[i], a_e[i], f_mult(a_m[i], g_m), a_e[i] + g_e, &mut a_e[i]);
        }

        a_m[t] = g_m;
        a_e[t] = g_e;

        // err = err * (1 - g * g) = err - err*g*g = err + s * g;
        err_m = f_add_norm(err_m, err_e, f_mult(s_m, g_m), s_e + g_e, &mut err_e);
    }

    for t in 0..p {
        a_e_max = f_max(a_e_max, a_e[t]);
    }

    for t in 0..p {
        let scale = a_e_max - a_e[t];
        if scale < DFRACT_BITS {
            lpc_coeff_m[t] = a_m[t] >> scale;
        } else {
            lpc_coeff_m[t] = 0;
        }
    }
    *lpc_coeff_e = a_e_max;
}

/// `CLpc_AutoToLpc(lpcCoeff_m, lpcCoeff_e, R, R_e, m, rc, is_ltpf)`: Levinson-Durbin algorithm to
/// compute the `FIXP_LPC` LPC coefficients from the given `m + 1` autocorrelation values.
///
/// `rc` (reflection coefficients) is not implemented, like in C++.
pub fn clpc_auto_to_lpc(
    lpc_coeff_m: &mut [FixpLpc],
    lpc_coeff_e: &mut i32,
    r: &[FixpDbl],
    r_e: i32,
    m: i32,
    _rc: Option<&mut [FixpDbl]>,
    is_ltpf: i32,
) {
    let mut a_m = [0 as FixpDbl; LPC_MAX_ORDER];
    let mut a_e = [0i32; LPC_MAX_ORDER];
    let mut a_e_max = 0;
    let mut err_e = 0;
    let p = m as usize;
    let mut rc_m = [0 as FixpDbl; LPC_MAX_ORDER];
    let mut rc_e = [0i32; LPC_MAX_ORDER];

    let acorr_headroom = f_norm(r[0]);
    let acorr_nrg = r[0] << acorr_headroom;

    // First iteration
    rc_m[0] = f_div_norm_signed_noexp(r[1] << acorr_headroom, acorr_nrg).wrapping_neg();
    a_m[0] = rc_m[0];
    a_e[0] = 0;
    let mut sigma2_m = (acorr_nrg >> 1) + f_mult_div2(r[1] << acorr_headroom, rc_m[0]);
    let mut sigma2_e = r_e + 1 - acorr_headroom;

    let mut err_m = f_add_norm(
        acorr_nrg,
        r_e,
        f_mult(r[1] << acorr_headroom, a_m[0]),
        r_e + a_e[0],
        &mut err_e,
    );

    for t in 1..p {
        a_e[t] = 0;
        a_m[t] = 0;
        let mut s_m = r[t + 1] << acorr_headroom;
        let mut s_e = r_e;

        for i in 0..t {
            s_m = f_add_norm(
                s_m,
                s_e,
                f_mult(a_m[i], r[t - i] << acorr_headroom),
                a_e[i] + r_e,
                &mut s_e,
            );
        }

        let mut g_e = 0;
        let mut g_m = f_div_norm_signed(s_m, err_m, &mut g_e);
        g_m = g_m.wrapping_neg();
        g_e += s_e - err_e;

        rc_m[t] = g_m;
        rc_e[t] = g_e;

        if is_ltpf != 0 {
            let rc_x_rc_m = f_pow2_div2(g_m);
            let rc_x_rc_e = g_e * 2 + 1;
            let sigma2_x_rc_x_rc_m = f_mult(sigma2_m, rc_x_rc_m);
            let sigma2_x_rc_x_rc_e = sigma2_e + rc_x_rc_e;
            let mut e = 0;
            sigma2_m = f_add_norm(
                sigma2_m,
                sigma2_e,
                sigma2_x_rc_x_rc_m.wrapping_neg(),
                sigma2_x_rc_x_rc_e,
                &mut e,
            );
            sigma2_e = e;

            let mut result_e = 0;
            let result_m = f_add_norm(
                sigma2_m,
                sigma2_e,
                fl2fxconst_dbl(-1.0e-09_f32 as f64),
                0,
                &mut result_e,
            );
            // if (Sigma2<=1.0E-09f): the autocorrelation matrix is not positive definite
            if result_m <= 0 {
                for i in 0..p {
                    rc_m[i] = 0;
                    rc_e[i] = 0;
                    a_m[i] = 0;
                    a_e[i] = 0;
                }
                break;
            }
        }

        let mut i = 0;
        while i < t / 2 {
            let a0_m = a_m[i];
            let a1_m = a_m[t - 1 - i];
            let a0_e = a_e[i];
            let a1_e = a_e[t - 1 - i];
            a_m[i] = f_add_norm(a0_m, a0_e, f_mult(a1_m, rc_m[t]), a1_e + rc_e[t], &mut a_e[i]);
            a_m[t - 1 - i] =
                f_add_norm(f_mult(a0_m, rc_m[t]), a0_e + rc_e[t], a1_m, a1_e, &mut a_e[t - 1 - i]);
            i += 1;
        }
        if t & 1 != 0 {
            a_m[i] =
                f_add_norm(a_m[i], a_e[i], f_mult(a_m[i], rc_m[t]), a_e[i] + rc_e[t], &mut a_e[i]);
        }

        a_m[t] = rc_m[t];
        a_e[t] = rc_e[t];

        // err = err * (1 - g * g) = err - err*g*g = err + s * g;
        err_m = f_add_norm(err_m, err_e, f_mult(s_m, rc_m[t]), s_e + rc_e[t], &mut err_e);
    }

    for t in 0..p {
        a_e_max = f_max(a_e_max, a_e[t]);
    }

    for t in 0..p {
        let scale = a_e_max - a_e[t];
        if scale < DFRACT_BITS {
            lpc_coeff_m[t] = fx_dbl2fx_lpc(a_m[t] >> scale);
        } else {
            lpc_coeff_m[t] = 0;
        }
    }
    *lpc_coeff_e = a_e_max;
    if a_e_max > 15 {
        for t in 0..p {
            lpc_coeff_m[t] = 0;
        }
        *lpc_coeff_e = 0;
    }
}
