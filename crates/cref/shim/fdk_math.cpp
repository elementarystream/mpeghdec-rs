// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.

// C shims exposing libFDK fixed-point primitives (common_fix.h, fixpoint_math.h, scale.h) with C
// linkage for differential testing.

#include "common_fix.h"
#include "fixpoint_math.h"
#include "scale.h"

extern "C" {

// fixmul.h
INT cref_fMult_DD(INT a, INT b) { return fMult((FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMult_DS(INT a, SHORT b) { return fMult((FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMult_SD(SHORT a, INT b) { return fMult((FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fMult_SS(SHORT a, SHORT b) { return fMult((FIXP_SGL)a, (FIXP_SGL)b); }
INT cref_fMultDiv2_DD(INT a, INT b) { return fMultDiv2((FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultDiv2_DS(INT a, SHORT b) { return fMultDiv2((FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultDiv2_SD(SHORT a, INT b) { return fMultDiv2((FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fMultDiv2_SS(SHORT a, SHORT b) { return fMultDiv2((FIXP_SGL)a, (FIXP_SGL)b); }
INT64 cref_fMult_QQ(INT64 a, INT64 b) { return fMult(a, b); }
INT64 cref_fMult_QD(INT64 a, INT b) { return fMult(a, (LONG)b); }
INT64 cref_fMultDiv2_QQ(INT64 a, INT64 b) { return fMultDiv2(a, b); }
INT64 cref_fMultDiv2_QD(INT64 a, INT b) { return fMultDiv2(a, (LONG)b); }
INT cref_fMultBitExact_DD(INT a, INT b) { return fMultBitExact((FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultBitExact_DS(INT a, SHORT b) { return fMultBitExact((FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultBitExact_SD(SHORT a, INT b) { return fMultBitExact((FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fMultDiv2BitExact_DD(INT a, INT b) { return fMultDiv2BitExact((FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultDiv2BitExact_DS(INT a, SHORT b) { return fMultDiv2BitExact((FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultDiv2BitExact_SD(SHORT a, INT b) { return fMultDiv2BitExact((FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fPow2_D(INT a) { return fPow2((FIXP_DBL)a); }
INT cref_fPow2_S(SHORT a) { return fPow2((FIXP_SGL)a); }
INT cref_fPow2Div2_D(INT a) { return fPow2Div2((FIXP_DBL)a); }
INT cref_fPow2Div2_S(SHORT a) { return fPow2Div2((FIXP_SGL)a); }

// fixmadd.h
INT cref_fMultAdd_DD(INT x, INT a, INT b) { return fMultAdd((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultAdd_DS(INT x, INT a, SHORT b) { return fMultAdd((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultAdd_SD(INT x, SHORT a, INT b) { return fMultAdd((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fMultAdd_SS(INT x, SHORT a, SHORT b) { return fMultAdd((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_SGL)b); }
INT cref_fMultAddDiv2_DD(INT x, INT a, INT b) { return fMultAddDiv2((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultAddDiv2_DS(INT x, INT a, SHORT b) { return fMultAddDiv2((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultAddDiv2_SD(INT x, SHORT a, INT b) { return fMultAddDiv2((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fMultAddDiv2_SS(INT x, SHORT a, SHORT b) { return fMultAddDiv2((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_SGL)b); }
INT cref_fMultSub_DD(INT x, INT a, INT b) { return fMultSub((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultSub_DS(INT x, INT a, SHORT b) { return fMultSub((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultSub_SD(INT x, SHORT a, INT b) { return fMultSub((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fMultSub_SS(INT x, SHORT a, SHORT b) { return fMultSub((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_SGL)b); }
INT cref_fMultSubDiv2_DD(INT x, INT a, INT b) { return fMultSubDiv2((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultSubDiv2_DS(INT x, INT a, SHORT b) { return fMultSubDiv2((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultSubDiv2_SD(INT x, SHORT a, INT b) { return fMultSubDiv2((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fMultSubDiv2_SS(INT x, SHORT a, SHORT b) { return fMultSubDiv2((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_SGL)b); }
INT cref_fMultAddDiv2BitExact_DD(INT x, INT a, INT b) { return fMultAddDiv2BitExact((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultAddDiv2BitExact_DS(INT x, INT a, SHORT b) { return fMultAddDiv2BitExact((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultAddDiv2BitExact_SD(INT x, SHORT a, INT b) { return fMultAddDiv2BitExact((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fMultSubDiv2BitExact_DD(INT x, INT a, INT b) { return fMultSubDiv2BitExact((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fMultSubDiv2BitExact_DS(INT x, INT a, SHORT b) { return fMultSubDiv2BitExact((FIXP_DBL)x, (FIXP_DBL)a, (FIXP_SGL)b); }
INT cref_fMultSubDiv2BitExact_SD(INT x, SHORT a, INT b) { return fMultSubDiv2BitExact((FIXP_DBL)x, (FIXP_SGL)a, (FIXP_DBL)b); }
INT cref_fPow2AddDiv2_D(INT x, INT a) { return fPow2AddDiv2((FIXP_DBL)x, (FIXP_DBL)a); }
INT cref_fPow2AddDiv2_S(INT x, SHORT a) { return fPow2AddDiv2((FIXP_DBL)x, (FIXP_SGL)a); }
INT cref_fPow2Add_D(INT x, INT a) { return fPow2Add((FIXP_DBL)x, (FIXP_DBL)a); }
INT cref_fPow2Add_S(INT x, SHORT a) { return fPow2Add((FIXP_DBL)x, (FIXP_SGL)a); }

// cplx_mul.h (results are written to out[0], out[1])
void cref_cplxMultDiv2_DD(INT* out, INT ar, INT ai, INT br, INT bi) { cplxMultDiv2(&out[0], &out[1], ar, ai, (FIXP_DBL)br, (FIXP_DBL)bi); }
void cref_cplxMultDiv2_DS(INT* out, INT ar, INT ai, SHORT br, SHORT bi) { cplxMultDiv2(&out[0], &out[1], ar, ai, (FIXP_SGL)br, (FIXP_SGL)bi); }
void cref_cplxMult_DD(INT* out, INT ar, INT ai, INT br, INT bi) { cplxMult(&out[0], &out[1], ar, ai, (FIXP_DBL)br, (FIXP_DBL)bi); }
void cref_cplxMult_DS(INT* out, INT ar, INT ai, SHORT br, SHORT bi) { cplxMult(&out[0], &out[1], ar, ai, (FIXP_SGL)br, (FIXP_SGL)bi); }
void cref_cplxMultDiv2_SPK(INT* out, INT ar, INT ai, SHORT br, SHORT bi) { FIXP_SPK w; w.v.re = br; w.v.im = bi; cplxMultDiv2(&out[0], &out[1], ar, ai, w); }
void cref_cplxMultAddDiv2_DD(INT* out, INT ar, INT ai, INT br, INT bi) { cplxMultAddDiv2(&out[0], &out[1], ar, ai, (FIXP_DBL)br, (FIXP_DBL)bi); }
void cref_cplxMultAddDiv2_DS(INT* out, INT ar, INT ai, SHORT br, SHORT bi) { cplxMultAddDiv2(&out[0], &out[1], ar, ai, (FIXP_SGL)br, (FIXP_SGL)bi); }
void cref_cplxMultSubDiv2_DD(INT* out, INT ar, INT ai, INT br, INT bi) { cplxMultSubDiv2(&out[0], &out[1], ar, ai, (FIXP_DBL)br, (FIXP_DBL)bi); }
void cref_cplxMultSubDiv2_DS(INT* out, INT ar, INT ai, SHORT br, SHORT bi) { cplxMultSubDiv2(&out[0], &out[1], ar, ai, (FIXP_SGL)br, (FIXP_SGL)bi); }
void cref_cplxMultDiv2_SS(INT* out, SHORT ar, SHORT ai, SHORT br, SHORT bi) { cplxMultDiv2(&out[0], &out[1], (FIXP_SGL)ar, (FIXP_SGL)ai, (FIXP_SGL)br, (FIXP_SGL)bi); }
void cref_cplxMultDiv2_SS_sgl(SHORT* out, SHORT ar, SHORT ai, SHORT br, SHORT bi) { cplxMultDiv2(&out[0], &out[1], (FIXP_SGL)ar, (FIXP_SGL)ai, (FIXP_SGL)br, (FIXP_SGL)bi); }

// abs.h, clz.h
INT cref_fAbs_D(INT x) { return fAbs((FIXP_DBL)x); }
SHORT cref_fAbs_S(SHORT x) { return fAbs((FIXP_SGL)x); }
INT64 cref_fAbs_Q(INT64 x) { return fAbs((FIXP_QDL)x); }
INT cref_fNorm_D(INT x) { return fNorm((FIXP_DBL)x); }
INT cref_fNorm_S(SHORT x) { return fNorm((FIXP_SGL)x); }
INT cref_fNormz_D(INT x) { return fNormz((FIXP_DBL)x); }
INT cref_fNormz_S(SHORT x) { return fNormz((FIXP_SGL)x); }
INT cref_fNormz_Q(INT64 x) { return fNormz((INT64)x); }

// scale.h
INT cref_scaleValue_D(INT v, INT s) { return scaleValue((FIXP_DBL)v, s); }
SHORT cref_scaleValue_S(SHORT v, INT s) { return scaleValue((FIXP_SGL)v, s); }
INT cref_scaleValueSaturate(INT v, INT s) { return scaleValueSaturate((FIXP_DBL)v, s); }
INT cref_scaleValueInPlace(INT v, INT s) { FIXP_DBL x = v; scaleValueInPlace(&x, s); return x; }
INT cref_SATURATE_RIGHT_SHIFT(INT src, INT scale, INT dBits) { return SATURATE_RIGHT_SHIFT(src, scale, dBits); }
INT cref_SATURATE_LEFT_SHIFT(INT src, INT scale, INT dBits) { return SATURATE_LEFT_SHIFT(src, scale, dBits); }
INT cref_SATURATE_SHIFT(INT src, INT scale, INT dBits) { return SATURATE_SHIFT(src, scale, dBits); }
INT cref_SATURATE_RIGHT_SHIFT_RND(INT src, INT scale, INT dBits) { return SATURATE_RIGHT_SHIFT_RND(src, scale, dBits); }
INT cref_SATURATE_LEFT_SHIFT_ALT(INT src, INT scale, INT dBits) { return SATURATE_LEFT_SHIFT_ALT(src, scale, dBits); }
INT cref_SATURATE_RIGHT_SHIFT_ALT(INT src, INT scale, INT dBits) { return SATURATE_RIGHT_SHIFT_ALT(src, scale, dBits); }
void cref_scaleValuesWithFactor(INT* v, INT factor, INT len, INT s) { scaleValuesWithFactor(v, factor, len, s); }
void cref_scaleValues_S(SHORT* v, INT len, INT s) { scaleValues(v, len, s); }
void cref_scaleValues_D(INT* v, INT len, INT s) { scaleValues(v, len, s); }
void cref_scaleValues_DD(INT* d, const INT* src, INT len, INT s) { scaleValues(d, src, len, s); }
void cref_scaleValuesSaturate_D(INT* v, INT len, INT s) { scaleValuesSaturate(v, len, s); }
void cref_scaleValuesSaturate_DD(INT* d, const INT* src, INT len, INT s) { scaleValuesSaturate(d, src, len, s); }
void cref_scaleValuesSaturate_SD(SHORT* d, const INT* src, INT len, INT s) { scaleValuesSaturate(d, src, len, s); }
void cref_scaleValuesSaturate_S(SHORT* v, INT len, INT s) { scaleValuesSaturate(v, len, s); }
void cref_scaleValuesSaturate_SS(SHORT* d, const SHORT* src, INT len, INT s) { scaleValuesSaturate(d, src, len, s); }
INT cref_getScalefactorShort(const SHORT* v, INT len) { return getScalefactorShort(v, len); }
INT cref_getScalefactorPCM(const INT* v, INT len, INT stride) { return getScalefactorPCM(v, len, stride); }
INT cref_getScalefactor_D(const INT* v, INT len) { return getScalefactor((const FIXP_DBL*)v, len); }
INT cref_getScalefactor_S(const SHORT* v, INT len) { return getScalefactor((const FIXP_SGL*)v, len); }

// fixpoint_math.h
INT cref_fIsLessThan_D(INT am, INT ae, INT bm, INT be) { return fIsLessThan((FIXP_DBL)am, ae, (FIXP_DBL)bm, be); }
INT cref_fIsLessThan_S(SHORT am, INT ae, SHORT bm, INT be) { return fIsLessThan((FIXP_SGL)am, ae, (FIXP_SGL)bm, be); }
INT cref_CalcInvLdData(INT x) { return CalcInvLdData(x); }
INT cref_CalcLdInt(INT i) { return CalcLdInt(i); }
INT cref_sqrtFixp_lookup(INT x) { return sqrtFixp_lookup(x); }
INT cref_sqrtFixp_lookup_e(INT x, INT* e) { return sqrtFixp_lookup(x, e); }
INT cref_invSqrtNorm2(INT op, INT* shift) { return invSqrtNorm2(op, shift); }
INT cref_sqrtFixp(INT op) { return sqrtFixp(op); }
INT cref_invFixp(INT op) { return invFixp(op); }
INT cref_invFixp_e(INT op, INT* e) { return invFixp(op, e); }
INT cref_schur_div(INT num, INT den, INT count) { return schur_div(num, den, count); }
INT cref_fMultNorm_e(INT a, INT b, INT* e) { return fMultNorm(a, b, e); }
INT cref_fMultNorm(INT a, INT b) { return fMultNorm(a, b); }
INT cref_fMultNorm_to(INT a, INT ae, INT b, INT be, INT re) { return fMultNorm(a, ae, b, be, re); }
INT cref_fMultI(INT a, INT b) { return fMultI(a, b); }
INT cref_fMultIfloor(INT a, INT b) { return fMultIfloor(a, b); }
INT cref_fMultIceil(INT a, INT b) { return fMultIceil(a, b); }
INT cref_fDivNorm_e(INT a, INT b, INT* e) { return fDivNorm(a, b, e); }
INT cref_fDivNorm(INT a, INT b) { return fDivNorm(a, b); }
INT cref_fDivNormSigned_e(INT a, INT b, INT* e) { return fDivNormSigned(a, b, e); }
INT cref_fDivNormSigned(INT a, INT b) { return fDivNormSigned(a, b); }
INT cref_fDivNormHighPrec(INT a, INT b, INT* e) { return fDivNormHighPrec(a, b, e); }
INT cref_fAdjust(INT a, INT* e) { return fAdjust(a, e); }
INT cref_fAddNorm_e(INT am, INT ae, INT bm, INT be, INT* re) { return fAddNorm(am, ae, bm, be, re); }
INT cref_fAddNorm_to(INT am, INT ae, INT bm, INT be, INT re) { return fAddNorm(am, ae, bm, be, re); }
INT cref_f2Pow_e(INT m, INT e, INT* re) { return f2Pow(m, e, re); }
INT cref_f2Pow(INT m, INT e) { return f2Pow(m, e); }
INT cref_fPow(INT bm, INT be, INT em, INT ee, INT* re) { return fPow(bm, be, em, ee, re); }
INT cref_fLdPow_e(INT bm, INT be, INT em, INT ee, INT* re) { return fLdPow(bm, be, em, ee, re); }
INT cref_fLdPow(INT bm, INT be, INT em, INT ee) { return fLdPow(bm, be, em, ee); }
INT cref_fPowInt(INT bm, INT be, INT n, INT* re) { return fPowInt(bm, be, n, re); }
INT cref_fLog2_e(INT m, INT e, INT* re) { return fLog2(m, e, re); }
INT cref_fLog2(INT m, INT e) { return fLog2(m, e); }
SHORT cref_fAddSaturate_S(SHORT a, SHORT b) { return fAddSaturate((FIXP_SGL)a, (FIXP_SGL)b); }
INT cref_fAddSaturate_D(INT a, INT b) { return fAddSaturate((FIXP_DBL)a, (FIXP_DBL)b); }
INT cref_fNegateSaturate_D(INT a) { return fNegateSaturate((FIXP_DBL)a); }
SHORT cref_fNegateSaturate_S(SHORT a) { return fNegateSaturate((FIXP_SGL)a); }
INT cref_fixp_floorToInt(INT a, INT sf) { return fixp_floorToInt(a, sf); }
INT cref_fixp_floor(INT a, INT sf) { return fixp_floor(a, sf); }
INT cref_fixp_ceilToInt(INT a, INT sf) { return fixp_ceilToInt(a, sf); }
INT cref_fixp_ceil(INT a, INT sf) { return fixp_ceil(a, sf); }
INT cref_fixp_truncateToInt(INT a, INT sf) { return fixp_truncateToInt(a, sf); }
INT cref_fixp_truncate(INT a, INT sf) { return fixp_truncate(a, sf); }
INT cref_fixp_roundToInt(INT a, INT sf) { return fixp_roundToInt(a, sf); }
INT cref_fixp_round(INT a, INT sf) { return fixp_round(a, sf); }
INT cref_GetInvInt(INT i) { return GetInvInt(i); }

}  // extern "C"
