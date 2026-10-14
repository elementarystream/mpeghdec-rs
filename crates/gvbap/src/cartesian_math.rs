// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libgVBAPRenderer/include/cartesianMath.h and libgVBAPRenderer/src/cartesianMath.cpp.

//! Math functions for Vector Base Amplitude Panning (VBAP): spherical and Cartesian points.
//!
//! The C++ operator overloads of `PointCartesian` are the methods [`PointCartesian::sub`],
//! [`PointCartesian::add`], [`PointCartesian::scale`] (`FIXP_DBL * PointCartesian`),
//! [`PointCartesian::shr_assign`], [`PointCartesian::shl_assign`] and [`PointCartesian::neg`].

use mpeghdec_fdk::digest::StateDigest;

use mpeghdec_fdk::common_fix::*;
use mpeghdec_fdk::fixpoint_math::{inv_fixp_e, sqrt_fixp};
use mpeghdec_fdk::scale::scale_value_saturate;
use mpeghdec_fdk::trig_fcts::inline_fixp_cos_sin;

/// `PointSpherical`: point in spherical coordinates.
///
/// Azimuth and elevation are mapped from -180 ... 180 degrees (-pi ... pi) to -1 ... 1. The
/// radius has an exponent of 4 (0.5 ... 16 m).
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PointSpherical {
    /// `azi`
    pub azi: FixpDbl,
    /// `ele`
    pub ele: FixpDbl,
    /// `rad`
    pub rad: FixpDbl,
}

/// `PointCartesian`: point (or vector) in Cartesian coordinates.
///
/// The exponent for object data is normally 4 (because of the required radius of maximal 16 m),
/// the exponent for speaker positions is 0.
#[derive(StateDigest, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PointCartesian {
    /// `x`
    pub x: FixpDbl,
    /// `y`
    pub y: FixpDbl,
    /// `z`
    pub z: FixpDbl,
}

impl PointCartesian {
    /// `operator-(const PointCartesian& lhs, const PointCartesian& rhs)`
    #[inline]
    pub fn sub(&self, rhs: &PointCartesian) -> PointCartesian {
        PointCartesian {
            x: self.x.wrapping_sub(rhs.x),
            y: self.y.wrapping_sub(rhs.y),
            z: self.z.wrapping_sub(rhs.z),
        }
    }

    /// `operator+(const PointCartesian& lhs, const PointCartesian& rhs)`
    #[inline]
    pub fn add(&self, rhs: &PointCartesian) -> PointCartesian {
        PointCartesian {
            x: self.x.wrapping_add(rhs.x),
            y: self.y.wrapping_add(rhs.y),
            z: self.z.wrapping_add(rhs.z),
        }
    }

    /// `operator*(const FIXP_DBL& lhs, const PointCartesian& rhs)`: `lhs * self`.
    #[inline]
    pub fn scale(&self, lhs: FixpDbl) -> PointCartesian {
        PointCartesian { x: f_mult(lhs, self.x), y: f_mult(lhs, self.y), z: f_mult(lhs, self.z) }
    }

    /// `operator>>=(PointCartesian& lhs, const FIXP_DBL& rhs)`
    #[inline]
    pub fn shr_assign(&mut self, rhs: i32) {
        self.x = self.x.wrapping_shr(rhs as u32);
        self.y = self.y.wrapping_shr(rhs as u32);
        self.z = self.z.wrapping_shr(rhs as u32);
    }

    /// `operator<<=(PointCartesian& lhs, const FIXP_DBL& rhs)`
    #[inline]
    pub fn shl_assign(&mut self, rhs: i32) {
        self.x = self.x.wrapping_shl(rhs as u32);
        self.y = self.y.wrapping_shl(rhs as u32);
        self.z = self.z.wrapping_shl(rhs as u32);
    }

    /// `operator-(PointCartesian& lhs)`
    #[inline]
    pub fn neg(&self) -> PointCartesian {
        PointCartesian {
            x: self.x.wrapping_neg(),
            y: self.y.wrapping_neg(),
            z: self.z.wrapping_neg(),
        }
    }
}

/// `sphericalToCartesian(PointSpherical sph)`: converts a spherical point (azimuth and elevation
/// mapped from -pi ... pi to -1 ... 1) to a Cartesian point where x, y and z are between -1 ... 1.
pub fn spherical_to_cartesian(sph: PointSpherical) -> PointCartesian {
    /* Scale sph from -pi ... pi to a power of 2 */
    /* New exponent will be 2 (-4 ... 4) */
    let azi = f_mult(sph.azi, 1686629713i32); /* FL2FXCONST_DBL(M_PI/4.0) */
    let ele = f_mult(sph.ele, 1686629713i32); /* FL2FXCONST_DBL(M_PI/4.0) */

    let out = inline_fixp_cos_sin(azi, ele, 2); /* out = cos(azi), sin(azi), cos(ele), sin(ele) */

    let mut cart =
        PointCartesian { x: f_mult(out[0], out[2]), y: f_mult(out[1], out[2]), z: out[3] };

    if sph.rad < (1i32 << (DFRACT_BITS - 1 - 4)) {
        let rad = sph.rad << 4; /* apply radius exponent 4 */

        cart.x = f_mult(cart.x, rad);
        cart.y = f_mult(cart.y, rad);
        cart.z = f_mult(cart.z, rad);
    }

    cart
}

/// `crossProduct(PointCartesian* a, PointCartesian* b)`: `a x b`. Both exponents have to be the
/// same; the result has an exponent of 1.
pub fn cross_product(a: &PointCartesian, b: &PointCartesian) -> PointCartesian {
    /* a and b don't need to be scaled if length of input vectors is < 1, because the length
     * of result vector is equal to the area of the parallelogram spanned by the vector a and b.
     * It must be less or equal to 1 => every entry of the result vector must be less or equal to
     * +-1.
     * To keep this function general we scaled a and b with an exponent of 1 (divide by 2).
     * Because in worst case we have to handle a result vector of length 3,
     * where every entry will be less than +-2 */

    /* Multiplicate and prescale for addition */
    PointCartesian {
        x: f_mult_div2(a.y, b.z).wrapping_sub(f_mult_div2(a.z, b.y)),
        y: f_mult_div2(a.z, b.x).wrapping_sub(f_mult_div2(a.x, b.z)),
        z: f_mult_div2(a.x, b.y).wrapping_sub(f_mult_div2(a.y, b.x)),
    }
}

/// `dotProduct(PointCartesian* a, PointCartesian* b)`: `a . b` with an exponent of 2.
pub fn dot_product(a: &PointCartesian, b: &PointCartesian) -> FixpDbl {
    /* Prescale with exponent of 2 (divide by 4), because in worst case the value of result could
     * be 3 */
    let px = f_mult_div2(a.x, b.x);
    let py = f_mult_div2(a.y, b.y);
    let pz = f_mult_div2(a.z, b.z);
    (px >> 1) + (py >> 1) + (pz >> 1)
}

/// `normalize(PointCartesian* a)`: normalizes the vector `a` (the result has an exponent of 0).
/// Returns 0 on success, -1 for a zero vector (which is left unchanged).
pub fn normalize(a: &mut PointCartesian) -> i32 {
    let mut e_sf = 0;
    let mut a_x_abs = f_abs(a.x);
    let mut a_y_abs = f_abs(a.y);
    let mut a_z_abs = f_abs(a.z);

    /* if the entries are very small we could not calculate a normalized vector.
     * Therefore we will scale it first */
    let tmp = a_x_abs | a_y_abs | a_z_abs;
    let mut e_min = count_leading_bits(tmp);

    e_min -= 1;
    if e_min >= 0 {
        a_x_abs = a_x_abs.wrapping_shl(e_min as u32);
        a_y_abs = a_y_abs.wrapping_shl(e_min as u32);
        a_z_abs = a_z_abs.wrapping_shl(e_min as u32);
    } else {
        a_x_abs >>= -e_min;
        a_y_abs >>= -e_min;
        a_z_abs >>= -e_min;
    }
    let mut scalefactor =
        f_pow2(a_x_abs).wrapping_add(f_pow2(a_y_abs)).wrapping_add(f_pow2(a_z_abs));
    if scalefactor == 0 {
        /* it must be a zero vector. There is no way to normalize it => do nothing */
        return -1;
    }

    scalefactor = sqrt_fixp(scalefactor);

    scalefactor = f_max(scalefactor, a_x_abs);
    scalefactor = f_max(scalefactor, a_y_abs);
    scalefactor = f_max(scalefactor, a_z_abs);
    scalefactor = scalefactor.wrapping_add(1);

    scalefactor = inv_fixp_e(scalefactor, &mut e_sf);

    a.x = scale_value_saturate(f_mult(a.x, scalefactor), e_sf + e_min);
    a.y = scale_value_saturate(f_mult(a.y, scalefactor), e_sf + e_min);
    a.z = scale_value_saturate(f_mult(a.z, scalefactor), e_sf + e_min);

    0
}
