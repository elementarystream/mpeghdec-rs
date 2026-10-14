// Rust port of the Fraunhofer FDK MPEG-H Software (mpeghdec r4.0.1).
// Third-Party Modified Version of the Fraunhofer FDK MPEG-H Software.
// See LICENSE.txt for the complete license text.
//
// Ported from libFDK/include/FDK_matrixCalloc.h and libFDK/src/FDK_matrixCalloc.cpp.

//! Matrix memory allocation.
//!
//! `fdkCallocMatrix1D()` becomes a zero-initialized `Vec` and `fdkCallocMatrix2D()` a
//! [`Matrix2D`]. Like in C++, the rows of a 2D matrix are stored contiguously (the C++ code
//! relies on `p[0]` pointing to the whole `dim1 * dim2` block, e.g. for `FDKmemclear`), see
//! [`Matrix2D::as_flat`]. Freeing is done by dropping the value.

use crate::digest::StateDigest;

use core::ops::{Index, IndexMut};

/// `fdkCallocMatrix1D(dim1, size)` / `fdkCallocMatrix1D_aligned(dim1, size)`: zero-initialized
/// vector of `dim1` elements.
pub fn fdk_calloc_matrix_1d<T: Clone + Default>(dim1: u32) -> Vec<T> {
    vec![T::default(); dim1 as usize]
}

/// Zero-initialized 2D matrix with contiguous row-major storage (`fdkCallocMatrix2D`).
#[derive(StateDigest, Clone, Debug, Default, PartialEq, Eq)]
pub struct Matrix2D<T> {
    data: Vec<T>,
    dim1: usize,
    dim2: usize,
}

impl<T: Clone + Default> Matrix2D<T> {
    /// Allocate a zero-initialized `dim1` x `dim2` matrix.
    pub fn new(dim1: usize, dim2: usize) -> Self {
        Matrix2D { data: vec![T::default(); dim1 * dim2], dim1, dim2 }
    }
}

impl<T> Matrix2D<T> {
    /// Number of rows.
    pub fn dim1(&self) -> usize {
        self.dim1
    }

    /// Number of columns.
    pub fn dim2(&self) -> usize {
        self.dim2
    }

    /// All elements in row-major order (`p[0][0 .. dim1 * dim2]` in C++).
    pub fn as_flat(&self) -> &[T] {
        &self.data
    }

    /// All elements in row-major order (`p[0][0 .. dim1 * dim2]` in C++).
    pub fn as_flat_mut(&mut self) -> &mut [T] {
        &mut self.data
    }

    /// Iterator over the rows.
    pub fn rows(&self) -> core::slice::ChunksExact<'_, T> {
        self.data.chunks_exact(self.dim2.max(1))
    }

    /// Mutable iterator over the rows (allows accessing several rows at once).
    pub fn rows_mut(&mut self) -> core::slice::ChunksExactMut<'_, T> {
        self.data.chunks_exact_mut(self.dim2.max(1))
    }
}

impl<T> Index<usize> for Matrix2D<T> {
    type Output = [T];

    fn index(&self, row: usize) -> &[T] {
        assert!(row < self.dim1, "Matrix2D row index out of range");
        &self.data[row * self.dim2..(row + 1) * self.dim2]
    }
}

impl<T> IndexMut<usize> for Matrix2D<T> {
    fn index_mut(&mut self, row: usize) -> &mut [T] {
        assert!(row < self.dim1, "Matrix2D row index out of range");
        &mut self.data[row * self.dim2..(row + 1) * self.dim2]
    }
}

/// `fdkCallocMatrix2D(dim1, dim2, size)`: zero-initialized `dim1` x `dim2` matrix, `None` if a
/// dimension is zero (C++ returns `NULL`).
pub fn fdk_calloc_matrix_2d<T: Clone + Default>(dim1: u32, dim2: u32) -> Option<Matrix2D<T>> {
    if dim1 == 0 || dim2 == 0 {
        return None;
    }
    Some(Matrix2D::new(dim1 as usize, dim2 as usize))
}
