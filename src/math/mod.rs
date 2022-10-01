mod glam_ext;
mod lcp;
mod matrix;
mod vector;

use bevy::prelude::*;
pub use glam_ext::*;
pub use lcp::lcp_gauss_seidel;
pub use matrix::{MatMN, MatN};
pub use vector::VecN;

pub(crate) fn dot<const N: usize>(a: &[f32; N], b: &[f32; N]) -> f32 {
    a.iter()
        .zip(b.iter())
        .fold(0.0, |dot, (&lhs, &rhs)| dot + lhs * rhs)
}

pub fn quat_left(q: Quat) -> Mat4 {
    Mat4::from_cols(
        Vec4::new(q.w, -q.x, -q.y, -q.z),
        Vec4::new(q.x, q.w, -q.z, q.y),
        Vec4::new(q.y, q.z, q.w, -q.x),
        Vec4::new(q.z, -q.y, q.x, q.w),
    )
}

pub fn quat_right(q: Quat) -> Mat4 {
    Mat4::from_cols(
        Vec4::new(q.w, -q.x, -q.y, -q.z),
        Vec4::new(q.x, q.w, q.z, -q.y),
        Vec4::new(q.y, -q.z, q.w, q.x),
        Vec4::new(q.z, q.y, -q.x, q.w),
    )
}
