mod debug_render;
pub use debug_render::*;

use std::ops::{Add, AddAssign};

use bevy::{    
    prelude::*,
    render::{
        mesh::Indices,
        render_resource::{
             PrimitiveTopology,
        },        
    },
};

use bevy_inspector_egui::Inspectable;

#[derive(Debug, Inspectable, Copy, Clone)]
pub struct Aabb {
    pub mins: Vec3,
    pub maxs: Vec3,
}

impl Default for Aabb {
    fn default() -> Self {
        Self {
            mins: Vec3::splat(std::f32::MAX),
            maxs: Vec3::splat(std::f32::MIN),
        }
    }
}

impl From<&Aabb> for Mesh {
    fn from(aabb: &Aabb) -> Self {
        let verts = vec![
            [aabb.maxs.x, aabb.maxs.y, aabb.mins.z],
            [aabb.mins.x, aabb.maxs.y, aabb.mins.z],
            [aabb.mins.x, aabb.maxs.y, aabb.maxs.z],
            [aabb.maxs.x, aabb.maxs.y, aabb.maxs.z],
            [aabb.maxs.x, aabb.mins.y, aabb.mins.z],
            [aabb.mins.x, aabb.mins.y, aabb.mins.z],
            [aabb.mins.x, aabb.mins.y, aabb.maxs.z],
            [aabb.maxs.x, aabb.mins.y, aabb.maxs.z],
        ];

        let uvs = vec![[0.0, 0.0]; 8];
        let normals = vec![[0.0, 0.0, 1.0]; 8];

        let indices = Indices::U32(vec![
            0, 1, 1, 2, 2, 3, 3, 0, // Top ring
            4, 5, 5, 6, 6, 7, 7, 4, // Bottom ring
            0, 4, 1, 5, 2, 6, 3, 7, // Verticals
        ]);

        let mut mesh = Mesh::new(PrimitiveTopology::LineList);
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, verts);
        mesh.insert_attribute(Mesh::ATTRIBUTE_UV_0, uvs);
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
        mesh.set_indices(Some(indices));
        mesh
    }
}

impl Add<Vec3> for Aabb {
    type Output = Self;
    fn add(self, pt: Vec3) -> Self::Output {
        Aabb {
            mins: Vec3::select(pt.cmplt(self.mins), pt, self.mins),
            maxs: Vec3::select(pt.cmpgt(self.maxs), pt, self.maxs),
        }
    }
}

impl AddAssign<Vec3> for Aabb {
    fn add_assign(&mut self, pt: Vec3) {
        self.mins = Vec3::select(pt.cmplt(self.mins), pt, self.mins);
        self.maxs = Vec3::select(pt.cmpgt(self.maxs), pt, self.maxs);
    }
}

impl Aabb {
    pub fn new(mins: Vec3, maxs: Vec3) -> Aabb {
        Aabb { mins, maxs }
    }

    pub fn intersection(&self, b: &Aabb) -> bool {
        // Exit with no intersection if separated along an axis
        if self.maxs[0] < b.mins[0] || self.mins[0] > b.maxs[0] {
            return false;
        }
        if self.maxs[1] < b.mins[1] || self.mins[1] > b.maxs[1] {
            return false;
        }
        if self.maxs[2] < b.mins[2] || self.mins[2] > b.maxs[2] {
            return false;
        }
        // Overlapping on all axes means AABBs are intersecting
        true
    }

    // TODO: preformance test form_points vs grow vs add_assign vs expand_by_point, all doing same thing
    pub fn from_points(pts: &[Vec3]) -> Self {
        pts.iter().fold(Aabb::default(), |acc, pt| acc + *pt)
    }

    pub fn grow(&mut self, p: Vec3) {
        self.mins = self.mins.min(p);
        self.maxs = self.maxs.max(p);
    }

    pub fn grow_aabb(&mut self, b: &Aabb) {
        self.grow(b.mins);
        self.grow(b.maxs);
    }

    pub fn area(&self) -> f32 {
        let e = self.maxs - self.mins; // box extent
        e.x * e.y + e.y * e.z + e.z * e.x
    }

    pub fn expand_by_point(&mut self, rhs: Vec3) {
        self.mins = Vec3::select(rhs.cmplt(self.mins), rhs, self.mins);
        self.maxs = Vec3::select(rhs.cmpgt(self.maxs), rhs, self.maxs);
    }

    pub fn width(&self) -> Vec3 {
        self.maxs - self.mins
    }
}

#[derive(Debug, Deref, DerefMut, Default, Component, Inspectable)]
pub struct AabbWorld(pub Aabb);
