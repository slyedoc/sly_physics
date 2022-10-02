use bevy::prelude::*;
use bevy_inspector_egui::Inspectable;
use std::ops::{Add, AddAssign};

#[derive(Component, Inspectable, Debug, Reflect, Copy, Clone)]
#[reflect(Component)]
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


impl Add<Aabb> for Aabb {
    type Output = Self;
    fn add(self, b: Aabb) -> Self::Output {
        Aabb {
            mins: Vec3::min(self.mins, b.mins),
            maxs: Vec3::max(self.maxs, b.maxs),
        }
    }
}

impl Add<&Aabb> for &Aabb {
    type Output = Aabb;
    fn add(self, b: &Aabb) -> Self::Output {
        Aabb {
            mins: Vec3::min(self.mins, b.mins),
            maxs: Vec3::max(self.maxs, b.maxs),
        }
    }
}

impl AddAssign<Aabb> for Aabb {
    fn add_assign(&mut self, rhs: Aabb) {
        self.mins = self.mins.min(rhs.mins);
        self.maxs = self.mins.max(rhs.maxs);
    }
}

#[test]
fn test_aabb_add() {
    let a = Aabb {
        mins: Vec3::new(0.0, 0.0, 0.0),
        maxs: Vec3::new(1.0, 1.0, 1.0),
    };
    let b = Aabb {
        mins: Vec3::new(1.0, 1.0, 1.0),
        maxs: Vec3::new(2.0, 2.0, 2.0),
    };
    let c = a + b;
    assert_eq!(c.mins, Vec3::new(0.0, 0.0, 0.0));    
    assert_eq!(c.maxs, Vec3::new(2.0, 2.0, 2.0));
}

impl Aabb {
    pub fn new(mins: Vec3, maxs: Vec3) -> Aabb {
        Aabb { mins, maxs }
    }

    #[inline]
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

    // TODO: performance test form_points vs grow vs add_assign vs expand_by_point, all doing same thing
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
