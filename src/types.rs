use std::{
    ops::{Add, AddAssign},
};

use bevy::{prelude::*, render::mesh::{Indices, PrimitiveTopology}};
use bevy_inspector_egui::Inspectable;

use crate::{MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED_SQ};

#[derive(Debug)]
pub struct BroadContact {
    pub a: Entity,
    pub b: Entity,
}

#[derive(Copy, Clone, Debug)]
pub struct Contact {
    pub a: Entity,
    pub b: Entity,
    pub world_point_a: Vec3,
    pub world_point_b: Vec3,
    pub local_point_a: Vec3,
    pub local_point_b: Vec3,
    pub normal: Vec3,
    pub separation_dist: f32,
    pub time_of_impact: f32,
}


#[derive(Component, Inspectable, Debug)]
pub struct RBHelper;

impl RBHelper {
    pub fn world_to_local(trans: &Transform, com: &CenterOfMass, world_point: Vec3) -> Vec3 {
        let com_world = trans.translation + trans.rotation * com.0;
        let inv_orientation = trans.rotation.conjugate();
        inv_orientation * (world_point - com_world)
    }

    pub fn local_to_world(trans: &Transform, com: &CenterOfMass, local_point: Vec3) -> Vec3 {
        let com_world = trans.translation + trans.rotation * com.0;
        com_world + trans.rotation * local_point
    }

    pub fn inv_intertia_tensor_world(
        trans: &Transform,
        inv_inertia_tensor: &InverseInertiaTensor,
    ) -> Mat3 {
        let orientation = Mat3::from_quat(trans.rotation);
        orientation * inv_inertia_tensor.0 * orientation.transpose()
    }

    pub fn centre_of_mass_world(t: &Transform, com: &CenterOfMass) -> Vec3 {
        t.translation + t.rotation * com.0
    }

    pub fn apply_impulse(
        trans: &Transform,
        linear_velocity: &mut LinearVelocity,
        angular_velocity: &mut AngularVelocity,
        inv_mass: &InverseMass,
        com: &CenterOfMass,
        inv_inertia_tensor: &InverseInertiaTensor,
        impulse_point: Vec3,
        impulse: Vec3,
    ) {
        if inv_mass.0 == 0.0 {
            return;
        }
        // impulse_point is in world space location of the applied impulse
        // impulse is in world space direction and magnitude of the impulse
        RBHelper::apply_impulse_linear(linear_velocity, inv_mass, impulse);

        let r = impulse_point - RBHelper::centre_of_mass_world(trans, com);
        let dl = r.cross(impulse); // this is in world space
        RBHelper::apply_impulse_angular(trans, angular_velocity, inv_mass, inv_inertia_tensor, dl);
    }

    pub fn apply_impulse_linear(
        linear_velocity: &mut LinearVelocity,
        inv_mass: &InverseMass,
        impulse: Vec3,
    ) {
        if inv_mass.0 == 0.0 {
            return;
        }
        // p = mv
        // dp = m dv = J
        // => dv = J / m
        linear_velocity.0 += impulse * inv_mass.0;
    }
    pub fn apply_impulse_angular(
        trans: &Transform,
        angular_velocity: &mut AngularVelocity,
        inv_mass: &InverseMass,
        inv_inertia_tensor: &InverseInertiaTensor,
        impulse: Vec3,
    ) {
        if inv_mass.0 == 0.0 {
            return;
        }

        // L = I w = r x p
        // dL = I dw = r x J
        // => dw = I^-1 * (r x J)
        angular_velocity.0 +=
            RBHelper::inv_intertia_tensor_world(trans, inv_inertia_tensor) * impulse;

        // clamp angular_velocity
        if angular_velocity.0.length_squared() > MAX_ANGULAR_SPEED_SQ {
            angular_velocity.0 = angular_velocity.0.normalize() * MAX_ANGULAR_SPEED;
        }
    }

    pub fn update(
        transform: &mut Transform,
        angular_velocity: &mut AngularVelocity,
        linear_velocity: &LinearVelocity,
        com: &CenterOfMass,
        inertia_tensor: &InertiaTensor,
        dt: f32,
    ) {
        transform.translation += linear_velocity.0 * dt;
        // we have an angular velocity around the centre of mass, this needs to be converted to
        // relative body translation. This way we can properly update the rotation of the model
        let com_world = transform.translation + transform.rotation * com.0;
        let com_to_position = transform.translation - com_world;
        // total torque is equal to external applied torques + internal torque (precession)
        // T = T_external + omega x I * omega
        // T_external = 0 because it was applied in the collision response function
        // T = Ia = w x I * w
        // a = I^-1 (w x I * w)
        let orientation = Mat3::from_quat(transform.rotation);
        let inertia_tensor = orientation * inertia_tensor.0 * orientation.transpose();
        let alpha = inertia_tensor.inverse()
            * (angular_velocity
                .0
                .cross(inertia_tensor * angular_velocity.0));
        angular_velocity.0 += alpha * dt;

        // update orientation
        let d_angle = angular_velocity.0 * dt;
        let angle = d_angle.length();
        let rcp_angle = angle.recip();
        let dq = if rcp_angle.is_finite() {
            Quat::from_axis_angle(d_angle * rcp_angle, angle)
        } else {
            Quat::IDENTITY
        };
        transform.rotation = (dq * transform.rotation).normalize();
        // now get the new body position
        transform.translation = com_world + dq * com_to_position;
    }
}

#[derive(Component, Inspectable, Debug, PartialEq, Eq)]
pub enum RigidBodyMode {
    Static,
    Dynamic,
}

impl Default for RigidBodyMode {
    fn default() -> Self {
        RigidBodyMode::Dynamic
    }
}

#[derive(Component, Inspectable, Debug)]
pub struct Static;

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct LinearVelocity(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct AngularVelocity(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug)]
pub struct Elasticity(pub f32); // assumed [0,1]

impl Default for Elasticity {
    fn default() -> Elasticity {
        Elasticity(0.9)
    }
}

#[derive(Component, Deref, DerefMut, Inspectable, Debug)]
pub struct Friction(pub f32); // assumed [0,1]

impl Default for Friction {
    fn default() -> Friction {
        Friction(0.5)
    }
}

#[derive(Component, Deref, DerefMut, Inspectable, Debug)]
pub struct Mass(pub f32);

impl Default for Mass {
    fn default() -> Mass {
        Mass(1.0)
    }
}

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct InverseMass(pub f32);

#[derive(Component, Inspectable, Debug, Default)]
pub struct CenterOfMass(pub Vec3);

#[derive(Component, Deref, DerefMut, Inspectable, Debug, Default)]
pub struct InertiaTensor(pub Mat3);

#[derive(Component, Inspectable, Debug)]
pub struct Damping {
    pub linear: f32,
    pub angular: f32,
}

impl Default for Damping {
    fn default() -> Damping {
        Damping {
            linear: 0.01,
            angular: 0.01,
        }
    }
}

#[derive(Component, Inspectable, Debug, Default)]
pub struct InverseInertiaTensor(pub Mat3);

#[derive(Debug, Default)]
pub struct CenterOfMassWorld(pub Vec3);

#[derive(Debug, Default)]
pub struct InverseInertiaTensorWorld(pub Mat3);


#[derive(Debug, Component, Inspectable, Copy, Clone)]
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

    pub fn clear(&mut self) {
        *self = Aabb::default();
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

    pub fn get_world_aabb(&self, trans: &Transform) -> AabbWorld {
        let corners = [
            Vec3::new(self.mins.x, self.mins.y, self.mins.z),
            Vec3::new(self.mins.x, self.mins.y, self.maxs.z),
            Vec3::new(self.mins.x, self.maxs.y, self.mins.z),
            Vec3::new(self.maxs.x, self.mins.y, self.mins.z),
            Vec3::new(self.maxs.x, self.maxs.y, self.maxs.z),
            Vec3::new(self.maxs.x, self.maxs.y, self.mins.z),
            Vec3::new(self.maxs.x, self.mins.y, self.maxs.z),
            Vec3::new(self.mins.x, self.maxs.y, self.maxs.z),
        ];

        let mut aabb = Aabb::default();
        for pt in &corners {
            let pt = (trans.rotation * *pt) + trans.translation;
            aabb.expand_by_point(pt);
        }

        AabbWorld(aabb)
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
