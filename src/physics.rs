#![allow(unused)]

use crate::consts::*;
use crate::util::*;
use macroquad::prelude::*;
use rapier2d::na::Isometry2;
use rapier2d::na::{Point2, Vector2};
use rapier2d::prelude::*;
use std::collections::HashSet;
use std::f32::consts::PI;

pub struct PhysicsProperities {
    pub friction: f32,
    pub restitution: f32,
    pub density: f32,
    pub linear_damping: f32,
    pub angular_damping: f32,
}

impl Default for PhysicsProperities {
    
    fn default() -> Self {
        Self { friction: 0.5, restitution: 0.5, density: 0.5, linear_damping: 0.1, angular_damping: 0.9 }
    }
}

impl PhysicsProperities {
    
    pub fn new(friction: f32, restitution: f32, density: f32, linear_damping: f32, angular_damping: f32) -> Self {
        Self { friction, restitution, density, linear_damping, angular_damping }
    }

    pub fn bounce() -> Self {
        Self { friction: 0.0, restitution: 1.0, density: 1.0, linear_damping: 0.1, angular_damping: 0.1 }
    }

    pub fn free() -> Self {
        Self { friction: 0.0, restitution: 1.0, density: 0.1, linear_damping: 0.01, angular_damping: 0.01 }
    }
}


pub struct Physics {
    pub attract_num: u32,
    pub rigid_bodies: RigidBodySet,
    pub colliders: ColliderSet,
    gravity: Vector2<f32>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: BroadPhase,
    narrow_phase: NarrowPhase,
    pub impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    physics_hooks: (),
    event_handler: (),
}

impl Physics {

    pub fn new() -> Self {
        Self {
            attract_num: 0,
            rigid_bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            gravity: Vector2::new(0.0, 0.0),
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: BroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            physics_hooks: (),
            event_handler: (),
        }
    }

    pub fn step_physics(&mut self) {
        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_bodies,
            &mut self.colliders,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            Some(&mut self.query_pipeline),
            &self.physics_hooks,
            &self.event_handler,
        );
    }

    pub fn remove_physics_object(&mut self, body_handle: RigidBodyHandle) {
        _ = self.rigid_bodies.remove(body_handle, &mut self.island_manager, &mut self.colliders, &mut self.impulse_joint_set, &mut self.multibody_joint_set, true);
    }

    pub fn get_physics_obj_num(&self) -> usize {
        let body_num = self.rigid_bodies.len();
        return body_num;
    }

    fn get_body_handle_from_collider(&self, collider_handle: ColliderHandle) -> Option<RigidBodyHandle> {
        let collider: &Collider;
        match self.colliders.get(collider_handle) {
            Some(col) => {
                collider = col;
            }
            None => {
                return None;
            }
        };
        match collider.parent() {
            Some(rbh) => {
                return Some(rbh);
            }
            None => {
                return None;
            }
        }
    }

    pub fn get_collider_size(&self, rb_handle: RigidBodyHandle) -> f32 {
        let rb = self.rigid_bodies.get(rb_handle).unwrap();
        let col_handle = rb.colliders().first().unwrap();
        let collider = self.colliders.get(*col_handle).unwrap();
        return collider.shape().as_ball().unwrap().radius;
    }

    fn iso_to_vec2_rot(&self, isometry: &Isometry<Real>) -> (Vec2, f32) {
        let pos = Vec2::new(isometry.translation.x, isometry.translation.y);
        let rot = isometry.rotation.angle() + PI;
        return (pos, rot);
    }

    pub fn add_dynamic_rigidbody(&mut self, key: u64, position: &Vec2, rotation: f32, linear_damping: f32, angular_damping: f32) -> RigidBodyHandle {
        let pos = Isometry2::new(Vector2::new(position.x, position.y), rotation);
        let dynamic_body = RigidBodyBuilder::dynamic().position(pos).linear_damping(linear_damping).angular_damping(angular_damping)
            .user_data(key as u128).build();
        return self.rigid_bodies.insert(dynamic_body);
    }

    pub fn add_static_rigidbody(&mut self, key: u64, position: &Vec2, rotation: f32) -> RigidBodyHandle {
        let pos = Isometry2::new(Vector2::new(position.x, position.y), rotation);
        let static_body = RigidBodyBuilder::fixed().position(pos).build();
        return self.rigid_bodies.insert(static_body);
    }

    pub fn add_collider(&mut self, body_handle: RigidBodyHandle, rel_position: &Vec2, rotation: f32, shape: SharedShape, physics_props: PhysicsProperities) -> ColliderHandle {
        let iso = make_isometry(rel_position.x, rel_position.y, rotation);
        let collider = match shape.shape_type() {
            ShapeType::Ball => {
                let radius = shape.0.as_ball().unwrap().radius;
                ColliderBuilder::ball(radius).position(iso).density(physics_props.density).friction(physics_props.friction).restitution(physics_props.restitution)
                    .active_collision_types(ActiveCollisionTypes::all()).active_events(ActiveEvents::COLLISION_EVENTS).build()
            },
            ShapeType::Triangle => {
                let verts = shape.0.as_triangle().unwrap().vertices().to_vec();
                ColliderBuilder::triangle(verts[0], verts[1], verts[2]).position(iso).density(physics_props.density).friction(physics_props.friction).restitution(physics_props.restitution)
                .active_collision_types(ActiveCollisionTypes::default()).active_events(ActiveEvents::COLLISION_EVENTS).build()
            },
            ShapeType::Cuboid => {
                let hx = shape.0.as_cuboid().unwrap().half_extents.x; let hy = shape.0.as_cuboid().unwrap().half_extents.y;
                ColliderBuilder::cuboid(hx, hy).position(iso).density(physics_props.density).friction(physics_props.friction).restitution(physics_props.restitution)
                .active_collision_types(ActiveCollisionTypes::default()).active_events(ActiveEvents::COLLISION_EVENTS).build()
            },
            ShapeType::ConvexPolygon => {
                let verts = shape.0.as_convex_polygon().unwrap().points();
                ColliderBuilder::convex_polyline(verts.to_vec()).unwrap().density(physics_props.density).friction(physics_props.friction).restitution(physics_props.restitution)
                .active_collision_types(ActiveCollisionTypes::all()).active_events(ActiveEvents::COLLISION_EVENTS).build()
            },
            _ => {
                ColliderBuilder::ball(5.0).position(iso).build()
            },
        };
        return self.colliders.insert_with_parent(collider, body_handle, &mut self.rigid_bodies);
    }

    pub fn add_dynamic(&mut self, key: u64, position: &Vec2, rotation: f32, shape: SharedShape, physics_props: PhysicsProperities) -> RigidBodyHandle {
        let rbh = self.add_dynamic_rigidbody(key, position, rotation, physics_props.linear_damping, physics_props.angular_damping);
        let _colh = self.add_collider(rbh, &Vec2::ZERO, 0.0, shape, physics_props);
        let rb = self.rigid_bodies.get_mut(rbh).unwrap();
        let impulse = random_unit_vec2()*(SPEED as f32);
        rb.apply_impulse(vector![impulse.x, impulse.y], true);
        return rbh;
    }

    pub fn add_static(&mut self, key: u64, position: &Vec2, rotation: f32, shape: SharedShape, physics_props: PhysicsProperities) -> RigidBodyHandle {
        let rbh = self.add_static_rigidbody(key, position, rotation);
        let _colh = self.add_collider(rbh, &Vec2::ZERO, 0.0, shape, physics_props);
        let rb = self.rigid_bodies.get_mut(rbh).unwrap();
        //let impulse = random_unit_vec2()*(SPEED as f32);
        //rb.apply_impulse(vector![impulse.x, impulse.y], true);
        return rbh;
    }

    pub fn add_ground(&mut self) -> RigidBodyHandle {
        let pos = Isometry2::new(Vector2::new(WORLD_W/2.0, 30.0), 0.0);
        let ground_body = RigidBodyBuilder::fixed().position(pos).build();
        let ground = self.rigid_bodies.insert(ground_body);
        let collider = ColliderBuilder::cuboid(WORLD_W/2.0, 10.0).active_collision_types(ActiveCollisionTypes::DYNAMIC_FIXED).build();
        self.colliders.insert_with_parent(collider, ground, &mut self.rigid_bodies);
        return ground
    }

    pub fn get_physics_data(&self, handle: RigidBodyHandle) -> PhysicsData {
        if let Some(rb) = self.rigid_bodies.get(handle) {
            let iso = rb.position();
            let (pos, rot) = self.iso_to_vec2_rot(iso);
            let force = Vec2::new(rb.user_force().data.0[0][0], rb.user_force().data.0[0][1]);
            let data = PhysicsData {
                position: pos,
                rotation: rot,
                mass: rb.mass(),
                kin_eng: Some(rb.kinetic_energy()),
                force: Some(force),
            };
            return data;
        } else {
            return PhysicsData {
                position: Vec2::new(WORLD_W / 2., WORLD_H / 2.),
                rotation: 0.0,
                mass: 0.0,
                kin_eng: Some(0.0),
                force: None,
            };
        }
    }

}

pub struct PhysicsData {
    pub position: Vec2,
    pub rotation: f32,
    pub mass: f32,
    pub kin_eng: Option<f32>,
    pub force: Option<Vec2>,
}
