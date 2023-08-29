#![allow(unused)]


use std::collections::hash_map::{Iter, IterMut};
use std::collections::HashMap;
use std::f32::consts::PI;
use crate::consts::*;
//use crate::sim::*;
use crate::util::*;
use crate::physics::*;
use crate::collector::*;
use macroquad::{color, prelude::*};
use macroquad::rand::*;
use rapier2d::geometry::*;
use rapier2d::na::Vector2;
use rapier2d::prelude::{RigidBody, RigidBodyHandle};


pub struct PartDef {
    pub shape_size: f32,
    pub relative_position: Vec2,
}

pub struct PartsScheme {
    pub parts: Vec<PartDef>
}

pub struct BodyPart {
    pub rel_pos: Vec2,
    pub color: Color,
    pub shape: SharedShape,
    handle: Option<ColliderHandle>,
}

impl BodyPart {
    pub fn add_new(relative_position: Vec2, size: f32, color: Color) -> Self {
        Self {
            color,
            rel_pos: relative_position,
            shape: SharedShape::ball(size),
            handle: None,
        }
    }

    pub fn draw_circle(&self, position: &Vec2, rot: f32) {
        let mut pos = Vec2::from_angle(rot).rotate(self.rel_pos);
        pos += position.clone();
        let size = self.shape.as_ball().unwrap().radius;
        draw_circle(pos.x, pos.y, size, self.color); 
    }

    pub fn get_rel_position(&self) -> Vec2 {
        return self.rel_pos
    }

    pub fn get_color(&self) -> Color {
        return self.color;
    }

    pub fn get_shape(&self) -> SharedShape {
        return self.shape.clone();
    }

    pub fn set_collider_handle(&mut self, collider_handle: ColliderHandle) {
        self.handle = Some(collider_handle);
    }

    pub fn get_collider_handler(&self) -> Option<ColliderHandle> {
        return self.handle;
    }
}

pub struct Unit {
    pub key: u64,
    pub pos: Vec2,
    pub rot: f32,
    pub size: f32,
    pub color: color::Color,
    pub shape: SharedShape,
    pub physics_handle: RigidBodyHandle,
    pub data: Vec<(f32, Vec2, f32)>,
    pub body_parts: Vec<BodyPart>,
    
}

impl Unit {
    
    pub fn new(physics: &mut Physics) -> Self {
        let size = rand::gen_range(SIZE_MIN, SIZE_MAX) as f32;
        let shape = SharedShape::ball(size);
        let color = random_color();
        let mut parts: Vec<BodyPart> = vec![];
        let step = 2.0*PI/3.0;
        let key = gen_range(u64::MIN, u64::MAX);
        let pos = random_position(WORLD_W as f32, WORLD_H as f32);
        let rbh = physics.add_dynamic(key, &pos, 0.0, shape.clone(), PhysicsProperities::default());
        for i in 0..3 {
            let rel_pos = Vec2::from_angle(i as f32 * step) * 2.0*size;
            let mut part = BodyPart::add_new(rel_pos, size, color);
            let coll_handle = physics.add_collider(rbh, &rel_pos, 0.0, part.get_shape(), PhysicsProperities::free());
            part.set_collider_handle(coll_handle);
            parts.push(part);
        }
        Self {
            key,
            pos,
            //rot: random_rotation(),
            rot: 0.0,
            size,
            color,
            shape: shape,
            physics_handle: rbh,
            data: vec![],
            body_parts: parts,
        }
    }

    pub fn draw(&self) {
        let x0 = self.pos.x;
        let y0 = self.pos.y;
        for part in self.body_parts.iter() {
            part.draw_circle(&self.pos, self.rot);
        }
        draw_circle(x0, y0, self.size, RED);
    }    

    pub fn update(&mut self, dt: f32, physics: &mut Physics) {
        self.update_physics(physics);
    }

    fn draw_circle(&self) {
        let x0 = self.pos.x;
        let y0 = self.pos.y;
        draw_circle_lines(x0, y0, self.size, 3.0, self.color);
        //self.draw_front();
    }

    fn update_physics(&mut self, physics: &mut Physics) {
        let physics_data = physics.get_physics_data(self.physics_handle);
        self.pos = physics_data.position;
        self.rot = physics_data.rotation;
    }
}
