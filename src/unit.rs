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
    pub shapes: Vec<SharedShape>,
    pub physics_handle: RigidBodyHandle,
    pub data: Vec<(f32, Vec2, f32)>,
    pub body_parts: Vec<BodyPart>,
    
}

impl Unit {
    
    pub fn new(physics: &mut Physics) -> Self {
        let size = rand::gen_range(SIZE_MIN, SIZE_MAX) as f32;
        let shape = SharedShape::ball(size);
        let key = gen_range(u64::MIN, u64::MAX);
        let pos = random_position(WORLD_W as f32, WORLD_H as f32);
        let rbh = physics.add_dynamic(key, &pos, 0.0, shape.clone(), PhysicsProperities::default());
        Self {
            key,
            pos,
            //rot: random_rotation(),
            rot: 0.0,
            size,
            color: random_color(),
            shapes: vec![shape],
            physics_handle: rbh,
            data: vec![],
            body_parts: vec![],
        }
    }

    pub fn new_dyn_circle(size: f32,  position: Vec2, physics: &mut Physics) -> Self {
        //let size = rand::gen_range(SIZE_MIN, SIZE_MAX) as f32;
        let shape = SharedShape::ball(size);
        let key = gen_range(u64::MIN, u64::MAX);
        let pos = position;
        let rbh = physics.add_dynamic(key, &pos, 0.0, shape.clone(), PhysicsProperities::bounce());
        Self {
            key,
            pos,
            //rot: random_rotation(),
            rot: 0.0,
            size,
            color: random_color(),
            shapes: vec![shape],
            physics_handle: rbh,
            data: vec![],
            
        }
    }

    pub fn new_static_rect(size: [f32; 2],  position: Vec2, physics: &mut Physics) -> Self {
        //let size = rand::gen_range(SIZE_MIN, SIZE_MAX) as f32;
        let shape = SharedShape::cuboid(size[0], size[1]);
        let key = gen_range(u64::MIN, u64::MAX);
        let pos = position;
        let rbh = physics.add_static(key, &pos, 0.0, shape.clone(), PhysicsProperities::bounce());
        Self {
            key,
            pos,
            //rot: random_rotation(),
            rot: 0.0,
            size: size[0],
            color: LIGHTGRAY,
            shapes: vec![shape],
            physics_handles: vec![rbh],
            data: vec![],
        }
    }

    pub fn new_dyn_poly(vertices: Vec<Vec2>, position: Vec2, physics: &mut Physics) -> Self {
        let points = vec2_to_point2_collection(&vertices);
        let shape = match SharedShape::convex_polyline(points) {
            Some(poly_shape) => poly_shape,
            None => SharedShape::ball(5.0),
        };
        let key = gen_range(u64::MIN, u64::MAX);
        let pos = position;
        let rbh = physics.add_dynamic(key, &pos, 0.0, shape.clone(), PhysicsProperities::free());
        Self {
            key,
            pos,
            //rot: random_rotation(),
            rot: 0.0,
            size: 5.0,
            color: LIGHTGRAY,
            shapes: vec![shape],
            physics_handles: vec![rbh],
            data: vec![],
        }
    }


    pub fn new_complex(parts: PartsScheme, physics: &mut Physics) -> Self {
        let mut shapes = vec![];
        let mut handles = vec![];
        let pos = random_position(WORLD_W as f32, WORLD_H as f32);
        let key = gen_range(u64::MIN, u64::MAX);
        for part in parts.parts.iter() {
            let shape = SharedShape::ball(part.shape_size);
            let rel_pos = part.relative_position + pos;
            let rbh = physics.add_dynamic(key, &rel_pos, 0.0, shape.clone(), PhysicsProperities::bounce());
            shapes.push(shape);
            handles.push(rbh);
        }
        let size = rand::gen_range(SIZE_MIN, SIZE_MAX) as f32;
        Self {
            key: key,
            pos: random_position(WORLD_W as f32, WORLD_H as f32),
            //rot: random_rotation(),
            rot: 0.0,
            size: size,
            color: random_color(),
            shapes: shapes,
            physics_handles: handles,
            data: vec![],
        }
    }

    pub fn draw(&self) {
        let x0 = self.pos.x;
        let y0 = self.pos.y;
        match self.shapes[0].shape_type() {
            ShapeType::Ball => {
                for (s, v, r) in self.data.iter() {
                    self.draw_circle(s, v, r);
                }
            },
            ShapeType::Cuboid => {
                
                let mut wh = self.shapes[0].0.as_cuboid().unwrap().half_extents.data;
                let w = wh.as_slice()[0];
                let h = wh.as_slice()[1];
                self.draw_cuboid(x0-w, y0-h, w*2.0, h*2.0);
            },
            ShapeType::ConvexPolygon => {
                let points = self.shapes[0].as_convex_polygon().unwrap().points().to_vec();
                let mut pre_point = points[points.len()-1];
                let mut v0 = Vec2::new(x0+pre_point.x, y0+pre_point.y);
                for p in points {
                    //let v0 = Vec2::new(pre_point.x, pre_point.y);
                    let v1 = Vec2::new(x0+p.x, y0+p.y);
                    draw_line(v0.x, v0.y, v1.x, v1.y, 2.0, self.color);
                    v0 = v1.clone();
                }
            }
            _ => {},
        }
        draw_circle(x0, y0, 6.0, RED);
    }    

    pub fn update(&mut self, dt: f32, physics: &mut Physics) {
        self.update_physics_complex(physics);
    }

    fn draw_cuboid(&self, x: f32, y: f32, w: f32, h: f32) {
        draw_rectangle_lines(x, y, w, h, 4.0, LIGHTGRAY);
    }

    fn draw_circle(&self, s: &f32, v: &Vec2, r: &f32) {
        let x0 = self.pos.x;
        let y0 = self.pos.y;
        draw_circle_lines(v.x, v.y, *s, 3.0, self.color);
        //self.draw_front();
    }

/*     fn update_physics(&mut self, physics: &mut Physics) {
        match self.physics_handle {
            Some(handle) => {
                let physics_data = physics.get_physics_data(handle);
                self.pos = physics_data.position;
                self.rot = physics_data.rotation;
            }
            None => {}
        }
    } */

    fn update_physics_complex(&mut self, physics: &Physics) {
        if self.shapes[0].shape_type() == ShapeType::Ball {
            self.data.clear();
            for rbh in self.physics_handles.iter() {
                let s = physics.get_collider_size(rbh.clone());
                let physics_data = physics.get_physics_data(rbh.clone());
                let pos = physics_data.position;
                let rot = physics_data.rotation;
                self.data.push((s, pos, rot));
            }
        }
        if self.shapes[0].shape_type() == ShapeType::ConvexPolygon {
            self.data.clear();
            for rbh in self.physics_handles.iter() {
                let s = 5.0;
                let physics_data = physics.get_physics_data(rbh.clone());
                let pos = physics_data.position;
                let rot = physics_data.rotation;
                self.pos = pos;
                self.data.push((s, pos, rot));
            }
        }
    }
}
