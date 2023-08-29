#![allow(unused)]

use std::f32::consts::PI;
use crate::consts::*;
//use crate::sim::*;
use crate::util::*;
use crate::physics::*;
use macroquad::{color, prelude::*};
use macroquad::rand::*;
use rapier2d::geometry::*;
use rapier2d::na::Point2;
use rapier2d::na::Vector2;
use rapier2d::prelude::vector;
use rapier2d::prelude::{RigidBody, RigidBodyHandle};


pub struct JetEng {
    pub pos: Vec2,
    pub jet_vec: Vec2,
    pub max_impulse: f32,
    pub impulse: f32,
}

impl JetEng {
    
    pub fn new(pos: Vec2, jet_vec: Vec2, max_impulse: f32) -> Self {
        Self{pos, jet_vec, max_impulse, impulse: 0.0}
    }

    pub fn power(&mut self, thrust: f32) {
        let thrust = thrust.clamp(-1.0, 1.0);
        self.impulse = self.max_impulse*thrust;
    }

    pub fn get_impulse(&self) -> Vec2 {
        return self.jet_vec*self.impulse;
    }

    pub fn draw(&self, x0: f32, y0: f32, rot: f32, color: Color) {
        let pos = Vec2::from_angle(rot).rotate(self.pos);
        draw_circle(pos.x+x0, pos.y+y0, 3.0, color);
        let mut imp = self.get_impulse();
        imp = Vec2::from_angle(rot).rotate(imp)*1000.0;
        draw_line(pos.x+x0, pos.y+y0, pos.x+x0+imp.x, pos.y+y0+imp.y, 2.0, RED);
    }
}

pub struct Jet {
    pub key: u64,
    pub pos: Vec2,
    pub rot: f32,
    pub size: f32,
    pub color: color::Color,
    pub shape: SharedShape,
    pub physics_handle: RigidBodyHandle,
    pub engines: Vec<JetEng>,    
}

impl Jet {

    pub fn new_dyn_poly(vertices: Vec<Vec2>, position: Vec2, physics: &mut Physics) -> Self {
        let points = vec2_to_point2_collection(&vertices);
        let points2 = points.as_slice();
        let shape = SharedShape::convex_hull(points2).unwrap();
        let key = gen_range(u64::MIN, u64::MAX);
        let pos = position;
        let rbh = physics.add_dynamic(key, &pos, 0.0, shape.clone(), PhysicsProperities::default());
        Self {
            key,
            pos,
            //rot: random_rotation(),
            rot: 0.0,
            size: 5.0,
            color: LIGHTGRAY,
            shape: shape,
            physics_handle: rbh,
            engines: vec![
                JetEng::new(Vec2::new(-5.0, -10.0), Vec2::new(1.0, 0.0), 0.5),
                JetEng::new(Vec2::new(-5.0, 10.0), Vec2::new(-1.0, 0.0), 0.5),
            ]
        }
    }

    pub fn go(&mut self, thrust: f32) {
        for eng in self.engines.iter_mut() {
            eng.power(thrust);
        }        
    }

    pub fn turn_left(&mut self) {
        self.engines[0].power(-1.0);
        self.engines[1].power(1.0);
    }

    pub fn turn_right(&mut self) {
        self.engines[0].power(1.0);
        self.engines[1].power(-1.0);
    }

    pub fn draw(&self) {
        let x0 = self.pos.x;
        let y0 = self.pos.y;
        let points = self.shape.as_convex_polygon().unwrap().points().to_vec();
        let mut pre_point = points[points.len()-1];
        let mut v0 = Vec2::new(pre_point.x, pre_point.y);
        v0 = Vec2::from_angle(self.rot).rotate(v0);
        for p in points {
            let mut v1 = Vec2::new(p.x, p.y);
            v1 = Vec2::from_angle(self.rot).rotate(v1);
            draw_line(x0+v0.x, y0+v0.y, x0+v1.x, y0+v1.y, 2.0, self.color);
            v0 = v1.clone();
        }
        draw_circle(x0, y0, 6.0, RED);
        self.draw_engs();
    }    

    fn draw_engs(&self) {
        for eng in self.engines.iter() {
            eng.draw(self.pos.x, self.pos.y, self.rot, SKYBLUE);
        }
    }

    pub fn update(&mut self, dt: f32, physics: &mut Physics) {
        self.update_physics(physics);
    }

    fn update_physics(&mut self, physics: &mut Physics) {
        let physics_data = physics.get_physics_data(self.physics_handle.clone());
        let mut spec_impulse0: Vec2 = Vec2::ZERO;
        let mut spec_impulse1: Vec2 = Vec2::ZERO;
        spec_impulse0 = self.engines[0].get_impulse();
        spec_impulse1 = self.engines[1].get_impulse();
        self.pos = physics_data.position;
        self.rot = physics_data.rotation;
        let mut rb = physics.rigid_bodies.get_mut(self.physics_handle.clone()).unwrap();
        let v0 =Vec2::new(self.engines[0].pos.x, self.engines[0].pos.y).to_owned();
        rb.apply_impulse_at_point(Vector2::new(spec_impulse0.x, spec_impulse0.y), Point2::new(v0.x, v0.y), true);
        let v1 =Vec2::new(self.engines[1].pos.x, self.engines[1].pos.y).to_owned();
        rb.apply_impulse_at_point(Vector2::new(spec_impulse1.x, spec_impulse1.y), Point2::new(v1.x, v1.y), true);
    }
}
