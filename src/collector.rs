#![allow(unused)]


use std::collections::hash_map::{Iter, IterMut};
use std::collections::HashMap;
use std::f32::consts::PI;
use crate::consts::{WORLD_H, WORLD_W};
//use crate::sim::*;
use crate::util::*;
use crate::physics::*;
use crate::unit::*;
use macroquad::{color, prelude::*};
use macroquad::rand::*;
use rapier2d::geometry::*;
use rapier2d::na::Vector2;
use rapier2d::prelude::{RigidBody, RigidBodyHandle};



pub struct Collector {
    pub units: HashMap<u64, Unit>,
}

impl Collector {
    pub fn new() -> Self {
        Self {
            units: HashMap::new(),
        }
    }

    pub fn add_many_units(&mut self, units_num: usize, physics: &mut Physics) {
        for _ in 0..units_num {
            let unit = Unit::new(physics);
            _ = self.add_unit(unit, physics);
        }
    }

    pub fn add_complex(&mut self, physics: &mut Physics) -> u64 {
        let parts = PartsScheme {
            parts: vec![
                PartDef {shape_size: 10.0, relative_position: Vec2::ZERO},
                PartDef {shape_size: 6.0, relative_position: Vec2::new(-20.0, -20.0)},
                PartDef {shape_size: 6.0, relative_position: Vec2::new(-20.0, 20.0)},
                PartDef {shape_size: 6.0, relative_position: Vec2::new(20.0, -20.0)},
                PartDef {shape_size: 6.0, relative_position: Vec2::new(20.0, 20.0)},
            ]
        };
        let unit = Unit::new_complex(parts, physics);
        let key = unit.key;
        self.units.insert(key, unit);
        return key;
    }

    pub fn add_unit(&mut self, mut unit: Unit, physics: &mut Physics) -> u64 {
        let key = unit.key;
        //let handle = physics.add_dynamic(key, &unit.pos, unit.rot, unit.shape.clone(), PhysicsProperities::default());
        //unit.physics_handle = Some(handle);
        self.units.insert(key, unit);
        return key;
    }

    pub fn get(&self, id: u64) -> Option<&Unit> {
        return self.units.get(&id);
    }

    pub fn _remove(&mut self, id: u64) {
        self.units.remove(&id);
    }

    pub fn get_iter(&self) -> Iter<u64, Unit> {
        return self.units.iter();
    }

    pub fn get_iter_mut(&mut self) -> IterMut<u64, Unit> {
        return self.units.iter_mut();
    }

    pub fn _count(&self) -> usize {
        return self.units.len();
    }
}