#![allow(unused)]

use macroquad::prelude::*;
use rapier2d::prelude::*;
use crate::collector::Collector;
use crate::consts::WORLD_H;
use crate::consts::WORLD_W;
use crate::physics::*;
use crate::camera::*;
use crate::util::*;
use crate::unit::*;
use crate::jet::*;

pub struct Simulation {
    pub physics: Physics,
    pub camera: Camera2D,
    pub units: Collector,
    pub selected_unit: Option<u64>,
    pub joint: Option<ImpulseJointHandle>,
    pub jet: Vec<Jet>,
}

impl Simulation {

    pub fn new() -> Self {
        Self {
            physics: Physics::new(),
            camera: create_camera(),
            units: Collector::new(),
            selected_unit: None,
            joint: None,
            jet: vec![],
        }
    }

    pub fn init(&mut self) {
        let hull = vec![
            Vec2::new(15.0, 0.0),
            Vec2::new(5.0, -5.0),
            Vec2::new(0.0, -10.0),
            Vec2::new(-3.0, -11.0),
            Vec2::new(-8.0, -5.0),
            Vec2::new(-10.0, -6.0),
            Vec2::new(-10.0, 0.0),
            Vec2::new(-10.0, 6.0),
            Vec2::new(-8.0, 5.0),
            Vec2::new(-3.0, 11.0),
            Vec2::new(0.0, 10.0),
            Vec2::new(5.0, 5.0),
        ];
        let jet = Jet::new_dyn_poly(hull, Vec2::new(600.0, 400.0), &mut self.physics);
        self.jet.push(jet);

        //let s1 = Unit::new_static_rect([20.0, 900.0], Vec2::new(0.0, 450.0), &mut self.physics);
        //self.units.add_unit(s1, &mut self.physics);
        //let s2 = Unit::new_static_rect([20.0, 900.0], Vec2::new(1200.0, 450.0), &mut self.physics);
        //self.units.add_unit(s2, &mut self.physics);
        //let s3 = Unit::new_static_rect([1200.0, 20.0], Vec2::new(600.0, 0.0), &mut self.physics);
        //self.units.add_unit(s3, &mut self.physics);
        //let s4 = Unit::new_static_rect([1200.0, 20.0], Vec2::new(600.0, 900.0), &mut self.physics);
        //self.units.add_unit(s4, &mut self.physics);
        //for _ in 0..64 {
        //    let pos = random_position(1000.0, 800.0);
        //    let verts = vec![Vec2::new(5.0, 5.0)*3.0, Vec2::new(-2.0, 6.0)*3.0, Vec2::new(-5.0, 5.0)*3.0, Vec2::new(2.0, -3.0)*3.0, Vec2::new(-5.0, -5.0)*3.0, Vec2::new(5.0, -5.0)*3.0];
        //    let poly = Unit::new_dyn_poly(verts, pos, &mut self.physics);
        //    self.units.add_unit(poly, &mut self.physics);
        //}
        self.units.add_many_units(48, &mut self.physics);
        //let ball01 = Unit::new_dyn_circle(25.0, Vec2::new(600.0, 600.0), &mut self.physics);
        //let ball02 = Unit::new_dyn_circle(25.0, Vec2::new(650.0, 650.0), &mut self.physics);
        //let static_obj = Unit::new_static_rect([800.0, 10.0], Vec2::new(600.0, 40.0), &mut self.physics);
        //let static_obj2 = Unit::new_static_rect([25.0, 25.0], Vec2::new(600.0, 720.0), &mut self.physics);
        //let joint = PrismaticJointBuilder::new(Vector::y_axis()).local_anchor1(point![0.0, 0.0]).local_anchor2(point![0.0, 0.0]).limits([10.0, 300.0]).motor_velocity(0.0, 0.0).build();
        //self.joint = Some(self.physics.impulse_joint_set.insert(ball02.physics_handles[0], ball01.physics_handles[0], joint, true));
        //self.units.add_unit(ball01, &mut self.physics);
        //self.units.add_unit(ball02, &mut self.physics);
        //self.units.add_unit(static_obj, &mut self.physics);
        //self.units.add_unit(static_obj2, &mut self.physics);
        //for _ in 0..14 {
        //    self.units.add_complex(&mut self.physics);
        //}
    }

    pub fn update(&mut self) {
        self.update_units();
        for jet in self.jet.iter_mut() {
            jet.update(get_frame_time(), &mut self.physics);
        }
    }

    pub fn step_physics(&mut self) {
        self.physics.step_physics();
    }

    pub fn update_units(&mut self) {
        let dt = get_frame_time();
        for (_, unit) in self.units.get_iter_mut() {
            unit.update(dt, &mut self.physics);
        }
    }

    pub fn draw(&self) {
        set_camera(&self.camera);
        clear_background(BLACK);
        for jet in self.jet.iter() {
            jet.draw();
        }        
        //match self.joint {
        //    None => {},
        //    Some(joint_handle) => {
        //        let joint = self.physics.impulse_joint_set.get(joint_handle).unwrap();
        //        let rb1 = self.physics.rigid_bodies.get(joint.body1).unwrap();
        //        let rb2 = self.physics.rigid_bodies.get(joint.body2).unwrap();
        //        let (p1, r1) = iso_to_vec2_rot(rb1.position());
        //        let (p2, r2) = iso_to_vec2_rot(rb1.position());
        //        draw_line(p1.x, p1.y, p2.x, p2.y, 2.0, RED);
        //    }
        //}
        self.draw_units();
    }

    pub fn draw_units(&self) {
        let dt = get_frame_time();
        for (_, unit) in self.units.get_iter() {
            unit.draw();
        }
    }

    pub fn input(&mut self) {
        self.mouse_input();
        control_camera(&mut self.camera);
        self.keyboard();
    }

    fn keyboard(&mut self) {
        let jet = self.jet.first_mut().unwrap();
        if is_key_pressed(KeyCode::W) {
            jet.go(1.0);
            println!("go");
        }
        if is_key_pressed(KeyCode::S) {
            jet.go(-1.0);
            println!("back");
        }
        if is_key_pressed(KeyCode::A) {
            jet.turn_left();
            println!("left");
        }
        if is_key_pressed(KeyCode::D) {
            jet.turn_right();
            println!("right");
        }
    }

    fn mouse_input(&mut self) {
        if is_mouse_button_released(MouseButton::Left) {
            self.selected_unit = None;
            let (mouse_posx, mouse_posy) = mouse_position();
            let mouse_pos = Vec2::new(mouse_posx, mouse_posy);
            let rel_coords = self.camera.screen_to_world(mouse_pos);
            for (id, unit) in self.units.get_iter() {
                if contact_mouse(rel_coords, unit.pos, unit.size) {
                    self.selected_unit = Some(*id);
                    break;
                }
            }
        }
    }

}