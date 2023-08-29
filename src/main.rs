#![allow(unused)]

mod sim;
mod camera;
mod physics;
mod consts;
mod util;
mod unit;
mod collector;
mod jet;

use macroquad::prelude::*;
use crate::sim::*;

fn app_configuration() -> Conf {
    Conf {
        window_title: env!("CARGO_PKG_NAME").to_string().to_uppercase(),
        window_width: 1200,
        window_height: 900,
        sample_count: 16,
        window_resizable: false,
        ..Default::default()
    }
}

#[macroquad::main(app_configuration)]
async fn main() {
    let mut sim = Simulation::new();
    sim.init();
    loop {
        sim.input();
        sim.update();
        sim.draw();
        sim.step_physics();
        next_frame().await;
    }
}