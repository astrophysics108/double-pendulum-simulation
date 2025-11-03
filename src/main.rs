use macroquad::prelude::*;
use num_traits::pow;
use ode_solvers::System;
use ode_solvers::rk4::Rk4;
use nalgebra::SVector;

const G: f32 =  9.81;

// gonna try ode_solvers aaah
// so according to the internet I put the base stuff here???
struct DoublePendulum {
...
}

#[macroquad::main("pendulum")]
async fn main() {
    // pendulum stuff
    let l1 = 150.0;
    let l2 = 150.0;
    let m1 = 50.0;
    let m2 = 50.0;
    let origin = vec2(400.0, 200.0);

    let mut phi1 = std::f32::consts::PI / 5.0;
    let mut phi2 = std::f32::consts::PI / 5.0;
    let mut omega1 = 0.0;
    let mut omega2 = 0.0;
    
    let change_in_time = 0.1; 
    loop {
        let change_in_angle = phi2 - phi1;

        // euqations
        let ang_acceleration_1 = (m2*l1*pow(omega1, 2)*change_in_angle.sin()*change_in_angle.cos()+m2*G*phi2.sin()*change_in_angle.cos()+m2*l2*pow(omega2, 2)*change_in_angle.sin()-(m1+m2)*G*phi1.sin())/(l1*(m1+m2) - m2*l1*(pow(change_in_angle.cos(),2)));
        let ang_acceleration_2 = -l1*(m2*l1*pow(omega1, 2)*change_in_angle.sin()*change_in_angle.cos()+(m1+m2)*(G*phi1.sin()*change_in_angle.cos()-l1*pow(omega1, 2)*change_in_angle.sin()-G*phi2.sin()))/(l2*l1*(m1+m2) - m2*l1*(pow(change_in_angle.cos(), 2)));
        
        // fake integration to update ang frequency etc
        omega1 += ang_acceleration_1 * change_in_time;
        omega2 += ang_acceleration_2 * change_in_time;
        phi1 += omega1 * change_in_time;
        phi2 += omega2 * change_in_time;
        // not moving properly for now
        let pos1 = origin + vec2(l1*phi1.sin(), l1*phi1.cos());
        let pos2 = pos1 + vec2(l2*phi2.sin(), l2*phi2.cos());
        clear_background(LIGHTGRAY);
        draw_line(origin.x, origin.y, pos1.x, pos1.y, 5.0, RED);
        draw_line(pos1.x, pos1.y, pos2.x, pos2.y, 5.0, RED);
        draw_circle(pos1.x, pos1.y, 20.0, BLUE);
        draw_circle(pos2.x, pos2.y, 20.0, BLUE);
        next_frame().await;

    }
}
