use macroquad::prelude::*;
use num_traits::pow;
use ode_solvers::System;
use ode_solvers::rk4::Rk4;
// use nalgebra::SVector;
use nalgebra::OVector;
use nalgebra::Const;

// can this go in here?
const G: f32 =  9.81;

// I HAVE NO IDEA WHAT IM DOING BSIYFBIWEUFBIO2FB
type State = OVector<f32, Const<4>>;

// gonna try ode_solvers aaah
// so according to the internet I put the base stuff here???
//what I just imply type uhhh
struct DoublePendulum {
    l1: f32,
    l2: f32,
    m1: f32,
    m2: f32,
}

// then I have to define a system... uhhh
impl System<f32, State> for DoublePendulum {
    fn system(&self, _t: f32, y: &OVector<f32, Const<4>> , dy: &mut OVector<f32, Const<4>> ) {

        // defining the base values that are shown in the struct
        let l1 = self.l1;
        let l2 = self.l2;
        let m1 = self.m1;
        let m2 = self.m2;

        // defining the y vector which we take the derivatives of - basically if I need to take the derivative of it then shove it in this
        let phi1 = y[0];
        let phi2 = y[1];
        let omega1 = y[2];
        let omega2 = y[3];
        
        // more key numbers
        let change_in_angle = phi2 - phi1;

        // euqations for ang acceleration (will be solved with RK4)
        let ang_acceleration_1 = (m2*l1*pow(omega1, 2)*change_in_angle.sin()*change_in_angle.cos()+m2*G*phi2.sin()*change_in_angle.cos()+m2*l2*pow(omega2, 2)*change_in_angle.sin()-(m1+m2)*G*phi1.sin())/(l1*(m1+m2) - m2*l1*(pow(change_in_angle.cos(),2)));
        let ang_acceleration_2 = -l1*(m2*l1*pow(omega1, 2)*change_in_angle.sin()*change_in_angle.cos()+(m1+m2)*(G*phi1.sin()*change_in_angle.cos()-l1*pow(omega1, 2)*change_in_angle.sin()-G*phi2.sin()))/(l2*l1*(m1+m2) - m2*l1*(pow(change_in_angle.cos(), 2)));

        // derivative vector components (i think?)
        dy[0] = omega1;
        dy[1] = omega2;
        dy[2] = ang_acceleration_1;
        dy[3] = ang_acceleration_2;
            
        }
}

#[macroquad::main("pendulum")]
async fn main() {
    
    // use the system here ig
    let origin = vec2(400.0, 200.0);

    let mut phi1 = std::f32::consts::PI / 5.0;
    let mut phi2 = std::f32::consts::PI / 5.0;
    let mut omega1 = 0.0;
    let mut omega2 = 0.0;
    
    



    loop {
        let system = DoublePendulum{l1:150.0, l2:150.0, m1:50.0, m2:50.0};
    let dt: f32= 0.01; 
    let t_start: f32 = 0.0;
    let t_end: f32 = 0.1;
    let l1 = 150.0;
    let l2 = 150.0;
    // initial y vector which we use the system to differentiate and update
    let y0 = State::from([phi1, phi2, omega1, omega2]);

    // solve with Rk4
    let mut delta = Rk4::new(system, t_start, y0, t_end, dt);  
           

            // integrate properly yay
        delta.integrate().unwrap();
        let new_state = delta.y_out().last().unwrap();
        phi1 = new_state[0];
        phi2 = new_state[1];
        omega1 = new_state[2];
        omega2 = new_state[3];

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
