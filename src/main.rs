// IMPORTS
use macroquad::prelude::*;
use num_traits::pow;
use ode_solvers::System;
use ode_solvers::rk4::Rk4;
use nalgebra::OVector;
use nalgebra::Const;

// CONSTANTS
const G: f32 =  9.81;
type State = OVector<f32, Const<4>>;
const ORIGIN: Vec2 = vec2(400.0, 200.0);

// Define system variable structure
struct DoublePendulum {
    l1: f32,
    l2: f32,
    m1: f32,
    m2: f32,
}

// system definition
impl System<f32, State> for DoublePendulum {

    fn system(&self, _t: f32, y: &OVector<f32, Const<4>> , dy: &mut State ) {

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

        // derivative vector components 
        // dy[0] = omega1;
        // dy[1] = omega2;
        // dy[2] = ang_acceleration_1;
        // dy[3] = ang_acceleration_2;
        *dy = State::from([
            omega1, omega2, 
            ang_acceleration_1, ang_acceleration_2
        ]);
            
        }
}

fn draw_slider(mut value: f32, pos: Vec2) -> f32 { 
    let size = vec2(300.0, 20.0);
    let min: f32 = 1.0;
    let max: f32 = 100.0;

    draw_rectangle(pos.x, pos.y, size.x, size.y, DARKGRAY);

    // moves the recatngle to the mouse place
    if is_mouse_button_down(MouseButton::Left) {

        let mouse = mouse_position();
        let mx = mouse.0;
        if mx >= pos.x && mx <= pos.x + size.x &&
           mouse.1 >= pos.y && mouse.1 <= pos.y + size.y
        {
            value = min + (mx - pos.x) / size.x * (max - min);
        }
    }

    let handle_x = pos.x + (value - min) / (max - min) * size.x;
    draw_rectangle(handle_x - 5.0, pos.y - 5.0, 10.0, size.y + 10.0, RED);

    return value
}

// drawing function
// Note to self: so this with list comprehensions and for loops when you figure that out
fn draw_everything(pos1: Vec2, pos2: Vec2, values: Vec4) -> Vec4 {

    clear_background(LIGHTGRAY);

    // slider positions
    let pos_slider_1 = vec2(50.0, 50.0);
    let pos_slider_2 = vec2(50.0, 100.0);
    let pos_slider_3 = vec2(50.0, 150.0);
    let pos_slider_4 = vec2(50.0, 200.0);

    // sliders for lengths and masses
    let value1 = draw_slider(values.x, pos_slider_1);
    let value2 = draw_slider(values.y, pos_slider_2);
    let value3 = draw_slider(values.z, pos_slider_3);
    let value4 = draw_slider(values.w, pos_slider_4);

    // pendulum rods
    draw_line(ORIGIN.x, ORIGIN.y, pos1.x, pos1.y, 5.0, RED);
    draw_line(pos1.x, pos1.y, pos2.x, pos2.y, 5.0, RED);

    // pendulum bobs
    draw_circle(pos1.x, pos1.y, 20.0, BLUE);
    draw_circle(pos2.x, pos2.y, 20.0, BLUE);

    return vec4(value1, value2, value3, value4);
}

#[macroquad::main("pendulum")]
// main logic loop 
async fn main() {

    // define angles and angular velocities
    let mut phi1 = std::f32::consts::PI / 5.0;
    let mut phi2 = std::f32::consts::PI / 5.0;
    let mut omega1 = 0.0;
    let mut omega2 = 0.0;
    let mut value1: f32 = 50.0;
    let mut value2: f32 = 50.0;
    let mut value3: f32 = 50.0;
    let mut value4: f32 = 50.0;

    let mut len1 = 150.0;
    let mut len2 = 150.0;

    let mut mass1 = 50.0;
    let mut mass2 = 50.0;

    loop {
        let system = DoublePendulum{l1:len1, l2:len2, m1:mass1, m2:mass2};
        let dt: f32= 0.01; 
        let t_start: f32 = 0.0;
        let t_end: f32 = 0.1;
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

        // get the position based on the angle
        let pos1 = ORIGIN + vec2(len1*phi1.sin(), len1*phi1.cos());
        let pos2 = pos1 + vec2(len2*phi2.sin(), len2*phi2.cos());

        // draw it all and await the next frame
        let values = draw_everything(pos1, pos2, vec4(value1, value2, value3, value4));
        value1 = values.x;
        value2 = values.y;
        value3 = values.z;
        value4 = values.w;
        len1 = value1 * 3.0;
        len2 = value2 * 3.0;
        mass1 = value3;
        mass2 = value4;
        next_frame().await;

    }
}
