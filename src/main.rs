// IMPORTS
use macroquad::prelude::*;
use ode_solvers::System;
use ode_solvers::rk4::Rk4;
use nalgebra::OVector;
use nalgebra::Const;

// CONSTANTS
const G: f32 =  9.81;
type State = OVector<f32, Const<4>>;
const WIDTH: f32 = 1000.0;
const HEIGHT: f32 = WIDTH / 2.0;
const ORIGIN: Vec2 = vec2(WIDTH / 2.0, HEIGHT / 6.0);


fn conf() -> Conf {
    Conf {
        window_title: "Double Pendulum".to_owned(),
        window_width: (WIDTH * 2.0) as i32,
        window_height: (HEIGHT * 2.0) as i32,
        high_dpi: true,
        fullscreen: false,
        window_resizable: false,
        icon: None,
        platform: Default::default(),
        sample_count: 1,
    }
}

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
        

        // euqations for ang acceleration (will be solved with RK4)
        let delta = phi1 - phi2;
        let sin_delta = delta.sin();
        let cos_delta = delta.cos();

        let denom = 2.0 * m1 + m2 - m2 * (2.0 * delta).cos();
        let denom1 = l1 * denom;
        let denom2 = l2 * denom;

        // Standard double-pendulum angular accelerations
        let ang_acceleration_1 =
            (-G * (2.0 * m1 + m2) * phi1.sin()
             - m2 * G * (phi1 - 2.0 * phi2).sin()
             - 2.0 * sin_delta * m2 * (omega2.powi(2) * l2 + omega1.powi(2) * l1 * cos_delta))
            / denom1;

        let ang_acceleration_2 =
            (2.0 * sin_delta
             * (omega1.powi(2) * l1 * (m1 + m2)
                + G * (m1 + m2) * phi1.cos()
                + omega2.powi(2) * l2 * m2 * cos_delta))
            / denom2;


        // derivative vector components 
        dy[0] = omega1;
        dy[1] = omega2;
        dy[2] = ang_acceleration_1;
        dy[3] = ang_acceleration_2;
        *dy = State::from([
            omega1, omega2, 
            ang_acceleration_1, ang_acceleration_2
        ]);
            
        }
}

fn draw_slider(mut value: f32, pos: Vec2, colour: Color) -> f32 { 
    let size = vec2(WIDTH/4.0, HEIGHT/25.0);
    let min: f32 = 1.0;
    let max: f32 = 100.0;

    draw_rectangle(pos.x, pos.y, size.x, size.y, colour);

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
    draw_rectangle(handle_x - 5.0, pos.y - 5.0, 10.0, size.y + 10.0, DARKGRAY);

    return value
}

// drawing function
// Note to self: so this with list comprehensions and for loops when you figure that out
fn draw_everything(pos1: Vec2, pos2: Vec2, values: Vec4, ball_texture: &Texture2D, phi1: f32, phi2: f32, paths: &Vec<Texture2D>, bob_path:usize) -> Vec<f32> {

    clear_background(LIGHTGRAY);

    // match rod and bob colour
    let rod_colours: Vec<Vec<i32>> = vec![
        vec![235, 170, 101],
        vec![89, 10, 44],
        vec![40, 10, 89],
        vec![99, 18, 21],
        vec![189, 78, 9],
        vec![207, 168, 14],
        vec![80, 166, 10],
        vec![6, 115, 102],
        vec![66, 166, 219],
        vec![21, 15, 112],
        vec![88, 23, 163],
    ];

    let colour = &rod_colours[bob_path];
    let colour_of_rods = Color::new((colour[0] as f32)/256.0, (colour[1] as f32)/256.0, (colour[2] as f32)/256.0, 1.0);

    // slider positions
    let pos_slider_1 = vec2(50.0, 50.0);
    let pos_slider_2 = vec2(50.0, 100.0);
    let pos_slider_3 = vec2(50.0, 150.0);
    let pos_slider_4 = vec2(50.0, 200.0);

    // sliders for lengths and masses
    let value1 = draw_slider(values.x, pos_slider_1, colour_of_rods);
    let value2 = draw_slider(values.y, pos_slider_2, colour_of_rods);
    let value3 = draw_slider(values.z, pos_slider_3, colour_of_rods);
    let value4 = draw_slider(values.w, pos_slider_4, colour_of_rods);

    
    // pendulum rods
    draw_line(ORIGIN.x, ORIGIN.y, pos1.x, pos1.y, 5.0, colour_of_rods);
    draw_line(pos1.x, pos1.y, pos2.x, pos2.y, 5.0, colour_of_rods);

    // pendulum bobs
    draw_texture_ex(
        &ball_texture, 
        pos1.x - 20.0, pos1.y - 20.0, 
        WHITE,
        DrawTextureParams {
            rotation: -phi1,
            dest_size: Some(vec2(40.0, 40.0)),
            ..Default::default()
        },
    );
    draw_texture_ex(
        &ball_texture, 
        pos2.x - 20.0, pos2.y - 20.0, 
        WHITE,
        DrawTextureParams {
            rotation: -phi2,
            dest_size: Some(vec2(40.0, 40.0)),
            ..Default::default()
        },
    );

    let bob_path = draw_aesthetics_menu(paths.to_vec());

    return vec![value1, value2, value3, value4, bob_path as f32];
}

fn draw_aesthetics_menu(paths: Vec<Texture2D>) -> i32 {
    let gap = vec2(WIDTH/15.0, 0.0);
    let gap_y = vec2(0.0, WIDTH/15.0);
    
    let pos_start = vec2(WIDTH/20.0, HEIGHT/2.0);
    let mut bob_path: i32 = 1000;
    let mut all_positions = vec![];

    for (i, img) in paths.iter().enumerate() {
        let ind = i as f32;
        let pos = pos_start + ((ind % 3.0) * gap) + (((ind as i32)/3) as f32 * gap_y);
        all_positions.push(pos);
        draw_texture_ex(
        &img, 
        pos.x , pos.y, 
        WHITE,
        DrawTextureParams {
            dest_size: Some(vec2(WIDTH/18.0, WIDTH/18.0)),
            ..Default::default()
        },
    );
    }

    // checks if the user selected a different bob path
    if is_mouse_button_down(MouseButton::Left) {

        let mouse = mouse_position();
        let mx = mouse.0;
        let my = mouse.1;

        for (i, position) in all_positions.iter().enumerate() {
            if (mx -100.0 < position.x) & (position.x < mx ) & (my - 100.0 < position.y) & (position.y < my) {
                bob_path = i as i32;
            }
        }
    }

    // let handle_x = pos.x + (value - min) / (max - min) * size.x;
    // draw_rectangle(handle_x - 5.0, pos.y - 5.0, 10.0, size.y + 10.0, RED);

    return bob_path
}

#[macroquad::main(conf())]
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
    let mut bob_path = 0;
    let img_paths = vec![
        r"images\ball_yellow.png", 
        r"images\magentaball.png", 
        r"images\starryball.png",
        r"images\red.png",
        r"images\orange.png",
        r"images\yellow.png",
        r"images\green.png",
        r"images\teal.png",
        r"images\lightblue.png",
        r"images\darkblue.png",
        r"images\purple.png"
        ];
    
    let mut paths = vec![];
    for path in img_paths.iter() {
            paths.push(load_texture(&path).await.unwrap());
    }

    loop {
        let system = DoublePendulum{l1:len1, l2:len2, m1:mass1, m2:mass2};
        let dt: f32= 0.01; 
        let t_start: f32 = 0.0;
        let t_end: f32 = 0.05;
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
        let values = draw_everything(pos1, pos2, vec4(value1, value2, value3, value4), &paths[bob_path], phi1, phi2, &paths, bob_path);
        value1 = values[0];
        value2 = values[1];
        value3 = values[2];
        value4 = values[3];
        let bob = values[4] as usize;
        if bob < 100 {
            bob_path = bob;
        }
        len1 = value1 * 3.0;
        len2 = value2 * 3.0;
        mass1 = value3;
        mass2 = value4;
        next_frame().await;

    }
}
