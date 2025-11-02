use macroquad::prelude::*;

#[macroquad::main("pendulum")]
async fn main() {
    loop {
        clear_background(BLUE);
        draw_circle(400.0, 300.0, 10.0, RED);
        next_frame().await;
    }
}
