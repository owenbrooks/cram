use ndarray::prelude::*;
use nannou::prelude::*;
fn main() {
    println!("Hello, world!");
    let x: Array2<f64> = Array::zeros((3,3));
    println!("{}", x);
    nannou::app(model).update(update).run();
}

struct Model {
    mouse_pos: Vec2,
}

fn model(app: &App) -> Model {
    app.new_window()
        .title("Cram")
        .event(event)
        .view(view)
        .build()
        .unwrap();

    Model {
        mouse_pos: pt2(0.0, 0.0),
    }
}

fn update(_app: &App, model: &mut Model, _update: Update) {

}

fn event(_app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        WindowEvent::MouseMoved(pos) => {
            model.mouse_pos = pos;
        }
        _other => (),
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    // Prepare to draw.
    let draw = app.draw();

    // Clear the background to purple.
    draw.background().color(WHITE);

    draw.to_frame(app, &frame).unwrap();
}