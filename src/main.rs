use ndarray::prelude::*;
use ndarray::stack;
use nannou::prelude::*;
fn main() {
    nannou::app(model).update(update).run();
}

struct Model {
    mouse_pos: Vec2,
    clouds: Vec<Array2<f32>>,
}

fn model(app: &App) -> Model {
    app.new_window()
        .title("Cram")
        .event(event)
        .view(view)
        .build()
        .unwrap();

    let x = Array::linspace(-3., 3., 100);
    let y = x.map(|t| t.sin());
    let h = Array::ones(x.len());
    let pc = stack![Axis(1), x, y, h];

    let rmat = cram::transforms::angle_to_rmat(std::f32::consts::FRAC_PI_6);
    let trans = array![2., 0.];
    let t = array![[rmat[[0,0]], rmat[[0,1]], trans[0]], 
                    [rmat[[1,0]], rmat[[1,1]], trans[1]],
                    [0., 0., 1.],
                ];

    let mut pc2 = Array2::zeros((pc.nrows(), pc.ncols()));
    for (i, mut row) in pc2.axis_iter_mut(Axis(0)).enumerate() {
        let pt = pc.slice(s![i, ..]);
        let new_pt = t.dot(&pt);
        row.assign(&new_pt);
    }

    Model {
        mouse_pos: pt2(0.0, 0.0),
        clouds: vec![pc, pc2],
    }
}

fn update(_app: &App, _model: &mut Model, _update: Update) {

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
    let draw = app.draw();
    draw.background().color(WHITE);

    let m2pixel = 100.0;
    for pc in &model.clouds {
        for row in pc.outer_iter() {
            let x = row[0]*m2pixel;
            let y = row[1]*m2pixel;
            draw.ellipse().x_y(x, y).radius(3.0).color(nannou::color::BLACK);
        }
    }

    draw.to_frame(app, &frame).unwrap();
}