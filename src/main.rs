use ndarray::prelude::*;
use ndarray::stack;
use nannou::prelude::*;
use cram::icp::nearest_neighbours;
fn main() {
    nannou::app(model).update(update).run();
}

struct Model {
    mouse_pos: Vec2,
    cloud_ref: Array2<f32>,
    cloud_target: Array2<f32>,
    correspondences: Array1<usize>,
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
    let cloud_ref = stack![Axis(1), x, y, h];

    let rmat = cram::transforms::angle_to_rmat(std::f32::consts::FRAC_PI_6);
    let trans = array![2., 0.];
    let t = array![[rmat[[0,0]], rmat[[0,1]], trans[0]], 
                    [rmat[[1,0]], rmat[[1,1]], trans[1]],
                    [0., 0., 1.],
                ];

    let mut cloud_target = Array2::zeros((cloud_ref.nrows(), cloud_ref.ncols()));
    for (i, mut row) in cloud_target.axis_iter_mut(Axis(0)).enumerate() {
        let pt = cloud_ref.slice(s![i, ..]);
        let new_pt = t.dot(&pt);
        row.assign(&new_pt);
    }

    let correspondences = nearest_neighbours(&cloud_ref, &cloud_target);

    Model {
        mouse_pos: pt2(0.0, 0.0),
        cloud_ref,
        cloud_target, 
        correspondences,
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
    for row in model.cloud_ref.outer_iter() {
        let x = row[0]*m2pixel;
        let y = row[1]*m2pixel;
        draw.ellipse().x_y(x, y).radius(3.0).color(nannou::color::BLACK);
    }
    for row in model.cloud_target.outer_iter() {
        let x = row[0]*m2pixel;
        let y = row[1]*m2pixel;
        draw.ellipse().x_y(x, y).radius(3.0).color(nannou::color::MEDIUMSLATEBLUE);
    }

    for to in 0..model.correspondences.len() {
        let from = model.correspondences[to];
        let new_pt = model.cloud_ref.index_axis(Axis(0), from);
        let ref_pt = model.cloud_target.index_axis(Axis(0), to);

        let new_pt = vec2(new_pt[0], new_pt[1])*m2pixel;
        let ref_pt = vec2(ref_pt[0], ref_pt[1])*m2pixel;

        draw.line().points(new_pt, ref_pt).color(nannou::color::LIME);
    }

    draw.to_frame(app, &frame).unwrap();
}