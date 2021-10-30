use ndarray::prelude::*;
use ndarray::stack;
use ndarray_linalg::Inverse;
use nannou::prelude::*;
use cram::icp::nearest_neighbours;

fn main() {
    nannou::app(model).update(update).run();
}

struct Model {
    mouse_pos: Vec2,
    cloud_ref: Array2<f64>,
    cloud_target: Array2<f64>,
    computed_transform: Array2<f64>,
    correspondences: Array1<usize>,
    should_step: bool,
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

    let rmat = cram::transforms::angle_to_rmat(std::f64::consts::FRAC_PI_6);
    let tvec = array![2., 0.];
    let tmat = cram::transforms::rmat_and_tvec_to_tmat(&rmat, &tvec);

    let noise = x.map(|x| 1.2*(100.*x).sin() + x.sin());
    let noisy_ref = stack![Axis(1), x, noise, h];
    let cloud_target = cram::transforms::transformed_cloud(&noisy_ref, &tmat);

    let correspondences = nearest_neighbours(&cloud_ref, &cloud_target);

    Model {
        mouse_pos: pt2(0.0, 0.0),
        cloud_ref,
        cloud_target, 
        computed_transform: Array::eye(tvec.len()+1), // will eventually bring ref to target
        correspondences,
        should_step: false,
    }
}

fn update(_app: &App, model: &mut Model, _update: Update) {
    if model.should_step {
        model.should_step = false;
        // Perform one ICP iteration and update transform
        let transformed_ref = cram::transforms::transformed_cloud(&model.cloud_ref, &model.computed_transform);
        let new_transform = cram::icp::find_transform(&transformed_ref, &model.cloud_target, &model.correspondences);
        model.computed_transform = new_transform.dot(&model.computed_transform);

        println!("transform: {:?}", model.computed_transform);
        // Update correspondences
        let inv_transform = model.computed_transform.inv().unwrap();
        let transformed_target = cram::transforms::transformed_cloud(&model.cloud_target, &inv_transform);
        model.correspondences = nearest_neighbours(&model.cloud_ref, &transformed_target);
    }
}

fn event(_app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        WindowEvent::MouseMoved(pos) => {
            model.mouse_pos = pos;
        },
        KeyPressed(Key::Right) => model.should_step = true,
        _other => (),
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(WHITE);

    let m2pixel: f64 = 100.0;

    // Draw point clouds
    for row in model.cloud_ref.outer_iter() {
        let x = row[0]*m2pixel;
        let y = row[1]*m2pixel;
        draw.ellipse().x_y(x as f32, y as f32).radius(3.0).color(nannou::color::BLACK);
    }

    let inv_transform = model.computed_transform.inv().unwrap();
    let transformed_target = cram::transforms::transformed_cloud(&model.cloud_target, &inv_transform);
    for row in transformed_target.outer_iter() {
        let x = row[0]*m2pixel;
        let y = row[1]*m2pixel;
        draw.ellipse().x_y(x as f32, y as f32).radius(3.0).color(nannou::color::MEDIUMSLATEBLUE);
    }

    for row in model.cloud_target.outer_iter() {
        let x = row[0]*m2pixel;
        let y = row[1]*m2pixel;
        draw.ellipse().x_y(x as f32, y as f32).radius(3.0).color(nannou::color::RED);
    }

    // Draw lines to indicate correspondences
    for to in 0..model.correspondences.len() {
        let from = model.correspondences[to];
        let new_pt = model.cloud_ref.index_axis(Axis(0), from);
        let ref_pt = transformed_target.index_axis(Axis(0), to);

        let new_pt = vec2(new_pt[0] as f32, new_pt[1] as f32)*m2pixel as f32;
        let ref_pt = vec2(ref_pt[0] as f32, ref_pt[1] as f32)*m2pixel as f32;

        draw.line().points(new_pt, ref_pt).color(nannou::color::LIME);
    }

    draw.to_frame(app, &frame).unwrap();
}