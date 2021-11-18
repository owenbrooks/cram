use cram::icp::nearest_neighbours;
use nannou::prelude::*;
use ndarray::prelude::*;
use ndarray::stack;
use ndarray_linalg::Inverse;

fn main() {
    nannou::app(model).update(update).run();
}

struct Model {
    mouse_pos: Vec2,
    cloud_ref: Array2<f64>,
    cloud_target: Array2<f64>,
    computed_transform: Array2<f64>,
    orig_correspondences: Array1<usize>,
    tf_correspondences: Array1<usize>,
    show_transformed: bool,
    show_correspondences: bool,
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

    let noise = x.map(|x| 1.2 * (100. * x).sin() + x.sin());
    let noisy_ref = stack![Axis(1), x, noise, h];
    let cloud_target = cram::transforms::transformed_cloud(&noisy_ref, &tmat);

    let correspondences = nearest_neighbours(&cloud_ref, &cloud_target);

    // Perform one ICP iteration and update transform
    let computed_transform = cram::icp::find_transform(&cloud_ref, &cloud_target, &correspondences);

    println!("transform: {:?}", computed_transform);
    // Update correspondences
    let inv_transform = computed_transform.inv().unwrap();
    let transformed_target = cram::transforms::transformed_cloud(&cloud_target, &inv_transform);
    let tf_correspondences = nearest_neighbours(&cloud_ref, &transformed_target);
    let orig_correspondences = nearest_neighbours(&cloud_ref, &cloud_target);

    Model {
        mouse_pos: pt2(0.0, 0.0),
        cloud_ref,
        cloud_target,
        computed_transform: computed_transform, // will eventually bring ref to target
        orig_correspondences,
        tf_correspondences,
        show_transformed: false,
        show_correspondences: true,
    }
}

fn update(_app: &App, _model: &mut Model, _update: Update) {}

fn event(_app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        WindowEvent::MouseMoved(pos) => {
            model.mouse_pos = pos;
        }
        KeyPressed(Key::Right) => model.show_transformed = true,
        KeyPressed(Key::Left) => model.show_transformed = false,
        KeyPressed(Key::H) => model.show_correspondences = !model.show_correspondences,
        _other => (),
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    let draw = app.draw();
    draw.background().color(WHITE);

    let pixels_per_m: f64 = 100.0;

    // Draw point clouds
    for row in model.cloud_ref.outer_iter() {
        let x = row[0] * pixels_per_m;
        let y = row[1] * pixels_per_m;
        draw.ellipse()
            .x_y(x as f32, y as f32)
            .radius(3.0)
            .color(nannou::color::BLACK);
    }

    let inv_transform = model.computed_transform.inv().unwrap();
    let transformed_target =
        cram::transforms::transformed_cloud(&model.cloud_target, &inv_transform);
    let cloud_to_display = if model.show_transformed {
        for row in transformed_target.outer_iter() {
            let x = row[0] * pixels_per_m;
            let y = row[1] * pixels_per_m;
            draw.ellipse()
                .x_y(x as f32, y as f32)
                .radius(3.0)
                .color(nannou::color::MEDIUMSLATEBLUE);
        }
        &transformed_target
    } else {
        for row in model.cloud_target.outer_iter() {
            let x = row[0] * pixels_per_m;
            let y = row[1] * pixels_per_m;
            draw.ellipse()
                .x_y(x as f32, y as f32)
                .radius(3.0)
                .color(nannou::color::RED);
        }
        &model.cloud_target
    };

    let correspondences_to_display = if model.show_transformed {
        &model.tf_correspondences
    } else {
        &model.orig_correspondences
    };

    if model.show_correspondences {
        // Draw lines to indicate correspondences
        for to in 0..correspondences_to_display.len() {
            let from = correspondences_to_display[to];
            let new_pt = model.cloud_ref.index_axis(Axis(0), from);
            let ref_pt = cloud_to_display.index_axis(Axis(0), to);

            let new_pt = vec2(new_pt[0] as f32, new_pt[1] as f32) * pixels_per_m as f32;
            let ref_pt = vec2(ref_pt[0] as f32, ref_pt[1] as f32) * pixels_per_m as f32;

            draw.line()
                .points(new_pt, ref_pt)
                .color(nannou::color::LIME);
        }
    }

    draw.to_frame(app, &frame).unwrap();
}
