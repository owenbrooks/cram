use nannou::image::io::Reader as ImageReader;
use nannou::prelude::*;
use ndarray::prelude::*;
use cram::lidar;
fn main() {
    nannou::app(model).update(update).run();
}

struct Model {
    environment: lidar::Environment,
    mouse_pos: Vec2,
    texture: wgpu::Texture,
    scan: Vec<Point2>,
    show_ground_truth: bool,
}

fn model(app: &App) -> Model {
    app.new_window()
        .title("Cram")
        .size(1200 + 20, 567 + 20)
        .event(event)
        .view(view)
        .build()
        .unwrap();

    let assets = app.assets_path().unwrap();
    let img_path = assets.join("maps").join("floor.jpg");
    let texture = wgpu::Texture::from_path(app, img_path.clone()).unwrap();
    let img = ImageReader::open(img_path)
        .unwrap()
        .decode()
        .unwrap()
        .to_rgb8();

    let binary_pixels = img.pixels().map(|pixel| pixel[0] < 40 && pixel[1] < 40 && pixel[2] < 40).collect();
    let grid = Array::from_shape_vec((img.height() as usize, img.width() as usize), binary_pixels).unwrap();
    println!("{:?}", grid.shape());

    let environment_dims = lidar::Dimension{width: grid.shape()[1], height: grid.shape()[0]};

    let environment = lidar::Environment{
        grid,
        dimensions: environment_dims,
    };

    Model {
        environment,
        mouse_pos: pt2(0.0, 0.0),
        texture,
        scan: vec![],
        show_ground_truth: false,
    }
}

fn update(_app: &App, model: &mut Model, _update: Update) {
    // println!("{:?}", model.mouse_pos);
    let coords = lidar::point_to_pixel_coords(model.mouse_pos, model.environment.dimensions);
    if coords.is_some() {
        model.scan = lidar::scan_from_point(model.mouse_pos, &model.environment);
    }
}

fn event(_app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        WindowEvent::MouseMoved(pos) => {
            model.mouse_pos = pos;
        },
        KeyPressed(Key::M) => model.show_ground_truth = !model.show_ground_truth,
        _other => (),
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    // Prepare to draw.
    let draw = app.draw();

    if model.show_ground_truth {
        draw.background().color(WHITE);
        draw.texture(&model.texture);
    } else {
        draw.background().color(BLACK);
    }

    // Display the current scan points
    let scan_point_radius = 1.;
    for pt in &model.scan {
        draw.ellipse().x_y(pt.x, pt.y).radius(scan_point_radius).color(nannou::color::RED);
    }

    draw.to_frame(app, &frame).unwrap();
}
