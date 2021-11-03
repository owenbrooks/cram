use cram::{diff_drive, lidar};
use nannou::image::io::Reader as ImageReader;
use nannou::prelude::*;
use ndarray::prelude::*;
fn main() {
    nannou::app(model).update(update).run();
}

struct Model {
    environment: lidar::Environment,
    mouse_pos: Vec2,
    texture: wgpu::Texture,
    scan: Vec<Point2>,
    show_ground_truth: bool,
    robot: diff_drive::Robot,
}

const M2PIXEL: f32 = 100.0;

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

    let binary_pixels = img
        .pixels()
        .map(|pixel| pixel[0] < 40 && pixel[1] < 40 && pixel[2] < 40)
        .collect();
    let grid = Array::from_shape_vec((img.height() as usize, img.width() as usize), binary_pixels)
        .unwrap();
    println!("{:?}", grid.shape());

    let environment_dims = lidar::Dimension {
        width: grid.shape()[1],
        height: grid.shape()[0],
    };

    let environment = lidar::Environment {
        grid,
        dimensions: environment_dims,
    };

    Model {
        environment,
        mouse_pos: pt2(0.0, 0.0),
        texture,
        scan: vec![],
        show_ground_truth: false,
        robot: diff_drive::Robot::new(0.3, 0.2),
    }
}

fn update(_app: &App, model: &mut Model, _update: Update) {
    model.robot.step(0.167);
    let robot_coords = pt2(M2PIXEL*model.robot.state.pose.x, M2PIXEL*model.robot.state.pose.y);
    let coords = lidar::point_to_pixel_coords(robot_coords, model.environment.dimensions);
    if coords.is_some() {
        model.scan = lidar::scan_from_point(robot_coords, &model.environment);
    }
}

fn event(app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        WindowEvent::MouseMoved(pos) => {
            model.mouse_pos = pos;
        }
        KeyPressed(Key::M) => model.show_ground_truth = !model.show_ground_truth,
        KeyPressed(Key::Right) => model.robot.set_command(diff_drive::RobotCommand::TurnRight),
        KeyPressed(Key::Left) => model.robot.set_command(diff_drive::RobotCommand::TurnLeft),
        KeyPressed(Key::Up) => model.robot.set_command(diff_drive::RobotCommand::Forward),
        KeyPressed(Key::Down) => model.robot.set_command(diff_drive::RobotCommand::Back),
        KeyReleased(key) => {
            match key {
                Key::Right | Key::Left => {
                    if app.keys.down.contains(&Key::Up) {
                        model.robot.set_command(diff_drive::RobotCommand::Forward);
                    } else if app.keys.down.contains(&Key::Down) {
                        model.robot.set_command(diff_drive::RobotCommand::Back);
                    } else {
                        model.robot.set_command(diff_drive::RobotCommand::Stop);
                    }
                },
                Key::Up | Key::Down => {
                    if app.keys.down.contains(&Key::Right) {
                        model.robot.set_command(diff_drive::RobotCommand::TurnRight);
                    } else if app.keys.down.contains(&Key::Left) {
                        model.robot.set_command(diff_drive::RobotCommand::TurnLeft);
                    } else {
                        model.robot.set_command(diff_drive::RobotCommand::Stop);
                    }
                },
                _other => (),
            }
        }
        _other => (),
    }
}

fn view(app: &App, model: &Model, frame: Frame) {
    // Prepare to draw.
    let draw = app.draw();

    // Show or hide ground truth background
    if model.show_ground_truth {
        draw.background().color(WHITE);
        draw.texture(&model.texture);
    } else {
        draw.background().color(BLACK);
    }

    // Display the current scan points
    let scan_point_radius = 1.;
    for pt in &model.scan {
        draw.ellipse()
            .x_y(pt.x, pt.y)
            .radius(scan_point_radius)
            .color(nannou::color::RED);
    }

    // Display the current robot state
    // Circle for position
    let robot_pose = model.robot.state.pose;
    draw.ellipse()
        .x_y(M2PIXEL*robot_pose.x, M2PIXEL*robot_pose.y)
        .radius(5.)
        .color(nannou::color::ORANGE);
    // Line for orientation
    let line_length = 15.;
    let start = pt2(M2PIXEL*robot_pose.x, M2PIXEL*robot_pose.y);
    let end = start
        + pt2(
            line_length * robot_pose.theta.cos(),
            line_length * robot_pose.theta.sin(),
        );
    draw.line()
        .start(start)
        .end(end)
        .color(nannou::color::ORANGE);

    draw.to_frame(app, &frame).unwrap();
}
