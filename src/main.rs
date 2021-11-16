use cram::{diff_drive, draw, icp, lidar, pose_graph, transforms};
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
    prev_scan: Vec<Point2>,
    show_ground_truth: bool,
    robot: diff_drive::Robot,
    mouse_is_lidar: bool,
    pose_graph: pose_graph::PoseGraph,
    scan_continuously: bool,
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

    let pose_graph = pose_graph::PoseGraph {
        nodes: vec![],
        edges: vec![],
    };

    Model {
        environment,
        mouse_pos: pt2(0.0, 0.0),
        texture,
        scan: vec![],
        prev_scan: vec![],
        show_ground_truth: false,
        robot: diff_drive::Robot::new(0.3),
        pose_graph,
        mouse_is_lidar: false,
        scan_continuously: false,
    }
}

fn update(_app: &App, model: &mut Model, _update: Update) {
    model.robot.step(0.167);
    let robot_coords = if model.mouse_is_lidar {
        model.mouse_pos
    } else {
        pt2(
            M2PIXEL * model.robot.state.pose.x,
            M2PIXEL * model.robot.state.pose.y,
        )
    };
    let coords = lidar::point_to_pixel_coords(robot_coords, model.environment.dimensions);
    if coords.is_some() {
        model.scan = lidar::scan_from_point(model.robot.state.pose, &model.environment, M2PIXEL);
    }

    if model.scan_continuously {
        if model.prev_scan.len() > 0 {
            let pose_est = icp::estimate_pose(&model.scan, &model.prev_scan, &model.pose_graph);
            model.pose_graph.add_measurement(pose_est);
        } else {
            model.pose_graph.add_measurement(model.robot.state.pose); // use ground truth pose for initial estimate only
        }
        model.prev_scan = model.scan.clone();
    }
}

fn event(app: &App, model: &mut Model, event: WindowEvent) {
    match event {
        WindowEvent::MouseMoved(pos) => {
            model.mouse_pos = pos;
        }
        KeyPressed(Key::M) => model.show_ground_truth = !model.show_ground_truth,
        KeyPressed(Key::L) => model.mouse_is_lidar = !model.mouse_is_lidar,
        KeyPressed(Key::R) => {
            // reset pose graph
            model.pose_graph.nodes = vec![];
            model.pose_graph.edges = vec![];
            model.prev_scan = vec![];
        }
        KeyPressed(Key::C) => model.scan_continuously = !model.scan_continuously,
        // Take measurement with space bar
        KeyPressed(Key::Space) => {
            if model.prev_scan.len() > 0 {
                let pose_est = icp::estimate_pose(&model.scan, &model.prev_scan, &model.pose_graph);
                model.pose_graph.add_measurement(pose_est);
                println!(
                    "Actual tf {:?}",
                    transforms::pose_to_trans(model.robot.state.pose)
                );
            } else {
                model.pose_graph.add_measurement(model.robot.state.pose); // use ground truth pose for initial estimate only
            }
            model.prev_scan = model.scan.clone();
        }
        // Robot movement with arrow keys
        KeyPressed(Key::Right) => model.robot.set_command(diff_drive::RobotCommand::TurnRight),
        KeyPressed(Key::Left) => model.robot.set_command(diff_drive::RobotCommand::TurnLeft),
        KeyPressed(Key::Up) => model.robot.set_command(diff_drive::RobotCommand::Forward),
        KeyPressed(Key::Down) => model.robot.set_command(diff_drive::RobotCommand::Back),
        KeyReleased(key) => match key {
            Key::Right | Key::Left => {
                if app.keys.down.contains(&Key::Up) {
                    model.robot.set_command(diff_drive::RobotCommand::Forward);
                } else if app.keys.down.contains(&Key::Down) {
                    model.robot.set_command(diff_drive::RobotCommand::Back);
                } else {
                    model.robot.set_command(diff_drive::RobotCommand::Stop);
                }
            }
            Key::Up | Key::Down => {
                if app.keys.down.contains(&Key::Right) {
                    model.robot.set_command(diff_drive::RobotCommand::TurnRight);
                } else if app.keys.down.contains(&Key::Left) {
                    model.robot.set_command(diff_drive::RobotCommand::TurnLeft);
                } else {
                    model.robot.set_command(diff_drive::RobotCommand::Stop);
                }
            }
            _other => (),
        },
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

    // Display the current and previous scan points
    draw::draw_scan_points(
        &model.scan,
        &draw,
        M2PIXEL,
        nannou::color::RED,
        model.robot.state.pose,
    );
    if let Some(last_pose) = &model.pose_graph.nodes.last() {
        draw::draw_scan_points(
            &model.prev_scan,
            &draw,
            M2PIXEL,
            nannou::color::WHITE,
            transforms::trans_to_pose(last_pose),
        );
    }
    draw::draw_scan_points(
        &model.scan,
        &draw,
        M2PIXEL,
        nannou::color::GREEN,
        diff_drive::Pose {
            x: 0.,
            y: 0.,
            theta: 0.,
        },
    );

    // Display the current robot state
    if !model.mouse_is_lidar {
        draw::draw_pose(model.robot.state.pose, &draw, M2PIXEL, Rgba8::from(ORANGE));
    }

    // Display the pose graph
    draw::draw_pose_graph(&model.pose_graph, &draw, M2PIXEL);

    draw.to_frame(app, &frame).unwrap();
}
