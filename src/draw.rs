use crate::{diff_drive, pose_graph::PoseGraph, transforms};
use nannou::prelude::*;
use ndarray::prelude::*;

pub fn draw_pose(pose: diff_drive::Pose, draw: &Draw, m2pixel: f32, color: nannou::color::Rgba8) {
    // Circle for position
    draw.ellipse()
        .x_y(m2pixel * pose.x, m2pixel * pose.y)
        .radius(5.)
        .color(color);
    // Line for orientation
    let line_length = 15.;
    let start = pt2(m2pixel * pose.x, m2pixel * pose.y);
    let end = start
        + pt2(
            line_length * pose.theta.cos(),
            line_length * pose.theta.sin(),
        );
    draw.line()
        .start(start)
        .end(end)
        .color(color);
}

pub fn draw_pose_graph(pose_graph: &PoseGraph, draw: &Draw, m2pixel: f32) {
    let visible_poses = 10;
    let step: u8 = 255 / visible_poses;
    let alpha_vals = (0..pose_graph.nodes.len()).into_iter().rev().map(|i| {
        if i < visible_poses as usize {
            255 - step*(i as u8)
        } else {
            0
        }
    }).into_iter();
    for (node, alpha) in pose_graph.nodes.iter().zip(alpha_vals) {
        let pose = transforms::trans_to_pose(node);
		draw_pose(pose, draw, m2pixel, rgba(194, 61, 255, alpha));
    }
}

pub fn draw_scan_points(points: &Vec<Point2>, draw: &Draw, m2pixel: f32, color: nannou::color::Rgb8, origin: diff_drive::Pose) {
    let scan_point_radius = 1.;
    for pt in points {
        let tf = transforms::pose_to_trans(origin);
        let homog_pt = array![pt.x as f64, pt.y as f64, 1.];
        let tf_pt = tf.dot(&homog_pt);
        draw.ellipse()
            .x_y(m2pixel * tf_pt[0] as f32, m2pixel * tf_pt[1] as f32)
            .radius(scan_point_radius)
            .color(color);
    }
}
