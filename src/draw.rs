use crate::{diff_drive, pose_graph::PoseGraph};
use nannou::prelude::*;

pub fn draw_pose(pose: diff_drive::Pose, draw: &Draw, m2pixel: f32, color: nannou::color::Rgb8) {
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
    for node in &pose_graph.nodes {
        let theta = -node[[0, 1]].atan2(node[[0, 0]]);
        let x = node[[0, 2]];
        let y = node[[1, 2]];
		let pose = diff_drive::Pose {x, y, theta};
		draw_pose(pose, draw, m2pixel, nannou::color::PURPLE);
    }
}
