use crate::diff_drive::Pose;
use ndarray::prelude::*;

pub struct Edge {
    source_id: usize,
    target_id: usize,
    information: Array2<f64>,
}

pub struct PoseGraph {
    pub nodes: Vec<Array2<f64>>, // 3x3 homogeneous pose matrix for 2D slam
    pub edges: Vec<Edge>,
}

impl PoseGraph {
    pub fn add_measurement(&mut self, pose: Pose) {
        if self.nodes.len() == 0 {
            panic!("Ensure the graph is created with at least one node.");
        }

        let pose = array![
            [pose.theta.cos() as f64, -pose.theta.sin() as f64, pose.x as f64],
            [pose.theta.sin() as f64, pose.theta.cos() as f64, pose.y as f64],
            [0., 0., 1.],
        ];
        self.nodes.push(pose);

        let source_id = self.nodes.len() - 1;
        let target_id = source_id + 1;
        let information = array![[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]];

        let edge = Edge {
            source_id,
            target_id,
            information,
        };

		self.edges.push(edge);
    }
}
