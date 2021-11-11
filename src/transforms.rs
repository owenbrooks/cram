#![allow(clippy::many_single_char_names)]
use ndarray::prelude::*;
use crate::diff_drive::Pose;

pub fn angle_to_rmat(theta: f64) -> Array2<f64> {
    let c = theta.cos();
    let s = theta.sin();
    array![[c, -s], [s, c]]
}

pub fn rmat_and_tvec_to_tmat(rmat: &Array2<f64>, tvec: &Array1<f64>) -> Array2<f64> {
    let mut t = Array::zeros((rmat.nrows() + 1, rmat.ncols() + 1));
    t.slice_mut(s![..rmat.nrows(), ..rmat.ncols()])
        .assign(&rmat);
    t.slice_mut(s![..tvec.len(), rmat.nrows()]).assign(&tvec);
    t[[rmat.nrows(), rmat.ncols()]] = 1.;
    t
}

pub fn transformed_cloud(cloud_ref: &Array2<f64>, tmat: &Array2<f64>) -> Array2<f64> {
    let mut cloud_target = Array2::zeros((cloud_ref.nrows(), cloud_ref.ncols()));
    for (i, mut row) in cloud_target.axis_iter_mut(Axis(0)).enumerate() {
        let pt = cloud_ref.slice(s![i, ..]);
        let new_pt = tmat.dot(&pt);
        row.assign(&new_pt);
    }
    cloud_target
}

// Converts 3x3 2D transformation matrix to x,y,theta pose representation
pub fn trans_to_pose(tf_matrix: &Array2<f64>) -> Pose {
    let theta = -tf_matrix[[0, 1]].atan2(tf_matrix[[0, 0]]) as f32;
    let x = tf_matrix[[0, 2]] as f32;
    let y = tf_matrix[[1, 2]] as f32;
    Pose {x, y, theta}
}

pub fn pose_to_trans(pose: Pose) -> Array2<f64> {
    array![
        [pose.theta.cos() as f64, -pose.theta.sin() as f64, pose.x as f64],
        [pose.theta.sin() as f64, pose.theta.cos() as f64, pose.y as f64],
        [0., 0., 1.],
    ]
}