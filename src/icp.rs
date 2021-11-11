#![allow(clippy::many_single_char_names)]
use crate::{transforms::*, pose_graph, diff_drive::Pose};
use nannou::geom::Point2;
use approx::abs_diff_eq;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use ndarray::{prelude::*, stack};
use ndarray_linalg::{solve::Determinant, svd::*};
use std::cmp::min;

// input: two point clouds, reference (dimension mxn) and new (dimension wxn)
// output: vector of indices of the points in reference that are closest to the points in new, length w
pub fn nearest_neighbours(reference: &Array2<f64>, new: &Array2<f64>) -> Array1<usize> {
    let dimensions = reference.ncols() - 1;
    let mut kdtree = KdTree::new(dimensions);
    for i in 0..reference.nrows() {
        kdtree
            .add([reference[[i, 0]], reference[[i, 1]]], i)
            .unwrap();
    }
    let mut result = Vec::new();
    for i in 0..new.nrows() {
        let nearest = kdtree.nearest(&[new[[i, 0]], new[[i, 1]]], 1, &squared_euclidean);
        if let Ok(nearest) = nearest {
            if let Some(nearest) = nearest.first() {
                result.push(*nearest.1);
            }
        }
    }
    Array1::from_vec(result)
}

// performs one iteration of ICP algorithm to find transform from target to reference
// uses SVD-based algorithm described in Least-Squares Fitting of Two 3-D Point Sets by K. S. ARUN
pub fn find_transform(
    from: &Array2<f64>, // must be homogenous: x, y, 1
    to: &Array2<f64>, // must be homogenous: x, y, 1
    _correspondences: &Array1<usize>,
) -> Array2<f64> {
    let point_count = min(from.nrows(), to.nrows()); // TODO: use corresponding points rather than hardcoding min point count
    let p = from
        .slice(s![..point_count - 1, ..from.ncols() - 1])
        .mean_axis(Axis(0))
        .unwrap();
    let p_dash = to
        .slice(s![..point_count - 1, ..to.ncols() - 1])
        .mean_axis(Axis(0))
        .unwrap();

    let qi = from.slice(s![..point_count - 1, ..from.ncols() - 1]).to_owned() - &p;
    let qi_dash = to.slice(s![..point_count - 1, ..to.ncols() - 1]).to_owned() - &p;
            
    let h = qi.t().dot(&qi_dash);
    let svd = h.svd(true, true).unwrap();
    let u = svd.0.unwrap();
    let vt = svd.2.unwrap();
    let v = vt.t();
    let x = v.dot(&u.t());
    let det_x = x.det().unwrap();
    println!("v {:?}", v);

    let rmat = if abs_diff_eq!(det_x, 1., epsilon = 0.00001) {
        x
    } else {
        // Det is not +1, so it is -1
        // We take the rotation instead 
        println!("det(x) = {}", det_x);
        let mut v_dash: Array2<f64> = v.to_owned();
        println!("v_dash {:?}", v_dash);
        let neg_col2 = -v_dash.slice(s![.., 1]).to_owned();
        v_dash.slice_mut(s![.., 1]).assign(&neg_col2);
        v_dash.dot(&u.t())
        // TODO check whether the singular values are zero and use RANSAC instead
    };
    let t = p_dash - rmat.dot(&p);

    rmat_and_tvec_to_tmat(&rmat, &t)
}

fn vec_point2_to_homog(orig_vec: &Vec<Point2>) -> Array2<f64> {
    let x = Array::from_iter(orig_vec.into_iter().map(|point| point.x as f64));
    let y = Array::from_iter(orig_vec.into_iter().map(|point| point.y as f64));
    let h = Array::ones(orig_vec.len());
    
    stack![Axis(1), x, y, h]
}

pub fn estimate_pose(new_scan: &Vec<Point2>, prev_scan: &Vec<Point2>, pose_graph: &pose_graph::PoseGraph) -> Pose {
    let new_scan = vec_point2_to_homog(new_scan);
    let prev_scan = vec_point2_to_homog(prev_scan);

    let correspondences = nearest_neighbours(&new_scan, &prev_scan);
    let transform = find_transform(&new_scan, &prev_scan, &correspondences);
    let prev_pose = pose_graph.nodes.last();
    match prev_pose {
        Some(prev_pose) => {
            println!("tf {:?}", transform);
            let new_pose = transform.dot(prev_pose);
            println!("{:?}", trans_to_pose(&new_pose));
            trans_to_pose(&new_pose)
        },
        None => {
            Pose {
                x: 0.,
                y: 0.,
                theta: 0.,
            }
        }
    }
}