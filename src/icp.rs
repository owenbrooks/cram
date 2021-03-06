#![allow(clippy::many_single_char_names)]
use crate::transforms::*;
use approx::abs_diff_eq;
use kdtree::distance::squared_euclidean;
use kdtree::KdTree;
use ndarray::prelude::*;
use ndarray_linalg::{solve::Determinant, svd::*};

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
    from: &Array2<f64>,
    to: &Array2<f64>,
    _correspondences: &Array1<usize>,
) -> Array2<f64> {
    let p = from
        .slice(s![.., ..from.ncols() - 1])
        .mean_axis(Axis(0))
        .unwrap();
    let p_dash = to
        .slice(s![.., ..to.ncols() - 1])
        .mean_axis(Axis(0))
        .unwrap();

    let qi = from.slice(s![.., ..from.ncols() - 1]).to_owned() - &p;
    let qi_dash = to.slice(s![.., ..to.ncols() - 1]).to_owned() - &p;

    let h = qi.t().dot(&qi_dash);
    let svd = h.svd(true, true).unwrap();
    let u = svd.0.unwrap();
    let vt = svd.2.unwrap();
    let v = vt.t();
    let x = v.dot(&u.t());
    let det_x = x.det().unwrap();

    let rmat = if abs_diff_eq!(det_x, 1., epsilon = 0.00001) {
        x
    } else {
        println!("{}", det_x);
        panic!("Algorithm failed since det(x) != +1");
    }; // TODO: solve the non-1 case
    let t = p_dash - rmat.dot(&p);

    rmat_and_tvec_to_tmat(&rmat, &t)
}
