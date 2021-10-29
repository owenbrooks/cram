use ndarray::prelude::*;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use crate::transforms::*;
use ndarray_linalg::{solve::Determinant, Inverse, svd::*};

// input: two point clouds, reference (dimension mxn) and new (dimension wxn)
// output: vector of indices of the points in reference that are closest to the points in new, length w
pub fn nearest_neighbours(reference: &Array2<f64>, new: &Array2<f64>) -> Array1<usize> {
	let dimensions = reference.ncols() - 1;
	let mut kdtree = KdTree::new(dimensions);
	for i in 0..reference.nrows() {
		kdtree.add([reference[[i, 0]], reference[[i, 1]]], i).unwrap();
	}
	let mut result = Vec::new();
	for i in 0..new.nrows() {
		let nearest = kdtree.nearest(&[new[[i, 0]], new[[i, 1]]], i, &squared_euclidean);
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
pub fn find_transform(reference: &Array2<f64>, target: &Array2<f64>, correspondences: &Array1<usize>) -> Array2<f64> {
	let p = reference.slice(s![.., ..reference.ncols()-1]).mean_axis(Axis(0)).unwrap();
	println!("p {:?}", p);
	println!("{:?}", target);
	let p_dash = target.slice(s![.., ..target.ncols()-1]).mean_axis(Axis(0)).unwrap();

	let qi = reference.slice(s![.., ..reference.ncols()-1]).to_owned() - &p;
	let qi_dash = target.slice(s![.., ..target.ncols()-1]).to_owned() - &p;
	println!("qi {:?}", qi);
	
	let h = qi.t().dot(&qi_dash);
	println!("h {:?}", h);
	let svd = h.svd(true, true).unwrap();
	let u = svd.0.unwrap();
	let vt = svd.2.unwrap();
	println!("u {:?}", u);
	let v = vt.t();
	let x = v.dot(&u.t());
	let det_x = x.det().unwrap();
	println!("x {:?}", x);

	let rmat = if det_x == 1. {x} else {x.inv().unwrap()}; // TODO: solve the non-1 case
	let t =  rmat.dot(&p) - p_dash;
	
	let tmat = rmat_and_tvec_to_tmat(&rmat, &t);

	tmat
}