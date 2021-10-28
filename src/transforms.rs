use ndarray::prelude::*;
// use ndarray::{stack};

pub fn angle_to_rmat(theta: f32) -> Array2<f32> {
	let c = theta.cos();
	let s = theta.sin();
	array![[c, -s], 
		   [s, c]]
}

pub fn rmat_and_tvec_to_tmat(rmat: Array2<f32>, tvec: Array1<f32>) -> Array2<f32> {
	let mut t = Array::zeros((rmat.nrows()+1, rmat.ncols()+1));
	t.slice_mut(s![..rmat.nrows(), ..rmat.ncols()]).assign(&rmat);
	t.slice_mut(s![..tvec.len(), rmat.nrows()]).assign(&tvec);
	t[[rmat.nrows(), rmat.ncols()]] = 1.;
	t
}