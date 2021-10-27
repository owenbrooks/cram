use ndarray::prelude::*;
// use ndarray::{stack};

pub fn angle_to_rmat(theta: f32) -> Array2<f32> {
	let c = theta.cos();
	let s = theta.sin();
	array![	[c, -s], 
			[s, c]]
}

// pub fn trans_and_rmat_to_tmat(rmat: Array2<f32>, trans: Array1<f32>) {
// 	let mut t = Array::zeros((rmat.nrows(), rmat.ncols()));
// 	t[[..rmat.nrows(), ..rmat.ncols()]].assign(&rmat);
	
// 	println!("{:?}", t);
// }