use ndarray::prelude::*;
// use ndarray::{stack};

pub fn angle_to_rmat(theta: f64) -> Array2<f64> {
	let c = theta.cos();
	let s = theta.sin();
	array![[c, -s], 
		   [s, c]]
}

pub fn rmat_and_tvec_to_tmat(rmat: &Array2<f64>, tvec: &Array1<f64>) -> Array2<f64> {
	let mut t = Array::zeros((rmat.nrows()+1, rmat.ncols()+1));
	t.slice_mut(s![..rmat.nrows(), ..rmat.ncols()]).assign(&rmat);
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