use ndarray::prelude::*;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;

// input: two point clouds, reference (dimension mxn) and new (dimension wxn)
// output: vector of indices of the points in reference that are closest to the points in new, length w
pub fn nearest_neighbours(reference: &Array2<f32>, new: &Array2<f32>) -> Array1<usize> {
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