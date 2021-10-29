use ndarray::prelude::*;
use kdtree::KdTree;
use kdtree::distance::squared_euclidean;
use optimization_engine::{SolverError, Problem, panoc::{PANOCCache, PANOCOptimizer}, constraints};
use optimization_engine::*;
use crate::transforms::*;

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

fn sum_squared_distance(reference: &Array2<f64>, target: &Array2<f64>, correspondences: &Array1<usize>, transform: &Array2<f64>) -> f64 {
	let mut sum = 0.;
	for i in 0..correspondences.len() {
		let p = target.row(i).to_owned();
		let to = correspondences[i];
		let q = reference.row(to);
		let tq = transform.dot(&q);
		let diff = p - tq;
		let cost = diff.dot(&diff);
		sum += cost;
	}
	sum
}
fn sum_squared_distance_grad(reference: &Array2<f64>, target: &Array2<f64>, correspondences: &Array1<usize>, transform: &Array2<f64>, grad: &mut [f64]) {
	// println!("{:?}", transform);
	for i in 0..correspondences.len() {
		let p = target.row(i).to_owned();
		let to = correspondences[i];
		let q = reference.row(to);
		let tq = transform.dot(&q);
		let diff = p - tq;
		let sinw = transform[[0, 1]];
		let cosw = transform[[0, 0]];
		let dtdu = array![[-sinw, -cosw, 1.], [cosw, -sinw, 1.], [0., 0., 0.]];
		let elem = -2.0*(diff)*dtdu.dot(&q);

		grad[0] += elem[0];
		grad[1] += elem[1];
		grad[2] += elem[2];
	}
}

// performs one iteration of ICP algorithm to find transform from target to reference
pub fn find_transform(reference: &Array2<f64>, target: &Array2<f64>, correspondences: &Array1<usize>) -> Array2<f64> {
	/* USER PARAMETERS */
    let tolerance = 1e-14;
    let n = 3;
    let lbfgs_memory = 10;
    let max_iters = 80;
    let mut u = [0., 0., 0.]; // initial guess
	
	let f = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
		let w = u[0];
		let t = array![u[1], u[2]];
		let rmat = angle_to_rmat(w);
		let tmat = rmat_and_tvec_to_tmat(&rmat, &t);
		*c = sum_squared_distance(reference, target, correspondences, &tmat);
		Ok(())
	};

	let df = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
		let w = u[0];
		let t = array![u[1], u[2]];
		let rmat = angle_to_rmat(w);
		let tmat = rmat_and_tvec_to_tmat(&rmat, &t);
        sum_squared_distance_grad(reference, target, correspondences, &tmat, grad);
        Ok(())
    };

	// define the constraints
    let bounds = constraints::NoConstraints::new();

    /* PROBLEM STATEMENT */
    let problem = Problem::new(&bounds, df, f);
    let mut panoc_cache = PANOCCache::new(n, tolerance, lbfgs_memory);
    let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache)
            .with_max_iter(max_iters);

    // Invoke the solver
    let status = panoc.solve(&mut u);

	println!("{:?}", status);

	let w = u[0];
	let t = array![u[1], u[2]];
	let rmat = angle_to_rmat(w);
	let tmat = rmat_and_tvec_to_tmat(&rmat, &t);

	tmat
}