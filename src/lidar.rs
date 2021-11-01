use nannou::prelude::{Point2, pt2};
use iter_num_tools::{lin_space, arange};

#[derive(Copy, Clone, Debug)]
pub struct Dimension {
    pub width: usize,
    pub height: usize,
}

pub fn point_to_pixel_coords(mouse_pos: Point2, environment_dims: Dimension) -> Option<PixelCoord> {
	let in_bounds_x = mouse_pos.x > -(environment_dims.width as f32 / 2.) && mouse_pos.x < environment_dims.width as f32 / 2.;
	let in_bounds_y = mouse_pos.y > -(environment_dims.height as f32 / 2.) && mouse_pos.y < environment_dims.height as f32 / 2.;
	if !in_bounds_x || !in_bounds_y {
		return None
	}

	let x = (mouse_pos.x + environment_dims.width as f32 / 2.0).floor() as usize;
	let y = (-mouse_pos.y + environment_dims.height as f32 / 2.0).floor() as usize;

	Some(PixelCoord {
		x,
		y,
	})
}

pub fn scan_from_point(origin: Point2, environment: &Environment) -> Vec<Point2> {
	let scan_angles = lin_space(0.0..std::f32::consts::PI*2.0, 128);
	let max_scan_distance = 250.;
	let scan_step = 1.;
	let scan_range = arange(0.0..max_scan_distance, scan_step);

	let mut scan_points = vec![];
	let scan_range: Vec<f32> = scan_range.collect();
	
	for angle in scan_angles {
		for dist in &scan_range {
			let scan_ray = pt2(dist*angle.cos(), dist*angle.sin());
			let scan_point = origin + scan_ray;
			let scan_coords = point_to_pixel_coords(scan_point, environment.dimensions);
			if let Some(scan_coords) = scan_coords {
				let object_detected = environment.grid[[scan_coords.y, scan_coords.x]];
				if object_detected {
					scan_points.push(scan_point);
					break;
				}
			}
		}
	}

	scan_points
}

#[derive(Copy, Clone)]
pub struct PixelCoord {
    pub x: usize,
    pub y: usize,
}

pub struct Environment {
	pub grid: ndarray::Array2<bool>,
	pub dimensions: Dimension,
}
