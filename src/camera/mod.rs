use core::f64;
use opencv as cv;
use opencv::{core::*, videoio::*};

mod ecc;
use ecc::*;

use std::str::FromStr;

use rubikscube::prelude::*;

type CVResult<T> = cv::error::Result<T, cv::error::Error>;
type GenericResult<T> = Result<T, Box<dyn std::error::Error>>;

const WHITE: VecN<f64, 4> = VecN::new(255.0, 255.0, 255.0, 255.0);
const BLACK: VecN<f64, 4> = VecN::new(0.0, 0.0, 0.0, 255.0);

pub const FOV_DEG: f64 = 55.0;
pub const FOV_RAD: f64 = FOV_DEG / 180.0 * f64::consts::PI;

fn pythagoras(a: f64, b: f64) -> f64 {
	(a * a + b * b).sqrt()
}

/// Return the two points from a line
fn get_line_points(line: &Vec4i) -> (Point, Point) {
	(Point::new(line[0], line[1]), Point::new(line[2], line[3]))
}

/// Return the centroids from a vector of Points.
fn centroid(points: &Vec<Point>) -> Point {
	if points.is_empty() {
		return Point::new(-1, -1);
	}

	// Take the average of all x and y values
	let mut x = 0f64;
	let mut y = 0f64;
	for p in points.iter() {
		x += p.x as f64;
		y += p.y as f64;
	}

	let l = points.len() as f64;
	Point::new((x / l) as i32, (y / l) as i32)
}

/// Estimates the grid and constructs the points of the rest of the Rubik's Cube
fn construct_grid(camsize: Size, image_points: Vector<Point>) -> CVResult<Vec<Vec<Vec<Point>>>> {
	let diag = pythagoras(camsize.width as f64, camsize.height as f64);
	let focal_len: f64 = diag / (2.0 * (FOV_RAD / 2.0).tan()); // Calculate the focal length

	// The camera projection matrix
	let camera_matrix = Mat::from_slice_2d(&[
		[focal_len, 0.0, 320.0], // updated cx
		[0.0, focal_len, 240.0], // updated cy
		[0.0, 0.0, 1.0],
	])?;

	// The points in the world coordinate
	let object_points: Vector<Point3d> = vec![
		Point3d::new(0.0, 0.0, 0.0),
		Point3d::new(1.2, 0.0, 0.0),
		Point3d::new(0.0, 1.2, 0.0),
		Point3d::new(0.0, 0.0, 1.2),
	]
	.into();

	// Convert into Point2d
	let init_points: Vector<Point2d> = image_points
		.into_iter()
		.map(|p| {
			let x = p.x;
			let y = p.y;
			Point2d::new(x as f64, y as f64)
		})
		.collect::<Vec<_>>()
		.into();

	let dist_coeffs = Mat::from_slice_2d(&[[0.0], [0.0], [0.0], [0.0]])?;

	let mut rvec = Vector::<f64>::default();
	let mut tvec = Vector::<f64>::default();

	// Solve PnP
	let success = cv::calib3d::solve_pnp(
		&object_points,
		&init_points,
		&camera_matrix,
		&dist_coeffs,
		&mut rvec,
		&mut tvec,
		false,
		cv::calib3d::SOLVEPNP_SQPNP,
	)?;

	let mat_rvec = Mat::from_slice(rvec.as_slice())?;
	let mat_tvec = Mat::from_slice(tvec.as_slice())?;

	if !success {
		return Err(cv::error::Error::new(-1, "No success solving PNP"));
	}

	// Construct the grid of the entire Rubik's Cube
	let mut grid = vec![vec![vec![Point::new(0, 0); 4]; 4]; 4];

	for i in 0..4 {
		for j in 0..4 {
			for k in 0..4 {
				// Get the point in the world coordinate
				let fp: Vector<Point3d> = vec![Point3d::new(
					(i as f64) * 0.4,
					(j as f64) * 0.4,
					(k as f64) * 0.4,
				)]
				.into();

				let mut proj_points = Vector::<Point2d>::default();
				let mut jacobian = Mat::default();

				// Project the point onto the image
				cv::calib3d::project_points(
					&fp,
					&mat_rvec,
					&mat_tvec,
					&camera_matrix,
					&dist_coeffs,
					&mut proj_points,
					&mut jacobian,
					0.0,
				)?;

				let p = proj_points
					.get(0)
					.map_err(|_| cv::Error::new(-1, "Could not project points"))?;

				let x = p.x as i32;
				let y = p.y as i32;
				grid[i][j][k] = Point::new(x, y);
			}
		}
	}

	Ok(grid)
}

/// Convert a Vector<Point> to Vector<Point2f>
fn points_to_point2f(points: Vector<Point>) -> Vector<Point2f> {
	points
		.into_iter()
		.map(|p| {
			let x = p.x;
			let y = p.y;
			Point2f::new(x as f32, y as f32)
		})
		.collect::<Vec<_>>()
		.into()
}

/// Helper cvt_color function
fn cvt_color(src: &mut impl ToInputArray, code: i32) -> CVResult<Mat> {
	let mut dst = Mat::default();
	cv::imgproc::cvt_color(src, &mut dst, code, 0)?;
	Ok(dst)
}

/// Convert a single scalar between colorspaces
fn convert_scalar(scalar: Scalar, code: i32) -> CVResult<Vec3b> {
	let img = Mat::new_size_with_default(Size::new(1, 1), cv::core::CV_8UC3, scalar)?;
	let mut conv = Mat::default();

	cv::imgproc::cvt_color(&img, &mut conv, code, 0)?;

	let scal_conv = *conv.at_2d::<Vec3b>(0, 0)?;
	Ok(scal_conv)
}

/// Helper threshold function
fn threshold(frame: &impl ToInputArray, threshval: f64, typ: i32) -> CVResult<Mat> {
	let mut thresh = Mat::default();
	cv::imgproc::threshold(frame, &mut thresh, threshval, 255.0, typ)?;
	Ok(thresh)
}

/// Reads the colors given the four side points
fn read_side(frame: &mut impl ToInputArray, points: Vector<Point>) -> CVResult<Vec<Vec<Scalar>>> {
	// Length of the perspective transformation. Arbitrarely chosen
	const S: i32 = 128;

	// The corner of the final image
	let dstpts = vec![
		Point::new(S, 0),
		Point::new(0, 0),
		Point::new(0, S),
		Point::new(S, S),
	]
	.into();

	let mut srcp = points_to_point2f(points.clone());
	let mut dstp = points_to_point2f(dstpts);

	// Get transformation matrix
	let mat = cv::imgproc::get_perspective_transform(&mut srcp, &mut dstp, cv::core::DECOMP_LU)?;

	let mut dst = Mat::default();

	// Apply perspective transformation
	cv::imgproc::warp_perspective(
		frame,
		&mut dst,
		&mat,
		Size::new(S, S),
		cv::imgproc::INTER_LINEAR,
		cv::core::BORDER_CONSTANT,
		Scalar::default(),
	)?;

	// Convert to Lab
	let lab = cvt_color(&mut dst, cv::imgproc::COLOR_RGB2Lab)?;

	// Thresholding (values are manually chosen)
	let gray = cvt_color(&mut dst, cv::imgproc::COLOR_RGB2GRAY)?;
	let mask_dark = threshold(&gray, 55.0, cv::imgproc::THRESH_BINARY)?;
	let mask_bright = threshold(&gray, 240.0, cv::imgproc::THRESH_BINARY_INV)?;

	let mut out = vec![vec![Scalar::default(); 3]; 3];

	#[cfg(debug_assertions)]
	let mut outframe = dst.clone();

	// Offset
	const O: i32 = 12;

	for i in 0..3 {
		for j in 0..3 {
			// Position of the area
			let x1 = (S / 3) * i + O;
			let x2 = (S / 3) * (i + 1) - O;
			let y1 = (S / 3) * j + O;
			let y2 = (S / 3) * (j + 1) - O;

			let w = x2 - x1;
			let h = y2 - y1;
			let rect = Rect::new(x1, y1, w, h);

			#[cfg(debug_assertions)]
			cv::imgproc::rectangle(&mut outframe, rect, BLACK, 2, 0, 0)?;

			// Only look a the are inside of the rect
			let sliced_dst = Mat::roi(&lab, rect)?;
			let sliced_mask_d = Mat::roi(&mask_dark, rect)?;
			let sliced_mask_b = Mat::roi(&mask_bright, rect)?;

			// Calculate masked to non masked ratio
			let rat = {
				let tot = w * h;
				let cnt = cv::core::count_non_zero(&sliced_mask_b)?;
				cnt as f64 / tot as f64
			};

			if rat < 0.20 {
				// if more than 80% filtered out, it's considered white
				let scal = convert_scalar(WHITE, cv::imgproc::COLOR_RGB2Lab)?;
				out[i as usize][j as usize] = scal.into();
			} else {
				// Maske the image
				let masked = {
					let mut masked = Mat::default();
					cv::core::bitwise_and(
						&sliced_mask_b,
						&sliced_mask_d,
						&mut masked,
						&cv::core::no_array(),
					)?;
					masked
				};

				// Read the color
				let scalar = cv::core::mean(&sliced_dst, &masked)?;
				out[i as usize][j as usize] = scalar;
			}
		}
	}

	#[cfg(debug_assertions)]
	{
		let mut masked = Mat::default();
		cv::core::copy_to(&mut outframe, &mut masked, &mask_bright)?;
		cv::highgui::imshow("masked", &masked)?;
	}

	Ok(out)
}

#[cfg(debug_assertions)]
/// Generates a 2D-image with a[i][j] = (0,i,j)
fn generate_spectrum() -> CVResult<Mat> {
	let mut spectrum = Mat::new_size_with_default(
		Size::new(255, 255),
		CV_8UC3,
		Scalar::new(0.0, 0.0, 0.0, 0.0),
	)?;

	// Iterate over the 255x255 matrix
	for i in 0..255 {
		for j in 0..255 {
			// Access pixel (j, i) and set the BGR values
			let pixel = spectrum.at_2d_mut::<Vec3b>(i, j)?;

			pixel[0] = 255;
			pixel[1] = j as u8;
			pixel[2] = i as u8;
		}
	}

	Ok(spectrum)
}

/// Find out the grid of the cube given 4 estimated points as reference
fn find_out_grid(frame: &mut Mat, points: Vec<Point>) -> CVResult<Vec<Vec<Vec<Point>>>> {
	let edge_thresh1 = 100.0;
	let edge_thresh2 = 200.0;

	let mut filter = Mat::default();
	cv::imgproc::bilateral_filter(frame, &mut filter, 1, 300.0, 300.0, 0)?;

	let mut edges = Mat::default();

	// Apply edge detection
	cv::imgproc::canny(
		&mut filter,
		&mut edges,
		edge_thresh1,
		edge_thresh2,
		3,
		false,
	)?;

	// Apply Probalistic Hough Line Transform
	let lines = {
		let mut lines: Vector<Vec4i> = vec![].into();

		let rho = 1.0;
		let theta = f64::consts::PI / 180.0;
		let numinters = 10;

		cv::imgproc::hough_lines_p(&mut edges, &mut lines, rho, theta, numinters, 50.0, 10.0)?;
		lines
	};

	let mut init_points = points;

	// The distance threshold to be considered a corner
	const DIST: f64 = 20.0;

	// Check if there is a nearer point
	for point in init_points.iter_mut() {
		let mut near = vec![];
		for line in lines.iter() {
			let (p1, p2) = get_line_points(&line);

			let d1 = (*point - p1).norm();
			let d2 = (*point - p2).norm();

			let (p, d) = if d1 < d2 { (p1, d1) } else { (p2, d2) };

			if d < DIST {
				near.push(p);
			}
		}

		if !near.is_empty() {
			*point = centroid(&near);
		}
	}

	let size = frame.size()?;
	construct_grid(size, init_points.into())
}

/// Reads the absolut color values in LAB
fn read_colors(frame: &mut Mat, grid: &Vec<Vec<Vec<Point>>>) -> CVResult<Vec<Vec<Vec<Scalar>>>> {
	// the rectangles covering each grid
	let p1 = vec![grid[3][0][0], grid[0][0][0], grid[0][3][0], grid[3][3][0]].into();
	let p2 = vec![grid[3][0][0], grid[0][0][0], grid[0][0][3], grid[3][0][3]].into();
	let p3 = vec![grid[0][3][0], grid[0][0][0], grid[0][0][3], grid[0][3][3]].into();

	// Read each side
	let grid1 = read_side(frame, p1)?;
	let grid2 = read_side(frame, p2)?;
	let grid3 = read_side(frame, p3)?;

	let cols = vec![grid1, grid2, grid3];

	Ok(cols)
}

/// Draw the grid on a frame
#[cfg(debug_assertions)]
fn draw_grid(frame: &mut impl ToInputOutputArray, grid: &Vec<Vec<Vec<Point>>>) -> CVResult<()> {
	#[cfg(debug_assertions)]
	for i in 0..4 {
		cv::imgproc::line(frame, grid[i][0][0], grid[i][3][0], WHITE, 2, 0, 0)?;
		cv::imgproc::line(frame, grid[i][0][0], grid[i][0][3], WHITE, 2, 0, 0)?;

		cv::imgproc::line(frame, grid[0][i][0], grid[0][i][3], WHITE, 2, 0, 0)?;
		cv::imgproc::line(frame, grid[0][i][0], grid[3][i][0], WHITE, 2, 0, 0)?;

		cv::imgproc::line(frame, grid[0][0][i], grid[0][3][i], WHITE, 2, 0, 0)?;
		cv::imgproc::line(frame, grid[0][0][i], grid[3][0][i], WHITE, 2, 0, 0)?;
	}

	Ok(())
}

/// Camera struct
/// Using Logitech C270 webcams
pub struct Camera {
	cam: VideoCapture,
	rotateflag: i32,
}

impl Camera {
	pub fn try_new(id: i32, rotateflag: i32) -> CVResult<Self> {
		let mut cam = VideoCapture::default()?;
		cam.open(id, CAP_ANY)?;
		let open = cam.is_opened()?;

		if !open {
			return Err(cv::error::Error::new(-1, "Could not open camera!"));
		}

		// opencv4.10 is broken at setting properties but 4.6 works.
		cam.set(cv::videoio::CAP_PROP_BRIGHTNESS, 0.0)?;
		cam.set(cv::videoio::CAP_PROP_SATURATION, 128.0)?;
		cam.set(cv::videoio::CAP_PROP_CONTRAST, 32.0)?;
		cam.set(cv::videoio::CAP_PROP_HUE, 0.0)?;

		// This is crucial to make grabbing work!!!
		cam.set(cv::videoio::CAP_PROP_BUFFERSIZE, 1.0)?;

		println!("BRIGHTNESS: {}", cam.get(cv::videoio::CAP_PROP_BRIGHTNESS)?);
		println!("SATURATION: {}", cam.get(cv::videoio::CAP_PROP_SATURATION)?);
		println!("CONTRAST: {}", cam.get(cv::videoio::CAP_PROP_CONTRAST)?);
		println!("HUE: {}", cam.get(cv::videoio::CAP_PROP_HUE)?);

		let out = Self { cam, rotateflag };
		Ok(out)
	}

	/// Grab the most recent image
	pub fn grab(&mut self) -> bool {
		let _ = self.cam.grab();
		match self.cam.grab() {
			Ok(r) => r,
			Err(_) => false,
		}
	}

	/// Retrieve the grabbed image
	pub fn retrieve(&mut self) -> CVResult<Mat> {
		let mut frame = Mat::default();
		let success = self.cam.retrieve(&mut frame, 0)?;

		if success {
			let mut dst = Mat::default();
			cv::core::rotate(&mut frame, &mut dst, self.rotateflag)?;
			Ok(dst)
		} else {
			Err(cv::error::Error::new(-1, "Could not open camera!"))
		}
	}

	pub fn read(&mut self) -> CVResult<Mat> {
		let mut frame = Mat::default();
		self.cam.read(&mut frame)?;
		let mut dst = Mat::default();
		cv::core::rotate(&mut frame, &mut dst, self.rotateflag)?;
		Ok(dst)
	}

	#[cfg(debug_assertions)]
	pub fn insert_helping_points(&mut self) -> CVResult<Vec<Point>> {
		let points = std::sync::Arc::from(std::sync::Mutex::from(Vec::<Point>::new()));
		let points_clone = std::sync::Arc::clone(&points);

		let handler = move |ev, x, y, _flags| {
			if ev == cv::highgui::EVENT_MBUTTONDOWN {
				if let Ok(mut p) = points_clone.lock() {
					if p.len() < 4 {
						p.push(Point::new(x, y));

						if p.len() == 4 {
							for pp in p.iter() {
								println!("Point::new({}, {}),", pp.x, pp.y);
							}
						}
					}
				}
			}
		};

		cv::highgui::named_window("main", cv::highgui::WINDOW_AUTOSIZE)?;
		cv::highgui::set_mouse_callback("main", Some(Box::new(handler)))?;

		'mainloop: loop {
			let mut frame = self.read()?;

			if let Ok(points_m) = points.lock() {
				if points_m.len() == 4 {
					break 'mainloop;
				}

				for p in points_m.iter() {
					cv::imgproc::circle(&mut frame, *p, 5, BLACK, -1, cv::imgproc::LINE_8, 0)?;
				}
			}

			cv::highgui::imshow("main", &frame)?;

			match cv::highgui::wait_key(1)? as u8 as char {
				'q' => break 'mainloop,
				_ => {}
			}
		}

		let res = points.lock();

		match res {
			Ok(r) => Ok(r.clone()),
			Err(_) => Err(cv::error::Error::new(-1, "Could not get points!")),
		}
	}
}

pub fn calibrate(cam1: &mut Camera, cam2: &mut Camera, data: &mut CalibrationData) -> CVResult<()> {
	let mut cnt = vec![(Scalar::default(), 0usize); 6];

	// Take 50 pictures
	for _ in 0..50 {
		let mut frame1 = cam1.read()?;
		let mut frame2 = cam2.read()?;
		let grid1 = find_out_grid(&mut frame1, data.corners1.clone())?;
		let grid2 = find_out_grid(&mut frame2, data.corners2.clone())?;

		let cols1 = read_colors(&mut frame1, &grid1)?;
		let cols2 = read_colors(&mut frame2, &grid2)?;

		for idx in 0..3 {
			for i in 0..3 {
				for j in 0..3 {
					if i == j && 0 < i {
						continue;
					}

					cnt[idx].0 += cols1[idx][i][j];
					cnt[idx + 3].0 += cols2[idx][i][j];
				}
			}
			cnt[idx].1 += 7;
			cnt[idx + 3].1 += 7;
		}
	}

	// Determine the centroids of all colors
	let out = cnt
		.into_iter()
		.map(|(s, n)| {
			let out = s / (n as f64);
			println!(
				"Scalar::new({}, {}, {}, {}.0),",
				out[0], out[1], out[2], out[3]
			);
			out
		})
		.collect();

	data.centroids = out;

	Ok(())
}

impl Drop for Camera {
	fn drop(&mut self) {
		if let Ok(opened) = self.cam.is_opened() {
			if opened {
				let _ = self.cam.release();
			}
		}
	}
}

/// Distance functions for two scalar
fn distance(s1: Scalar, s2: Scalar) -> f64 {
	let w = s2[1] - s1[1];
	let h = s2[2] - s1[2];
	pythagoras(w, h)
}

/// Return the index of the nearest Scalar to s
fn get_nearest(s: Scalar, vs: &Vec<Scalar>) -> usize {
	let mut best = (f64::INFINITY, 0);
	for (idx, p) in vs.iter().enumerate() {
		let dist = distance(s, *p);
		if dist < best.0 {
			best = (dist, idx);
		}
	}
	best.1
}

/// Prints the color read as a cube
#[cfg(debug_assertions)]
fn print_cvec(vec: &Vec<char>) {
	for row in DISPLAY_GRID {
		for col in row {
			if col < 54 {
				if let Some(side) = Side::from_repr(vec[col] as u8 - b'a') {
					print!("{}▀ ", get_ansii_color(side));
				} else {
					print!("X ");
				}
			} else {
				print!("  ");
			}
		}
		println!();
	}
	// Reset ansii color
	print!("\x1b[00m");
}

/// Reads the absolut color values given the corners and frames
fn read_all_colors(
	frame1: &mut Mat,
	points1: Vec<Point>,
	frame2: &mut Mat,
	points2: Vec<Point>,
) -> CVResult<Vec<Vec<Vec<Scalar>>>> {
	let skel1 = find_out_grid(frame1, points1)?;
	let grid1 = read_colors(frame1, &skel1)?;

	let skel2 = find_out_grid(frame2, points2)?;
	let grid2 = read_colors(frame2, &skel2)?;

	let mut out = vec![];
	for g in grid1 {
		out.push(g);
	}
	for g in grid2 {
		out.push(g);
	}

	Ok(out)
}

/// Groups each color in colors to one of the centroids
fn colors_to_cube(colors: &Vec<Vec<Vec<Scalar>>>, centroids: &Vec<Scalar>) -> Vec<char> {
	// one huge translation table!
	const fn help(side: Side, x: usize, y: usize) -> usize {
		(side as usize) * CUBE_AREA + y * CUBE_DIM + x
	}

	// Convert the index entry of the grids into actual cube permutation indices.
	const IDX: [[[usize; 3]; 3]; 6] = [
		[
			// red (left)
			[
				help(Side::Left, 0, 0),
				help(Side::Left, 0, 1),
				help(Side::Left, 0, 2),
			],
			[
				help(Side::Left, 1, 0),
				help(Side::Left, 1, 1),
				help(Side::Left, 1, 2),
			],
			[
				help(Side::Left, 2, 0),
				help(Side::Left, 2, 1),
				help(Side::Left, 2, 2),
			],
		],
		[
			// white (up)
			[
				help(Side::Up, 0, 0),
				help(Side::Up, 1, 0),
				help(Side::Up, 2, 0),
			],
			[
				help(Side::Up, 0, 1),
				help(Side::Up, 1, 1),
				help(Side::Up, 2, 1),
			],
			[
				help(Side::Up, 0, 2),
				help(Side::Up, 1, 2),
				help(Side::Up, 2, 2),
			],
		],
		[
			// green (back)
			[
				help(Side::Back, 2, 0),
				help(Side::Back, 1, 0),
				help(Side::Back, 0, 0),
			],
			[
				help(Side::Back, 2, 1),
				help(Side::Back, 1, 1),
				help(Side::Back, 0, 1),
			],
			[
				help(Side::Back, 2, 2),
				help(Side::Back, 1, 2),
				help(Side::Back, 0, 2),
			],
		],
		[
			// blue (front)
			[
				help(Side::Front, 2, 2),
				help(Side::Front, 2, 1),
				help(Side::Front, 2, 0),
			],
			[
				help(Side::Front, 1, 2),
				help(Side::Front, 1, 1),
				help(Side::Front, 1, 0),
			],
			[
				help(Side::Front, 0, 2),
				help(Side::Front, 0, 1),
				help(Side::Front, 0, 0),
			],
		],
		[
			// yellow (down)
			[
				help(Side::Down, 2, 0),
				help(Side::Down, 2, 1),
				help(Side::Down, 2, 2),
			],
			[
				help(Side::Down, 1, 0),
				help(Side::Down, 1, 1),
				help(Side::Down, 1, 2),
			],
			[
				help(Side::Down, 0, 0),
				help(Side::Down, 0, 1),
				help(Side::Down, 0, 2),
			],
		],
		[
			// orange (right)
			[
				help(Side::Right, 0, 2),
				help(Side::Right, 1, 2),
				help(Side::Right, 2, 2),
			],
			[
				help(Side::Right, 0, 1),
				help(Side::Right, 1, 1),
				help(Side::Right, 2, 1),
			],
			[
				help(Side::Right, 0, 0),
				help(Side::Right, 1, 0),
				help(Side::Right, 2, 0),
			],
		],
	];

	// Assign each color a letter
	const COL_CHAR: [char; 6] = [
		'e', // left
		'a', // up
		'c', // back
		'd', // front
		'b', // down
		'f', // right
	];

	let mut rcols: Vec<char> = vec!['x'; 54];
	// Assign all center facelets
	for i in (4..54).step_by(9) {
		rcols[i] = ((i / 9) as u8 + 'a' as u8) as char;
	}

	for (k, grid) in colors.iter().enumerate() {
		for i in 0..3 {
			for j in 0..3 {
				// Ignore blocked facelets
				if i == j && 0 < i {
					continue;
				}

				// Get nearest centroid (color)
				let col = get_nearest(grid[i][j], centroids);
				let idx = IDX[k][i][j]; // index on the permutation
				rcols[idx] = COL_CHAR[col];
			}
		}
	}

	rcols
}

/// Data used for reading the cubes configuration
#[derive(Clone)]
pub struct CalibrationData {
	pub corners1: Vec<Point>,   // 4 corners of cam 1
	pub corners2: Vec<Point>,   // 4 corners of cam 2
	pub centroids: Vec<Scalar>, // the centoids of the 6 colors
}

impl Default for CalibrationData {
	fn default() -> Self {
		// manually typed
		let corners1 = vec![
			Point::new(253, 316),
			Point::new(477, 196),
			Point::new(221, 590),
			Point::new(4, 156),
		];

		// manually typed
		let corners2 = vec![
			Point::new(239, 280),
			Point::new(24, 396),
			Point::new(254, 5),
			Point::new(472, 407),
		];

		// Measured
		let centroids = vec![
			Scalar::new(
				103.15802020230214,
				197.78614767782156,
				33.79034788774039,
				0.0,
			),
			Scalar::new(
				190.83742090115115,
				142.85967690602453,
				131.30614075925973,
				0.0,
			),
			Scalar::new(
				216.01276455026465,
				56.15974867724871,
				208.23642857142858,
				0.0,
			),
			Scalar::new(
				168.21700617283938,
				162.74526455026455,
				196.80681657848336,
				0.0,
			),
			Scalar::new(
				222.51574955908288,
				96.47886243386245,
				112.46783950617284,
				0.0,
			),
			Scalar::new(
				146.6680952380952,
				163.54761022927704,
				61.664444444444484,
				0.0,
			),
		];

		Self {
			corners1,
			corners2,
			centroids,
		}
	}
}

/// Attempts to read a cube. Return None if the cube couldn't be read properly
pub fn read_cube(
	frame1: &mut Mat,
	frame2: &mut Mat,
	data: CalibrationData,
) -> CVResult<Option<CubieCube>> {
	// Estimated Coordinates of the 4 corners

	let CalibrationData {
		corners1,
		corners2,
		centroids,
	} = data;

	let colors = read_all_colors(frame1, corners1, frame2, corners2)?;
	let rcols = colors_to_cube(&colors, &centroids);

	#[cfg(debug_assertions)]
	print_cvec(&rcols);

	// Correct errors
	if let Some(vec) = correct_errors(rcols) {
		let s: String = vec.iter().collect();
		// Convert into arraycube
		if let Ok(array) = ArrayCube::from_str(s.as_str()) {
			// Convert into cubiecube
			if let Ok(cubie) = CubieCube::try_from(array) {
				// Only return if solvable
				if cubie.is_solvable() {
					return Ok(Some(cubie));
				}
			}
		}
	}

	Ok(None)
}

// === DEBUGGING ===

#[cfg(debug_assertions)]
pub fn camera_testing() -> GenericResult<()> {
	std::env::set_var("RUST_BACKTRACE", "1");

	let CalibrationData {
		corners1,
		corners2,
		centroids,
	} = CalibrationData::default();

	let mut cam1 = Camera::try_new(2, cv::core::ROTATE_90_CLOCKWISE)?;
	let mut cam2 = Camera::try_new(4, cv::core::ROTATE_90_COUNTERCLOCKWISE)?;

	// cam1.calibrate(corners1)?;
	// cam2.calibrate(corners2)?;

	// cam1.insert_helping_points()?;
	// cam2.insert_helping_points()?;

	loop {
		let mut frame1 = cam1.read()?;
		let mut frame2 = cam2.read()?;

		let mut spectrum = cvt_color(&mut generate_spectrum()?, cv::imgproc::COLOR_Lab2RGB)?;
		let colors = read_all_colors(&mut frame1, corners1.clone(), &mut frame2, corners2.clone())?;
		let rcols = colors_to_cube(&colors, &centroids);

		for s in centroids.iter() {
			let p = Point::new(s[1] as i32, s[2] as i32);
			cv::imgproc::circle(&mut spectrum, p, 5, BLACK, -1, cv::imgproc::LINE_8, 0)?;
		}

		for grid in colors.iter() {
			for (i, row) in grid.iter().enumerate() {
				for (j, entry) in row.iter().enumerate() {
					if i == j && 0 < i {
						continue;
					}
					let p = Point::new(entry[1] as i32, entry[2] as i32);
					// println!("{}/{}", p.x, p.y);
					cv::imgproc::circle(&mut spectrum, p, 2, BLACK, -1, cv::imgproc::LINE_8, 0)?;
				}
			}
		}

		let grid1 = find_out_grid(&mut frame1, corners1.clone())?;
		let grid2 = find_out_grid(&mut frame2, corners2.clone())?;

		cv::highgui::imshow("f1bef", &frame1)?;
		cv::highgui::imshow("f2bef", &frame2)?;

		draw_grid(&mut frame1, &grid1)?;
		draw_grid(&mut frame2, &grid2)?;

		cv::highgui::imshow("f1", &frame1)?;
		cv::highgui::imshow("f2", &frame2)?;
		cv::highgui::imshow("spectrum", &spectrum)?;

		let s_bef: String = rcols.iter().collect();
		println!("Try convert cube: {}", s_bef);
		print_cvec(&rcols);
		match correct_errors(rcols) {
			Some(rcols) => {
				let s: String = rcols.iter().collect();
				println!("Errors corrected: {}", s);

				match ArrayCube::from_str(s.as_str()) {
					Ok(cube) => {
						let array = cube.clone();
						// array.print();
						match CubieCube::try_from(cube) {
							Ok(cubie) => {
								match cubie.check_solvability() {
									Ok(_) => {
										array.print();
										println!("{}", s);
										println!("SOLVABLE!!!!!!!!!!!!!!!");
										// break;
									}
									Err(e) => eprintln!("Unsolvable: {}", e),
								}
							}
							Err(e) => {
								array.print();
								eprintln!("Could not convert: {}", e);
							}
						}
					}
					Err(e) => {
						print_cvec(&rcols);
						eprintln!("Could not parse cube: {}", e);
					}
				}
			}
			// None => {},
			None => eprintln!("Could not autocomplete cube!"),
		}

		if let Ok(key) = cv::highgui::wait_key(16) {
			match key as u8 as char {
				'q' => break,
				_ => {}
			}
		} else {
			break;
		}
	}

	Ok(())
}
