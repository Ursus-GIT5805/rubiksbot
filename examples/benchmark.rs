use rubikscube::prelude::*;

use indicatif::*;
use plotters::prelude::*;

/// Number of cubes to solve
const N: usize = 2000;
/// Number of turns the random generated sequence is
const NT: usize = 20;

/// Number of milliseconds it takes to execute
const STEP_QUARTER: usize = 130; // 52 * 2.5ms
const STEP_HALF: usize = 255; // 102 * 2.5ms

const DELAY_TURN: usize = 60;

const DATA_PATH: &str = "kociemba.dat";

/// Count the time needed to
fn count_time(seq: Vec<Turn>) -> usize {
	let l = seq.len();

	let time_turns: usize = seq.iter().map(|t| if t.wise == TurnWise::Double {
		STEP_HALF
	} else {
		STEP_QUARTER
	}).sum();

	let time_between = (l-1)*DELAY_TURN;

	time_turns + time_between
}

/// Get the number of occurrences of the most occuring item
fn cnt_most_frequent<T>(vec: &Vec<T>) -> usize
where T: std::hash::Hash + PartialEq + Eq + Copy {
	let mut map = std::collections::HashMap::<T, usize>::new();

	for ele in vec.iter() {
		match map.get_mut(ele) {
			Some(v) => *v += 1,
			None => {
				map.insert(*ele, 1);
			},
		}
	}

	let max = map.iter()
		.map(|(_, cnt)| cnt)
		.max();

	*max.unwrap()
}

/// Create a histogram from the data and save it to 'path'
fn plot_data(x_label: &str, path: &str, shape: ShapeStyle, vec: Vec<usize>) -> Result<(),Box<dyn std::error::Error>> {
	let min = *vec.iter().min().unwrap() as u32 - 1;
	let max = *vec.iter().max().unwrap() as u32 + 1;
	let cnt = cnt_most_frequent(&vec) as u32 + 1;

	let root = BitMapBackend::new(path, (640, 480))
		.into_drawing_area();

    root.fill(&WHITE)?;

	let axis = min..max;
    let mut chart = ChartBuilder::on(&root)
        .x_label_area_size(40)
        .y_label_area_size(40)
        .margin(5)
        .build_cartesian_2d(axis.into_segmented(), 0u32..cnt)?;

	chart
        .configure_mesh()
        .disable_x_mesh()
        .y_desc("Count")
        .x_desc(x_label)
        .axis_desc_style(("sans-serif", 20))
        .draw()?;

	chart.draw_series(
		Histogram::vertical(&chart)
            .style(shape)
            .data(vec.iter().map(|x: &usize| (*x as u32, 1))),
    )?;

	let tot: usize = vec.iter().sum();
	let avg = tot as f64 / vec.len() as f64;

	println!("{}: Average={}", path, avg);

	root.present()?;

	Ok(())
}

fn main() {
	println!("Generating data...");
	let data = match KociembaData::load(DATA_PATH) {
		Ok(k) => k,
		Err(_) => {
			let data = KociembaData::generate(true);
			let _ = data.save(DATA_PATH);
			data
		}
	};


	let mut solver = Solver::new(data);


	println!("Loaded! Start benchmarking!");

	let mut res = vec![];
	let mut turnlen = vec![];

	// Random cubes
	for _ in (0..N).progress() {
		let cube = CubieCube::random();
		let seq = solver.solve(cube).unwrap();
		turnlen.push(seq.len());
		res.push( count_time(seq) );
	}

	plot_data("Execution Time [ms]", "histogram-random.png", RED.filled(), res).unwrap();
	plot_data("Turns", "histogram-random-turns.png", BLUE.filled(), turnlen).unwrap();

	let mut res = vec![];
	let mut turnlen = vec![];

	// Random sequence cubes
	for _ in (0..N).progress() {
		let cube = {
			let mut c = CubieCube::new();
			let turns = random_sequence(NT);
			c.apply_turns(turns);
			c
		};
		let seq = solver.solve(cube).unwrap();
		turnlen.push(seq.len());
		res.push( count_time(seq) );
	}

	plot_data("Execution time [ms]", "histogram-seq-random.png", RED.filled(), res).unwrap();
	plot_data("Turns", "histogram-seq-random-turns.png", BLUE.filled(), turnlen).unwrap();
}
