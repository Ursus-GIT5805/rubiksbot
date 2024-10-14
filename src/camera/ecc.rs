use rubikscube::prelude::*;

use std::str::FromStr;
use strum::IntoEnumIterator;

const fn get_side(c: char) -> Option<Side> {
	Side::from_repr(c as u8 - b'a')
}

const fn to_char(x: u8) -> char {
	(x + b'a') as char
}

fn correct_corners(cube: Vec<char>) -> Option<Vec<char>> {
	let mut cube = cube;

	let sides: Vec<[Side; 3]> = vec![
		[Side::Up, Side::Right, Side::Front],
		[Side::Up, Side::Back, Side::Right],
		[Side::Down, Side::Left, Side::Front],
		[Side::Down, Side::Front, Side::Right],
		[Side::Up, Side::Left, Side::Back],
		[Side::Up, Side::Front, Side::Left],
		[Side::Down, Side::Right, Side::Back],
		[Side::Down, Side::Back, Side::Left],
	];

	'cornerloop: for corner in Corner::iter() {
		let (i1, i2, i3) = corner_to_indices(corner);

		let (c1, c2, mis) = if cube[i1] == 'x' {
			(cube[i2], cube[i3], i1)
		} else if cube[i2] == 'x' {
			(cube[i3], cube[i1], i2)
		} else if cube[i3] == 'x' {
			(cube[i1], cube[i2], i3)
		} else {
			continue;
		};

		let (s1, s2) = (c1 as u8 - b'a', c2 as u8 - b'a');

		for side in sides.iter() {
			for i in 0..3 {
				let i1 = (i + 1) % 3;
				if side[i] as u8 == s1 && side[i1] as u8 == s2 {
					let i2 = (i + 2) % 3;

					let chr = (side[i2] as u8 + b'a') as char;
					cube[mis] = chr;
					// println!("{} -> {}", mis, chr);

					continue 'cornerloop;
				}
			}
		}

		return None;
	}

	Some(cube)
}

fn is_cvec_solvable(cvec: Vec<char>) -> bool {
	let s: String = cvec.iter().collect();

	if let Ok(array) = ArrayCube::from_str(s.as_str()) {
		if let Ok(cubie) = CubieCube::try_from(array) {
			return cubie.is_solvable();
		}
	}

	false
}

/// Fixes the cube if and only if there is one color replaced
fn single_off(vec: Vec<char>) -> Option<Vec<char>> {
	let mut rvec = vec;
	let mut distr = vec![0; NUM_SIDES];
	for c in rvec.iter() {
		distr[get_side(*c)? as usize] += 1;
	}

	let (from, to) = {
		let mut from = NUM_SIDES;
		let mut to = NUM_SIDES;
		for (i, cnt) in distr.iter().enumerate() {
			match (*cnt as usize).cmp(&CUBE_AREA) {
				std::cmp::Ordering::Less => to = i,
				std::cmp::Ordering::Equal => continue,
				std::cmp::Ordering::Greater => from = i,
			}
		}
		(from, to)
	};

	let from = to_char(from as u8);
	let to = to_char(to as u8);

	// there are better solutions but just replacing all is also correct
	// and so simple to implement

	for i in 0..rvec.len() {
		if rvec[i] == from {
			rvec[i] = to;
			if is_cvec_solvable(rvec.clone()) {
				return Some(rvec);
			}
			rvec[i] = from;
		}
	}

	None
}

fn is_single_off(vec: &Vec<char>) -> bool {
	let mut distr = vec![0; NUM_SIDES];
	for c in vec.iter() {
		if let Some(side) = get_side(*c) {
			distr[side as usize] += 1;
		}
	}

	let mut cnt8 = 0;
	let mut cnt9 = 0;
	let mut cnt10 = 0;
	for num in distr {
		match num {
			8 => cnt8 += 1,
			9 => cnt9 += 1,
			10 => cnt10 += 1,
			_ => return false,
		}
	}

	cnt8 == 1 && cnt9 == 4 && cnt10 == 1
}

pub fn correct_errors(vec: Vec<char>) -> Option<Vec<char>> {
	let mut vec = correct_corners(vec)?;

	if is_single_off(&vec) {
		vec = single_off(vec)?;
	}

	Some(vec)
}
