use adafruit_motorkit::{
	stepper::{StepDirection, StepStyle, StepperMotor},
	Motor, MotorError,
};

use linux_embedded_hal::I2cdev;
use pwm_pca9685::{Address, Pca9685};
use rubikscube::prelude::*;

/// Delay between steps and turns
const STEP_DELAY: std::time::Duration = std::time::Duration::from_micros(2500);
const STEP_DELAY_ADVANCED: std::time::Duration = STEP_DELAY;
const TURN_DELAY: std::time::Duration = std::time::Duration::from_millis(60);

/// Number of extra steps because of the magnets holding the cube back.
const EXTRA_STEPS: usize = 2;

type Pwm = Pca9685<I2cdev>;
type MotorResult<T> = Result<T, MotorError>;

/// StepperMotor Controller
///
/// Wrapper to control the six stepper motors
pub struct StepperController {
	pwm: [Pwm; 3],
}

/// Setup communication with the Stepper motor HAT
fn init_pca(address: impl Into<Address>) -> MotorResult<Pwm> {
	let i2cdev = linux_embedded_hal::I2cdev::new("/dev/i2c-1").map_err(|_| MotorError::I2cError)?;
	let mut pwm = Pca9685::new(i2cdev, address).map_err(|_| MotorError::PwmError)?;

	pwm.enable().map_err(|_| MotorError::PwmError)?;
	pwm.set_prescale(4).map_err(|_| MotorError::PwmError)?;

	Ok(pwm)
}

/// Return the number of steps and direction for the given TurnWise
fn get_instruction(wise: TurnWise) -> (usize, StepDirection) {
	match wise {
		TurnWise::Clockwise => (50 + EXTRA_STEPS, StepDirection::Backward),
		TurnWise::Double => (100 + EXTRA_STEPS, StepDirection::Backward),
		TurnWise::CounterClockwise => (50 + EXTRA_STEPS, StepDirection::Forward),
	}
}

impl StepperController {
	pub fn try_new() -> MotorResult<Self> {
		// Note, you can see the address with `i2cdetect -y 1`
		Ok(Self {
			pwm: [init_pca(0x60)?, init_pca(0x61)?, init_pca(0x62)?],
		})
	}

	/// Return the indices of the motor which turns the given side
	pub fn get_motor(&self, side: Side) -> (usize, Motor) {
		match side {
			Side::Up => (0, Motor::Stepper2),
			Side::Down => (2, Motor::Stepper2),
			Side::Back => (2, Motor::Stepper1),
			Side::Front => (1, Motor::Stepper1),
			Side::Left => (0, Motor::Stepper1),
			Side::Right => (1, Motor::Stepper2),
		}
	}

	/// Execute a single instruction
	fn execute_single(
		&mut self,
		pwmidx: usize,
		motor: Motor,
		wise: TurnWise,
	) -> Result<(), MotorError> {
		let (steps, dir) = get_instruction(wise);

		let mut stepper = StepperMotor::try_new(&mut self.pwm[pwmidx], motor, None)?;

		for _ in 0..steps {
			stepper.step_once(&mut self.pwm[pwmidx], dir, StepStyle::Double)?;
			std::thread::sleep(STEP_DELAY);
		}
		stepper.stop(&mut self.pwm[pwmidx])?;

		Ok(())
	}

	/// Execute two instructions in parallel
	/// You should only turn two opposite sides together
	fn execute_double(
		&mut self,
		pwmi1: usize,
		motor1: Motor,
		wise1: TurnWise,
		pwmi2: usize,
		motor2: Motor,
		wise2: TurnWise,
	) -> MotorResult<()> {
		let (steps1, dir1) = get_instruction(wise1);
		let (steps2, dir2) = get_instruction(wise2);

		// They have to step the same amount!
		if steps1 != steps2 {
			return Err(MotorError::InvalidMotorError);
		}

		let mut stepper1 = StepperMotor::try_new(&mut self.pwm[pwmi1], motor1, None)?;
		let mut stepper2 = StepperMotor::try_new(&mut self.pwm[pwmi2], motor2, None)?;

		for _ in 0..steps1 {
			stepper1.step_once(&mut self.pwm[pwmi1], dir1, StepStyle::Double)?;
			stepper2.step_once(&mut self.pwm[pwmi2], dir2, StepStyle::Double)?;
			std::thread::sleep(STEP_DELAY_ADVANCED);
		}
		stepper1.stop(&mut self.pwm[pwmi1])?;
		stepper2.stop(&mut self.pwm[pwmi2])?;

		Ok(())
	}

	/// Execute the given turn and send it to the motors
	pub fn execute_turn(&mut self, turn: Turn) -> MotorResult<()> {
		match turn.typ {
			// Single side turns
			TurnType::U | TurnType::D | TurnType::B | TurnType::F | TurnType::L | TurnType::R => {
				let side = turn.typ.get_side()
					.ok_or(MotorError::InvalidMotorError)?;
				let (idx, mot) = self.get_motor(side);
				self.execute_single(idx, mot, turn.wise)?;
			}
			// Advanced turns, needs 2 motors
			_ => {
				let (t1, t2) = analyze_advanced_turn(turn)
					.ok_or(MotorError::InvalidMotorError)?;

				let w1 = t1.wise;
				let w2 = t2.wise;

				let s1 = t1.typ.get_side().ok_or(MotorError::InvalidMotorError)?;
				let s2 = t2.typ.get_side().ok_or(MotorError::InvalidMotorError)?;

				let (i1, m1) = self.get_motor(s1);
				let (i2, m2) = self.get_motor(s2);

				self.execute_double(i1, m1, w1, i2, m2, w2)?;
			}
		}
		// Turn executed, have a delay until the next one
		std::thread::sleep(TURN_DELAY);

		Ok(())
	}

	/// Unpower all motors
	pub fn unpower_motors(&mut self) -> MotorResult<()> {
		for pwm in self.pwm.iter_mut() {
			let mut stepper1 = StepperMotor::try_new(pwm, Motor::Stepper1, None)?;
			let mut stepper2 = StepperMotor::try_new(pwm, Motor::Stepper2, None)?;
			stepper1.stop(pwm)?;
			stepper2.stop(pwm)?;
		}
		Ok(())
	}
}
