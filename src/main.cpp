#include "main.h"
#include "okapi/api/odometry/odomState.hpp"

// motor definitions: put negative if reversed
 const int DRIVE_MOTOR_LEFT_FRONT = 2;
 const int DRIVE_MOTOR_LEFT_BACK = 11;
 const int DRIVE_MOTOR_RIGHT_FRONT = -9;
 const int DRIVE_MOTOR_RIGHT_BACK = -20;

 pros::Motor left_front_motor(2);
	pros::Motor left_back_motor(11);
	pros::Motor right_front_motor(9);
	pros::Motor right_back_motor(20);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "OKAPI INITIALIZED");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	OdomState state;
	std::shared_ptr<OdomChassisController> chassis =
  	ChassisControllerBuilder()
    	.withMotors(
			{DRIVE_MOTOR_LEFT_BACK, DRIVE_MOTOR_LEFT_FRONT},
			{DRIVE_MOTOR_RIGHT_BACK, DRIVE_MOTOR_RIGHT_FRONT}
		)
		.withSensors(
			ADIEncoder{'E', 'F'},
			ADIEncoder{'G', 'H', true},
			ADIEncoder{'C', 'D', true}
		)
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 10_in}, imev5GreenTPR})
		.withOdometry({{2.75_in, 10_in, 0.75_in, 2.75_in}, quadEncoderTPR}, StateMode::CARTESIAN)
    	.buildOdometry();
	chassis->setState({0_in, 0_in, 0_deg});
	pros::delay(20);
	
	// chassis->moveDistance(1_ft);
	// pros::delay(20);
	chassis->turnAngle(90_deg);
	pros::lcd::set_text(4, "DONE!");
	
	//pros::lcd::set_text(3, OdomState.x, OdomState.y, );
	
}
   
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// OdomState state;
	std::shared_ptr<OdomChassisController> chassis =
  	ChassisControllerBuilder()
    	.withMotors(
			{DRIVE_MOTOR_LEFT_BACK, DRIVE_MOTOR_LEFT_FRONT},
			{DRIVE_MOTOR_RIGHT_BACK, DRIVE_MOTOR_RIGHT_FRONT}
		)
		
		.withSensors(
			ADIEncoder{'E', 'F'},
			ADIEncoder{'G', 'H', true},
			ADIEncoder{'C', 'D', true}
		)
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 10_in}, imev5GreenTPR})
		.withOdometry({{2.75_in, 10_in, 0.75_in, 2.75_in}, quadEncoderTPR}, StateMode::CARTESIAN)
    	.buildOdometry();

	chassis->setState({0_in, 0_in, 0_deg});
	pros::delay(20);
	while (true) {
		left_front_motor = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		left_back_motor = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		right_front_motor = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		right_back_motor = -master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

		// cout << state.str() << '\n';
		double x1 = chassis->getState().x.convert(inch);
		double y1 = chassis->getState().y.convert(inch);
		double ang = chassis->getState().theta.convert(degree);
		
		// string state = chassis->getState().str();
		// pros::delay(50);
		// pros::lcd::set_text(5, state);
		
		// cout << x1 << " " << y1 << " " << ang << '\n'; 
		pros::delay(50);
		master.print(0,0, "X: %lf", x1);
		pros::delay(50);
		master.print(1,0, "Y: %lf", y1);
		pros::delay(50);
		master.print(2,0, "A: %lf", ang);
		pros::delay(20);
	}
}
