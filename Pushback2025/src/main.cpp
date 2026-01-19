#include "main.h"
// ==============================
// GLOBALS
// ==============================
// Drivetrain motors (6-motor tank)

// Motor groups for easy control
pros::MotorGroup leftDrive({11, 12, 14},pros::MotorGearset::blue);
pros::MotorGroup rightDrive({-1, -2, -3},pros::MotorGearset::blue);
// PID constants for driving forward
double kP = 0.5;  // Proportional
double kI = 0.0;  // Integral (start with 0, we’ll tune later)
double kD = 0.1;  // Derivative
void drivePID(double target) {
    // Reset motor positions
    leftFront.tare_position();
    leftMiddle.tare_position();
    leftBack.tare_position();
    rightFront.tare_position();
    rightMiddle.tare_position();
    rightBack.tare_position();

    double error = target; // initial error
    double lastError = 0;
    double integral = 0;

    while (fabs(error) > 5) { // loop until within 5 encoder ticks
        // Average current position
        double leftPos = (leftFront.get_position() + leftMiddle.get_position() + leftBack.get_position()) / 3.0;
        double rightPos = (rightFront.get_position() + rightMiddle.get_position() + rightBack.get_position()) / 3.0;
        double currentPos = (leftPos + rightPos) / 2.0;

        // Calculate error
        error = target - currentPos;

        // PID calculations
        integral += error;
        double derivative = error - lastError;
        double power = kP * error + kI * integral + kD * derivative;

        // Cap power to -100 to 100
        if (power > 100) power = 100;
        if (power < -100) power = -100;

        // Move motors
        leftDrive.move(power);
        rightDrive.move(power);

        lastError = error;

        pros::delay(10); // small delay for stability
    }

    // Stop motors
    leftDrive.move(0);
    rightDrive.move(0);
}

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
	pros::lcd::set_text(1, "Hello PROS User!");

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
    // Drive forward ~1000 encoder ticks using PID
    drivePID(1000);
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
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
}