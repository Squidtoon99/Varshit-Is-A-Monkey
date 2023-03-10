#include "main.h"

// vars and bools for you fools
bool initStuff = true ;



/////
// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////


// Chassis constructor
Drive chassis (
  // Left Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  {4, -5, -6}

  // Right Chassis Ports (negative port will reverse it!)
  //   the first port is the sensored port (when trackers are not used!)
  ,{-3, 17, 8}

  // IMU Port
  ,21

  // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
  //    (or tracking wheel diameter)
  ,4.125

  // Cartridge RPM
  //   (or tick per rotation if using tracking wheels)
  ,600

  // External Gear Ratio (MUST BE DECIMAL)
  //    (or gear ratio of tracking wheel)
  // eg. if your drive is 84:36 where the 36t is powered, your RATIO would be 2.333.
  // eg. if your drive is 36:60 where the 60t is powered, your RATIO would be 0.6.
  ,2.333

  // Uncomment if using tracking wheels
  /*
  // Left Tracking Wheel Ports (negative port will reverse it!)
  // ,{1, 2} // 3 wire encoder
  // ,8 // Rotation sensor

  // Right Tracking Wheel Ports (negative port will reverse it!)
  // ,{-3, -4} // 3 wire encoder
  // ,-9 // Rotation sensor
  */

  // Uncomment if tracking wheels are plugged into a 3 wire expander
  // 3 Wire Port Expander Smart Port
  // ,1
);



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::print_ez_template();

  pros::delay(500); // Stop the user from doing anything while legacy ports configure.

  // setup positions of subsystems
  sixlock(true) ;
  // set pneumatic positions
  clawLift(false) ;
  claw(true) ;
  // set_lift_position (-10, 100) ; // push lift down

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true); // Enables modifying the controller curve with buttons on the joysticks
  chassis.set_active_brake(0); // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(1, 0); // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  default_constants(); // Set the drive to your own constants from autons.cpp!
  exit_condition_defaults(); // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.set_left_curve_buttons (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If using tank, only the left side is used.
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,    pros::E_CONTROLLER_DIGITAL_A);

  // runs task for auton
  pros::Task autonUnJammer(intakeTask) ;

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.add_autons(
  {
    Auton("test code", testerAuton),
    Auton("Rush tall goal with doinker", TallMogoWing),
    Auton("Fakeout to tall goal", tallnuetfake),
    Auton("Rush small right nuet with doinker, grab tall goal", Tallsmallneut),
    Auton("Rush small right nuet, right AWP", rightneut),

    Auton("wing - Left Neut rush and ally Left ; set the bot angled nerds", neutRushAndAllyLeft),
    Auton("wing - rush neut and hold with sixlock", wingTurn),
    Auton("wing - AWP Left + Left Neut + AWP Right", wingAWPFull),
    Auton("justAWP", winPointAuton),
    Auton("scoop enemy homezone Right", scoopEnemyZoneRight),
    Auton("skill", skillAuton),
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();

  // uses limit switch to recallibrate everything post-auton-setup
  // while (initStuff)
  // {
  //   if (reInitSwitch ())
  //   {
  //     chassis.initialize();
  //   }
  // }

}



/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
int disabledCount = 0 ;
int disabledLock = 0 ;

void disabled() {
  setClawOp(false) ;
  if (disabledLock == 0)
  {
    disabledCount ++ ;
    disabledLock = 1 ;
  }
  if (disabledCount == 2)
  {
    tallMogo (true) ;
  }
  clawLift (false);
  claw(true);
  sixlock(true);
}



/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // . . .
}



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

  // set bools false
  // initStuff = false ;
  chassis.reset_pid_targets(); // Resets PID targets to 0
  chassis.reset_gyro(); // Reset gyro position to 0
  chassis.reset_drive_sensor(); // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD); // Set motors to hold.  This helps autonomous consistency.

  ez::as::auton_selector.call_selected_auton(); // Calls selected auton from autonomous selector.

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

// set bools and vars
  disabledLock = 0 ;
  // turn off intake Task
  intake(0) ;
  useAutonTask (false) ;
  // set bools false
  initStuff = false ;
  setClawOp (true) ;
  setLiftStart(1) ;
  doubleclaw(true) ;

  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  // run subsystems
  pros::Task Intakes(Intake_Control);
  pros::Task F4Bar(lift_control);
  pros::Task Claw(clawControl) ;




  while (true) {

    // run drivebase
    chassis.tank(); // Tank control



    pros::delay(ez::util::DELAY_TIME); // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
