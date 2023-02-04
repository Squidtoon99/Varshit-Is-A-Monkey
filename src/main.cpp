#include "main.h"
#include "EZ-Template/util.hpp"
#include "Intakes.hpp"
#include "Flywheel.hpp"
#include "dashboard.hpp"
#include "display/lv_misc/lv_task.h"
#include "pros/llemu.hpp"
#include "pros/screen.hpp"
#include <charconv>
#include <string>
#include <vector>
#include <map>


#define DISC_SIG 1
#define DISC_SIG2 2
#define DISC_SIG3 3
#define DISC_SIG4 4
#define VISION_OFFSET 5
#define GOAL_SIG1 9
// vars and bools for you fools
bool initStuff = true;

// For instalattion, upgrading, documentations and tutorials, check out website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
Drive chassis(
    // Left Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    {0, -1, -2}

    // Right Chassis Ports (negative port will reverse it!)
    //   the first port is the sensored port (when trackers are not used!)
    ,
    {0, 6, 7}

    // IMU Port
    ,
    21

    // Wheel Diameter (Remember, 4" wheels are actually 4.125!)
    //    (or tracking wheel diameter)
    ,
    3.25

    // Cartridge RPM
    //   (or tick per rotation if using tracking wheels)
    ,
    600

    // External Gear Ratio (MUST BE DECIMAL)
    //    (or gear ratio of tracking wheel)
    // eg. if your drive is 84:36 where the 36t is powered, your RATIO would
    // be 2.333. eg. if your drive is 36:60 where the 36t is powered, your RATIO
    // would be 1.6666
    ,
    1.6666

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
  // ez::print_ez_template();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure.

  // setup positions of subsystems

  // set pneumatic positions

  // set_lift_position (-10, 100) ; // push lift down

  // Configure your chassis controls
  chassis.toggle_modify_curve_with_controller(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.reset_gyro();                                // Resets the gyro
  chassis.set_active_brake(0);                        // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(1, 0);                    // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  //default_constants();                                // Set the drive to your own constants from autons.cpp!
//  exit_condition_defaults();
  set_indexer(IN);
set_expansion(DOWN);                     // Set the exit conditions to your own constants from autons.cpp!

  // These are already defaulted to these buttons, but you can change the
  // left/right curve buttons here! chassis.set_left_curve_buttons
  // (pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT); // If
  // using tank, only the left side is used.
  // chassis.set_right_curve_buttons(pros::E_CONTROLLER_DIGITAL_Y,
  // pros::E_CONTROLLER_DIGITAL_A);

  // runs task for auton

  // Autonomous Selector using LLEMU
  // ez::as::auton_selector.add_autons({

  // Auton("Han Solo ", Han_Solo),
  // Auton("speed_test two", speed_test2),
  // Auton("speed_test", speed_test),
  // Auton("help me ObiWan i need discs", help_me_ObiWan),
  // Auton(" Rogue one but not everyone dies! ", rogue_one),
  // });

  // Initialize chassis and auton selector
chassis.initialize();
pros::lcd::initialize();
// ez::as::initialize();

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
void competition_initialize() {
  // . . .
}


// void intake_task_fn() {
//   set_intake(127);
//   pros::delay(4000);
//   set_intake(0);
// }
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
  chassis.reset_pid_targets();                // Resets PID targets to 0
  chassis.reset_gyro();                       // Reset gyro position to 0
  chassis.reset_drive_sensor();               // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps
  // autonomous consistency.
  pros::Vision vision_sensor(8, pros::E_VISION_ZERO_CENTER);
  vision_sensor.set_exposure(50);

  // pros::vision_signature_s_t DISC_SIG_1 = pros::Vision::signature_from_utility(1, 1153, 1511, 1332, -4387, -3979, -4183, 1.300, 0);
  // pros::vision_signature_s_t DISC_SIG_2 = pros::Vision::signature_from_utility(2, 1177, 1977, 1577, -4953, -4179, -4566, 2.500, 0);
  // pros::vision_signature_s_t DISC_SIG_3 = pros::Vision::signature_from_utility(3, 1411, 1859, 1635, -4361, -3845, -4103, 1.400, 0);

  // Done with exposure 
  pros::vision_signature_s_t DISC_SIG_4 = pros::Vision::signature_from_utility(1, -1, 853, 426, -4175, -3723, -3949, 3.000, 0);
  pros::vision_signature_s_t GOAL_SIG = pros::Vision::signature_from_utility(2, -2259, -775, -1517, 1595, 5493, 3544, 1.400, 0);

  // vision_sensor.set_signature(DISC_SIG, &DISC_SIG_1);
  // vision_sensor.set_signature(DISC_SIG2, &DISC_SIG_2);
  // vision_sensor.set_signature(DISC_SIG3, &DISC_SIG_3);
  vision_sensor.set_signature(DISC_SIG4, &DISC_SIG_4);
  vision_sensor.set_signature(GOAL_SIG1, &GOAL_SIG);
  int intake_run = 0;
  auto iteration = 0;
  while (true) {
    pros::vision_object_s_t obj = vision_sensor.get_by_size(0);
    if (obj.signature == DISC_SIG4) {
      pros::lcd::print(0, "X: %d", obj.x_middle_coord);
      pros::lcd::print(1, "Y: %d", obj.y_middle_coord);
      pros::lcd::print(2, "Width: %d", obj.width);
      pros::lcd::print(3, "Height: %d", obj.height);

      int area = obj.width * obj.height;
      pros::lcd::print(4, "Area: %d", area);

      int left_move = (((4800 - min(area, 2500)) * 0.04) + (obj.x_middle_coord + VISION_OFFSET) * 0.2);
      int right_move = (((4800 - min(area, 2500)) * 0.04) - (obj.x_middle_coord + VISION_OFFSET) * 0.2);
      pros::lcd::print(5, "Left: %d", left_move);
      pros::lcd::print(6, "Right: %d", right_move);
      for (pros::Motor mtr : chassis.left_motors) {
        mtr.move_velocity(left_move);
      }
      
      for (pros::Motor mtr : chassis.right_motors) {
        mtr.move_velocity(right_move);
      }

      if (area > 1000) {
        // create pros task
        intake_run = 100;
        set_intake(127);
      }
      pros::lcd::print(7, "Iteration: %d", iteration);
    } else if (obj.signature == GOAL_SIG1) {
      printf("Width: %d\n", obj.width);
    } else {
      pros::lcd::print(0, "No object detected");
      // clear lines 1-7
      for (int i = 1; i < 8; i++) {
        pros::lcd::print(i, "");
      }

      //  stop moving
      for (pros::Motor mtr : chassis.left_motors) {
        mtr.move_velocity(0);
      }

      for (pros::Motor mtr : chassis.right_motors) {
        mtr.move_velocity(0);
      }

      if (intake_run > 0) {
        intake_run--;
      } else {
        set_intake(0);
      }
      pros::lcd::print(7, "Iteration: %d", iteration);
    }
    pros::delay(20);
    iteration++;
    if (iteration > 10000) {
      iteration = 0;
    }
  }
  // ez::as::auton_selector.call_selected_auton();  // Calls selected auton from autonomous selector.

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
  
//chassis.set_drive_brake(MOTOR_BRAKE_HOLD);

chassis.reset_pid_targets();
//reset PID targets to 0
chassis.reset_gyro();
//reset gyro position to 0
chassis.reset_drive_sensor();
//reset drive sensors to 0

chassis.set_drive_brake(MOTOR_BRAKE_HOLD);
//set motors to hold. This helps autonomous consistancy
chassis.set_active_brake(0.0);
//get match load
// set_indexer(OUT);
// flywheel(-69);
// set_topintake(-43);
// set_intake(-20);
// pros::delay(700);
// set_intake(0);
// set_topintake(40);
// set_indexer(IN);
// flywheel(69);
// set_intake(0);
// pros::delay(900);

// back up and turn
// chassis.set_drive_pid(8, 80);
// chassis.wait_drive();

// chassis.set_turn_pid(-88.5, 70);
// chassis.wait_drive();

// shoot 3 discs
// set_indexer(OUT);
// set_topintake(127);
// pros::delay(300);
// set_flywheel(74);
// set_intake(127);
// pros::delay(950);

set_indexer(IN);
set_intake(0);
set_topintake(0);

//get back to match load position
//chassis.set_drive_pid(-0.9, 90);
//chassis.wait_drive();

// chassis.set_turn_pid(-1, 80);
// chassis.wait_drive();

// flywheel(-60);
// chassis.set_drive_pid(-10.4, 70);

//get match load
// set_indexer(OUT);
// pros::delay(300);
// set_topintake(-42);
// set_intake(-20);
// pros::delay(800);
// set_intake(0);
// pros::delay(1100);
// set_indexer(IN);
// flywheel(69);
// set_topintake(40);
// set_intake(0);
// pros::delay(900);

// back up and turn
// chassis.set_drive_pid(8, 80);
// chassis.wait_drive();

// chassis.set_turn_pid(-88.5, 70);
// chassis.wait_drive();

// shoot another 3 discs x1
// set_indexer(OUT);
// set_topintake(127);
// pros::delay(300);
// set_flywheel(72);
// set_intake(127);
// pros::delay(950);

// set_indexer(IN);
// set_intake(0);
// set_topintake(0);

//get back to match load position
// chassis.set_turn_pid(-1, 80);
// chassis.wait_drive();

// flywheel(-60);
// chassis.set_drive_pid(-11, 70);

// //get match load
// set_indexer(OUT);
// pros::delay(300);
// flywheel(-63);
// set_topintake(-42);
// set_intake(-20);
// pros::delay(900);
// set_intake(0);
// pros::delay(1500);
// set_indexer(IN);
// flywheel(72);
// set_topintake(30);
// set_intake(0);
// pros::delay(900);

// back up and turn
// chassis.set_drive_pid(8, 80);
// chassis.wait_drive();

// chassis.set_turn_pid(-88.5, 70);
// chassis.wait_drive();

// // shoot another 3 discs x2
// set_indexer(OUT);
// set_topintake(127);
// pros::delay(300);
// set_flywheel(72);
// set_intake(127);
// pros::delay(1520);

// set_indexer(IN);
// set_intake(0);
// set_topintake(0);

  // This is preference to what you like to drive on.
  chassis.set_drive_brake(MOTOR_BRAKE_COAST);

  // run subsystems
  pros::Task Intakes(Intake_Control);
  pros::Task Flywheel(Flywheel_Control);
  pros::Task TopIntakes(TopIntake_Control);
  pros::Task Indexer(Indexer_Control);
  pros::Task Expansion(Expansion_Control);
  pros::Task Dashboard(dashboard_task_fn);

  // pros::Task Dashboard(dashboard_task_fn);
  //  pros::Task Slider(Slider_Control);

  // bool is_tank = true;
  bool is_tank = true;

  while (true) {
    // run drivebase
    if (master.get_digital_new_press(DIGITAL_X)) {
      is_tank = !is_tank;
    }
    if (is_tank)
      chassis.tank();  // Tank control
    else
      chassis.arcade_standard(ez::SPLIT);

    
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!
                                        // Keep this ez::util::DELAY_TIME
  }
}
