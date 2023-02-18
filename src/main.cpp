#include "main.h"

#include <charconv>
#include <codecvt>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "EZ-Template/auton.hpp"
#include "EZ-Template/util.hpp"
#include "Flywheel.hpp"
#include "Intakes.hpp"
#include "TopIntake.hpp"
#include "dashboard.hpp"
#include "display/lv_misc/lv_task.h"
#include "pros/adi.h"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include "pros/vision.h"

// Disc Signature ID's
#define DISC_SIG_1 1
#define DISC_SIG_2 2
#define DISC_SIG_3 3
// Goal Signature ID's
#define RED_GOAL_SIG_ID_1 4
#define RED_GOAL_SIG_ID_2 5
#define RED_GOAL_SIG_ID_3 6
#define BLUE_GOAL_SIG_ID 7

#define LOW_VISION_OFFSET 5
#define HIGH_VISION_OFFSET -5
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
    4.125

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

pros::Vision low_sensor(5, pros::E_VISION_ZERO_CENTER);
pros::Vision high_sensor(4, pros::E_VISION_ZERO_CENTER);

pros::vision_signature_s_t DISC_OBJ_1 = pros::Vision::signature_from_utility(DISC_SIG_1, -1, 1309, 654, -4641, -2681, -3661, 1.600, 0);
pros::vision_signature_s_t DISC_OBJ_2 = pros::Vision::signature_from_utility(DISC_SIG_2, 151, 1639, 895, -3079, -1517, -2298, 2.500, 0);

pros::vision_signature_s_t DISC_OBJ_3 = pros::Vision::signature_from_utility(DISC_SIG_3, -1, 483, 241, -4637, -3779, -4208, 3.000, 0);
// pros::vision_signature_s_t GOAL_SIG = pros::Vision::signature_from_utility(2, -2259, -775, -1517, 1595, 5493, 3544, 1.400, 0);
// //fill in sig

// TODO: Configure BLUE_GOAL_SIG
pros::vision_signature_s_t BLUE_GOAL_SIG = pros::Vision::signature_from_utility(BLUE_GOAL_SIG_ID, -1, 1, 0, -1, 1, 0, 1.000, 0);
// pros::vision_signature_s_t GOAL_SIG_2 = pros::Vision::signature_from_utility(3, 4755, 7509, 6132, -2875, -403, -1638, 2.500, 0);
pros::vision_signature_s_t RED_GOAL_SIG = pros::Vision::signature_from_utility(RED_GOAL_SIG_ID_1, 2303, 9413, 5858, -1703, -1, -852, 1.000, 0);

pros::vision_signature_s_t RED_GOAL_SIG_2 = pros::Vision::signature_from_utility(RED_GOAL_SIG_ID_2, 1859, 7047, 4452, -2535, 1, -1266, 1.000, 0);

auto intake_run = 0;
auto discs = 2;
auto iteration = 0;
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
  chassis.reset_gyro();                               // Resets the gyro
  chassis.set_active_brake(0);                        // Sets the active brake kP. We recommend 0.1.
  chassis.set_curve_default(1, 0);                    // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)
  // default_constants();                                // Set the drive to your own constants from autons.cpp!
  //  exit_condition_defaults();
  set_indexer(IN);
  set_expansion(DOWN);  // Set the exit conditions to your own constants from autons.cpp!

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

void disc_detection() {
  pros::vision_object_s_t obj = low_sensor.get_by_size(0);
  // printf("OBJ: %d: %d\n", obj.signature, obj.width);
  if (obj.signature > 10) {
    return;
  } else {
    if (iteration % 100 == 0) {
      printf("OBJ: %d: %d\n", obj.signature, obj.width);
    }
  }

  pros::lcd::print(0, "X: %d", obj.x_middle_coord);
  pros::lcd::print(1, "Y: %d", obj.y_middle_coord);
  pros::lcd::print(2, "Width: %d", obj.width);
  pros::lcd::print(3, "Height: %d", obj.height);
  // printf("[DISC] width: %d\n", obj.width);

  int area = obj.width * obj.height;
  pros::lcd::print(4, "Area: %d", area);

  int left_move = (((4800 - min(area, 2500)) * 0.04) + (obj.x_middle_coord + LOW_VISION_OFFSET) * 0.3);
  int right_move = (((4800 - min(area, 2500)) * 0.04) - (obj.x_middle_coord + LOW_VISION_OFFSET) * 0.3);
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
    intake_run = 225;
    set_intake(127);
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
  }
}

void goal_detection() {
  pros::vision_object_s_t goal = high_sensor.get_by_size(0);
  if (goal.signature > 10) {
    if (iteration % 100 == 0) {
      printf("Turning\n");
      for (pros::Motor mtr : chassis.left_motors) {
        mtr.move_velocity(90);
      }
      for (pros::Motor mtr : chassis.right_motors) {
        mtr.move_velocity(-90);
      }

      pros::delay(20);
    }
    return;
  } else if (iteration % 100 == 0) {
    printf("[GOAL] SIGNATURE: %d\n", goal.signature);
  }

  chassis.reset_pid_targets();
  pros::lcd::print(0, "[GOAL] X: %d", goal.x_middle_coord);
  pros::lcd::print(1, "[GOAL] Y: %d", goal.y_middle_coord);
  pros::lcd::print(2, "[GOAL] Width: %d", goal.width);
  pros::lcd::print(3, "[GOAL] Height: %d", goal.height);
  pros::lcd::print(4, "[GOAL] Area: %d", goal.width * goal.height);
  if (iteration %  50 == 0 || goal.width * goal.height > 2500) {
    printf("[GOAL] w: %d h: %d [A]: %d\n", goal.width, goal.height, goal.width * goal.height);
  }
  
  //printf("[GOAL] ( x: %d, y: %d )\n",  goal.x )
  int area = goal.width * goal.height;
  //pros::lcd::print(4, "Area: %d", area);
  if (area < 500 || goal.height < 8) {
    printf("FALSE POSITIVE\n");
    return;
  }
  // If too close don't; move
  if (area > 2500) {
    
    for (pros::Motor mtr : chassis.left_motors) {
      mtr.move_velocity(0);
    }

    for (pros::Motor mtr : chassis.right_motors) {
      mtr.move_velocity(0);
    }
    
    while (discs > 0) {
      printf("DISCS: %d\n", discs);
      flywheel(9000);
      while (flywheel_speed() < 350) {
        pros::delay(10);
      }
      printf("FIRE\n");
      set_topintake(127);  // m ax: 127
      pros::delay(75);
      set_topintake(-127);
      printf("FIRE DONE\n");
      pros::delay(200);
      
      discs--;
      if (discs == 0) {
        flywheel(0);
        set_topintake(0);
        printf("ALL DONE\n");
      }
    }
    return;
  }
  printf("GOAL DETECTED: %d: (h: %d, w: %d, a: %d)\n", goal.signature, goal.height, goal.width, area);
  // Center the robot on the goal
  int left_move = (((4800 - min(area, 2500)) * 0.04) + (goal.x_middle_coord + HIGH_VISION_OFFSET) * 0.2);
  int right_move = (((4800 - min(area, 2500)) * 0.04) - (goal.x_middle_coord + HIGH_VISION_OFFSET) * 0.2);

  for (pros::Motor mtr : chassis.left_motors) {
    mtr.move_velocity(left_move);
  }

  for (pros::Motor mtr : chassis.right_motors) {
    mtr.move_velocity(right_move);
  }
}


void autonomous() {
  // set bools false
  // initStuff = false ;

  chassis.reset_pid_targets();                // Resets PID targets to 0
  chassis.reset_gyro();                       // Reset gyro position to 0
  chassis.reset_drive_sensor();               // Reset drive sensors to 0
  chassis.set_drive_brake(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps
  // autonomous consistency.
  chassis.toggle_auto_print(true);     // Toggle printing of PID values
  low_sensor.set_exposure(50);
  pros::ADIDigitalIn sensor('B');

  // pros::vision_signature_s_t DISC_OBJ_1 = pros::Vision::signature_from_utility(1, 1153, 1511, 1332, -4387, -3979, -4183, 1.300, 0);
  // pros::vision_signature_s_t DISC_SIG_2 = pros::Vision::signature_from_utility(2, 1177, 1977, 1577, -4953, -4179, -4566, 2.500, 0);
  // pros::vision_signature_s_t DISC_SIG_3 = pros::Vision::signature_from_utility(3, 1411, 1859, 1635, -4361, -3845, -4103, 1.400, 0);
  // Done with exposure
  // pros::vision_signature_s_t DISC_OBJ_2 = pros::Vision::signature_from_utility(2, -1, 853, 426, -4175, -3723, -3949, 3.000, 0);
  // Home DISC SIG
  low_sensor.set_signature(DISC_SIG_1, &DISC_OBJ_1);

  // School Disc Sig
  low_sensor.set_signature(DISC_SIG_2, &DISC_OBJ_2);

  // Home Disc SIG 2
  low_sensor.set_signature(DISC_SIG_3, &DISC_OBJ_3);

  // pros::vision_color_code_t disc_code = low_sensor.create_color_code(DISC_SIG_1, DISC_SIG_2, DISC_SIG_3);
  // Red Goal[School]

  high_sensor.set_exposure(50);

  high_sensor.set_signature(RED_GOAL_SIG_ID_1, &RED_GOAL_SIG);
  high_sensor.set_signature(RED_GOAL_SIG_ID_2, &RED_GOAL_SIG_2);
  // high_sensor.set_signature(BLUE_GOAL_SIG_ID, &BLUE_GOAL_SIG);

  intake_run = 0;
  iteration = 0;
  sensor.get_new_press();
  while (true) {
    if (sensor.get_new_press()) {
      discs++;
      printf("NEW DISC\n");
    } else {
      if (iteration % 200 == 0) {
        printf("NO NEW DISC\n");
      }
    }

    if (discs >= 2) {
      goal_detection();
    } else {
      disc_detection();
    }
    
    if (intake_run > 0) {
      intake_run--;
    } else {
      set_intake(0);
    }

    pros::delay(20);
    iteration++;
    if (iteration > 10000) {
      iteration = 0;
    } else if (iteration % 100 == 0) {
      printf("Iteration: %d Discs: %d\n", iteration, discs);
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
  // chassis.set_drive_brake(MOTOR_BRAKE_HOLD);
  // printf("Entering AUTON\n");
  // autonomous();
  // printf("EXITING AUTON\n");
  chassis.reset_pid_targets();
  // reset PID targets to 0
  chassis.reset_gyro();
  // reset gyro position to 0
  chassis.reset_drive_sensor();
  // reset drive sensors to 0

  chassis.set_drive_brake(MOTOR_BRAKE_HOLD);
  // set motors to hold. This helps autonomous consistancy
  chassis.set_active_brake(0.0);
  // get match load
  //  set_indexer(OUT);
  //  flywheel(-69);
  //  set_topintake(-43);
  //  set_intake(-20);
  //  pros::delay(700);
  //  set_intake(0);
  //  set_topintake(40);
  //  set_indexer(IN);
  //  flywheel(69);
  //  set_intake(0);
  //  pros::delay(900);

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

  // get back to match load position
  // chassis.set_drive_pid(-0.9, 90);
  // chassis.wait_drive();

  // chassis.set_turn_pid(-1, 80);
  // chassis.wait_drive();

  // flywheel(-60);
  // chassis.set_drive_pid(-10.4, 70);

  // get match load
  //  set_indexer(OUT);
  //  pros::delay(300);
  //  set_topintake(-42);
  //  set_intake(-20);
  //  pros::delay(800);
  //  set_intake(0);
  //  pros::delay(1100);
  //  set_indexer(IN);
  //  flywheel(69);
  //  set_topintake(40);
  //  set_intake(0);
  //  pros::delay(900);

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

  // get back to match load position
  //  chassis.set_turn_pid(-1, 80);
  //  chassis.wait_drive();

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
  bool is_tank = false;

  while (true) {
    // run drivebase
    if (master.get_digital_new_press(DIGITAL_X)) {
      is_tank = !is_tank;
      // printf("tank");
      // pros::lcd::print(6, "Height: %d", 5);
    }
    if (is_tank)
      chassis.tank();  // Tank control
    else
      chassis.arcade_standard(ez::SPLIT);

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!
                                        // Keep this ez::util::DELAY_TIME
  }
}
