#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include <cmath>
#include <string>
#include <vector>
#include "constants.hpp"

pros::Motor Flywheel(12, pros::E_MOTOR_GEARSET_06); //no true
pros::Motor Flywheel2(13, pros::E_MOTOR_GEARSET_06, true); //true

std::vector<std::string> flywheel_dash_slice() {
  std::vector<std::string> lines;
  lines.push_back("Flywheel");
  lines.push_back("Speed\tTemperature");
  lines.push_back(std::to_string(Flywheel.get_actual_velocity()) + " " + std::to_string(Flywheel.get_temperature()));
  lines.push_back(std::to_string(Flywheel2.get_actual_velocity()) + " " + std::to_string(Flywheel2.get_temperature()));

  return lines;
}

// TODO: Split into a vector
double flywheel_speed(){
  return (Flywheel.get_actual_velocity()+Flywheel2.get_actual_velocity())/2;
}

double flywheel_temp(){
  return (Flywheel.get_temperature()+Flywheel2.get_temperature())/2;
}

std::vector<int> speeds =
    {0, 8500, 10000, 11000};

bool speed = false;

void flywheel(int power) {
  Flywheel.move_voltage(power);
  Flywheel2.move_voltage(power);
}

int flywheel_index = 0;
void set_flywheel(int input) { flywheel_index = input; }
int get_flywheel() { return flywheel_index; }

void Flywheel_Control(void *) {
  while (true) {

    if (master.get_digital_new_press(DIGITAL_R2)) {
      flywheel_index++;
      if (flywheel_index >= speeds.size()) {
        flywheel_index = 0;
      }
    }
    if (master.get_digital_new_press(DIGITAL_Y)) {
      flywheel_index = 0;
    }

    flywheel(speeds[flywheel_index]);
    // pros::vision_object_s_t goal = get_high_sensor().get_by_size(0);

    // if (goal.signature < 10) {
    //     printf("g %d\n", goal.signature);
    //     auto area = goal.width * goal.height;
    //   if (area < 500 || goal.height < 8) {
    //     printf("FALSE POSITIVE [Flywheel]\n");
    //     return;
    //   }
    //   // Function Here
    //   int rpm = 37.63 * (4011.2 * std::pow(area, -0.5892)) + 5876;
    //   printf("RPM: %d\n", rpm);
    //   flywheel(rpm);
    //   set_flywheel(1);

      // if (abs(goal.x_middle_coord - 10) < 20) {
      //   continue;
      // }
      // // Center the robot on the goal
      // int left_move = (goal.x_middle_coord - 5) * 0.2;
      // int right_move = (goal.x_middle_coord - 5) * -0.2;

      // for (pros::Motor mtr : chassis.left_motors) {
      //   mtr.move_velocity(left_move);
      // }

      // for (pros::Motor mtr : chassis.right_motors) {
      //   mtr.move_velocity(right_move);
      // }
    // }

    // if (!speed) {
    //   set_flywheel(0);
    //   flywheel(0);
    // }
    // flywheel(speeds[flywheel_index]);
    pros::delay(20);
  }
}
