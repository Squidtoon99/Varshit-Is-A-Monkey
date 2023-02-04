#include "Intakes.hpp"

#include "main.h"
#include "pros/misc.h"
#include "pros/motors.h"


// -mouth----------------------------------------------------------------------------

pros::Motor Conveyor(10);  //<-- reversed

double get_conveyor_temperature() { return Conveyor.get_temperature(); }
double get_conveyor_actual_velocity() { return Conveyor.get_actual_velocity(); }

void set_intake(int power) { Conveyor.move(power); }

void Intake_Control(void *) {
  while (true) {
    // printf("master: %d\n", master.get_digital(DIGITAL_L1));
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) or master.get_digital(pros::E_CONTROLLER_DIGITAL_R1 )) {
      set_intake(127);  // m ax: 127
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      set_intake(-127);
    } else {
      set_intake(0);
    }

    pros::delay(20);
  }
}
