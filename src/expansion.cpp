#include "main.h"
#include "pros/misc.h"

// Driver Control Variables
bool expansion = true;
int expansion1 = 0;
pros::ADIDigitalOut expansion2('C');

void set_expansion(bool input) {
  if (input == UP) {
    expansion2.set_value(true);
    
  }
  if (input == DOWN)
    expansion2.set_value(false);
}

void Expansion_Control(void *b) {
  while (true) {
    // toggle for lock4
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && expansion) {
      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
      //expansion = false;
      set_expansion(UP);
    } else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_UP))
      set_expansion(DOWN);
    pros::delay(20);
  }
}

void expand() {
  set_expansion(true);
  for (pros::Motor mtr : chassis.left_motors) {
    mtr.move_velocity(-200);
  }

  for (pros::Motor mtr : chassis.right_motors) {
    mtr.move_velocity(-150);
  }
}