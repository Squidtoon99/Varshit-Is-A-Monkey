#include "main.h"
#include "pros/motors.h"
#include <cmath>
#include <string>
#include <vector>

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

// From 5000 to 12000 RPM by orders of 500
std::vector<int> speeds =
    {0, 5000, 5500, 6000, 6500, 7000, 7500, 8000, 8500, 9000, 9500, 10000, 10500, 11000, 11500, 12000};

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
      if (flywheel_index >= speeds.size())
        flywheel_index = 0;
      printf("flywheel: %d\n", speeds[flywheel_index]);
    } else if (master.get_digital_new_press(DIGITAL_L2)) {
      flywheel_index--;
      if (flywheel_index < 0)
        flywheel_index = speeds.size() - 1;
      printf("flywheel: %d\n", speeds[flywheel_index]);
    }
    if (master.get_digital_new_press(DIGITAL_Y)) {
      flywheel_index = 0;
      printf("flywheel: %d\n", speeds[flywheel_index]);
    }

    flywheel(speeds[flywheel_index]);
    pros::delay(20);
  }
}
