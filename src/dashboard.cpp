#include <charconv>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>


#include "Flywheel.hpp"
#include "Intakes.hpp"
#include "main.h"
#include "pros/colors.h"
#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"

using namespace std;


// std::vector<std::string> chassis_dash_slice() {
//   std::vector<std::string> lines;
//   lines.push_back("Drivetrain");
//   lines.push_back("Speed\tTemperature");

//   for (auto mtr : chassis.left_motors) {
    
//   }
//   for (auto mtr : chassis.right_motors) {
//     lines.push_back(std::to_string(mtr->get_actual_velocity()) + "\t" + std::to_string(mtr->get_temperature()));
//   }

//   return lines;
// }

void dashboard_task_fn() {
  // pros::lcd::initialize();
  pros::screen::set_pen(COLOR_BLACK);
  pros::screen::fill_rect(0,0,480,272);
  pros::screen::set_pen(COLOR_WHITE);
  while (true) {
    printf("\033[H\033[2J");
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Intake:           Flywheel:");
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "%.3d   %.3d   %.3d   %.3d", get_conveyor_actual_velocity(), get_conveyor_temperature(), flywheel_speed(), flywheel_temp());
    pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Drivetrain");
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "%.3d   %.3d", chassis.left_motors[0].get_actual_velocity(), chassis.left_motors[0].get_temperature());
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "%.3d   %.3d", chassis.left_motors[1].get_actual_velocity(), chassis.left_motors[1].get_temperature());
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "%.3d   %.3d", chassis.right_motors[0].get_actual_velocity(), chassis.right_motors[0].get_temperature());
    pros::screen::print(pros::E_TEXT_MEDIUM, 4, "%.3d   %.3d", chassis.right_motors[0].get_actual_velocity(), chassis.right_motors[0].get_temperature());
    pros::delay(20);
  }
}