#pragma once

#include <string>
#include <vector>
#include "pros/motors.hpp"
void set_flywheel(int input);
int get_flywheel();

void Flywheel_Control(void *);
void flywheel(int power);

std::vector<std::string> flywheel_dash_slice();

double flywheel_speed();
double flywheel_temp();