#include "main.h"
#include "globals.hpp"
#include "mechLib.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <sstream>  // For std::ostringstream
#include <cmath>    // For std::trunc

bool clamp_up = false;
bool forceClampRelease = false;
bool pistonActivated = false;
Motor frontRight(DRIVE_FR_P);
Motor frontLeft(DRIVE_FL_P);
Motor backRight(DRIVE_BR_P);
Motor backLeft(DRIVE_BL_P);

Motor intakeF(INTAKE_F);
Motor intakeC(INTAKE_C);
Motor clamp (CLAMP_P);
Controller master(CONTROLLER_MASTER);
adi::DigitalOut piston (PNEUMATIC_P);

void clamp_tilt_task_fn(void *params) {
  float error;

  while (true) {
    if (clamp_up && !forceClampRelease) {
      error = CLAMP_TILT_DEGREES - clamp.get_position();
      // master.print(0, 0, "%f", error);
    } else if (forceClampRelease) {
      error = CLAMP_FORCE_DEGREES - clamp.get_position();
      clamp_up = false;
    } else {
      error = CLAMP_DEFAULT_DEGREES - clamp.get_position();
    }

    clamp.move((error * CLAMP_TILT_KP) + 10);
    delay(25);
    if (forceClampRelease && clamp.get_actual_velocity() == 0) {
      clamp.tare_position();
      forceClampRelease = false;
    }
  }
}

// void getRoInfo(void *params) {
//   while (true) {
//     master.set_text(1, 0, "Clamp Temp: " + std::to_string(clamp.get_temperature()));
//     master.set_text(0, 0, std::to_string(master.get_battery_capacity()));
//     master.set_text(2, 0, "Mean intk temp: " + std::to_string((intakeC.get_temperature() + intakeF.get_temperature()) / 2));
//     delay(500);
//   }
// }

void moveDelay() {
  delay(MOVEDELAY);
}

void initialize() {
  // Drive
  DEFINE_MOTOR(frontRight, DRIVE_FR_P, v5::MotorGears::green);
  frontRight.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  DEFINE_MOTOR(frontLeft, DRIVE_FL_P, v5::MotorGears::green);
  frontLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  DEFINE_MOTOR(backLeft, DRIVE_BL_P, v5::MotorGears::green);
  backLeft.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
  DEFINE_MOTOR(backright, DRIVE_BR_P, v5::MotorGears::green);
  backright.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

  // Intakes
  DEFINE_MOTOR(intakeF, INTAKE_F, v5::MotorGears::green);
  DEFINE_MOTOR(intakeC, INTAKE_C, v5::MotorGears::green);

  // Clamp
  DEFINE_MOTOR(clamp, CLAMP_P, v5::MotorGears::red);

  // adi::DigitalOut piston (PNEUMATIC_P);

  // Master
  Controller master(CONTROLLER_MASTER);

  // clamp pid
  Task clampTiltTask(clamp_tilt_task_fn, (void *)"PROS", TASK_PRIORITY_DEFAULT,
                     TASK_STACK_DEPTH_DEFAULT, "clampPID");
                    
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  // NON POSITIVE CORNERS
  if (QUADRANT == 1 || QUADRANT == 2) {
    delay(200); // Wait for init
    baseMove(-21);
    moveDelay();
    baseTurn(STAKEANGLE);    // Face clamp
    clamp_up = false; // Just incase
    moveDelay();
    baseMove(-11);
    moveDelay();
    clamp_up = true;
    delay(600); // Wait for clamp
    intakeF.move(127);
    intakeC.move(127);
    baseTurn(FIRSTDONUT);    // Face the first donut
    moveDelay();
    baseMove(24.2);
    delay(700);
    if (intakeC.get_actual_velocity() < 50) {
      intakeC.move_voltage(-127);
      delay(500);
      intakeC.move_voltage(127);
    }
    baseTurn(SECONDDONUT); // Face second donut
    baseMove(20);
    delay(500);
    baseMove(-6);
    moveDelay();
    baseTurn(FACEADJUST);
    delay(1000);
    intakeF.move(-127);
    intakeC.move(-127);
    baseMove(17);
    moveDelay();
    baseTurn(THIRDDONUT);
    intakeF.move(127);
    intakeC.move(127);
    moveDelay();
    baseMove(16);
    delay(600);
    baseMove(-18);
    moveDelay();
    baseTurn(LADDER);
    moveDelay();
    baseMove(50);
    if (intakeC.get_actual_velocity() < 50) {
      intakeC.move_voltage(-127);
      delay(300);
      intakeC.move_voltage(127);
    }
  } else if (QUADRANT == 3 || QUADRANT == 4) { // POSITIVE CORNERS
    delay(200); // Wait for init
    baseMove(-23);
    moveDelay();
    baseTurn(STAKEANGLE);    // Face clamp
    clamp_up = false; // Just incase
    moveDelay();
    baseMove(-10);
    moveDelay();
    clamp_up = true;
    delay(600); // Wait for clamp
    intakeF.move(127);
    intakeC.move(127);
    baseTurn(FIRSTDONUT);    // Face the first donut
    baseMove(24.2);
    delay(700);
    baseTurn(LADDER);
    moveDelay();
    baseMove(50);
  }
}

void opcontrol() {
  int intake_voltage = 0;
  double left, right;
  bool startup = false;
  int count = 0;

  int32_t r1,r2;
  master.clear();
  while (true) {
    left = master.get_analog(ANALOG_LEFT_Y);
    right = master.get_analog(ANALOG_RIGHT_Y);

    frontRight.move(right);
    backRight.move(right);

    backLeft.move(left);
    frontLeft.move(left);

    r1 = master.get_digital(E_CONTROLLER_DIGITAL_R1);
    r2 = master.get_digital(E_CONTROLLER_DIGITAL_R2);

    if (r1 || r2) {
      intakeC.move(127 * (r1 - r2));
      intakeF.move(127 * (r1 - r2));
      // master.print(0, 0, "%f", (127 * (r1 - r2)));
    }

    if (master.get_digital_new_press(DIGITAL_L1)) {
      forceClampRelease = false;
      clamp_up = !clamp_up;
    }

    if (master.get_digital_new_press(DIGITAL_A)) {
      clamp_up = false;
      forceClampRelease = true;
    }

    if (master.get_digital_new_press(DIGITAL_L2)) {
      pistonActivated = !pistonActivated;
      piston.set_value(pistonActivated);
    } else if (master.get_digital_new_press(DIGITAL_X)) {
      pistonActivated = !pistonActivated;
      piston.set_value(pistonActivated);
    }

    if (master.get_digital(DIGITAL_B)) {
      intakeC.move(127);
      intakeF.move(127);
      int rpm = intakeC.get_actual_velocity();
      if (startup) {
        if (rpm < 180) {
          
        }
      } else {
        if (rpm > 170) {
          startup = true;
        }
      }
      // master.set_text(1, 0, "RPM: " + std::to_string(rpm));
    } else if (!(r1 || r2)) {
      intakeC.move(0);
      intakeF.move(0);
    }

    if (intakeC.get_actual_velocity() == 0) {
      startup = false;
    }
    
    if (!(count % 25)) {
        // Battery
        double battery_capacity = c::battery_get_capacity();
        std::ostringstream battery_stream;
        battery_stream << static_cast<int>(trunc(battery_capacity));
        master.set_text(0, 0, "Battery: " + battery_stream.str() + "%");
        delay(75);

        // Clamp Temp
        double clamp_temp = clamp.get_temperature();
        std::ostringstream clamp_temp_stream;
        clamp_temp_stream << static_cast<int>(trunc(clamp_temp));
        master.set_text(1, 0, "ClampTemp: " + clamp_temp_stream.str() + "°C");
        delay(75);

        // Intake Temp (average of two temperatures)
        double intake_temp = (intakeC.get_temperature() + intakeF.get_temperature()) / 2.0;
        std::ostringstream intake_temp_stream;
        intake_temp_stream << static_cast<int>(trunc(intake_temp));
        master.set_text(2, 0, "Intk temp: " + intake_temp_stream.str() + "°C");
    }

    count++;
    delay(20); // Run for 20 ms then update
  }
}