#include "main.h"
#include "mechLib.hpp"

bool clamp_up = false;
bool forceClampRelease = false;

void moveRPM(int left, int right) {
  Motor frontRight(DRIVE_FR_P);
  Motor frontLeft(DRIVE_FL_P);
  Motor backRight(DRIVE_BR_P);
  Motor backLeft(DRIVE_BL_P);

  backRight.move_velocity(right);
  backLeft.move_velocity(left);

  frontRight.move_velocity(right);
  frontLeft.move_velocity(left);
}

void clamp_tilt_task_fn(void *params) {
  Controller master(E_CONTROLLER_MASTER);
  Motor clamp(CLAMP_P);
  float error;

  while (true) {
    if (clamp_up) {
      error = CLAMP_TILT_DEGREES - clamp.get_position();
      master.print(0, 0, "%f", error);
    } else {
      if (forceClampRelease) {
        error = CLAMP_FORCE_DEGREES - clamp.get_position();
        forceClampRelease = false;
        clamp_up = false;
      } else {
        error = CLAMP_DEFAULT_DEGREES - clamp.get_position();
      }
    }
    clamp.move((error * CLAMP_TILT_KP) + 10);
    delay(20);
  }
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
  DEFINE_MOTOR(intakeF, INTAKE_F, v5::MotorGears::blue);
  DEFINE_MOTOR(intakeC, INTAKE_C, v5::MotorGears::green);

  // Clamp
  DEFINE_MOTOR(clamp, CLAMP_P, v5::MotorGears::red);

  lcd::initialize();

  // Master
  Controller master(CONTROLLER_MASTER);

  // clamp pid
  Task clampTiltTask(clamp_tilt_task_fn, (void *)"PROS", TASK_PRIORITY_DEFAULT,
                     TASK_STACK_DEPTH_DEFAULT, "clampPID");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  // Motor frontRight(DRIVE_FR_P);
  // Motor frontLeft(DRIVE_FL_P);
  // Motor backRight(DRIVE_BR_P);
  // Motor backLeft(DRIVE_BL_P);

  // Motor intakeF(INTAKE_F);
  // Motor intakeC(INTAKE_C);

  // // Move backwards towards clamp
  // // moveRPM(-200, -200);
  // backRight.move_velocity(-150);
  // backLeft.move_velocity(-150);

  // frontRight.move_velocity(-150);
  // frontLeft.move_velocity(-150);
  // delay(1200);
  // moveRPM(0, 0);
  // delay(200);

  // // Activate clamp to get stake
  // clamp_up = true;
  // delay(1000); // Wait for clamp

  // // Turn 90 degrees to face a donut (lots of tuning required)
  // moveRPM(150, -150); // for red negative corner, blue positive corner
  // delay(300);
  // moveRPM(0, 0);
  // delay(200);

  // // Move forward and start intake
  // moveRPM(200, 200);
  // intakeF.move_velocity(600);
  // delay(700);
  // moveRPM(0, 0);
  // delay(300);

  // // Score two donuts
  // intakeC.move_velocity(300);
  // for (int i = 0; i < 6; i++) { // Checks if conveyer is stuck 6 times at intervals of 1s. Total of 6 seconds of conveyer moving.
  //   if (intakeC.get_actual_velocity() <= 50) {
  //     intakeC.move_velocity(0);
  //     break;
  //   } else {
  //     delay(1000);
  //   }
  // }

  // // Stop
  // intakeC.move_velocity(0);
  // intakeF.move_velocity(0);
  baseTurn(90);
}

void opcontrol() {
  Motor frontRight(DRIVE_FR_P);
  Motor frontLeft(DRIVE_FL_P);
  Motor backRight(DRIVE_BR_P);
  Motor backLeft(DRIVE_BL_P);

  Motor intakeF(INTAKE_F);
  Motor intakeC(INTAKE_C);
  Motor clamp(CLAMP_P);
  Controller master(CONTROLLER_MASTER);

  int intake_voltage = 0;
  double left, right;

  while (true) {
    left = master.get_analog(ANALOG_LEFT_Y);
    right = master.get_analog(ANALOG_RIGHT_Y);

    frontRight.move(right);
    backRight.move(right);

    backLeft.move(left);
    frontLeft.move(left);

    intakeC.move(127 * (master.get_digital(E_CONTROLLER_DIGITAL_R1) -
                        master.get_digital(E_CONTROLLER_DIGITAL_R2)));
    intakeF.move(127 * (master.get_digital(E_CONTROLLER_DIGITAL_R1) -
                        master.get_digital(E_CONTROLLER_DIGITAL_R2)));

    if (master.get_digital_new_press(DIGITAL_L1)) {
      clamp_up = !clamp_up;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      clamp_up = false;
      forceClampRelease = true;
    }

    delay(20); // Run for 20 ms then update
  }
}