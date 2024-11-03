#include "main.h"
#include "globals.hpp"
#include "mechLib.hpp"
#include "pros/rtos.h"

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
    } else if (forceClampRelease) {
      error = CLAMP_FORCE_DEGREES - clamp.get_position();
      forceClampRelease = false;
      clamp_up = false;
    } else {
      error = CLAMP_DEFAULT_DEGREES - clamp.get_position();
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
  Motor intakeF(INTAKE_F);
  Motor intakeC(INTAKE_C);

  // delay(100); // Try to fix angle problem
  // baseMove(-30, -30);
  // delay(100);
  // clamp_up = true;
  // delay(1000);
  // intakeF.move(127);
  // intakeC.move(127);
  // baseTurn(
  //     100); // Positive bearing = red-, blue+, Negative bearing = red+, blue-
  // delay(100);
  // baseMove(28, 28);
  // delay(1000);
  // intakeF.move(0);
  // intakeC.move(0);
  // baseTurn(
  //     93);
  // // delay(400);
  // // intakeF.move(127);
  // // intakeC.move(127);
  // // delay(100);
  // // baseMove(23, 23);
  // // delay(1000);
  // // intakeF.move(0);
  // // intakeC.move(0);
  // baseMove(15, 15);
  // delay(200);
  // // baseMove(-13, -13);
  // // delay(200);
  // baseTurn(
  //     95);
  // delay(100);
  // baseMove(45, 45);

  // SIGMA
  delay(100); // Wait for init (see if work)
  baseMove(-25.1);
  delay(MOVEDELAY);
  baseTurn(35.1);
  baseMove(-9.2);
  clamp_up = true;
  delay(1000); // Wait for clamp
  baseMove(9.2);
  delay(MOVEDELAY);
  baseTurn(32.5);
  delay(MOVEDELAY);
  intakeF.move(127);
  intakeC.move(127);
  baseMove(24.2);
  delay(MOVEDELAY);
  baseTurn(90 - 32.5 - 35.1);  // Change to absolute value once auton is good
  delay(MOVEDELAY);
  baseMove(18.8);
  delay(700);             // Wait for intake to get 1 of two stacks
  baseMove(-12.9);
  delay(MOVEDELAY);
  baseTurn(33.2);
  delay(MOVEDELAY);
  baseMove(16.9);
  delay(700);             // Wait for intake to get 2 of two stacks
  baseMove(-10);
  intakeF.move(0);
  delay(1000);
  intakeC.move(0);
}

void opcontrol() {
  Motor frontRight(DRIVE_FR_P);
  Motor frontLeft(DRIVE_FL_P);
  Motor backRight(DRIVE_BR_P);
  Motor backLeft(DRIVE_BL_P);

  Motor intakeF(INTAKE_F);
  Motor intakeC(INTAKE_C);
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