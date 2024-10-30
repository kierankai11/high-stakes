#include "main.h"

bool clamp_up = false;
bool forceClampRelease = false;

void clamp_tilt_task_fn(void *params) {
  Controller master(E_CONTROLLER_MASTER);
  Motor clamp(CLAMP_P);
  float error;

  while (true) {
    if (clamp_up) {
        error = CLAMP_TILT_DEGREES - clamp.get_position();
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
  DEFINE_DRIVE(frontRight, DRIVE_FR_P);
  DEFINE_DRIVE(frontLeft, DRIVE_FL_P);
  DEFINE_DRIVE(backLeft, DRIVE_BL_P);
  backLeft.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  DEFINE_DRIVE(backright, DRIVE_BR_P);
  backright.set_brake_mode(E_MOTOR_BRAKE_HOLD);

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
  Motor frontRight(DRIVE_FR_P);
  Motor frontLeft(DRIVE_FL_P);
  Motor backRight(DRIVE_BR_P);
  Motor backLeft(DRIVE_BL_P);

  Motor intakeF(INTAKE_F);
  Motor intakeC(INTAKE_C);
  Motor clamp(CLAMP_P);

  frontRight.move_velocity(200);
  frontLeft.move_velocity(200);
  backRight.move_velocity(200);
  backLeft.move_velocity(200);

  intakeF.move_velocity(200);

  delay(700);

  frontRight.move_velocity(0);
  frontLeft.move_velocity(0);
  backRight.move_velocity(0);
  backLeft.move_velocity(0);
  
  intakeF.move_velocity(0);
  // for next time
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

    intakeC.move(127*(master.get_digital(E_CONTROLLER_DIGITAL_R1)-master.get_digital(E_CONTROLLER_DIGITAL_R2)));
    intakeF.move(127*(master.get_digital(E_CONTROLLER_DIGITAL_R1)-master.get_digital(E_CONTROLLER_DIGITAL_R2)));

    if (master.get_digital_new_press(DIGITAL_L1)) {
      clamp_up = !clamp_up;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      clamp_up = false;
      forceClampRelease  = true;
    }

    delay(20); // Run for 20 ms then update
  }
}