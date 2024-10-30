#define DRIVE_BL_P             15
#define DRIVE_BR_P             -16

#define DRIVE_FL_P             8
#define DRIVE_FR_P             -9

// INTAKE
#define INTAKE_F               10
#define INTAKE_C               11

// CLAMP
#define CLAMP_P                17
#define CLAMP_TILT_DEGREES     200
#define CLAMP_DEFAULT_DEGREES  -70  // works
#define CLAMP_TILT_KP          0.7          // important for accuracy

#define DEFINE_DRIVE(name, port) pros::Motor name (port, pros::v5::MotorGears::red, pros::v5::MotorEncoderUnits::degrees)
#define DEFINE_MOTOR(name, port, gear) pros::Motor name (port, gear, pros::v5::MotorEncoderUnits::degrees) 