using namespace vex;

extern brain Brain;

// VEXcode devices
extern drivetrain Drivetrain;
extern motor_group roboHand_MotorGroup;
extern motor intakeMotor;
extern controller Controller1;
extern digital_out pneaumtaicSystem;
extern distance DistanceSensor;
extern optical opticalVision;
extern inertial inertialSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );