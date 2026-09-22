/**
 * \file robot.h
 *
 * Shared handles for every device and subsystem on the robot, plus the entry
 * point into the autonomous routes.
 *
 * The objects themselves are DEFINED once in src/main.cpp. This header only
 * declares them so other translation units (src/autons.cpp, etc.) can use them.
 *
 * Routes live in src/autons.cpp and are selected inside runAuton(), so adding
 * a new one never requires touching this header.
 */

#pragma once

#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/imu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern lemlib::Drivetrain drivetrain;
extern lemlib::Chassis chassis;

extern pros::Distance frontSensor;
extern pros::Distance leftSensor;
extern pros::Distance rightSensor;
extern pros::Distance backBottomSensor;
extern pros::Distance backTopSensor;
extern pros::Distance clawSensor;


extern pros::Imu imu;
extern pros::Rotation vertical_rotation_sensor;
extern pros::Rotation horizontal_rotation_sensor;
extern lemlib::TrackingWheel vertical_tracking_wheel;
extern lemlib::TrackingWheel horizontal_tracking_wheel;
extern lemlib::OdomSensors sensors;

extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
extern lemlib::ExpoDriveCurve throttle_curve;
extern lemlib::ExpoDriveCurve steer_curve;

extern pros::MotorGroup cascade;
extern pros::Rotation cascade_sensor;
extern lemlib::PID cascade_pid;
void moveCascadeTo(double target);

extern pros::MotorGroup arm_motor;
extern pros::Rotation arm_sensor;
extern lemlib::PID arm_pid;
void moveArmTo(double target);

inline constexpr double downArmDegPinAndCup = 3;
inline constexpr double downArmDegJustPin = 3;
inline constexpr double allianceGoalWithNothing = 5;
inline constexpr double normalAllianceGoal = 5;



extern pros::adi::DigitalOut clawShut;
extern bool flipBool;
extern pros::adi::DigitalOut leftToggle;
extern pros::adi::DigitalOut rightToggle;


// ---------------------------------------------------------------- controller
extern pros::Controller controller;

// -------------------------------------------------------- autonomous (autons.cpp)
/// Runs the selected autonomous route. Pick which one inside src/autons.cpp.
void runAuton();

// Tuning routines, callable from opcontrol() for testing.
void darwin_test();
void frictiontest();
void torquetest();

// Suppresses the brain-screen odometry readout so the tuner can own the screen.
extern bool printingDistances;

/// Blocking turn-PID auto-tuner (~30-60s). Give it clear floor space and call it
/// once. startKD = 0 picks a starting kD automatically from kP.
void turnTunerAuton(double kP, double startKD = 0);

/// Set true from any task to abort turnTunerAuton(); holding X also works.
extern bool stopTuning;

void L1Button();
void L2Button();
void R1Button();
void toggleMech();
