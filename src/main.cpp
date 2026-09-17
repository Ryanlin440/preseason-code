#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "robot.h"
pros::MotorGroup right_motors({11, 12, -7}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup left_motors({-20, -19, 9}, pros::MotorGearset::blue); // right motors use 200 RPM cartridges
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.6, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// pros::Distance backSensor(20);
pros::Distance frontSensor(20);
pros::Distance leftSensor(17);
pros::Distance rightSensor(16);

pros::Imu imu(2);
pros::Rotation vertical_rotation_sensor(-5);
pros::Rotation horizontal_rotation_sensor(3);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor, lemlib::Omniwheel::NEW_275, -1);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation_sensor, lemlib::Omniwheel::NEW_275, -1.5);

pros::MotorGroup cascade ({10, -1}, pros::v5::MotorGears::blue);//2 11w
pros::Rotation cascade_sensor(6);
lemlib::PID cascade_pid(0,0,0,0);
void moveCascadeTo(double target){

    cascade_pid.reset();
    while (true) {
        float current_pos = cascade_sensor.get_position() / 100.0; 
        float error = target - current_pos;
        float output = cascade_pid.update(error);

        cascade.move(output);

        if (std::abs(error) < 1.0) {
            cascade.move(0);
            break;
        }
        pros::delay(10);
    }
}

// TODO: placeholder port/gearset -- update to the real toggles port(s)
pros::MotorGroup toggles({13}, pros::v5::MotorGears::green);

pros::adi::DigitalOut clawShut(1, false);

bool flipBool = false;
pros::adi::DigitalOut clawFlip(2, flipBool);
pros::Distance claw_sensor(14);


pros::MotorGroup arm_motor ({15, -16}, pros::v5::MotorGears::green);//2 5.5
pros::Rotation arm_sensor(8);
lemlib::PID arm_pid(0,0,0,0);
void moveArmTo(double target){

    arm_pid.reset();
    while (true) {
        float current_pos = arm_sensor.get_position() / 100.0; 
        float error = target - current_pos;
        float output = arm_pid.update(error);

        arm_motor.move(output);

        if (std::abs(error) < 1.0) {
            arm_motor.move(0);
            break;
        }
        pros::delay(10);
    }
}

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, 
                        &steer_curve
);
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
int count = 20;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    cascade_sensor.reset_position();
    // while (true) {
    //         // print robot location to the brain screen
    //         // pros::lcd::print(3, "Front sensor: %.2f", (double)(frontSensor.get() / 25.4));
    //         pros::lcd::print(4, "asdl %d", count);
    //         // delay to save resources
    //         count+=5;
    //         pros::delay(200);
    //     }
    pros::Task screenTask([&]() {
        // pros::lcd::print(0, "TASK RUNNING");
        // count = 20;
        while (true) {
            // print robot location to the brain screen

            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Front sensor: %.2f", (double)(frontSensor.get() / 25.4));
            pros::lcd::print(4, "Cascade Sensor: %.2f", (double)(cascade_sensor.get_position()/100.0));
            // pros::lcd::print(4, "asdl %d", count);
            // count+=2;
            // delay to save resources
            pros::delay(40);
            //
        }
    });
    // pros::Task armTask([&]() {
    //     while (true) {
    //         if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
    //             moveArmTo(upArmDeg);
    //         }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
    //             moveArmTo(downArmDeg);
    //         }
    //         pros::delay(20);
    //         }
    //     });

}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() { runAuton(); }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void opcontrol() {
    // loop forever
    // autonomous();
    while (true) {
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);


        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && claw_sensor.get_distance() < 20){//opening and closing claw
            clawShut.set_value(true);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            clawShut.set_value(false);
        }
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){//cascade movement
            cascade.move(127);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
            cascade.move(-127);
            std::cout << "Testing" << std::endl;
        }else{
            cascade.move(0);
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)){//flipping the claw yaw
            flipBool = !flipBool;
            clawFlip.set_value(flipBool);
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){//moving the toggles
            toggles.move(-127);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            toggles.move(-50);
        }else{
            toggles.move(0);
        }
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){//ARM movement
            arm_motor.move(127);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            arm_motor.move(-127);
        }else{
            arm_motor.move(0);
        }

        //cascade winch toggle
        // delay to save resources
        pros::delay(25);
    }
}
