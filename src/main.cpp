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
#include <cmath>
#include <cstdio>
pros::MotorGroup right_motors({11, 12, -13}, pros::MotorGearset::blue); // left motors use 600 RPM cartridges
pros::MotorGroup left_motors({-20, -19, 18}, pros::MotorGearset::blue); // right motors use 200 RPM cartridges
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              12.6, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// pros::Distance backSensor(20);
pros::Distance frontSensor(15);
pros::Distance leftSensor(17);
pros::Distance rightSensor(14);

pros::Imu imu(5);
pros::Rotation vertical_rotation_sensor(-21);
pros::Rotation horizontal_rotation_sensor(4);
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_rotation_sensor, lemlib::Omniwheel::NEW_275, -1);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_rotation_sensor, lemlib::Omniwheel::NEW_275, -1.5);

pros::MotorGroup cascade ({1, -2}, pros::v5::MotorGears::blue);//2 11w
pros::Rotation cascade_sensor(3);
lemlib::PID cascade_pid(2,0,0,0);
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
bool clawBool = false;
pros::adi::DigitalOut clawShut(1, clawBool);
// TODO: placeholder ADI ports -- update to the real toggle ports
bool flipBool = false;
pros::adi::DigitalOut leftToggle(2, false);
pros::adi::DigitalOut rightToggle(3, false);

pros::Distance claw_sensor(9);
pros::Distance backBottomSensor(8);
pros::Distance backTopSensor(8);

pros::MotorGroup arm_motor ({6, -7}, pros::v5::MotorGears::green);//2 5.5
pros::Rotation arm_sensor(10);
lemlib::PID arm_pid(2,0,0,0);
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
int pressNum = 0;
// Cleared by turnTunerAuton() so the screen task stops overwriting its readout.
bool printingDistances = true;
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    cascade_sensor.reset_position();
    arm_sensor.reset_position();
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
            if (not printingDistances) {
                pros::delay(40);
                continue;
            }

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
    pros::Task armTask([&]() {
        while (true) {
            L1Button();
            // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
            //     clawBool=!clawBool;
            //     clawShut.set_value(clawBool);
            // }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){//maybe switch the order of these two?
            //     if(clawBool==false && claw_sensor.get_distance() < 100){
            //         clawBool=true;
            //         clawShut.set_value(clawBool);
            //     }
            // }else if(clawBool==false){
            //     moveArmTo(downArmDegPinAndCup);
            // }
            L2Button();
            // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){//maybe switch the order of these two?
            //     if(clawBool==false){
            //         moveArmTo(downArmDegJustPin);
            //     }else{//claw bool == true
            //         clawShut.set_value(false);
            //         pros::delay(200);
            //         clawShut.set_value(true);
            //     }
            // }
            R1Button();
            // if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){//ARM movement
            //     if(pressNum++ == 0){
            //         moveArmTo(170);
            //     }else if (pressNum == 1){
            //         if(backBottomSensor.get_distance() > 500 && backTopSensor.get_distance() > 500){
            //             moveArmTo(270);
            //         }else if(backBottomSensor.get_distance() < 200 && backTopSensor.get_distance() > 500){
            //             moveArmTo(360);
            //         }else if(backBottomSensor.get_distance() < 200 && backTopSensor.get_distance() < 200){
            //             while(backTopSensor.get_distance() < 200){
            //                 cascade.move(127);
            //             }
            //             moveArmTo(360);
            //         }

            //         pressNum=0;
            //         clawShut.set_value(false);
            //         clawBool=false;
            //     }
            //     arm_motor.move(127);
            // }

            pros::delay(20);
            }
        });

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
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            turnTunerAuton(2.0);   // your angular kP; second arg 0 = auto-pick starting kD
        }
        // get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);

        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){//cascade movement
            cascade.move(127);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            cascade.move(-127);
            std::cout << "Testing" << std::endl;
        }else{
            cascade.move(0);
        }
        toggleMech();
        
        //cascade winch toggle
        // delay to save resources
        pros::delay(25);
    }
}
void L1Button(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
        clawBool=!clawBool;
        clawShut.set_value(clawBool);
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){//maybe switch the order of these two?
        if(clawBool==false && claw_sensor.get_distance() < 100){
            clawBool=true;
            clawShut.set_value(clawBool);
        }
    }else if(clawBool==false){
        moveArmTo(downArmDegPinAndCup);
    }
};
void L2Button(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){//maybe switch the order of these two?
        if(clawBool==false){
            moveArmTo(downArmDegJustPin);
        }else{//claw bool == true
            clawShut.set_value(false);
            pros::delay(200);
            clawShut.set_value(true);
        }
    }
};
void R1Button(){
    if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){//ARM movement
        if(pressNum++ == 0){
            moveArmTo(170);
        }else if (pressNum == 1){
            if(backBottomSensor.get_distance() > 500 && backTopSensor.get_distance() > 500){
                moveArmTo(270);
            }else if(backBottomSensor.get_distance() < 200 && backTopSensor.get_distance() > 500){
                moveArmTo(360);
            }else if(backBottomSensor.get_distance() < 200 && backTopSensor.get_distance() < 200){
                while(backTopSensor.get_distance() < 200){
                    cascade.move(127);
                }
                moveArmTo(360);
            }

            pressNum=0;
            clawShut.set_value(false);
            clawBool=false;
        }
        arm_motor.move(127);
    }
};
void toggleMech(){
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        leftToggle.set_value(true);
        rightToggle.set_value(true);
    }else{
        leftToggle.set_value(false);
        rightToggle.set_value(false);
    }
};

// =========================================================================
//   Turn PID auto-tuner
// =========================================================================
//
// You set the desired kP. The tuner repeatedly turns the robot IN PLACE
// (alternating direction so it never drifts across the field) and:
//   Phase 1 (BRACKET): climbs kD until overshoot/oscillation is gone.
//   Phase 2 (REFINE):  golden-section searches that region to minimize a cost
//                      built from {overshoot, oscillations, settle time,
//                      steady-state error}.
// It reports the lowest-cost kD it found on the Brain + Controller screens.
//
// kI WHILE TUNING: keep it 0. The integral only acts within windupRange degrees
// of the target -- exactly the settling region the tuner measures -- so a
// nonzero kI adds its own slow limit-cycle + phase lag that corrupts the
// overshoot/oscillation metric you're minimizing. Tune kD with kI = 0, then
// add a small kI (~0.01) afterward purely to erase steady-state error.
//
// HOW TO RUN IT: this is a blocking routine (it loops until the search finishes,
// ~30-60s). Trigger it ONCE -- e.g. behind a button in the driver loop:
//     if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
//         turnTunerAuton(2.0);
//     }
// Give the robot clear floor space first. Hardcode the printed kD into
// angular_controller when it's done.
//
// KILL SWITCH: hold X on the controller during a test to abort. The best-so-far
// values stay frozen on the screen.

struct TurnTuneCfg {
    double kP = 0;              // set per call
    double kI = 0;             // keep 0 while tuning (see note above)

    double testAngle = 90;     // degrees per test turn

    double kDStart = 0;        // bracket start (0 = auto: kP * 2)
    double kDMin = 0;          // bracket floor for the downward search (0 = auto: kP * 0.5)
    double kDMax = 0;          // bracket ceiling (0 = auto: kP * 200)
    double growth = 1.4;       // multiply/divide kD by this each bracket step
    int    refineIters = 6;    // golden-section iterations

    double testDuration = 2500; // ms observed per turn (must outlast settling)
    double settleTol = 0.75;    // deg counted as "at target"
    double settleHold = 150;    // ms continuously within settleTol = settled
    double ssWindow = 300;      // ms tail window averaged for steady-state error

    double overshootTol = 1.5;  // deg overshoot allowed to count as "oscillation gone"
    int    oscTol = 1;          // error sign-changes allowed

    double wOvershoot = 2.0;    // cost per degree of overshoot       (punishes kD too LOW)
    double wOscillation = 3.0;  // cost per oscillation                (punishes kD too LOW)
    double wRise = 2.0;         // cost per second of rise time        (punishes kD too HIGH / early decel)
    double wSettle = 1.0;       // cost per second of settle time
    double wSteady = 4.0;       // cost per degree of steady-state error

    double settleBetween = 400; // ms pause between tests
    double maxPower = 127;      // motor command cap -- PID output is unbounded
};

struct TurnTestResult {
    double overshoot;
    int    oscillations;
    double riseTime;     // ms to first reach the target band -- large = decelerated too early
    double settleTime;
    double steadyError;
    double cost;
    bool   valid;        // false if the test was cut short by the kill switch
};

// Set true internally once an abort has been latched -- breaks out of every loop.
static bool g_tuneAborted = false;

// Kill switch. Set this true from anywhere (e.g. a button in another task) to
// stop the tuner; the best-so-far values stay frozen on screen.
bool stopTuning = false;

static bool tunerKilled() {
    // Direct physical kill. turnTunerAuton() blocks the driver loop, so a bool you
    // set from that loop can never be reached -- holding X here flips the bool from
    // inside the tuner instead. You can still set stopTuning = true from any task.
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
        stopTuning = true;
    }
    return stopTuning;
}

// -1 / 0 / +1. Returns 0 inside the deadband so IMU jitter around the target
// isn't counted as a zero-crossing.
static int tuneSign(double x, double deadband) {
    if (x > deadband) return 1;
    if (x < -deadband) return -1;
    return 0;
}

// Wrap a heading into [0, 360).
static double wrapDeg(double deg) {
    deg = std::fmod(deg, 360.0);
    if (deg < 0) deg += 360.0;
    return deg;
}

// Command an in-place turn. Positive power turns clockwise (increasing heading),
// matching lemlib's convention where Chassis::arcade's turn term is left +, right -.
static void tuneTurnDrive(double power, double maxPower) {
    if (power > maxPower) power = maxPower;
    if (power < -maxPower) power = -maxPower;
    left_motors.move(power);
    right_motors.move(-power);
}

// One test turn to targetH using kP/kI/kD. Measures the transient + cost.
static TurnTestResult runTurnTest(const TurnTuneCfg& cfg, double kD, double targetH) {
    TurnTestResult r;
    r.overshoot = 0;
    r.oscillations = 0;
    r.riseTime = cfg.testDuration;
    r.settleTime = cfg.testDuration;
    r.steadyError = 0;
    r.cost = 0;
    r.valid = true;

    // lemlib::PID stores its gains as const, so a fresh controller per test is
    // how you change kD -- there is no setConstants()/tempConstants().
    lemlib::PID turnPID(cfg.kP, cfg.kI, kD);

    double startTime = pros::millis();
    double elapsed = 0;

    int    initialSign = 0;
    int    lastSign = 0;
    bool   reached = false;
    bool   settled = false;
    double settleEnterTime = -1;

    double ssSum = 0;
    int    ssCount = 0;

    while (elapsed < cfg.testDuration) {
        if (tunerKilled()) {
            g_tuneAborted = true;
            r.valid = false;
            break;
        }

        double error = lemlib::angleError(targetH, chassis.getPose().theta, false);
        double turnPow = turnPID.update(error);
        tuneTurnDrive(turnPow, cfg.maxPower);

        int s = tuneSign(error, cfg.settleTol);

        if (initialSign == 0 && s != 0) {
            initialSign = s;
            lastSign = s;
        }

        // oscillation = error crossing zero (sign flip). The deadband in
        // tuneSign() means only a genuine excursion past the target band counts,
        // not sensor noise jittering across zero while parked on target.
        if (s != 0 && lastSign != 0 && s != lastSign) {
            r.oscillations++;
        }
        if (s != 0) {
            lastSign = s;
        }

        // overshoot = error on the opposite side of where it started
        if (initialSign != 0 && s == -initialSign) {
            double mag = std::fabs(error);
            if (mag > r.overshoot) {
                r.overshoot = mag;
            }
        }

        // rise = first moment it reaches the target band. Stays large for an
        // overdamped kD that decelerates early and crawls in.
        if (not reached && std::fabs(error) <= cfg.settleTol) {
            reached = true;
            r.riseTime = elapsed;
        }

        // settle = stayed within settleTol continuously for settleHold ms
        if (std::fabs(error) <= cfg.settleTol) {
            if (settleEnterTime < 0) {
                settleEnterTime = elapsed;
            }
            if (not settled && (elapsed - settleEnterTime) >= cfg.settleHold) {
                settled = true;
                r.settleTime = settleEnterTime;
            }
        } else {
            settleEnterTime = -1;
        }

        // steady-state error = average |error| over the tail window
        if (elapsed >= cfg.testDuration - cfg.ssWindow) {
            ssSum += std::fabs(error);
            ssCount++;
        }

        pros::delay(10);
        elapsed = pros::millis() - startTime;
    }

    tuneTurnDrive(0, cfg.maxPower);

    if (ssCount > 0) {
        r.steadyError = ssSum / ssCount;
    }

    r.cost = cfg.wOvershoot * r.overshoot
           + cfg.wOscillation * (double)r.oscillations
           + cfg.wRise * (r.riseTime / 1000.0)
           + cfg.wSettle * (r.settleTime / 1000.0)
           + cfg.wSteady * r.steadyError;

    return r;
}

static void printTuneStatus(const char* phase, double kP, double kD,
                            const TurnTestResult& r, double bestKD, double bestCost) {
    pros::lcd::print(0, "Turn PID Tuner  [%s]", phase);
    pros::lcd::print(1, "kP %.3f   testing kD %.3f", kP, kD);
    pros::lcd::print(2, "overshoot    %.2f deg", r.overshoot);
    pros::lcd::print(3, "oscillations %d", r.oscillations);
    pros::lcd::print(4, "rise %.0f  settle %.0f ms", r.riseTime, r.settleTime);
    pros::lcd::print(5, "steady err   %.2f deg", r.steadyError);
    pros::lcd::print(6, "cost %.2f   best kD %.3f (%.2f)", r.cost, bestKD, bestCost);
}

// Runs one test against a fixed pair of headings, and tracks the best result.
// Targets alternate between baseH and baseH + testAngle -- both absolute -- so a
// test that ends short of its target can't drag the next one across the field.
static TurnTestResult runTestAt(const TurnTuneCfg& cfg, double kD, const char* phase,
                                double baseH, int& leg, double& bestKD, double& bestCost) {
    double target = wrapDeg(baseH + (leg ? cfg.testAngle : 0.0));
    leg = 1 - leg;

    TurnTestResult r = runTurnTest(cfg, kD, target);

    // An aborted test stops partway through, so its metrics describe a turn that
    // never happened. Scoring it could pin bestKD to a value that was never
    // actually measured.
    if (r.valid && r.cost < bestCost) {
        bestCost = r.cost;
        bestKD = kD;
    }

    printTuneStatus(phase, cfg.kP, kD, r, bestKD, bestCost);
    if (not g_tuneAborted) {
        pros::delay(cfg.settleBetween);
    }
    return r;
}

void turnTunerAuton(double kP, double startKD) {
    printingDistances = false;
    g_tuneAborted = false;
    stopTuning = false;
    pros::lcd::clear();

    TurnTuneCfg cfg;
    cfg.kP = kP;
    cfg.kDStart = startKD;   // 0 = auto (kP * 2)

    double kDStart = cfg.kDStart;
    if (kDStart <= 0) {
        kDStart = cfg.kP * 2.0;
    }
    if (kDStart <= 0) {
        kDStart = 1.0;
    }

    double kDMax = cfg.kDMax;
    if (kDMax <= 0) {
        kDMax = cfg.kP * 200.0;
    }
    if (kDMax <= 0) {
        kDMax = 200.0;
    }

    double kDMin = cfg.kDMin;
    if (kDMin <= 0) {
        kDMin = cfg.kP * 0.5;
    }
    if (kDMin <= 0) {
        kDMin = 0.1;
    }

    // An explicit startKD outside the auto-derived bracket would otherwise leave
    // the golden-section search with an inverted interval.
    if (kDMax < kDMin) {
        kDMax = kDMin;
    }
    if (kDStart < kDMin) {
        kDStart = kDMin;
    }
    if (kDStart > kDMax) {
        kDStart = kDMax;
    }

    // Anchor every test turn to the heading the robot starts at.
    double baseH = wrapDeg(chassis.getPose().theta);
    int leg = 1;
    double bestKD = kDStart;
    double bestCost = 1e18;

    // Phase 1: bracket the optimum. We want kDLow to OSCILLATE (kD too low) and
    // kDHigh to be STABLE (kD high enough), so the refine search sits between them.
    double kDLow = kDMin;
    double kDHigh = kDMax;

    TurnTestResult r0 = runTestAt(cfg, kDStart, "BRACKET", baseH, leg, bestKD, bestCost);
    bool startStable = (r0.overshoot <= cfg.overshootTol) && (r0.oscillations <= cfg.oscTol);

    if (startStable) {
        // kDStart already kills oscillation -- it may be TOO high (decelerating
        // early). Walk DOWN until oscillation reappears, so we find the smallest
        // stable kD (the fastest one) instead of trusting the high starting value.
        kDHigh = kDStart;
        double kD = kDStart / cfg.growth;
        while (kD >= kDMin) {
            if (g_tuneAborted) {
                break;
            }
            TurnTestResult r = runTestAt(cfg, kD, "DESCEND", baseH, leg, bestKD, bestCost);
            bool stable = (r.overshoot <= cfg.overshootTol) && (r.oscillations <= cfg.oscTol);
            if (not stable) {
                kDLow = kD;     // found the oscillation boundary below
                break;
            }
            kDHigh = kD;        // still stable -- push the ceiling lower and keep going
            kD = kD / cfg.growth;
        }
    } else {
        // kDStart oscillates -- climb UP until oscillation is eliminated.
        kDLow = kDStart;
        bool found = false;
        double kD = kDStart * cfg.growth;
        while (kD <= kDMax) {
            if (g_tuneAborted) {
                break;
            }
            TurnTestResult r = runTestAt(cfg, kD, "ASCEND", baseH, leg, bestKD, bestCost);
            bool stable = (r.overshoot <= cfg.overshootTol) && (r.oscillations <= cfg.oscTol);
            if (stable) {
                kDHigh = kD;
                found = true;
                break;
            }
            kDLow = kD;
            kD = kD * cfg.growth;
        }
        if (not found) {
            kDHigh = kDMax;
        }
    }

    // Phase 2: golden-section refine within [kDLow, kDHigh], minimizing cost.
    double a = kDLow;
    double b = kDHigh;
    if (b < a) {
        // Aborting mid-bracket can leave the interval inverted; golden-section
        // would then evaluate kD values outside the bracket entirely.
        double swap = a;
        a = b;
        b = swap;
    }
    const double invphi = 0.6180339887;

    double c = b - invphi * (b - a);
    double d = a + invphi * (b - a);

    double fc = 0;
    double fd = 0;
    if (not g_tuneAborted) {
        fc = runTestAt(cfg, c, "REFINE", baseH, leg, bestKD, bestCost).cost;
    }
    if (not g_tuneAborted) {
        fd = runTestAt(cfg, d, "REFINE", baseH, leg, bestKD, bestCost).cost;
    }

    for (int i = 0; i < cfg.refineIters; i++) {
        if (g_tuneAborted) {
            break;
        }
        if (fc < fd) {
            b = d;
            d = c;
            fd = fc;
            c = b - invphi * (b - a);
            fc = runTestAt(cfg, c, "REFINE", baseH, leg, bestKD, bestCost).cost;
        } else {
            a = c;
            c = d;
            fc = fd;
            d = a + invphi * (b - a);
            fd = runTestAt(cfg, d, "REFINE", baseH, leg, bestKD, bestCost).cost;
        }
    }

    tuneTurnDrive(0, cfg.maxPower);

    // Freeze the best-so-far values on screen. printingDistances stays false so the
    // brain-screen task won't overwrite this; the readout persists until next run.
    pros::lcd::clear();
    if (g_tuneAborted) {
        pros::lcd::print(1, "TUNING ABORTED - best so far");
    } else {
        pros::lcd::print(1, "TUNING COMPLETE");
    }
    pros::lcd::print(3, "kP    %.3f", cfg.kP);
    pros::lcd::print(4, "kD    %.3f   <-- use this", bestKD);
    pros::lcd::print(5, "cost  %.2f", bestCost);

    controller.set_text(0, 0, "                ");
    controller.set_text(1, 0, "                ");
    pros::delay(60);
    char line[24];
    std::snprintf(line, sizeof(line), "kP %.3f", cfg.kP);
    controller.set_text(0, 0, line);
    pros::delay(60);
    std::snprintf(line, sizeof(line), "best kD %.3f", bestKD);
    controller.set_text(1, 0, line);
    controller.rumble("---");
}
