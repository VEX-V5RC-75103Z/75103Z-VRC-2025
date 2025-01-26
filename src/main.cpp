#include "main.h"

#include <cmath>
#include <numeric>

//#include "lemlib/api.hpp"  // IWYU pragma: keep
//#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
//#include "pros/llemu.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/ai_vision.h"
#include "pros/ai_vision.hpp"
#include "pros/colors.hpp"
#include "pros/screen.h"
#include "pros/screen.hpp"
#define pi 3.141592653589793

#define as(a, b, c, d) for (auto a = b; a < c; a += d)
#define de(a, b, c, d) for (auto a = b; a > c; a -= d)

int sortedColor = 0;                    // 0 = keep blue, 1 = keep red, 2 = none
bool autonElim;                         // F = qual auton, T = elim auton
bool autonSide;                         // T = close, F = far
const int wheelCirc = 220;              // in mm
const int driveEncoders = 300;          // ticks per revolution
const double trackWidth = 10.8 * 25.4;  // conversion to mm
int lbStates[3] = {0, 338, 217}; // list of all the states
int lbState = 0;                        // current state it is in
const int lbTotalStates = sizeof(lbStates) / sizeof(lbStates[0]);  // total number of states

pros::MotorGroup left({11, 12, 13}, pros::MotorGearset::blue);
pros::MotorGroup right({18, 19, 20}, pros::MotorGearset::blue);
pros::Motor roller(1, pros::MotorGearset::green);
pros::Motor chain(-8, pros::MotorGearset::blue);
pros::Motor lb(10, pros::MotorGearset::blue);

pros::Rotation lbRotation(10);
pros::Imu inertial(11);
//pros::Vision vision(4);
pros::AIVision vision(4);

pros::adi::Pneumatics mogoLeft('a', false);
pros::adi::Pneumatics mogoRight('b', false);

pros::screen_touch_status_s_t touchStatus;

/*pros::vision_object_s_t keepRed =
    vision.get_by_sig(0, 1);  // sorts out red donuts
pros::vision_object_s_t keepBlue =
    vision.get_by_sig(0, 2);  // sorts out blue donuts*/

pros::aivision_object_s_t colorSort = vision.get_object(0);

pros::Controller ctrl(CONTROLLER_MASTER);  // controller here

auto vector_sum(auto vector){
  return std::reduce(vector.begin(), vector.end());
}

void setupUI() {
  /*screen layout
    1: hi (can be changed later)
    2: temp flags
    3: comp mode flags

    then buttons (hopefully)
  */
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(TEXT_MEDIUM, 1, "hi");

  int leftTemp = std::round((left.get_temperature(0) + left.get_temperature(1) + left.get_temperature(2)) / 3.0);
  int rightTemp = std::round((right.get_temperature(0) + right.get_temperature(1) + right.get_temperature(2)) / 3.0);
  int rollerTemp = std::round(roller.get_temperature());
  int chainTemp = std::round(chain.get_temperature());
  int lbTemp = std::round(lb.get_temperature());
  pros::screen::print(TEXT_MEDIUM, 2, "L: %d R: %d I1: %d I2: %d LB: %d", leftTemp, rightTemp, rollerTemp, chainTemp, lbTemp);

  pros::screen::draw_rect(10, 190, 160, 230); //color button
  if(sortedColor == 0) {
    pros::screen::print(TEXT_MEDIUM_CENTER, 85, 210, "Color: Blue");
    pros::screen::set_pen(0x00FF4747);
    pros::screen::fill_rect(11, 191, 159, 229);
  }
  else if (sortedColor == 1) {
    pros::screen::print(TEXT_MEDIUM_CENTER, 85, 210, "Color: Red");
    pros::screen::set_pen(0x00478BFF);
    pros::screen::fill_rect(11, 191, 159, 229);
  }
  else {
    pros::screen::print(TEXT_MEDIUM_CENTER, 85, 210, "Color: none?");
    pros::screen::set_pen(0x00919191);
    pros::screen::fill_rect(11, 191, 159, 229);
  }

  pros::screen::draw_rect(165, 190, 315, 230);  //elim/qual button
  if(autonElim) {
    pros::screen::print(TEXT_MEDIUM_CENTER, 240, 210, "Auton Mode: Elims");
    pros::screen::set_pen(0x0047FF47);
    pros::screen::fill_rect(166, 191, 314, 229);
  }
  else {
    pros::screen::print(TEXT_MEDIUM_CENTER, 240, 210, "Auton Mode: Quals");
    pros::screen::set_pen(0x00FF4747);
    pros::screen::fill_rect(166, 191, 314, 229);
  }

  pros::screen::draw_rect(320, 190, 470, 230); //close/far button
  if(autonSide) {
    pros::screen::print(TEXT_MEDIUM_CENTER, 395, 210, "Auton Side: Close");
    pros::screen::set_pen(0x0047FF47);
    pros::screen::fill_rect(321, 191, 469, 229);
  }
  else {
    pros::screen::print(TEXT_MEDIUM_CENTER, 395, 210, "Auton Side: Far");
    pros::screen::set_pen(0x00FF4747);
    pros::screen::fill_rect(321, 191, 469, 229);
  }
}

void donut_detected() {
  chain.set_brake_mode(pros::MotorBrake::brake);  // to effectively fling
  ctrl.rumble(".");                               // alerts driver
  pros::delay(90);                                // adjustable
  chain.brake();
  pros::delay(200);
}

void donut_not_detected() {  // resets it to coast when not sorting
  chain.set_brake_mode(pros::MotorBrake::coast);
}

// Moves the robot forward and backward
void drive(int inchesDist, bool forward, int rpm) {
  left.tare_position_all();
  right.tare_position_all();
  double mmDist = inchesDist * 25.4;
  double rotations = round(10 * (mmDist / wheelCirc)) * 0.1;
  double ticks = round(rotations * driveEncoders);
  double pause = (rotations / rpm) * 60000;

  if (forward) {
    left.move_absolute(ticks, rpm);
    right.move_absolute(ticks, rpm);
  } else {
    left.move_absolute(-ticks, rpm);
    right.move_absolute(-ticks, rpm);
  }

  pros::delay(pause + 100);
}

void turn(double degrees, bool turnLeft, int rpm) {
  left.tare_position_all();
  right.tare_position_all();
  double turnCirc = pi * trackWidth;
  double arcLen = (degrees / 360) * turnCirc;
  double rotations = arcLen / wheelCirc;
  double ticks = round(rotations * driveEncoders);
  double pause = (rotations / rpm) * 60 * 1000;

  if (turnLeft) {
    left.move_absolute(-ticks, rpm);
    right.move_absolute(ticks, rpm);
  } else {
    left.move_absolute(ticks, rpm);
    right.move_absolute(-ticks, rpm);
  }

  pros::delay(pause + 100);
}
void ladyBrownCycle(bool forward) {
  if (forward) {
    lbState++;
  } else {
    lbState--;
  }
  lbState = lbState % lbTotalStates;
}
void ladyBrownSet() {
  double kp = 1.5;
  double error = (lbStates[lbState] - lbRotation.get_position());
  double movePower = kp * error;
  lb.move(-movePower);
}

void toHeading(double degrees, int rpm) {
  double kp = 0.5;
  while (true) {
    double error =
        degrees - inertial.get_heading();  // heading is clockwise adi :skull:
    error = fmod(degrees, 360);
    double turnControl = kp * error;
    if (error < 180) {  // turn right
      turnControl = kp * error;
      left.move(turnControl);
      right.move(-turnControl);
    }
    if (error > 180) {  // turn left
      turnControl = kp * (360 - error);
      left.move(-turnControl);
      right.move(turnControl);
    }

    // to break out of while true
    if (error <= 0.5 || error >= 359.5) {
      break;
    }
  }
  pros::delay(100);
}

// old turn func but hopefully better
void inertialTurn(double degrees, int rpm) {
  double kp = 0.5;
  // remember: sensor thinks CCW is negative, we say CCW is positive
  // so every time we get the inertial sensor rotation we mult by -1
  double initPos = -1 * inertial.get_rotation();
  double finalPos = initPos + degrees;
  while (true) {
    double needToTurn = finalPos - (-1 * inertial.get_rotation());
    double turnControl = kp * needToTurn;

    left.move(turnControl * -1);
    right.move(turnControl);

    // to break out of while true
    if (std::abs(needToTurn) <= 0.5) {
      break;
    }
  }

  pros::delay(100);
}

void mogoRetract() {
  mogoLeft.retract();
  mogoRight.retract();
}

void mogoExtend() {
  mogoLeft.extend();
  mogoRight.extend();
}

void intake() {
  roller.move(127);
  chain.move(127);
  while (true) {
    if (sortedColor == 0) {
      if (vision.get_object_count() > 0) { //keep red, fling blue
        donut_detected();
        break;
      } else {
        donut_not_detected();
      }
    } else {
      if (vision.get_object_count() > 0) { //keep blue, fling red
        donut_detected();
        break;
      } else {
        donut_not_detected();
      }
    }
  }
}


void initialize() {
  /*pros::lcd::initialize();*/  
  pros::Task([] {
      setupUI();
      //touch inputs (pls work)
      if((touchStatus.x > 10 && touchStatus.x < 160) && (touchStatus.y > 190 && touchStatus.y < 230)) {
        if(sortedColor < 2 && sortedColor > 0) {
          sortedColor++;
        }
        else {
          sortedColor = 0;
        }
      }
      if((touchStatus.x > 165 && touchStatus.x < 315) && (touchStatus.y > 190 && touchStatus.y < 230)) {autonElim = !autonElim;}
      if((touchStatus.x > 320 && touchStatus.x < 470) && (touchStatus.y > 190 && touchStatus.y < 230)) {autonSide = !autonSide;}

    /*if (sortedColor == 0) {  // auton color info
      pros::lcd::print(1, "LB: Sorting for BLUE");
      ctrl.print(1, 0, "LB: Sorting for BLUE");
    } else if (sortedColor == 1) {
      pros::lcd::print(1, "LB: Sorting for RED");
      ctrl.print(1, 0, "LB: Sorting for RED");
    } else {
      pros::lcd::print(1, "LB: Sorting for N/A");
      ctrl.print(1, 0, "LB: Sorting for N/A");
    }

    if (autonElim) {
      pros::lcd::print(2, "CB: ELIM auton");
    } else {
      pros::lcd::print(2, "CB: QUAL auton");
    }

    if (autonSide) {
      pros::lcd::print(3, "RB: CLOSE side auton");
    } else {
      pros::lcd::print(3, "RB: FAR side auton");
    }*/
    // insert temperature flags when all the motors are defined
  });
  pros::Task([] {
    ladyBrownSet();  // rotates the lady brown thing to the state
  });
  pros::Task([] {
    if(sortedColor == 0) {
      colorSort = vision.get_object(0);
    }

    else if(sortedColor == 1) {
      colorSort = vision.get_object(1);
    }
    if (sortedColor == 0) {
      if (vision.get_object_count() > 0) { //fling red, keep blue
        donut_detected();
      } else {
        donut_not_detected();
      }
    }
    if (sortedColor == 1) {
      if (vision.get_object_count() > 0) { //fling blue, keep red
        donut_detected();
      } else {
        donut_not_detected();
      }
    }
    if (sortedColor == 2) {
      donut_not_detected();
    }
  });
  /*It's good to have an lcd layout to give flags etc to the driver; you can do
  this through pros::lcd::print() which is to the brain or ctrl.print() which is
  to the controller, it's up to you to decide where! (example from mentor code):
  lcd layout (max 8 lines):
  0: hi (can be changed/removed later)
  1: left button setting - color sort fling
  2: mid button setting - auton type
  3: right button setting - auton side
  4: temp flags - overheat or not
  5: comp ctrl mode flag - what mode it is in right now
  */
  //pros::lcd::register_btn1_cb(on_center_button);
}

double dynamicCurve(double velocity) {
  const double minCurve = 0.4;
  const double maxCurve = 1.0;
  const double speedThresh = 300.0;

  double normalVelocity = std::min(std::abs(velocity) / speedThresh, 1.0);
  double curve = minCurve + normalVelocity * (maxCurve - minCurve);
  return (velocity < 0) ? -curve : curve;
}

void disabled() { /*pros::lcd::print(5, "Disabled");*/ }

void competition_initialize() { /*pros::lcd::print(5, "Competition Initialize");*/ }

void autonomous() {
  //pros::lcd::print(5, "Autonomous");  // COLOR IS ACCOUNTED IN intake()
  if (autonElim) {                    // Elimination auton here
    if (autonSide) {                  // close side
      drive(32, false, 400);
      inertialTurn(21.80140949, 300);
      mogoExtend();
      //intakeon;
      drive(12, true, 400);
      //intakeoff;
      inertialTurn(-90, 300);
      mogoRetract();
      drive(6, true, 400);
      inertialTurn(180, 300);
      drive(6, false, 400);
      mogoExtend();
      inertialTurn(-45, 300);
      //intakeon;
      ladyBrownCycle(false);
      drive(26, true, 600);
      //intakeoff;
    } else {  // far side
      // Elimination, far side
    }
  } else {            // Qualification auton here
    if (autonSide) {  // close side
      // Qualification, close side
      drive(32, false, 400);
      inertialTurn(21.80140949, 300);
      mogoExtend();
      //intakeon;
      drive(12, true, 400);
      //intakeoff;
      inertialTurn(-90, 300);
      mogoRetract();
      drive(6, true, 400);
      inertialTurn(180, 300);
      drive(6, false, 400);
      mogoExtend();
      inertialTurn(-45, 300);
      ladyBrownCycle(false);
      drive(9, false, 400);

    } else {  // far side
      // Qualification, far side
    }
  }
}

void opcontrol() {
  /*pros::lcd::print(5, "OpControl");
  while (true) {
    // temp flags
    float dtLeftOT =
        ((round(10.0 * ((vector_sum(left.get_temperature_all()) / 3.0))) / 10.0));
    float dtRightOT =
        ((round(10.0 * ((vector_sum(right.get_temperature_all()) / 3.0))) / 10.0));
    float chainOT = chain.get_temperature();
    float lbOT = lb.get_temperature();
    float rollerOT = roller.get_temperature();
    // printing the overtemp flags on to lcd
    pros::lcd::print(4, "DTL%.1f DTR%.1f Chain%.1f LB%.1f Roller%.1f", dtLeftOT,
                     dtRightOT, chainOT, lbOT, rollerOT);
    ctrl.print(0, 0, "DTL%.1f DTR%.1f Chain%.1f LB%.1f Roller%.1f", dtLeftOT,
               dtRightOT, chainOT, lbOT, rollerOT);*/
    // Arcade control scheme
    int power = ctrl.get_analog(ANALOG_LEFT_Y);
    int turn = ctrl.get_analog(ANALOG_RIGHT_X);
    double velocity = (vector_sum(left.get_actual_velocity_all()) + vector_sum(right.get_actual_velocity_all()) / 2.0);
    double curve = dynamicCurve(velocity);
    double powerL = power + turn * curve;
    double powerR = power - turn * curve;

    if (ctrl.get_digital(DIGITAL_Y)) {  // modifier
      turn = turn / 2;
    }

    // dt
    left.move(powerL);
    right.move(powerR);

    if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      roller.move(127);
    }
    if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      roller.move(-128);
    }
    if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      chain.move(128);
    }
    if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      chain.move(-128);
    }

    if (ctrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      ladyBrownCycle(true);
    }
    if (ctrl.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      ladyBrownCycle(false);
    }
    if (ctrl.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
      mogoExtend();
    } else {
      mogoRetract();
    }
    pros::delay(20);  // Run for 20 ms then update
}