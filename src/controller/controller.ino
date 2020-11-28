//#include "controller.h"
#include <EEPROM.h>
#include <TMCStepper.h>

#define LOC_RED (Point) {7, 13}
#define LOC_BLUE (Point) {0,0}
#define LOC_YELLOW (Point) {0,0}
#define LOC_BLACK (Point) {0,0}
#define LOC_WHITE (Point) {0,0}
#define LOC_DILUTE (Point) {0,0}
#define LOC_CLEAN (Point) {0,0}
#define HEIGHT_CALIBRATE 0 // Height for stall calibration, and lowest possible height. Should generally be 0
#define HEIGHT_LOAD HEIGHT_CALIBRATE
#define HEIGHT_MOVE 10 // Height for moving around. Should be tall enough to clear all obstacles, prob. want this to be your largest height
#define PLUNGER_MAX_HEIGHT 100 // Max height of plunger.
#define PLUNGER_DIAMETER 20
#define HEIGHT_DISPENSE HEIGHT_MOVE
#define HEIGHT_DISPENSE_WATER HEIGHT_DISPENSE // if you want to specify a water dispense height to minimize splashing
#define BB_DISPENSE_L (Point) {50,50} // closest to origin corner for dispensing bounding box
#define BB_DISPENSE_U (Point) (100,100} // farthest from origin corner for dispensing bounding box
#define CW HIGH
#define CCW LOW

#define STEPS_PER_REV (double) 200
#define MICROSTEPS (double) 16
#define DRIVE_PULLEY_RADIUS (double) 6.1 // in mm
#define DRIVE_SCREW_PITCH (double) 8.0 // in mm

#define STEP_DIST_SYRINGE ( DRIVE_SCREW_PITCH / ( STEPS_PER_REV * MICROSTEPS ))
#define STEP_DIST_X ( (2 * PI * DRIVE_PULLEY_RADIUS) / ( STEPS_PER_REV * MICROSTEPS ))
#define STEP_DIST_Y STEP_DIST_X

#define STALL_PIN_SYRINGE A0
#define STALL_PIN_PLUNGER A1
#define STALL_PIN_X A2
#define STALL_PIN_Y A3

#define MOTOR_SYRINGE StepperMotor(4, 5, STALL_PIN_SYRINGE, 3, STEP_DIST_SYRINGE, 140, CW, 500) // defining motor wholesale as macro instead of individual pins
#define MOTOR_PLUNGER StepperMotor(6, 7, STALL_PIN_PLUNGER, 3, STEP_DIST_SYRINGE, 90, CW, 500)
#define MOTOR_X StepperMotor(8, 9, STALL_PIN_X, A4, STEP_DIST_X, 280, CW, 500)
#define MOTOR_Y StepperMotor(10, 11, STALL_PIN_Y, A5, STEP_DIST_Y, 280, CCW, 500)

#define CONFIG_RX 12
#define CONFIG_TX 13
#define STALL_VALUE 40
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

TMC2209Stepper driver(CONFIG_RX, CONFIG_TX, R_SENSE, DRIVER_ADDRESS);

using namespace TMC2208_n;

class StepperMotor {
  public:
//    StepperMotor(int step_pin, int dir_pin, int stall_pin = -1, int enable_pin = -1, double mm_per_step = 0, int step_delay = 1) {
//      this->step_pin = step_pin;
//      this->dir_pin = dir_pin;
//      this->stall_pin = stall_pin;
//      this->enable_pin = enable_pin;
//      this->step_delay = step_delay;
//      this->mm_per_step = mm_per_step;
//      pinMode(step_pin, OUTPUT);
//      pinMode(dir_pin, OUTPUT);
//      if (stall_pin >= 0) {
//        this->stall_pin = stall_pin;
//        pinMode(stall_pin, INPUT);
//      }
//      if (enable_pin >= 0) {
//        this->enable_pin = enable_pin;
//        pinMode(enable_pin, OUTPUT);
//        digitalWrite(enable_pin, LOW);
//      }
//    }
//    StepperMotor( int step_pin, int dir_pin, double mm_per_step, int step_delay = 1) {
//    }
//    StepperMotor( int step_pin, int dir_pin, int stall_pin, double mm_per_step, int step_delay = 1) {
//     
//    }
    StepperMotor( int step_pin, int dir_pin, int stall_pin, int enable_pin, double mm_per_step, double max_pos, double increasing_direction = CW, int step_delay = 1) {
      this->step_pin = step_pin;
      this->dir_pin = dir_pin;
      this->stall_pin = stall_pin;
      this->enable_pin = enable_pin;
      this->mm_per_step = mm_per_step;
      this->step_delay = step_delay;
      this->max_pos = max_pos;
      this->inc_dir = increasing_direction;
      pinMode(step_pin, OUTPUT);
      pinMode(dir_pin, OUTPUT);
      pinMode(stall_pin, INPUT); // included with next gen design in mind. Currently not used internally, stall refs currently managed in global scope
      pinMode(enable_pin, INPUT);
    }
    void step_dist_cw(double dist, int step_delay = 1) {
      step_cw((int) dist / mm_per_step, step_delay);
    }
    void go_up(double dist) {
      double target_pos = pos + dist;
      if (target_pos > max_pos) {
        target_pos = max_pos;
      }
      digitalWrite(step_pin, LOW);
      digitalWrite(dir_pin, inc_dir);
      while (pos < target_pos) {
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(step_pin, LOW);
        pos += mm_per_step;
      }
    }

    void go_down(double dist) {
      double target_pos = pos - dist;
      if (target_pos < 0) {
        target_pos = 0;
      }
      digitalWrite(step_pin, LOW);
      digitalWrite(dir_pin, !inc_dir);
      while (pos > target_pos) {
        digitalWrite(step_pin, HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(step_pin, LOW);
        pos -= mm_per_step;
      }
    }

    void go_to_position(double target_position) {
//      Serial.println("Goin to tha position");
      if (target_position > max_pos) {
        return;
      }
      digitalWrite(step_pin, LOW);
      if (pos < target_position) {
        digitalWrite(dir_pin, inc_dir);
        while (pos < target_position) {
          digitalWrite(step_pin, HIGH);
          delayMicroseconds(step_delay);
          digitalWrite(step_pin, LOW);
          pos += mm_per_step;
        }
      } else if (pos > target_position) {
        digitalWrite(dir_pin, !inc_dir);
        while (pos > target_position) {
          digitalWrite(step_pin, HIGH);
          delayMicroseconds(step_delay);
          digitalWrite(step_pin, LOW);
          pos -= mm_per_step;
        }
      } 
    }
    void step_cw(int num_steps, int step_delay = 1) {
      step_(num_steps, CW, step_delay);
    }

    void step_ccw(int num_steps, int step_delay = 1) {
      step_(num_steps, CCW, step_delay);
    }

    void step(int dir, int delay_micros = 20) {
      digitalWrite(dir_pin, dir);
      digitalWrite(step_pin, HIGH);
      delayMicroseconds(delay_micros);
      digitalWrite(step_pin, LOW);
    }
    void calibrate() {
      digitalWrite(dir_pin, !inc_dir);
      digitalWrite(step_pin, LOW);
//      while(1) {
      for (int i = 0; i <= round(max_pos / mm_per_step); i++) { // times out if stall isn't triggered 
        if (isStalled()) {
          break;
        }
        digitalWrite(step_pin, HIGH);
        delay(step_delay);
        digitalWrite(step_pin, LOW);
      }
      pos = 0;
    }
    void calibrate_reverse() { // calibrate to max_pos instead of zero
      digitalWrite(dir_pin, inc_dir);
      digitalWrite(step_pin, LOW);
      for (int i = 0; i <= round(max_pos / mm_per_step); i++) { // times out if stall isn't triggered 
        if (isStalled()) {
          break;
        }
        digitalWrite(step_pin, HIGH);
        delay(step_delay);
        digitalWrite(step_pin, LOW);
      }
      pos = max_pos;
    }
//    void calibrate(int dir, int step_delay = -1) { // TODO: may want to add a max timeout option
//      digitalWrite(dir_pin, dir);
//      if (step_delay < 0) {
//        step_delay = this->step_delay;
//      }
//      while(1) {
//        if (isStalled()) {
//          break;
//        }
//        digitalWrite(step_pin, HIGH);
//        delay(step_delay);
//        digitalWrite(step_pin, LOW);
//      }
//      // clean-up so motor can be used again.
//      reset();
//    }
    void setStalled(bool isStalled) { // kludge so that global interrupt handlers can set this value
      stalled = isStalled;
    }
    bool isStalled() {
//      return stalled;
      if (stall_pin >= 0 &&
        digitalRead(stall_pin) == HIGH) {
         Serial.println("Fuck, we stalled");
        return true;
      }
      return false;
    }

    void reset() {
      this->disable();
      delay(this->reset_delay);
      this->enable();
      this->stalled = false;
    }

    void enable() {
      digitalWrite(enable_pin, LOW);
    }
    
    void disable() {
      digitalWrite(enable_pin, HIGH);
    }
    
  private:
    int step_pin;
    int dir_pin;
    int stall_pin;
    int enable_pin;
    int step_delay;
    double mm_per_step;
    int inc_dir = CW;
    double pos = 0;
    double max_pos;
    int reset_delay = 20;
    bool stalled = false;
//    void interrupt_handler () {
//      stalled = true;
//    }
    void step_(int num_steps, int dir, int step_delay = -1) {
      if (step_delay < 0) {
        step_delay = this->step_delay; // reset local reference
        // I think this is mem safe bc step_delay should be passed by value
      }
      digitalWrite(dir_pin, dir);
      for (int i = 0; i < num_steps; i++) {
        digitalWrite(step_pin, HIGH);
        delay(step_delay);
//        delayMicroseconds(100);
        digitalWrite(step_pin, LOW);
        if (isStalled()) {
          reset();
          break;
        }
      }
    }
};

//class GantryAxis {
//  public:
//    GantryAxis(StepperMotor motor) {
//    }
//}

class Syringe {
  public:
    Syringe(StepperMotor body, StepperMotor plunger, double diameter = 10.0, double step_distance = 0.1, int step_delay = 1){
      *(this->body_ptr) = body;
      *(this->plunger_ptr) = plunger;
      this->step_distance = step_distance;
      this->UP_DIR = CW;
      this->diameter = diameter;
      this->step_distance = step_distance;
      this->step_delay = step_delay;
    }
    void moveTo(float target_height){ // TODO: check that it moves within height bounds
      if (target_height > syringe_height) {
        while(target_height > syringe_height) {
          body.step(UP_DIR); // technically not truly synchronous...
          plunger.step(UP_DIR);
          delay(step_delay);
          syringe_height += step_distance;
        }
      } else {
        while(target_height < syringe_height) {
          body.step(!UP_DIR);
          plunger.step(!UP_DIR);
          delay(step_delay);
          syringe_height -= step_distance;
        }
      }
    }
    void load(double volume) { // volume in mm^3
      double target_height = plunger_height + heightFromVolume(volume); // v = PI * d^2 * h / 4
      while(plunger_height < target_height){
        plunger.step(UP_DIR);
        delay(step_delay);
        plunger_height += step_distance;
      }
    }
    void dispense(double volume) {
      double target_height = plunger_height - heightFromVolume(volume);
      while (plunger_height > target_height) {
        plunger.step(!UP_DIR);
        delay(step_delay);
        plunger_height -= step_distance;
      }
    }
  private:
    double heightFromVolume(double volume) {
      return (4.0 * volume) / (PI * diameter * diameter);
    }
    StepperMotor &body = *body_ptr; // kludge to reference by name
                              // should probably refactor at some point to
                              // declare with .h/.hpp and use an initializer list
    StepperMotor &plunger = *plunger_ptr;
    
    
    StepperMotor* body_ptr = nullptr;
    StepperMotor* plunger_ptr = nullptr;
    int UP_DIR;
    double syringe_height = 0.0;
    double plunger_height = 0.0;
    double step_distance;
    double diameter;
    int step_delay;
};

class Gantry {
  public:
    Gantry(double step_distance = 0.1){
      
    }
  private:
    double x, y;
};

typedef struct Point {
  int x;
  int y;
} Point;

class Color {
  public:
    Color(double r, double b, double y, double w, double k) {
      // basic color w/ no water dilution
      double sum = r + b + y + w + k;
      this->r_ = r /= sum;
      this->b_ = b /= sum;
      this->y_ = y /= sum;
      this->w_ = w /= sum;
      this->k_ = k /= sum;
      this->c_ = 0.0;
    }
    
    Color(double r, double b, double y, double w, double k, double c) {
      // allow for specifying water % for diluted / wet mixes
      double sum = r + b + y + w + k + c;
      this->r_ = r /= sum;
      this->b_ = b /= sum;
      this->y_ = y /= sum;
      this->w_ = w /= sum;
      this->k_ = k /= sum;
      this->c_ = c /= sum;
    }
    
    const double &r = r_;
    const double &b = b_;
    const double &y = y_;
    const double &w = w_;
    const double &k = k_;
    const double &c = c_;
  private:
    double r_;
    double b_;
    double y_;
    double w_;
    double k_;
    double c_;
};

void configure_stallguard() {
  driver.beginSerial(19200);
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(MICROSTEPS); // was originally set at 16
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);
}
/**
 * Basic motor tests checking for movement
 */
void motor_test_basic(int steps = 1500, int period = 3){
  // testing syringe motor
  if(Serial) {
    Serial.print("Testing syringe...");
  }
  MOTOR_SYRINGE.reset();
  MOTOR_SYRINGE.step_cw(steps, period);
  MOTOR_SYRINGE.step_ccw(steps, period);
  if (Serial) {
    Serial.println("done.");
  }
  // testing plunger motor
  if(Serial) {
    Serial.print("Testing plunger...");
  }
  MOTOR_PLUNGER.reset();
  MOTOR_PLUNGER.step_cw(steps, period);
  MOTOR_PLUNGER.step_ccw(steps, period);
  if (Serial) {
    Serial.println("done.");
  }
  // testing x carriage motor
  if(Serial) {
    Serial.print("Testing X carriage...");
  }
  MOTOR_X.reset();
  MOTOR_X.step_cw(steps, period);
  MOTOR_X.step_ccw(steps, period);
  if (Serial) {
    Serial.println("done.");
  }
  // testing y carriage motor
  if(Serial) {
    Serial.print("Testing Y carriage...");
  }
  MOTOR_Y.reset();
  MOTOR_Y.step_cw(steps, period);
  MOTOR_Y.step_ccw(steps, period);
  if (Serial) {
    Serial.println("done.");
  }
}

// callback for computer serial command interface
bool queue_events = false; // should events be queued
bool serial_blocked = false; // global flag to let us know if we should throw stuff away
void serialEvent() {
  String s = Serial.readStringUntil('\n');
  if (serial_blocked && !queue_events) { // throw away events received while you were running old event
    // unless you have queueing of events turned on.
    Serial.println("Ignoring message " + s); // Say that event is being thrown away
  } else { // main logic block goes here
      serial_blocked = true;
      serialCallback(s);
  }
  if (!Serial.available()) { // Unblocking new events if we've processed all stale events
      serial_blocked = false;
  }
}

void serialCallback(String msg) {
  String command = "";
  String payload = "";
  int argc = 1;
  String str = "";
  bool isCommand = true;
  for (char const &c: msg) {
    if (isCommand) {    
      if (c != ' ') {
        command += c;
      } else {
        isCommand = false;
      }
    } else {
      payload += c;
    }
  }
  if (command == "calibrate") {
    if (payload == "x") {
      Serial.print("Calibrating X-axis...");
      MOTOR_X.calibrate();
      Serial.println("done.");
    } else {
      Serial.println("Unrecognized argument " + payload + " for command " + command);
    }
  } else if (command == "movex") {
    double pos = payload.toDouble();
    if (pos || payload == "0" || payload == "0.0") {
      Serial.println(pos);
      MOTOR_X.go_to_position(pos);
      Serial.println("At requested position");
      
    } else {
      Serial.println("Unrecognized argument " + payload + " for command " + command);
    }
  } else if (command = "test") {
    Serial.println("testing motion");
    motor_test_basic(2400,1);
    Serial.println("done.");
  } else {
    Serial.println("Unrecognized command " + command);
  }

//  if (msg == "calibrate x") {
//    // do calibration
//  } else if ( msg
//      Serial.println("Hello " + msg);
//      delay(5000);
//      Serial.println("Goodbye " + msg);
}

void syringe_interrupt_handler() {
  MOTOR_SYRINGE.setStalled(true);
}

void plunger_interrupt_handler() {
  MOTOR_PLUNGER.setStalled(true);
}

void x_interrupt_handler() {
  Serial.println("X went wrong");
  MOTOR_X.setStalled(true);
}

void y_interrupt_handler() {
  MOTOR_Y.setStalled(true);
}



void setup() {
  // put your setup code here, to run once:
  configure_stallguard();
  Serial.begin(9600);
  while(!Serial){}
  Serial.println("Starting");
  // Below is kludge to hook up motor stall pins to interrupts
  // ideally this could be handled in the class directly, but the instance method prototypes can't be used directly with attachInterrupt
  // and I can't use non-standard C++ libraries in AVR w/o adding extra bulk
  // Could theoretically also do live polling in the main loop, but would then have to wrangle all of our tasks into short-lived enough to be viable
  // in that approach... 
  // This current approach is messy, and strews the innards of our motor abstraction all over the place (and also exposes internal state for stall setting)
  // but it *should* at least work for now without needing to add extra stuff to AVR deps
  // May revisit this later, and end up adding the extra dependencies for a less fubar approach
  pinMode(STALL_PIN_SYRINGE, INPUT);
  pinMode(STALL_PIN_PLUNGER, INPUT);
  pinMode(STALL_PIN_X, INPUT);
  pinMode(STALL_PIN_Y, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_SYRINGE), syringe_interrupt_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_PLUNGER), plunger_interrupt_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_X), x_interrupt_handler, RISING);
  attachInterrupt(digitalPinToInterrupt(STALL_PIN_Y), y_interrupt_handler, RISING);
//  readStringUntil(
  configure_stallguard();
//  motor_test_basic(2400,1);
//  Serial.println("Finishing");
  
//  Serial.begin(9600);
//  Serial.print("What up hello!\n");
//  Serial.print("Red x: ");
//  Serial.print(LOC_RED.x);
//  Serial.print(", y: ");
//  Serial.print(LOC_RED.y);
//  Serial.print("\n");
//  Serial.print("Dispensing height: ");
//  Serial.print(HEIGHT_DISPENSE);
//  delay(2000);
//  Serial.println("");
//  Serial.println("Starting: ");
//  MOTOR_SYRINGE.reset();
//  MOTOR_SYRINGE.step_cw(5000, 1);
//  MOTOR_SYRINGE.step_ccw(2400, 1);
//  Serial.print("steps done");

}

void calibrate_xy() {

}

void calibrate_needle() {
  // may want to run calibrate_xy before this.
//  go_to(LOC_CLEAN);  
}

void go_to(Point pt) {
  return;
}

void make_color(Point pt, Color color) {
}

void set_height() {
  
}

void calibrate() {
// go_to(
}

void loop() {
  // put your main code here, to run repeatedly:
 

}
