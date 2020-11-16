//#include "controller.h"
#include <TMCStepper.h>
#include <SoftwareSerial.h>

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
#define SW_RX 8
#define SW_TX 9
#define SERIAL_PORT SoftwareSerial(SW_RX, SW_TX)

#define MOTOR_SYRINGE StepperMotor(10,11,12,13) // defining motor wholesale as macro instead of individual pins

TMC2209Stepper driver(SW_RX, SW_TX, 0.11f, 0b00);

using namespace TMC2208_n;

class StepperMotor {
  public:
    StepperMotor(int step_pin, int dir_pin, int stall_pin = -1, int enable_pin = -1, int step_delay = 1) {
      this->step_pin = step_pin;
      this->dir_pin = dir_pin;
      this->stall_pin = stall_pin;
      this->enable_pin = enable_pin;
      this->step_delay = step_delay;
      pinMode(step_pin, OUTPUT);
      pinMode(dir_pin, OUTPUT);
      if (stall_pin >= 0) {
        this->stall_pin = stall_pin;
        pinMode(stall_pin, INPUT);
      }
      if (enable_pin >= 0) {
        this->enable_pin = enable_pin;
        pinMode(enable_pin, OUTPUT);
        digitalWrite(enable_pin, LOW);
      }
    }

    void step_cw(int num_steps, int step_delay = -1) {
      step_(num_steps, CW, step_delay);
    }

    void step_ccw(int num_steps, int step_delay = -1) {
      step_(num_steps, CCW, step_delay);
    }

    void calibrate(int dir, int step_delay = -1) { // TODO: may want to had a max timeout option
      digitalWrite(dir_pin, dir);
      if (step_delay < 0) {
        step_delay = this->step_delay;
      }
      while(1) {
        if (isStalled()) {
          break;
        }
        digitalWrite(step_pin, HIGH);
        delay(step_delay);
        digitalWrite(step_pin, LOW);
      }
      // clean-up so motor can be used again.
      reset();
    }

    bool isStalled() {
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
    int reset_delay = 20;
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
//
//class Syringe {
//  public:
//    Syringe(StepperMotor body, StepperMotor plunger)
//    {
//      *(this->body_ptr) = body;
//      *(this->plunger_ptr) = plunger;
//    }
//  private:
//    const StepperMotor &body = body_ptr; // kludge to reference by name
//                              // should probably refactor at some point to
//                              // declare with .h/.hpp and use an initializer list
//    const StepperMotor &plunger = plunger_ptr;
//    StepperMotor* body_ptr;
//    StepperMotor* plunger_ptr;
//
//};

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

void setup() {
  // put your setup code here, to run once:
//  SERIAL_PORT.begin(9600);
//  while(!SERIAL_PORT);
//  driver.beginSerial(115200);
//  driver.begin();
//  driver.pdn_disable(1);
//  driver.toff(4);
//  driver.pwm_autoscale(true);
//  driver.shaft(1);

//  driver.en_spreadCycle(0);
//  driver.SGTHRS(30);
  Serial.begin(9600);
  Serial.print("What up hello!\n");
  if (driver.stealth()) {
    Serial.print("Stealth Mode\n");
  } else {
    Serial.print("Spread Mode\n");
  }
  Serial.print("Red x: ");
  Serial.print(LOC_RED.x);
  Serial.print(", y: ");
  Serial.print(LOC_RED.y);
  Serial.print("\n");
  Serial.print("Dispensing height: ");
  Serial.print(HEIGHT_DISPENSE);
  delay(2000);
  Serial.println("");
  Serial.println("Starting: ");
  MOTOR_SYRINGE.reset();
  MOTOR_SYRINGE.step_cw(5000, 1);
  MOTOR_SYRINGE.step_ccw(2400, 1);
  Serial.print("steps done");

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
