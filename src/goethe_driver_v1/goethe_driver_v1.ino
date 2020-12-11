// Copyright 2020 Matt Fan
/* This is a simple single motor
 *  test driver to validate basic motion,
 *  config, and stall detection before scaling
 *  back to full driver. This is because motion
 *  stopped working after some software changes,
 *  and since the hardware and electronics are already
 *  kind sketch, this hopefully helps me validate the
 *  software with less variables, and more easily identify
 *  hardware issues that may be impacting performance
 */

#include <TMCStepper.h>

#define STEP_PIN_X 12
#define DIR_PIN_X 11
#define STALL_PIN_X 19
#define EN_PIN_X 13

#define STEP_PIN_Y 9
#define DIR_PIN_Y 8
#define STALL_PIN_Y 18
#define EN_PIN_Y 10

#define STEP_PIN_SHELL 3
#define DIR_PIN_SHELL 2
#define STALL_PIN_SHELL 20
#define EN_PIN_SHELL 4

#define STEP_PIN_PLUNGER 6
#define DIR_PIN_PLUNGER 5
#define STALL_PIN_PLUNGER 21
#define EN_PIN_PLUNGER 7

#define CONFIG_SERIAL_XY Serial3
#define CONFIG_SERIAL_SYRINGE Serial2
//
//#define STEP_PIN 3
//#define DIR_PIN 4
//#define STALL_PIN 2
//#define EN_PIN 5

#define CW HIGH
#define CCW LOW

#define STEPS_PER_REV (double) 200
#define MICROSTEPS (double) 16
#define DRIVE_PULLEY_RADIUS (double) 6.1 // in mm
#define DRIVE_SCREW_PITCH (double) 8.0 // in mm

#define STEP_DIST_SYRINGE ( DRIVE_SCREW_PITCH / ( STEPS_PER_REV * MICROSTEPS ))
#define STEP_DIST_X ( (2 * PI * DRIVE_PULLEY_RADIUS) / ( STEPS_PER_REV * MICROSTEPS ))
#define STEP_DIST_Y STEP_DIST_X

#define STALL_VALUE_XY 0
#define STALL_VALUE_SYRINGE 60
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

/**
 * Custom abstraction class for TMC2209 Motor drivers
 * Doesn't handle UART config (since we want to use single Serial to
 * config multiple drivers) so that needs to set externally
 * Also there's a limit of 4 motors concurrently bc of ISR glue methods
 */
class StepperMotor {
  public:
      static StepperMotor* instances[4];
      static int num_motors;
      static void stall0_isr() {
        if (StepperMotor::instances[0] != NULL) {
          StepperMotor::instances[0]->set_stalled(true);
        }
      }
      static void stall1_isr() {
        if (StepperMotor::instances[1] != NULL) {
          StepperMotor::instances[1]->set_stalled(true);
        }
      }
      static void stall2_isr() {
        if (StepperMotor::instances[2] != NULL) {
          StepperMotor::instances[2]->set_stalled(true);
        }
      }
      static void stall3_isr() {
        if (StepperMotor::instances[3] != NULL) {
          StepperMotor::instances[3]->set_stalled(true);
        }
      }
      StepperMotor( int step_pin, int dir_pin, int stall_pin, int enable_pin, double mm_per_step, double max_pos, double increasing_direction, int step_delay = 600) {
        this->idx = num_motors++;
        pinMode(stall_pin, INPUT);
        switch(idx) {
          case 0:
//            Serial.println("attaching stall pin 0");
            attachInterrupt(digitalPinToInterrupt(stall_pin), stall0_isr, RISING);
            break;
          case 1:
            attachInterrupt(digitalPinToInterrupt(stall_pin), stall1_isr, RISING);
            break;
          case 2:
            attachInterrupt(digitalPinToInterrupt(stall_pin), stall2_isr, RISING);
            break;
          case 3:
            attachInterrupt(digitalPinToInterrupt(stall_pin), stall3_isr, RISING);
            break;
          default:
            Serial.println("Error: too many motors initialized");
            return;
        }
        instances[idx] = this;
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
        pinMode(enable_pin, OUTPUT);
        this->reset();

      }

      void go_up(double dist) {
        enable();
        double target_pos = pos + dist;
        if (target_pos > max_pos) {
          target_pos = max_pos;
        }
        digitalWrite(step_pin, LOW);
//        digitalWrite(dir_pin, inc_dir);
        set_dir(inc_dir);
        while (pos < target_pos) {
          step_();
        }
      }

      void go_down(double dist) {
        enable();
        double target_pos = pos - dist;
        if (target_pos < 0) {
          target_pos = 0;
        }
        digitalWrite(step_pin, LOW);
//        digitalWrite(dir_pin, !inc_dir);
        set_dir(!inc_dir);
        while (pos > target_pos) {
          step_();
        }
      }

      void go_to(double target_position) {
        enable();
      // Serial.println("Goin to tha position");
        if (target_position > max_pos) {
          return;
        }
        digitalWrite(step_pin, LOW);
        if (pos < target_position) {
//          digitalWrite(dir_pin, inc_dir);
          set_dir(inc_dir);
          while (pos < target_position) {
            step_();
          }
        } else if (pos > target_position) {
//          digitalWrite(dir_pin, !inc_dir);
          set_dir(!inc_dir);
          while (pos > target_position) {
            step_();
          }
        } 
      }

      void calibrate() {
        enable();
//        digitalWrite(dir_pin, !inc_dir);
        set_dir(!inc_dir);
        digitalWrite(step_pin, LOW);
//      while(1) {
        for (int i = 0; i <= round(max_pos / mm_per_step); i++) { // times out if stall isn't triggered 
          if (this->get_stalled()) {
            break;
          }
          step_();
        }
        pos = 0;
      }

      void calibrate_reverse() { // calibrate to max_pos instead of zero
//        digitalWrite(dir_pin, inc_dir);
        enable();
        set_dir(inc_dir);
        digitalWrite(step_pin, LOW);
        for (int i = 0; i <= round(max_pos / mm_per_step); i++) { // times out if stall isn't triggered 
          if (this->get_stalled()) {
            break;
          }
          step_();
        }
        pos = max_pos;
      }
      
      void setDelayMicros(int step_delay) {
        this->step_delay = step_delay;
      }
      void setDelayMillis(int step_delay_millis) {
        this->step_delay = step_delay_millis * 1000;
      }
      void set_stalled(bool stalled) {
        this->stalled = stalled;
      }
      void reset() {
        this->disable();
//        delay(this->reset_delay);
//          delay(20);
        this->enable();
        this->set_stalled(false);
      }
      virtual void enable() {
        digitalWrite(enable_pin, LOW);
      }
      virtual void disable() {
        digitalWrite(enable_pin, HIGH);
      }
      int get_stall_pin() {
        return stall_pin;
      }
      virtual bool get_stalled() {
        return this->stalled;
      }
      int get_step_pin() {
        return step_pin;
      }
      int get_dir_pin() {
        return dir_pin;
      }
      protected:
        int idx;
        int step_pin;
        int dir_pin;
        int stall_pin;
        int enable_pin;
        int step_delay;
        double mm_per_step;
        int inc_dir = LOW;
        double pos = 0;
//        double pos = 200;
        double max_pos;
        int reset_delay = 20; // how long to wait in ms between toggling enable pin for reset
        volatile bool stalled = false;
        virtual void step_() {
          digitalWrite(step_pin, HIGH);
          delayMicroseconds(step_delay);
          digitalWrite(step_pin, LOW);
          if (digitalRead(dir_pin) == inc_dir) {
            pos += mm_per_step;
          } else {
            pos -= mm_per_step;
          }
        }
        virtual void set_dir(int dir) {
          digitalWrite(dir_pin, dir);
        }
};
/**
 * Wrapper to run two motors synchronously. Extends StepperMotor
 * and also takes a reference to another stepper motor. The joint
 * assembly can be controlled as a single motor here, and the
 * passed stepper motor can also be controlled directly.
 * Not thread safe
 */
class DoubleStepperMotor : public StepperMotor {
  // 
   public:
      DoubleStepperMotor( StepperMotor& baseMotor, int step_pin, int dir_pin, int stall_pin, int enable_pin, double mm_per_step, double max_pos, double increasing_direction, int step_delay = 600) :
        StepperMotor(step_pin, dir_pin, stall_pin, enable_pin, mm_per_step, max_pos, increasing_direction, step_delay), baseMotor(baseMotor){
      }
      bool get_stalled() override {
        return stalled || baseMotor.get_stalled();
      }
      void enable() override {
        digitalWrite(enable_pin, LOW);
        baseMotor.enable();
      }
      void disable() override {
        digitalWrite(enable_pin, HIGH);
        baseMotor.disable();
      }
      void diagnostics() {
        Serial.println("reference stuff: ");
        Serial.print("baseMotor step pin: ");
        Serial.println(baseMotor.get_step_pin());
      }
    protected:
      void step_() override {
        digitalWrite(step_pin, HIGH);
        digitalWrite(baseMotor.get_step_pin(), HIGH);
        delayMicroseconds(step_delay);
        digitalWrite(step_pin, LOW);
        digitalWrite(baseMotor.get_step_pin(), LOW);
        if (digitalRead(dir_pin) == inc_dir) {
            pos += mm_per_step;
          } else {
            pos -= mm_per_step;
          }
      }
      void set_dir(int dir) override {
        digitalWrite(dir_pin, dir);
        digitalWrite(baseMotor.get_dir_pin(), dir);
      }
      StepperMotor baseMotor;
      
};

// initializing static variables for StepperMotor bc C++ standard style restrictions
int StepperMotor::num_motors = 0;
StepperMotor* StepperMotor::instances[4] = { NULL, NULL, NULL, NULL };


StepperMotor x_motor(STEP_PIN_X, DIR_PIN_X, STALL_PIN_X, EN_PIN_X, STEP_DIST_X, 200, CW, 600);
StepperMotor y_motor(STEP_PIN_Y, DIR_PIN_Y, STALL_PIN_Y, EN_PIN_Y, STEP_DIST_Y, 200, CCW, 600);
StepperMotor plunger_motor(STEP_PIN_PLUNGER, DIR_PIN_PLUNGER, STALL_PIN_PLUNGER, EN_PIN_PLUNGER, STEP_DIST_SYRINGE, 90, CW, 500);
//StepperMotor shell_motor(STEP_PIN_SHELL, DIR_PIN_SHELL, STALL_PIN_SHELL, EN_PIN_SHELL, STEP_DIST_SYRINGE, 140, CW, 600);
DoubleStepperMotor syringe_motor(plunger_motor, STEP_PIN_SHELL, DIR_PIN_SHELL, STALL_PIN_SHELL, EN_PIN_SHELL, STEP_DIST_SYRINGE, 140, CW, 500);
//void x_stall_callback() {
//  Serial.println("stalled out!!");
//  x_motor.set_stalled();
//}

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
  Serial.println("Runnin it");
//  x_motor.go_down(20);
//  x_motor.reset();
//  y_motor.reset();
//  x_motor.calibrate();
//  y_motor.calibrate();
//  Serial.println("calibrrted");
//  x_motor.reset();
//  y_motor.reset();
////  y_motor.go_up(3);
//  x_motor.go_to(100);
//  y_motor.go_to(100);
//  plunger_motor.enable();
//  plunger_motor.go_down(20);
//  plunger_motor.disable();
//  syringe_motor.diagnostics();
    plunger_motor.reset();
    plunger_motor.calibrate();
    plunger_motor.go_up(3);
//    delay(1000);
    syringe_motor.reset();
    syringe_motor.calibrate_reverse();

//    syringe_motor.go_up(20);
//    syringe_motor.disable();
    Serial.println("El Finito");
//  x_motor.go_up(50);
//  x_motor.go_down(30);

}
//void serialCallback(String msg) {
//  Serial.println("Runnin it");
//  digitalWrite(EN_PIN, HIGH);
//  digitalWrite(STEP_PIN, LOW);
//  digitalWrite(DIR_PIN, LOW);
//  delay(1);
//  digitalWrite(EN_PIN, LOW);
//  delay(1000);
//  for (int i = 0; i < 2000; i++) {
//    digitalWrite(STEP_PIN, HIGH);
//    delayMicroseconds(700);
//    digitalWrite(STEP_PIN, LOW);
//  }
//  digitalWrite(EN_PIN, HIGH);
//}



//
//void serialCallback(String msg) {
//  String command = "";
//  String payload = "";
//  int argc = 1;
//  String str = "";
//  bool isCommand = true;
//  for (char const &c: msg) {
//    if (isCommand) {    
//      if (c != ' ') {
//        command += c;
//      } else {
//        isCommand = false;
//      }
//    } else {
//      payload += c;
//    }
//  }
//  if (command == "calibrate") {
//    if (payload == "x") {
//      Serial.print("Calibrating X-axis...");
//      MOTOR_X.calibrate();
//      Serial.println("done.");
//    } else {
//      Serial.println("Unrecognized argument " + payload + " for command " + command);
//    }
//  } else if (command == "movex") {
//    double pos = payload.toDouble();
//    if (pos || payload == "0" || payload == "0.0") {
//      Serial.println(pos);
//      MOTOR_X.go_to(pos);
//      Serial.println("At requested position");
//      
//    } else {
//      Serial.println("Unrecognized argument " + payload + " for command " + command);
//    }
//  } else if (command = "test") {
//    Serial.println("testing motion");
//    motor_test_basic(2400,1);
//    Serial.println("done.");
//  } else {
//    Serial.println("Unrecognized command " + command);
//  }

TMC2209Stepper driver_xy(&CONFIG_SERIAL_XY, R_SENSE, DRIVER_ADDRESS);
TMC2209Stepper driver_syringe(&CONFIG_SERIAL_SYRINGE, R_SENSE, DRIVER_ADDRESS);
using namespace TMC2208_n;

void configure_stallguard_xy() {
  CONFIG_SERIAL_XY.begin(115200);
  driver_xy.beginSerial(115200);
  driver_xy.begin();
  driver_xy.toff(4);
  driver_xy.blank_time(24);
  driver_xy.rms_current(400); // mA
  driver_xy.microsteps(MICROSTEPS); // was originally set at 16
  driver_xy.TCOOLTHRS(0xFFFFF); // 20bit max
//  driver.semin(5);
//  driver.semax(2);
  driver_xy.semin(0);
  driver_xy.semax(0);
  driver_xy.sedn(0b01);
  driver_xy.SGTHRS(STALL_VALUE_XY);
}

void configure_stallguard_syringe() {
  CONFIG_SERIAL_SYRINGE.begin(115200);
  driver_syringe.beginSerial(115200);
  driver_syringe.begin();
  driver_syringe.toff(4);
  driver_syringe.blank_time(24);
  driver_syringe.rms_current(400); // mA
  driver_syringe.microsteps(MICROSTEPS); // was originally set at 16
  driver_syringe.TCOOLTHRS(0xFFFFF); // 20bit max
  driver_syringe.semin(5);
  driver_syringe.semax(2);
//  driver_syringe.semin(0);
//  driver_syringe.semax(0);
  driver_syringe.sedn(0b01);
  driver_syringe.SGTHRS(STALL_VALUE_SYRINGE);
}


void setup() {
  
  Serial.begin(9600);
  while(!Serial){}
  Serial.println("howdy");
  configure_stallguard_xy();
  configure_stallguard_syringe();
  // put your setup code here, to run once:
//  pinMode(STEP_PIN, OUTPUT);
//  pinMode(DIR_PIN, OUTPUT);
//  pinMode(EN_PIN, OUTPUT);
//  pinMode(STALL_PIN, INPUT);

//  attachInterrupt(digitalPinToInterrupt(STALL_PIN), x_stall_callback, RISING);

  
//  digitalWrite(EN_PIN, HIGH);
//  x_motor.calibrate();
//  print(DA_THING);
//  print_da_thing();
}

void loop() {
  // put your main code here, to run repeatedly:

}
