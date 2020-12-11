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

#define STEP_PIN 2
#define DIR_PIN 3
#define STALL_PIN A0
#define EN_PIN 4

#define MICROSTEPS (double) 16

#define STALL_VALUE 0
#define DRIVER_ADDRESS 0b00 // TMC2209 Driver address according to MS1 and MS2
#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

bool isStalled = false;
TMC2209Stepper driver(&Serial1, R_SENSE, DRIVER_ADDRESS);
using namespace TMC2208_n;
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
  digitalWrite(EN_PIN, HIGH);
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  delay(1);
  digitalWrite(EN_PIN, LOW);
  delay(1000);
  for (int i = 0; i < 2000; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(700);
    digitalWrite(STEP_PIN, LOW);
  }
  digitalWrite(EN_PIN, HIGH);
}

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
//      MOTOR_X.go_to_position(pos);
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

void configure_stallguard() {
  Serial1.begin(115200);
  driver.beginSerial(115200);
  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA
  driver.microsteps(MICROSTEPS); // was originally set at 16
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
//  driver.semin(5);
//  driver.semax(2);
  driver.semin(0);
  driver.semax(0);
  driver.sedn(0b01);
  driver.SGTHRS(STALL_VALUE);
}

void setup() {
  Serial.begin(9600);
  configure_stallguard();
  // put your setup code here, to run once:
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  pinMode(STALL_PIN, INPUT);
  
  digitalWrite(EN_PIN, HIGH);
}

void loop() {
  // put your main code here, to run repeatedly:

}
