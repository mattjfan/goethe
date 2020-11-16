#ifndef _CONTROLLER_H
#define _CONTROLLER_H

  class StepperMotor {
    public:
      StepperMotor(int step_pin, int dir_pin, int stall_pin = -1, int enable_pin = -1, int step_delay = 20);
      void step_cw(int num_steps, int step_delay = -1);
      void step_ccw(int num_steps, int step_delay = -1);
  
      void calibrate(int dir, int step_delay = -1);
  
      bool isStalled();
  
      void reset();
      void enable();
      
      void disable();
      
    private:
      int step_pin;
      int dir_pin;
      int stall_pin;
      int enable_pin;
      int step_delay;
      int reset_delay = 20;
      void step_(int num_steps, int dir, int step_delay = -1);
  };

  class Syringe {
    public:
      Syringe(StepperMotor body, StepperMotor plunger);
    private:
      StepperMotor body_;
      StepperMotor plunger_;
  };
#endif 
