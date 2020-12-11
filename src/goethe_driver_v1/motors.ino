//#define MAX_NUM_MOTORS 4
//
//class StepperMotor {
//  public:
//      static int stall_pins[MAX_NUM_MOTORS];
//      static int num_motors 0;
//      StepperMotor( int step_pin, int dir_pin, int stall_pin, int enable_pin, double mm_per_step, double max_pos, double increasing_direction, int step_delay = 600) {
//        this->idx = num_motors++;
//        stall_pins[idx] = stall_pin;
//        this->step_pin = step_pin;
//        this->dir_pin = dir_pin;
//        this->stall_pin = stall_pin;
//        this->enable_pin = enable_pin;
//        this->mm_per_step = mm_per_step;
//        this->step_delay = step_delay;
//        this->max_pos = max_pos;
//        this->inc_dir = increasing_direction;
//        pinMode(step_pin, OUTPUT);
//        pinMode(dir_pin, OUTPUT);
//        pinMode(enable_pin, OUTPUT);
//        pinMode(stall_pin, INPUT); // included with next gen design in mind. Currently not used internally, stall refs currently managed in global scope
//      }
//
//      void go_up(double dist) {
//        double target_pos = pos + dist;
//        if (target_pos > max_pos) {
//          target_pos = max_pos;
//        }
//        digitalWrite(step_pin, LOW);
//        digitalWrite(dir_pin, inc_dir);
//        while (pos < target_pos) {
//          step_();
//        }
//      }
//
//      void go_down(double dist) {
//        double target_pos = pos - dist;
//        if (target_pos < 0) {
//          target_pos = 0;
//        }
//        digitalWrite(step_pin, LOW);
//        digitalWrite(dir_pin, !inc_dir);
//        while (pos > target_pos) {
//          step_();
//        }
//      }
//
//      void go_to_position(double target_position) {
//      // Serial.println("Goin to tha position");
//        if (target_position > max_pos) {
//          return;
//        }
//        digitalWrite(step_pin, LOW);
//        if (pos < target_position) {
//          digitalWrite(dir_pin, inc_dir);
//          while (pos < target_position) {
//            step_();
//          }
//        } else if (pos > target_position) {
//          digitalWrite(dir_pin, !inc_dir);
//          while (pos > target_position) {
//            step_();
//          }
//        } 
//      }
//
//      void calibrate() {
//        digitalWrite(dir_pin, !inc_dir);
//        digitalWrite(step_pin, LOW);
////      while(1) {
//        for (int i = 0; i <= round(max_pos / mm_per_step); i++) { // times out if stall isn't triggered 
//          if (this->stalled) {
//            break;
//          }
//          step_();
//        }
//        pos = 0;
//      }
//
//      void calibrate_reverse() { // calibrate to max_pos instead of zero
//        digitalWrite(dir_pin, inc_dir);
//        digitalWrite(step_pin, LOW);
//        for (int i = 0; i <= round(max_pos / mm_per_step); i++) { // times out if stall isn't triggered 
//          if (this->stalled) {
//            break;
//          }
//          step_();
//        }
//        pos = max_pos;
//      }
//      
//      void setDelayMicros(int step_delay) {
//        this->step_delay = step_delay;
//      }
//      void setDelayMillis(int step_delay_millis) {
//        this->step_delay = step_delay_millis * 1000;
//      }
//      void set_stalled() {
//        this->stalled = true;
//      }
//      void reset() {
//        this->disable();
//        delay(this->reset_delay);
//        this->enable();
//        this->stalled = false;
//      }
//      void enable() {
//        digitalWrite(enable_pin, LOW);
//      }
//      void disable() {
//        digitalWrite(enable_pin, HIGH);
//      }
//      int get_stall_pin() {
//        return stall_pin;
//      }
//      private:
//        int idx;
//        int step_pin;
//        int dir_pin;
//        int stall_pin;
//        int enable_pin;
//        int step_delay;
//        double mm_per_step;
//        int inc_dir = LOW;
//        double pos = 0;
//        double max_pos;
//        int reset_delay = 1;
//        bool stalled = false;
//        void step_() {
//          digitalWrite(step_pin, HIGH);
//          delay(step_delay);
//          digitalWrite(step_pin, LOW);
//          pos += (digitalRead(dir_pin) == inc_dir ? 1 : -1) * mm_per_step;
//        }
//};
//
////class StepperMotorFactory {
////  public:
////    static StepperMotor& create(int step_pin, int dir_pin, int stall_pin, int enable_pin, double mm_per_step, double max_pos, double increasing_direction, int step_delay = 600) {
////      StepperMotor* sm = new StepperMotor(step_pin, dir_pin, stall_pin, enable_pin,
////        mm_per_step, max_pos, increasing_direction, step_delay);
////      attachInterrupt(digitalPinToInterrupt(sm->get_stall_pin()), sm->set_stalled, RISING);
////      return *sm;
////    }
////};
