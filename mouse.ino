#include <IRremote.h>
#include <CircularBuffer.h>

#undef min
#undef max

#include "lib/arduino-tools/include/stepper_state_8.hpp"


struct interrupt_guard{
  interrupt_guard(){
    noInterrupts();
  }

  ~interrupt_guard(){
    interrupts();
  }
};


constexpr unsigned min(unsigned l, unsigned r){
  return l < r ? l : r;
}


enum class action: unsigned char{
  none,
  forward,
  backward,
  turn_left,
  turn_right
};

class chassis{
public:
  void move(action act, long steps){
    interrupt_guard lock;
    action_ = act;
    steps_ = steps;
    done_steps_ = 0;
  }

  void step_action(){
    auto steps = steps_;
    auto done_steps = done_steps_;
    if(steps == 0){
      left_.unset();
      right_.unset();
      return;
    }

    switch(action_){
      case action::forward:
        left_.prev();
        right_.prev();
      break;
      case action::backward:
        left_.next();
        right_.next();
      break;
      case action::turn_left:
        left_.prev();
        right_.next();
      break;
      case action::turn_right:
        left_.next();
        right_.prev();
      break;
    }

    left_.set();
    right_.set();

    --steps;
    ++done_steps;
    if(steps == 0){
      action_ = action::none;
    }

    steps_ = steps;
    done_steps_ = done_steps;
  }

  unsigned step_delay_us(){
    constexpr unsigned max_diff = max_step_delay_us - min_step_delay_us;
    return max_step_delay_us - min(done_steps_, min(steps_, max_diff));
  }

private:
  static constexpr unsigned max_step_delay_us = 1200;
  static constexpr unsigned min_step_delay_us = 650;

  unsigned long volatile done_steps_ = 0;
  unsigned long volatile steps_ = 0;
  action volatile action_ = action::none;
  tools::stepper_state_8<  5,  6,  7,  8 > right_;
  tools::stepper_state_8<  9, 10, 11, 12 > left_;
};

class chassis chassis;

constexpr unsigned long min_ir_pause_ms = 500;

constexpr int IR_RECV_PIN = 4;
IRrecv irrecv(IR_RECV_PIN);

void setup() {
  // motor right
  pinMode( 5, OUTPUT);
  pinMode( 6, OUTPUT);
  pinMode( 7, OUTPUT);
  pinMode( 8, OUTPUT);

  // motor left
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  // Buttons
  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);
  pinMode(16, INPUT);

  // Infrared receiver
  irrecv.enableIRIn();

  // init motors by one motor turn
  chassis.move(action::forward, 8);
  for(char i = 0; i < 8; ++i){
    chassis.step_action();
    delay(1);
  }

  // Timer 1 (16 bit)
  {
    interrupt_guard lock;
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 0xFFFF - chassis.step_delay_us() * 2; // timer start value
    TCCR1B |= (1 << CS11);                        // prescale: 8
    TIMSK1 |= (1 << TOIE1);                       // enable timer overflow interrupt
  }
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = 0xFFFFu - chassis.step_delay_us() * 2; // timer start value
  chassis.step_action();
}

void loop(){
  unsigned long last_ir_button = 0;
  unsigned long last_ir_delay = millis();

  CircularBuffer< action, 64 > movements;
  bool program_mode = true;

  for(;;){
    if(program_mode){
      if(digitalRead(15) == HIGH){
        movements.push(action::turn_right);
        delay(250);
      }else if(digitalRead(13) == HIGH){
        movements.push(action::backward);
        delay(250);
      }else if(digitalRead(14) == HIGH){
        movements.push(action::forward);
        delay(250);
      }else if(digitalRead(17) == HIGH){
        movements.push(action::turn_left);
        delay(250);
      }else if(digitalRead(16) == HIGH){
        program_mode = false;
        delay(250);
      }else{
        decode_results results;
        if(irrecv.decode(&results)){
          unsigned long const now = millis();
          if(last_ir_button != results.value || last_ir_delay < now){
            last_ir_delay = now + min_ir_pause_ms;
            last_ir_button = results.value;

            switch(last_ir_button){
              case 0xE0E006F9ul:
                chassis.move(action::forward, 3200);
              break;
              case 0xE0E046B9ul:
                chassis.move(action::turn_right, 3200);
              break;
              case 0xE0E08679ul:
                chassis.move(action::backward, 3200);
              break;
              case 0xE0E0A659ul:
                chassis.move(action::turn_left, 3200);
              break;
              case 0xE0E016E9ul:
                chassis.move(action::none, 0);
              break;
            }
          }

          irrecv.resume();
        }
      }
    }else{
      while(movements.size()){
        chassis.move(movements.pop(), 3200);
        delay(3500);
      }

      program_mode = true;
    }
  }
}
