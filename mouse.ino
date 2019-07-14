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
      m1.unset();
      m2.unset();
      m3.unset();
      return;
    }

    switch(action_){
      case action::forward:
        m1.next();
        m2.prev();
      break;
      case action::backward:
        m1.prev();
        m2.next();
      break;
      case action::turn_left:
        m1.next();
        m2.next();
        m3.next();
      break;
      case action::turn_right:
        m1.prev();
        m2.prev();
        m3.prev();
      break;
    }

    m1.set();
    m2.set();
    m3.set();

    --steps;
    ++done_steps;
    if(steps == 0){
      action_ = action::none;
    }

    steps_ = steps;
    done_steps_ = done_steps;
  }

  unsigned step_delay_us(){
    return step_delay_us_;
  }

private:
  static constexpr unsigned step_delay_us_ = 1000;

  unsigned long volatile done_steps_ = 0;
  unsigned long volatile steps_ = 0;
  action volatile action_ = action::none;
  tools::stepper_state_8<  5,  6,  7,  8 > m1;
  tools::stepper_state_8<  9, 10, 11, 12 > m2;
  tools::stepper_state_8< 14, 15, 16, 17 > m3;
};

class chassis chassis;

constexpr unsigned long min_ir_pause_ms = 500;

constexpr int IR_RECV_PIN = 4;
IRrecv irrecv(IR_RECV_PIN);

void setup() {
  // motor 1
  pinMode( 5, OUTPUT);
  pinMode( 6, OUTPUT);
  pinMode( 7, OUTPUT);
  pinMode( 8, OUTPUT);

  // motor 2
  pinMode( 9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  // motor 3
  pinMode(14, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(17, OUTPUT);

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

  for(;;){
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
}
