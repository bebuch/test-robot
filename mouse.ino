#include "lib/arduino-tools/include/stepper_state_8.hpp"


struct interrupt_guard{
  interrupt_guard(){
    noInterrupts();
  }

  ~interrupt_guard(){
    interrupts();
  }
};


enum class action{
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
  }

  void step_action(){
    auto steps = steps_;
    if(steps == 0){
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
    if(steps == 0){
      action_ = action::none;
    }

    steps_ = steps;
  }

private:
  unsigned long volatile steps_ = 0;
  action volatile action_ = action::none;
  tools::stepper_state_8<  5,  6,  7,  8 > right_;
  tools::stepper_state_8<  9, 10, 11, 12 > left_;
};

class chassis chassis;

constexpr unsigned microsecond_step = 1000;

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

  // init motors by one motor turn
  chassis.move(action::forward, 8);
  for(int i = 0; i < 8; ++i){
    chassis.step_action();
    delay(1);
  }

  // Timer 1 (16 bit)
  {
    interrupt_guard lock;
    TCCR1A = 0;
    TCCR1B = 0;

    TCNT1 = 0xFFFF - microsecond_step * 2; // timer start value
    TCCR1B |= (1 << CS11);                 // prescale: 8
    TIMSK1 |= (1 << TOIE1);                // enable timer overflow interrupt
  }
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = 0xFFFFu - microsecond_step * 2; // timer start value
  chassis.step_action();
}

void loop() {
  if(digitalRead(15) == HIGH){
    chassis.move(action::turn_right, 3200);
    delay(100);
  }else if(digitalRead(13) == HIGH){
    chassis.move(action::backward, 3200);
    delay(100);
  }else if(digitalRead(14) == HIGH){
    chassis.move(action::forward, 3200);
    delay(100);
  }else if(digitalRead(17) == HIGH){
    chassis.move(action::turn_left, 3200);
    delay(100);
  }else if(digitalRead(16) == HIGH){
    chassis.move(action::none, 0);
    delay(100);
  }
}
