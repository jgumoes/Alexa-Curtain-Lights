/*
 * this sketch runs a PWM on an ATTiny, with chip select and complementary outputs (to drive the H bridge).
 * this is the lowest form-factor way of doing this with what I already have. seriously.
 * 
 * this version contains a button debounce, effectively using the AtTiny as a digital filter:
 * - when the button pin is pulled low, a timer is started. if the button is still low when the timer expires, the press is registered as genuine.
 * - (if implemented with interrupts,) a falling edge starts a millisecond timer with preset OCR. when the timer expires, if the button pin is still low,
 *        the press is registered as genuine
 * - a rising edge could be used to cancel the timer, or, on compare, button_out is set to !button_pin state (button press pulls low)
 * 
 * the sketch has an output button_out, which will have two operating modes:
 *    1. initial: the button_out indicates the state of the LED
 *                  - the AtTiny operates independantly until the ESP has initiallised.
 *                  - i.e. the button presses toggle the LED state, and button_out reflects the LED state
 *                  - AtTiny remains in this state until the ESP pulls toggle_pin HIGH for the first time
 *    
 *    0. normal: button_out indicates a button press
 *                  - when a button press is registered, button_out goes HIGH until the toggle_pin changes state.
 *                  - 
 * 
 * direct registers are on page 89 of the datasheet
 * https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2586-AVR-8-bit-Microcontroller-ATtiny25-ATtiny45-ATtiny85_Datasheet.pdf
 */

#include <avr/io.h>
#include <avr/interrupt.h>

const int toggle_pin = 4;   // the toggle from the ESP. electrical pin 3
const int button_pin = 2; // the button input pin. electrical pin 7
const int button_out = 3;  // the button output pin. electrical pin 2
uint8_t button_mode = 1;   // flags the button mode. 1 = initial, 0 = normal
const uint8_t timer1 = 0b01010001;  // default settings of TCCR1
volatile uint8_t PWM_state = 1;
const uint8_t OCR0A_val = 194; // default value for OCR
volatile uint8_t toggle_flag = 0;
uint8_t prev_toggle = 0;
uint8_t toggle;
/*
ISR(PCINT0_vect){
  // handle the toggle_pin
  // sets timer 1 state to the state of toggle_pin
  toggle_flag = 1;
}
*/

ISR(INT0_vect){
  // initiate a 50mS timer. When the timer expires, it will set PB3 to the state of PB2
  TCNT0 = 0;    //reset timer register
  TCCR0B = 0b00000100;
}

volatile uint8_t button_flag = 0;

ISR(TIMER0_COMPA_vect){
  // toggle button_flag
  TCCR0B = 0;
  TCNT0 = 0;          // reset the timer register


  // button_flag goes high if level is low. it doesn't seem to go low if level is high.
  button_flag = ((255 ^ PINB) >> button_pin) & 1; // the state of the button press (the button is inverted (pulled high))
}

void setup() {
  
  pinMode(1, OUTPUT);
  pinMode(0, OUTPUT);

  pinMode(button_out, OUTPUT);
  
  // Prime the mS timer, but keep it off on boot
  //GTCCR = 0b10000000;   // make sure timer is off (first bit !pauses or !starts the timer)
  TCCR0A = 0b00000010;    // {1:0] WGM01, WGM00: CTC mode
  //TCCR0B = 0b00000100;
  TCCR0B = 0;             // [3] WGM02: CTC mode
  OCR0A = OCR0A_val; //194;
  TIMSK |= (1 << OCIE0A); //0b00010000;   // [6]: enable output compare interrupt on OCOA

  // setup an external interupt on pin 2. copied from https://www.instructables.com/ATtiny85-Interrupt-Barebones-Example/
  pinMode(button_pin, INPUT_PULLUP);   // set interrupt pin as input with a pullup to keep it stable
  MCUCR |= (1 << 1); //ISC01);      // set external interrupt to trigger on falling edge
  GIMSK |= (1 << INT0);// | (1 << PCIE);       // Enable PCINT interrupt and externl interrupt in the general interrupt mask
  
  // Prime the LED PWM, and start it at boot
  //TCCR1 = 0b01010001;
  TCCR1 = timer1;
  OCR1A = 127;
  PORTB |= (1 << button_out);   // turn button_out high on boot

  GTCCR = 0;  // make sure timer0 can run
  sei();      // make sure interuppts are enabled

  pinMode(toggle_pin, INPUT);   // toggle input. ESP raises high to turn PWM on. this should be pulled to ground
  PCMSK |= (1 << toggle_pin);   // enable pin change interrupt on toggle_pin

  uint8_t prev_button_flag = button_flag;
  //while(digitalRead(toggle_pin == LOW)){
  while(1){ // why does this kill the debouncing?
    // remain in setup mode until toggle_pin is pulled high for the first time
    if(button_flag){
      toggle = 1;
      PORTB |= 1 << button_out;   // i just had so much fun with direct port manipulation, i jst couldn't let it go!
      TCCR1 = timer1;
      PORTB &= 255;
    }
    else{
      toggle = 0;
      PORTB &= 255 - (1 << button_out);
      TCCR1 = 0;
      PORTB &= 252;   // 255 - 3
    }
    prev_button_flag = button_flag;
    while(button_flag == prev_button_flag){
      // wait until button_flag changes before looping again
      // todo: add a delay
    }
  }
  // todo: add a delay here also
  while(digitalRead(toggle_pin == HIGH)){
  }
  button_mode = 0;
}

int my_delay(int top){
  // a delay on timer0 OC0B. counts to top then returns 1
  OCR0B = TCNT0 + top;    // 
}

void switch_LED(){
  // switches the LEDs
}


void loop(){
  //if(button_flag & button_mode){  // button flagged and mode 1
  if(button_flag){ // if button_flag has been raised
    //PORTB ^= button_mode << button_out;  // toggle button_out if button_mode is 1
    PORTB |= 1 << button_out; // sets button_out to 1 if button_mode is 0
    //TCCR1 ^= timer1 * (button_mode & button_flag);  // toggle TCCR1 if button_flag & button_mode are high
  }
  //elif((button_mode - button_flag) == 1){ // if button_mode is 1 and button mode is 0. hoping I don't have to use this
    //}
  toggle = digitalRead(toggle_pin);
  if(toggle_flag != prev_toggle){ // if toggle_pin has changed state
    button_flag = 0;
    TCCR1 ^= timer1 * toggle;
    PORTB &= 255 ^ (3 * toggle);  // leave or zero pins 0 & 1 
  }
}
