// sntjoy.ino
// Author: mistepien@wp.pl
// Copyright 2025
//
// Inspired by https://github.com/jonthysell/SegaController
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

/*----------------------------------------
  |   DDRx   |   PORTx  |    result      |
  ---------------------------------------|
  ---------------------------------------|
  |    0     |     0    |     INPUT      |
  ----------------------------------------
  |    0     |     1    |  INPUT_PULLUP  |
  ----------------------------------------
  |    1     |     0    | OUTPUT (LOW)   |
  ----------------------------------------
  |    1     |     1    | OUTPUT (HIGH)  |
  ----------------------------------------

  LOW(0) state of both registers is DEFAULT,
  thus every pin is in INPUT mode without doing anything.

  ------------------            -----------------
  |  HARDWARE XOR  |            |  SOFTWARE XOR |
  ------------------            -----------------
  PINx = byte;    <========>    PORTx ^= byte;

  ------------------
  |  HARDWARE XOR  |
  ------------------

  "The port input pins I/O location is read only, while the data register and the
  data direction register are read/write. However, writing a logic one to a bit
  in the PINx register, will result in a toggle in the
  corresponding bit in the data register."

  That is more efficient since XOR operation is done in hardware, not software,
  and it saves cycles since in code there is no need to bother about XOR.*/

/*Source of these classic rapidfire frequencies - great input of Nightshft
  (Nightshft, thank you very much!):
  https://eab.abime.net/showpost.php?p=1364366&postcount=3
*/

#define __2X_VER__

#if !((F_CPU == 500000L) || (F_CPU == 1000000L))
#error This is designed only for 0.5MHz clock or for 1MHz!!!
#endif

#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
#define bitToggle(value, bit) ((value) ^= (1UL << (bit)))
#endif

//OUTPUT PINS FOR JOYSTICK
enum joystick_pinout {
  LBTN = 0,     //PB0
  DBTN = 1,     //PB1
  UBTN = 2,     //PB2
  F1BTN = 3,    //PB3
  F3BTN = 4,    //PB4
  F2BTN = 5,    //PB5
  C64MODE = 6,  //PB6
  RBTN = 7      //PB7
};

//UBTN/DBTN/LBTN/RBTN/F1BTN are in reversed logic (diodes between MCU and DB9)
//they are "NOT PRESSED" when they are HIGH
constexpr byte ATARI_JOY = bit(UBTN) | bit(DBTN) | bit(LBTN) | bit(RBTN) | bit(F1BTN);

#if defined(__2X_VER__)

//OUTPUT PINS FOR LEDS
enum leds_pullout_pinout {
  F3PULLUP = 0,    //PC0
  F2PULLUP = 1,    //PC1
  UNUSED_LP = 2,   //PC2
  CONF0LED = 3,    //PC3
  CONF1LED = 4,    //PC4
  AUTOFIRELED = 5  //PC5
};

#else

//OUTPUT PINS FOR LEDS
enum leds_pullout_pinout {
  F3PULLUP = 4,    //PC4
  F2PULLUP = 5,    //PC5
  UNUSED_LP = 1,   //PC0
  CONF0LED = 2,    //PC2
  CONF1LED = 3,    //PC1
  AUTOFIRELED = 0  //PC3
};

#endif

constexpr byte ALL_LEDS = bit(CONF0LED) | bit(CONF1LED) | bit(AUTOFIRELED);

constexpr byte ctl_off_led_bits[3] = { bit(AUTOFIRELED), bit(CONF1LED), bit(CONF0LED) };

#include <avr/wdt.h>
#include <avr/power.h>
#include <util/delay.h>
#include <EEPROM.h>
#include "SNES.h"
#include "Timer_params.h"

constexpr word _c64_amiga_combination = bit(SNES_BTN_A) | bit(SNES_BTN_B) | bit(SNES_BTN_Y) | bit(SNES_BTN_SELECT);
constexpr word _pullup_combination = bit(SNES_BTN_B) | bit(SNES_BTN_Y) | bit(SNES_BTN_SELECT) | bit(SNES_DPAD_RIGHT);


bool DPAD_UP = 0;
bool DPAD_DOWN = 0;
bool DPAD_LEFT = 0;
bool DPAD_RIGHT = 0;
bool BTN_UP = 0;
bool BTN_DOWN = 0;

word prev_gamepad_state = 0;
byte rapidfire_freq = 0;
bool timer_start_flag = 0;     //it will let to start timer together with update of PORT registers
volatile bool fire_rapid = 0;  //this variable is toggled by timer interrupt (0101010101)
volatile bool reading_controller_flag;
volatile bool blinking_led;
volatile bool save_eeprom_flag;
bool ctl_on = 0;  //with 0 it was false negative even with connected pad
volatile byte ctl_off_led_bit_number = 0;
bool rapidfire_button = 0;  //state of rapid button
bool autofire_button = 0;   //"artificial" state of autofire_button
bool fire_single = 0;       //to jest zwykły fire wciśnięty przez usera -- wciśnięty to strzał i tyle
bool rapid_up_down_btn = 0;
bool rapid_left_right_btn = 0;
bool start_btn = 0;
bool select_btn = 0;
bool reset_flag = 0;
byte ledstate = 0;
byte joystate = 0;
constexpr byte max_blinks = 10;


typedef struct {
  byte _counter;
  byte _packed_data;
} eeprom_buffer_struct;



/*
_packed_data byte:
7 - AMIGAmode_bit
6 - pullup_mode_bit
2, 3 - joyconf / 3, 4 - joyconf
1, 0 - rapidefire_freq
*/

constexpr byte AMIGAmode_bit = 7;
constexpr byte pullup_mode_bit = 6;

eeprom_buffer_struct eeprom_stuff;
byte eeprom_stuff_index;
constexpr byte sizeof_eeprom_stuff = sizeof(eeprom_stuff);
constexpr byte eeprom_stuff_last_before_max_index = ((255 / sizeof_eeprom_stuff) * sizeof_eeprom_stuff) - sizeof_eeprom_stuff;

void setup() {
  DDRB = 0xFF;  //OUTPUT LOW: PB0-7:
  //DDRB |= ATARI_JOY | bit(F2BTN) | bit(F3BTN) | bit(C64MODE);

  /*
  UP,DOWN,RIGHT,LEFT and F1BTN are in reversed logic -
  "not pressed" ("released") is 1, so OUTPUT is set to HIGH
  F2BTN and F3BTN are set to 0 */
  PINB = ATARI_JOY;  //PORTB is default to LOW, toggle to HIGH only ATARI_JOY


  //pull-up stuff
  DDRC |= ALL_LEDS | bit(F3PULLUP) | bit(F2PULLUP);  //set all (except UNUSED_LP)to output
                                                     //low by default
  //unused PIN in PORTC is set to INPUT_PULLUP
  //bitClear(DDRC, UNUSED_LP); INPUT IS DEFAULT
  bitSet(PORTC, UNUSED_LP);

  wdt_disable();

  //INITIALIZE SNES & SET UP REST OF PORTC
  snes.begin(2, 1, 0);
  PORTD |= bit(PD3) | bit(PD4) | bit(PD5) | bit(PD6) | bit(PD7);  //input_pullup

#if defined(__AVR_ATmega48PB__) || defined(__AVR_ATmega88PB__) || (__AVR_ATmega168PB__) || defined(__AVR_ATmega328PB__)
  byte _PE_Pins = bit(PE0) | bit(PE1) | bit(PE2) | bit(PE3);
  //DDRE &=  ~_PE_Pins;
  PORTE |= _PE_Pins;
#elif defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
  byte _PA_Pins = bit(PA0) | bit(PA1) | bit(PA2) | bit(PA3);
  //DDRA &=  ~_PA_Pins;
  PORTA |= _PA_Pins;

  PORTC |= bit(PC7);  //set to INPUT_PULLUP
#endif

  eeprom_buffer_struct tmp_eeprom_stuff = read_eeprom_stuff_packed_data();

  /* #############################################
  C64/AMIGA MODE stuff
  AMIGAmode = global variable -- that is a logic negation of SW1 NET on PCB
  AMIGAmode = 0 -- C64 mode -- so C64 mode does not draw power from pull_down resitor
                               C64MODE(PIN) is 
  AMIGAmode = 1 -- AMIGA mode  -- C64MODE(PIN) is set to LOW  */

  set_C64_AMIGA_MODE_in_setup(bitRead(tmp_eeprom_stuff._packed_data, AMIGAmode_bit));


  timer_instead_of_millis();


  byte _ledstate_setup = 0;
  bool __blinking_led__ = 0;
  byte reads_num = 0;
  word _snes_state_in_setup = 0;
  byte __mode_combination_pressed__ = 0;


  ///LOOP FOR C64-AMIGA/PULLUP combination
  while (reads_num < max_blinks) {

    if (__blinking_led__) {
      _ledstate_setup = tmp_eeprom_stuff._packed_data & bit(pullup_mode_bit) ? bit(CONF0LED) | bit(CONF1LED) : bit(CONF1LED) | bit(AUTOFIRELED);
    } else {
      _ledstate_setup = 0;
    }

    push_ledstate_to_register(_ledstate_setup);

    if (reading_controller_flag) {
      _snes_state_in_setup = snes.getState();
      if (is_C64_AMIGA_mode_combination_pressed_in_setup(_snes_state_in_setup)) {
        bitSet(__mode_combination_pressed__, AMIGAmode_bit);
      } else if (is_pullup_mode_combination_pressed_in_setup(_snes_state_in_setup)) {
        bitSet(__mode_combination_pressed__, pullup_mode_bit);
      }
      reading_controller_flag = 0;
    }

    if (__mode_combination_pressed__) {
      break;
    }

    __blinking_led__ = blinking_led;
    reads_num += catcher_timer_flag(__blinking_led__);
  }
  ////end of LOOP for combination

  tmp_eeprom_stuff._packed_data ^= __mode_combination_pressed__;

  if (__mode_combination_pressed__ & bit(AMIGAmode_bit)) {
    set_C64_AMIGA_MODE_in_setup(bitRead(tmp_eeprom_stuff._packed_data, AMIGAmode_bit));
  }

  set_pullup_mode_in_setup(tmp_eeprom_stuff._packed_data);

  eeprom_stuff = tmp_eeprom_stuff;

  if (__mode_combination_pressed__) {
    smart_eeprom_stuff_put();
    if (__mode_combination_pressed__ & bit(AMIGAmode_bit)) {
      blinking_leds_after_combination(ALL_LEDS);
    } else if (__mode_combination_pressed__ & bit(pullup_mode_bit)) {
      blinking_leds_after_combination(tmp_eeprom_stuff._packed_data & bit(pullup_mode_bit) ? bit(CONF0LED) | bit(CONF1LED) : bit(CONF1LED) | bit(AUTOFIRELED));
    }
  }

  //set joyconf
  byte tmp_joyconf = (tmp_eeprom_stuff._packed_data & (bit(CONF0LED) | bit(CONF1LED))) >> CONF0LED;
  set_joyconf(tmp_joyconf);

  //set rapidfire frequency
  rapidfire_freq = tmp_eeprom_stuff._packed_data & 3;

  //power stuff -- power only necessary things
  power_all_disable();
#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
  power_timer0_enable();  //millis()
#endif
  power_timer1_enable();  //autofire
#if !defined(__AVR_ATtiny88__) && !defined(__AVR_ATtiny48__)
  power_timer2_enable();  //reading controller
#endif

  timer_stop();

  wdt_enable(WDTO_15MS);
}

void set_C64_AMIGA_MODE_in_setup(bool ___AMIGAmode___) {
  /*if (___AMIGAmode___) {  //C64MODE pin is set to LOW, AMIGAmode flag is 1  -- that is AMIGA mode
    bitClear(PORTB, C64MODE);
  } else {
    bitSet(PORTB, C64MODE);  //C64MODE pin is set to HIGH, AMIGAmode flag is 0 -- that is C64 mode
  }*/
  bitWrite(PORTB, C64MODE, 1 ^ ___AMIGAmode___);
}

bool complex_bool_value(byte big_byte, byte tested_byte) {
  bool ___output = (big_byte & tested_byte) ^ tested_byte ? 0 : 1;
  return ___output;
}

void set_pullup_mode_in_setup(byte ___AMIGAmode___pullup_mode___) {
  if (complex_bool_value(___AMIGAmode___pullup_mode___, bit(AMIGAmode_bit) | bit(pullup_mode_bit))) {
    update_pull_up_register_in_AMIGA_mode(bit(F2BTN) | bit(F3BTN));
  } /* PULLUP is now LOW and F2BTB/F3BTN are LOW
         we set PULLUP to HIGH and F2BTB/F3BTN
         since now we can XOR PULLUP and F2BTB/F3BTN
         with same data and they will be "xored" to each
         other */
}

bool is_C64_AMIGA_mode_combination_pressed_in_setup(word _snes_state_) {
  bool _comb_pressed = 0;
  _comb_pressed = _snes_state_ ^ (_c64_amiga_combination | bit(SNES_CTL_ON));  //xor = 1 -- combination is NOT PRESSED
  _comb_pressed ^= 1;

  return _comb_pressed;
}

bool is_pullup_mode_combination_pressed_in_setup(word _snes_state_) {
  bool _comb_pressed = 0;
  _comb_pressed = _snes_state_ ^ (_pullup_combination | bit(SNES_CTL_ON));  //xor = 1 -- combination is NOT PRESSED
  _comb_pressed ^= 1;

  return _comb_pressed;
}

#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
void timer_instead_of_millis() {
  noInterrupts();
  // Clear registers
  TCNT0 = 0;

#if (F_CPU == 1000000L)
  // 269.3965517241379 Hz (1000000/((57+1)*64))
  OCR0A = 57;
#elif (F_CPU == 500000L)
  // 269.3965517241379 Hz (500000/((28+1)*64))
  OCR0A = 28;
#endif

  // CTC && Prescaler 64
  TCCR0A = bit(CTC0) | bit(CS01) | bit(CS00);

  // Output Compare Match A Interrupt Enable
  TIMSK0 = bit(OCIE0A);
  interrupts();
}
#else
void timer_instead_of_millis() {
  noInterrupts();
  // set CTC & clear all other bits in TCCR2A
  TCCR2A = bit(WGM21);

  TCNT2 = 0;

#if (F_CPU == 1000000L)
  // 271.7391304347826 Hz (1000000/((114+1)*32))
  OCR2A = 114;

  // Prescaler 32
  TCCR2B = bit(CS21) | bit(CS20);
#elif (F_CPU == 500000L)
  // 270.56277056277054 Hz (500000/((230+1)*8))
  OCR2A = 230;

  // Prescaler 8
  TCCR2B = bit(CS21);
#endif

  // Output Compare Match A Interrupt Enable
  TIMSK2 = bit(OCIE2A);
  interrupts();
}
#endif

#if defined(__AVR_ATtiny88__) || defined(__AVR_ATtiny48__)
ISR(TIMER0_COMPA_vect) {
#else
ISR(TIMER2_COMPA_vect) {
#endif
  reading_controller_flag = 1;

  static unsigned int _counter_eeprom;
  if (_counter_eeprom == 0) {
    save_eeprom_flag = 1;
  }

  if (_counter_eeprom++ > 1200) {  //1200 * 3,7ms = 4440 ms =4.44s
    _counter_eeprom = 0;
  }

  static byte _counter;
  if (1 ^ ctl_on) {
    if (_counter++ > 40) {  //40 * 3,7ms = 148ms
      _counter = 0;
      blinking_led ^= 1;
      if (ctl_off_led_bit_number++ > 2) {
        ctl_off_led_bit_number = 0;
      }
    }
  }
}

void timer_start(byte rspeed = 0) {
  noInterrupts();
  // set CTC & clear all other bits in TCCR1B
  TCCR1B = bit(WGM12);

  OCR1A = timer_params[rspeed].ocr1a;

#if (F_CPU == 1000000L)  //two prescalers: 1 & 8

  /*switch (timer_params[rspeed].prescaler) {
    case 0:  //1
      TCCR1B |= bit(CS10);
      break;
    case 1:  //8
      TCCR1B |= bit(CS11);
      break; 
  } */
  TCCR1B |= timer_params[rspeed].prescaler ? bit(CS11) : bit(CS10);
#elif (F_CPU == 500000L)  //only one prescaler: 1
  TCCR1B |= bit(CS10);  //1
#endif

  /*----------------------------------------------------------------------------
    look a file Timer_params.h -- what Prescalers are really used in Timer_params.h?
    ---------------------------------------------------------------------------*/
  // set prescaler
  /* switch (timer_params[rspeed].prescaler) {
    case 0:  //1
      TCCR1B |= bit(CS10);
      break;
    case 1:  //8
      TCCR1B |= bit(CS11);
      break;
    case 2:  //64
      TCCR1B |= bit(CS11) | bit(CS10);
      break;
    case 3:  //256
      TCCR1B |= bit(CS12);
      break;
    case 4:  //1024
      TCCR1B |= bit(CS12) | bit(CS10);
      break;
  } */

  // Output Compare Match A Interrupt Enable
#if defined(__AVR_ATmega8__)
  TIMSK |= bit(OCIE1A);
#else
  TIMSK1 |= bit(OCIE1A);
#endif


  interrupts();
}

void timer_stop() {
  noInterrupts();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

#if defined(__AVR_ATmega8__)
  TIMSK = 0;
#else
  TIMSK1 = 0;
#endif

  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  fire_rapid ^= 1;
}

void rapidfire_toggle(bool state) {
  static byte rapidfire_freq_prev;
  static bool rapidfire_toggle_state_prev;
  bool changed_rapidfire_toggle_state = state ^ rapidfire_toggle_state_prev;
  byte changed_rapidfire_freq = rapidfire_freq ^ rapidfire_freq_prev;

  if (changed_rapidfire_toggle_state || changed_rapidfire_freq) {
    if (changed_rapidfire_toggle_state) {
      rapidfire_toggle_state_prev = state;
    }
    if (changed_rapidfire_freq) {
      rapidfire_freq_prev = rapidfire_freq;
    }

    timer_stop();

    timer_start_flag = state;
    fire_rapid = state;
  }
}

void set_joyconf(byte conf) {
  static byte _prev_conf;
  if (_prev_conf ^ conf) {
    _prev_conf = conf;
    ledstate = (ledstate & ~(bit(CONF0LED) | bit(CONF1LED))) | (conf << CONF0LED);
    reset_flag = 1;
  }
}

void reset_buttons() {
  fire_single = 0;
  rapidfire_button = 0;
  rapid_up_down_btn = 0;
  rapid_left_right_btn = 0;
  autofire_button = 0;
  start_btn = 0;
  select_btn = 0;
  joystate &= ~(bit(F2BTN) | bit(F3BTN));
  BTN_UP = 0;
  BTN_DOWN = 0;
  prev_gamepad_state &= (bit(SNES_CTL_ON) | bit(SNES_DPAD_UP) | bit(SNES_DPAD_DOWN) | bit(SNES_DPAD_LEFT) | bit(SNES_DPAD_RIGHT));
}

void button(byte btn, bool btn_state) {  //that function is used in loop reading every bit of controller's state
  //and here we have a "choice" between different configurations
  button_basic(btn, btn_state);

  byte start_select_state = (start_btn | (select_btn << 1));
  switch (start_select_state) {
    case 1:  // 01
      button_pressed_WITH_start_btn(btn, btn_state);
      break;
    case 2:  // 10
      button_pressed_WITH_select_btn(btn, btn_state);
      break;
    default:  //0 or 3 -- 00 or 11
      button_common_pressed_WITHOUT_start_and_select_btns(btn, btn_state);
      switch ((ledstate >> CONF0LED) & 3) {
        case 0:
          button_0conf(btn, btn_state);
          break;
        case 1:
          button_1conf(btn, btn_state);
          break;
        case 2:
          button_2conf(btn, btn_state);
          break;
        case 3:
          button_3conf(btn, btn_state);
          break;
      }
  }
}

void button_basic(byte btn, bool btn_state) {
  switch (btn) {
    case SNES_CTL_ON:
      reset_flag = 1;
      ctl_on = btn_state;
      break;
    case SNES_BTN_START:
      start_btn = btn_state;
      break;
    case SNES_BTN_SELECT:
      select_btn = btn_state;
      break;
  }
}

void button_pressed_WITH_start_btn(byte btn, bool btn_state) {
  switch (btn) {
    case SNES_DPAD_UP:  //+ START BTN
      if (btn_state) {
        rapidfire_freq = 0;
      } else {
        DPAD_UP = 0;
      }
      break;
    case SNES_DPAD_DOWN:  //+ START BTN
      if (btn_state) {
        rapidfire_freq = 2;
      } else {
        DPAD_DOWN = 0;
      }
      break;
    case SNES_DPAD_LEFT:  //+ START BTN
      if (btn_state) {
        rapidfire_freq = 3;
      } else {
        DPAD_LEFT = 0;
      }
      break;
    case SNES_DPAD_RIGHT:  //+ START BTN
      if (btn_state) {
        rapidfire_freq = 1;
      } else {
        DPAD_RIGHT = 0;
      }
      break;
  }
}

void button_pressed_WITH_select_btn(byte btn, bool btn_state) {
  switch (btn) {
    case SNES_DPAD_UP:  //+ SELECT BTN
      if (btn_state) {
        set_joyconf(0);
      } else {
        DPAD_UP = 0;
      }
      break;
    case SNES_DPAD_DOWN:  //+ SELECT BTN
      if (btn_state) {
        set_joyconf(2);
      } else {
        DPAD_DOWN = 0;
      }
      break;
    case SNES_DPAD_LEFT:  //+ SELECT BTN
      if (btn_state) {
        set_joyconf(3);
      } else {
        DPAD_LEFT = 0;
      }
      break;
    case SNES_DPAD_RIGHT:  //+ SELECT BTN
      if (btn_state) {
        set_joyconf(1);
      } else {
        DPAD_RIGHT = 0;
      }
      break;
  }
}

void button_common_pressed_WITHOUT_start_and_select_btns(byte btn, bool btn_state) {
  switch (btn) {
    case SNES_DPAD_UP:  //DPAD_UP
      DPAD_UP = btn_state;
      break;
    case SNES_DPAD_DOWN:  //DPAD_DOWN
      DPAD_DOWN = btn_state;
      break;
    case SNES_DPAD_LEFT:  //DPAD_LEFT
      DPAD_LEFT = btn_state;
      break;
    case SNES_DPAD_RIGHT:  //DPAD_RIGHT
      DPAD_RIGHT = btn_state;
      break;
  }
}

void autofire_button_func(bool btn_state) {
  if (btn_state) {
    if (autofire_button) {
      bitClear(ledstate, AUTOFIRELED);
    }
    autofire_button ^= 1;
  }
}

void set_btn(byte joypin, bool btn_state) {
  bitWrite(joystate, joypin, btn_state);
}

void button_2conf(byte btn, bool btn_state) {
  switch (btn) {
    case SNES_BTN_X:  //UP BUTTON
      BTN_UP = btn_state;
      break;
    case SNES_BTN_Y:  //RAPIDFIRE
      rapidfire_button = btn_state;
      break;
    case SNES_BTN_B:  //DOWN BUTTON
      BTN_DOWN = btn_state;
      break;
    case SNES_BTN_A:  //FIRE1
      fire_single = btn_state;
      break;
    case SNES_BTN_TR:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SNES_BTN_TL:  //F3
      set_btn(F3BTN, btn_state);
      break;
  }
}

void button_0conf(byte btn, bool btn_state) {
  switch (btn) {
    case SNES_BTN_A:  //UP BUTTON
      BTN_UP = btn_state;
      break;
    case SNES_BTN_Y:  //RAPIDFIRE
      rapidfire_button = btn_state;
      break;
    case SNES_BTN_TL:  //AUTOFIRE BUTTON
      autofire_button_func(btn_state);
      break;
    case SNES_BTN_X:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SNES_BTN_TR:  //F3
      set_btn(F3BTN, btn_state);
      break;
    case SNES_BTN_B:  //FIRE1
      fire_single = btn_state;
      break;
  }
}

void button_3conf(byte btn, bool btn_state) {
  switch (btn) {
    case SNES_BTN_X:  //UP-DOWN RAPID BUTTON
      rapid_up_down_btn = btn_state;
      break;
    case SNES_BTN_Y:  //LEFT-RIGHT RAPID BUTTON
      rapid_left_right_btn = btn_state;
      break;
    case SNES_BTN_B:  //RAPIDFIRE
      rapidfire_button = btn_state;
      break;
    case SNES_BTN_A:  //FIRE1
      fire_single = btn_state;
      break;
    case SNES_BTN_TR:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SNES_BTN_TL:  //F3
      set_btn(F3BTN, btn_state);
      break;
  }
}

void button_1conf(byte btn, bool btn_state) {
  switch (btn) {
    case SNES_BTN_A:  //UP BUTTON
      BTN_UP = btn_state;
      break;
    case SNES_BTN_X:  //AUTOFIRE BUTTON
      autofire_button_func(btn_state);
      break;
    case SNES_BTN_Y:  //RAPIDFIRE
      rapidfire_button = btn_state;
      break;
    case SNES_BTN_B:  //FIRE1
      fire_single = btn_state;
      break;
    case SNES_BTN_TR:  //F2
      set_btn(F2BTN, btn_state);
      break;
    case SNES_BTN_TL:  //F3
      set_btn(F3BTN, btn_state);
      break;
  }
}

byte ctl_off_blinking(byte _ledstate) {
  return ctl_on ? _ledstate : ctl_off_led_bits[ctl_off_led_bit_number];
}

void push_ledstate_to_register(byte _ledstate) {
  //Pushing ledstate to PORTC
  static byte prev_ledstate;
  byte changed_ledstate = prev_ledstate ^ _ledstate;
  if (changed_ledstate) {
    //changed_ledstate &= ALL_LEDS; //that seems to be redundant
    prev_ledstate = _ledstate;
    PINC = changed_ledstate;  //hardware XOR
  }
}

void push_joystate_and_pullstate_to_register(byte _joystate) {
  //Pushing joystate to PORTB
  static byte prev_joystate;
  byte changed_joystate = prev_joystate ^ _joystate;
  if (changed_joystate) {
    //changed_joystate &= ~(bit(C64MODE)); //that seems to be redundant -- nothing is changing bit(C64MODE) in loop()
    PINB = changed_joystate;  //hardware XOR

    if (complex_bool_value(eeprom_stuff._packed_data, bit(AMIGAmode_bit) | bit(pullup_mode_bit))) {
      byte changed_F2F3 = changed_joystate & (bit(F2BTN) | bit(F3BTN));
      update_pull_up_register_in_AMIGA_mode(changed_F2F3);
    }

    prev_joystate = _joystate;
  }
}

// PULLUP for F2BTN and F3BTN
void update_pull_up_register_in_AMIGA_mode(byte _changed_F2F3) {
#if defined(__2X_VER__)
  _changed_F2F3 >>= F3BTN;
#endif

  PINC = _changed_F2F3;  //hardware XOR
                         /*both PULLUP PINS are since setup()
                          reversed to F2BTN/F3BTN -- they are in
                          antiphase
                          thus when F2 is 1->0, the same
                          time F2_PULLUP is 0->1 :) */
}

byte catcher_timer_flag(bool __flag_to_catch) {
  static bool __prev_flag_to_catch;
  if (__flag_to_catch ^ __prev_flag_to_catch) {
    __prev_flag_to_catch = __flag_to_catch;
    return 1;
  } else {
    return 0;
  }
}

void blinking_leds_after_combination(byte __leds__) {
  byte reads_num = 0;
  bool __blinking_led__ = 0;
  byte _ledstate_setup = 0;
  while (reads_num < max_blinks) {
    __blinking_led__ = blinking_led;
    _ledstate_setup = __blinking_led__ ? __leds__ : 0;
    push_ledstate_to_register(_ledstate_setup);
    reads_num += catcher_timer_flag(__blinking_led__);
  }
}

bool nod_bool(bool OUT_DIR, bool SECOND_DIR) {
  //bool value = ((!( OUT_DIR  || SECOND_DIR )) || OUT_DIR); //logic where LOW=pressed, HIGH=released
  bool value = ((!(OUT_DIR && SECOND_DIR)) && OUT_DIR);  //logic where HIGH=pressed, LOW=released
  return value;
}

void process_state(word current_gamepad_state) {

  //NO START_BTN and SELECT_BTN pressed at the same time
  current_gamepad_state = snes.nod(current_gamepad_state, bit(SNES_BTN_START) | bit(SNES_BTN_SELECT));

  ///////////////////////////////////////////////////////////
  //here everything happens only if current_state is changed
  ///////////////////////////////////////////////////////////
  word changed_gamepad_state = prev_gamepad_state ^ current_gamepad_state;
  if (changed_gamepad_state) {
    prev_gamepad_state = current_gamepad_state;

    for (byte index = 0; index < 13; index++) {
      if (changed_gamepad_state & 1) {
        button(index, current_gamepad_state & 1);
      } else if (!changed_gamepad_state) {
        break;
      }
      changed_gamepad_state >>= 1;
      current_gamepad_state >>= 1;
    }

    if (reset_flag) {
      reset_buttons();
      reset_flag = 0;
    }
    rapidfire_toggle(rapidfire_button | autofire_button | rapid_left_right_btn | rapid_up_down_btn);

    //AUTOFILE LED follows changes of autofire_button
    bitWrite(ledstate, AUTOFIRELED, autofire_button);


    //join together DPAD_DOWN and BTN_DOWN -- BTN_UP prevails over DPAD_DOWN
    DPAD_DOWN = nod_bool(DPAD_DOWN, BTN_UP);

    //BTN_UP prevails over BTN_DOWN
    BTN_DOWN = nod_bool(BTN_DOWN, BTN_UP);

    //join together DPAD_UP and BTN_UP -- BTN_DOWN prevails over DPAD_UP
    DPAD_UP = nod_bool(DPAD_UP, BTN_DOWN);

    //starting timer is always related to pressing controller button
    //so execute timer_start() only if current_state is changed
    if (timer_start_flag) {
      timer_start(rapidfire_freq);
      timer_start_flag = 0;  //if timer is already started it would be futile to start it once again
    }
  }
}

void push_stuff() {
  ///////////////////////////////////////////////////////////
  //here everything happens in EVERY cycle of loop()
  ///////////////////////////////////////////////////////////
  bool tmp_fire_rapid = fire_rapid; /*since fire_rapid is updated by ISR(TIMER1_COMPA_vect)
                                      and that used more than once in this function it is prone to
                                      be changed during execution of the function.
                                      tmp_fire_rapid is "frozen" value of fire_rapid
                                    */


  /*fire_rapid is updated by ISR(TIMER1_COMPA_vect)
    thus update of F1BTN/UP_DOWN_RAPID/LEFT_RIGHT_RAPID cannot depend upon
    change of SNES controller buttons
    so the whole joystate PORT have to be updated
    in every possible cycle */

  bool fire_output = (rapidfire_button | autofire_button) ? (fire_single | tmp_fire_rapid) : fire_single;
  set_btn(F1BTN, fire_output);


  if (rapid_up_down_btn) {
    bool BTN_UP_RAPID = tmp_fire_rapid;
    bool BTN_DOWN_RAPID = tmp_fire_rapid ^ 1;
    set_btn(UBTN, BTN_UP_RAPID);
    set_btn(DBTN, BTN_DOWN_RAPID);
  } else {
    set_btn(DBTN, DPAD_DOWN | BTN_DOWN);  //DOWN
    set_btn(UBTN, DPAD_UP | BTN_UP);      //UP
  }

  if (rapid_left_right_btn) {
    bool BTN_LEFT_RAPID = tmp_fire_rapid;
    bool BTN_RIGHT_RAPID = tmp_fire_rapid ^ 1;
    set_btn(LBTN, BTN_LEFT_RAPID);
    set_btn(RBTN, BTN_RIGHT_RAPID);
  } else {
    set_btn(RBTN, DPAD_RIGHT);
    set_btn(LBTN, DPAD_LEFT);
  }

  push_joystate_and_pullstate_to_register(joystate);

  push_ledstate_to_register(ctl_off_blinking(ledstate));

  try_push_stuff_to_EEPROM();
}

/*
###################################################################
########################### EEPROM STUFF
###################################################################
*/

byte read_eeprom_stuff_index() {
  byte _tmp_eeprom_stuff_index = EEPROM.read(0);

  switch (_tmp_eeprom_stuff_index) {
    case 0:
      _tmp_eeprom_stuff_index = 1;
      break;
    default:
      if ((_tmp_eeprom_stuff_index - 1) % sizeof_eeprom_stuff) {  //there is remainder -- thus _tmp_eeprom_stuff_index IS NOT dividible by sizeof_eeprom_stuff
        _tmp_eeprom_stuff_index = 1;
      }
  }

  return _tmp_eeprom_stuff_index;
}

eeprom_buffer_struct read_eeprom_stuff_packed_data() {
  //read values from EEPROM
  eeprom_buffer_struct _tmp_eeprom_stuff;
  eeprom_stuff_index = read_eeprom_stuff_index();
  EEPROM.get(eeprom_stuff_index, _tmp_eeprom_stuff);
  return _tmp_eeprom_stuff;
}

void smart_eeprom_stuff_put() {
  if (eeprom_stuff._counter == 255) {
    switch (eeprom_stuff_index) {
      case 1 ... eeprom_stuff_last_before_max_index:
        eeprom_stuff_index += sizeof_eeprom_stuff;
        break;
      default:
        eeprom_stuff_index = 1;
    }
  }
  eeprom_stuff._counter++;  //if 255 (byte) after ++ that makes 0
  noInterrupts();
  EEPROM.update(0, eeprom_stuff_index);
  EEPROM.put(eeprom_stuff_index, eeprom_stuff);
  interrupts();
}

byte pack_stuff_data(byte __packed_data__, byte _ledstate, byte in_rapidfire_freq) {
  in_rapidfire_freq |= (_ledstate & (bit(CONF0LED) | bit(CONF1LED)));
  in_rapidfire_freq |= (__packed_data__ & (bit(AMIGAmode_bit) | bit(pullup_mode_bit)));
  return in_rapidfire_freq;
}

void try_push_stuff_to_EEPROM() {
  if (save_eeprom_flag) {
    save_eeprom_flag = 0;
    byte _new_eeprom_stuff___packed_data = pack_stuff_data(eeprom_stuff._packed_data, ledstate, rapidfire_freq);
    if (eeprom_stuff._packed_data ^ _new_eeprom_stuff___packed_data) {
      eeprom_stuff._packed_data = _new_eeprom_stuff___packed_data;
      smart_eeprom_stuff_put();  //that takes time so before that eeprom_stuff's changes are checked earlier
    }
  }
}

//#########################################################################

void loop() {
  if (reading_controller_flag) {
    process_state(snes.getState());
    reading_controller_flag = 0;
  }
  push_stuff();
  wdt_reset();
}

int main(void) {
  interrupts();

// Set internal oscillator prescaler if defined (in boards.txt)
#if defined(CLKPR) && defined(OSC_PRESCALER)
  CLKPR = 0x80;           // Enable prescaler
  CLKPR = OSC_PRESCALER;  // Set prescaler
#endif

  setup();

  while (1) {
    loop();
  }

  return 0;
}