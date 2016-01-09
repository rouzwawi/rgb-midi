#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "xnormidi/serial_midi.h"
#include "xnormidi/midi_function_types.h"

// None, Wait, Attack, Hold, Decay, Sustain, Release, rePeat
#define ENV_N 0
#define ENV_W 1
#define ENV_A 2
#define ENV_H 3
#define ENV_D 4
#define ENV_S 5
#define ENV_P 6
#define ENV_R 7
#define ENV_END 8

typedef struct {
   uint8_t stage;
   uint8_t count;
   uint8_t w, a, h, d, s, p, r;
   bool release_disabled;
} Envelope;

void env_trigger(Envelope*, uint8_t);
void env_release(Envelope*, uint8_t);
void env_tick(Envelope*, uint8_t idx);
void env_next_stage(Envelope* e);

void cc_callback(MidiDevice * device, uint8_t chan, uint8_t num, uint8_t val);
void note_on_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t velocity);
void note_off_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t aftertouch);

static Envelope er = {
   ENV_W, // stage
       0, // count
       0, // w
       0, // a
       2, // h
     200, // d
       0, // s
     102, // p
       0, // r
       false, // release disabled
};
static Envelope eb = {
   ENV_W, // stage
       0, // count
       0, // w
       0, // a
       2, // h
     200, // d
       0, // s
     101, // p
       0, // r
       false, // release disabled
};
static Envelope eg = {
   ENV_W, // stage
       0, // count
       0, // w
       0, // a
       2, // h
     200, // d
       0, // s
     100, // p
       0, // r
       false, // release disabled
};
static Envelope* es[6];
static uint8_t levels[] = {0,0,0,0,0,0}; // r0,b0,g0,r1,b1,g1
static uint8_t selected_color = 0;

void set_output(
   uint8_t level,
   volatile uint8_t* ocr,
   volatile uint8_t* port,
   uint8_t pin,
   volatile uint8_t* tccra,
   uint8_t pwm_bit
) {
   if (level == 255) {
      // extreme value, disable PWM
      *tccra &= ~_BV(pwm_bit);
      *port |= _BV(pin);
   } else if (level == 0) {
      // extreme value, disable PWM
      *tccra &= ~_BV(pwm_bit);
      *port &= ~_BV(pin);
   } else {
      // normal value, enable PWM
      *tccra |= _BV(pwm_bit);
      *ocr = level;
   }
}

ISR(TIMER2_OVF_vect) {
   // R
   env_tick(&er, 0);
   set_output(levels[0], &OCR2A, &PORTB, PB3, &TCCR2A, COM2A1);
   set_output(levels[1], &OCR0A, &PORTD, PD6, &TCCR0A, COM0A1);
   // B
   env_tick(&eb, 1);
   set_output(levels[1], (volatile uint8_t*)&OCR1B, &PORTB, PB2, &TCCR1A, COM1B1);
   set_output(levels[2], &OCR0B, &PORTD, PD5, &TCCR0A, COM0B1);
   // G
   env_tick(&eg, 2);
   set_output(levels[2], (volatile uint8_t*)&OCR1A, &PORTB, PB1, &TCCR1A, COM1A1);
   set_output(levels[0], &OCR2B, &PORTD, PD3, &TCCR2A, COM2B1);
}

// OC0A = PD6 LED0R
// OC0B = PD5 LED0B
// OC1A = PB1 LED0G
// OC1B = PB2 LED1R
// OC2A = PB3 LED1B
// OC2B = PD3 LED1G


int main(void) {
   // PWM from timer/counter0
   TCCR0B |= _BV(CS00); // clk (no prescale)
   TCCR0A |= _BV(COM0A1); // non-inverting PWM on OC0A
   TCCR0A |= _BV(COM0B1); // non-inverting PWM on OC0B
   TCCR0A |= _BV(WGM01) | _BV(WGM00); // fast PWM mode with 0xFF as TOP
   DDRD   |= _BV(PD6); // OC0A as output  // RED_1
   DDRD   |= _BV(PD5); // OC0B as output  // BLUE_1
   OCR0A   = 0;
   OCR0B   = 0;

   // PWM from timer/counter1
   TCCR1B |= _BV(CS00); // clk (no prescale)
   TCCR1A |= _BV(COM1A1); // non-inverting PWM on OC1A
   TCCR1A |= _BV(COM1B1); // non-inverting PWM on OC1B
   TCCR1A |= _BV(WGM10); // fast PWM mode with 0xFF as TOP (8-bit on 16-bit counter)
   TCCR1B |= _BV(WGM12); // fast PWM mode with 0xFF as TOP (8-bit on 16-bit counter)
   DDRB   |= _BV(PB1); // OC1A as output  // GREEN_0
   DDRB   |= _BV(PB2); // OC1B as output  // BLUE_0
   OCR1A   = 0;
   OCR1B   = 0;

   // PWM from timer/counter2
   TCCR2B |= _BV(CS22) | _BV(CS21);// | _BV(CS20);
   // TCCR2B |= _BV(CS20); // clk (no prescale)
   TCCR2A |= _BV(COM0A1); // non-inverting PWM on OC2A
   TCCR2A |= _BV(COM0B1); // non-inverting PWM on OC2B
   TCCR2A |= _BV(WGM21) | _BV(WGM20); // fast PWM mode with 0xFF as TOP
   DDRB   |= _BV(PB3); // OC2A as output  // RED_0
   DDRD   |= _BV(PD3); // OC2B as output  // GREEN_1
   OCR2A   = 0;
   OCR2B   = 0;

   // Envelope tick generation from timer/counter2

   TIMSK2 |= _BV(TOIE2); // interrupt for envelope tick


   // init midi, give the clock rate setting, indicate that we want only input
   MidiDevice* midi_device = serial_midi_init(false, true);
   midi_register_cc_callback(midi_device, cc_callback);
   midi_register_noteon_callback(midi_device, note_on_callback);
   midi_register_noteoff_callback(midi_device, note_off_callback);


   es[0] = &er;
   es[1] = &eb;
   es[2] = &eg;

   DDRD  |= _BV(PD2); // PD2 as output
   PORTD |= _BV(PD2); // PD2 ON

   sei(); // enable global interrupts

   for(;;) {
      midi_device_process(midi_device);
   }

   return 0; /* never reached */
}

void env_trigger(Envelope* e, uint8_t velocity) {
   e->stage = ENV_W;
   e->count = e->w;
   PORTD ^= _BV(PD2); // toggle PD2
}

void env_release(Envelope* e, uint8_t aftertouch) {
   e->stage = ENV_R;
   e->count = e->r;
}

void env_next_stage(Envelope* e) {
   (e->stage)++;
   switch (e->stage) {
   case ENV_END: // reset
      e->stage = ENV_N;
      e->count = 0;
      break;
   case ENV_A:
      e->count = e->a;
      break;
   case ENV_H:
      e->count = e->h;
      break;
   case ENV_D:
      e->count = e->d;
      break;
   case ENV_S:
      if (e->p) e->count = e->p;
      break;
   case ENV_P:
      if (e->p) {
         e->stage = ENV_W;
         e->count = e->w;
         PORTD ^= _BV(PD2); // toggle PD2
      } else {
         (e->stage)--; // go back to sustain
      }
      break;
   case ENV_R:
      (e->stage)--;
      break;
   }
}

void env_tick(Envelope* e, uint8_t idx) {
   bool sustain = false;

   if (e->count) (e->count)--;
   while (e->count == 0 && e->stage != ENV_N && !sustain) {
      switch(e->stage) {
      case ENV_A:
         levels[idx] = 255;
         break;

      case ENV_D:
         levels[idx] = e->s;
         break;

      case ENV_S:
         sustain = true;
         break;

      case ENV_R:
         levels[idx] = 0;
         break;
      }
      env_next_stage(e);
   }

   switch(e->stage) {
   case ENV_A:
      levels[idx] = (uint8_t)( (e->a - e->count)/(float)(e->a) * 255 );
      break;
   case ENV_D:
      levels[idx] = (uint8_t)( (e->count)/(float)(e->d) * (255-e->s) ) + e->s;
      break;
   case ENV_S:
      levels[idx] = e->s;
      break;
   case ENV_R:
      levels[idx] = (uint8_t)( (e->count)/(float)(e->r) * e->s );
      break;
   }
}

void cc_callback(MidiDevice * device, uint8_t chan, uint8_t num, uint8_t val) {
   // use 22-31 for custom CC

   switch (num) {
   case 22: // led channel select
      if (val > 2) val = 2;
      if (val < 0) val = 0;
      selected_color = val;
      break;
   case 23: // W
      es[selected_color]->w = val; break;
   case 24: // A
      es[selected_color]->a = val; break;
   case 25: // H
      es[selected_color]->h = val; break;
   case 26: // D
      es[selected_color]->d = val; break;
   case 27: // S
      es[selected_color]->s = val*2; break;
   case 28: // R
      es[selected_color]->r = val; break;
   case 29: // P
      es[selected_color]->p = val; break;
   }
}

void note_on_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t velocity) {
   env_trigger(&er, velocity);
   env_trigger(&eb, velocity);
   env_trigger(&eg, velocity);
}
void note_off_callback(MidiDevice * device, uint8_t chan, uint8_t note, uint8_t aftertouch) {
   if (!er.release_disabled) env_release(&er, aftertouch);
   if (!eb.release_disabled) env_release(&eb, aftertouch);
   if (!eg.release_disabled) env_release(&eg, aftertouch);
}
