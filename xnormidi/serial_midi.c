#include "serial_midi.h"
#include <avr/interrupt.h>
#include "stdlib.h"

#define MIDI_BAUD_RATE (31250)
#define MIDI_CLOCK_RATE ((F_CPU/(MIDI_BAUD_RATE*16UL))-1)

static MidiDevice midi_device;

void serial_midi_send(MidiDevice* device, uint8_t cnt, uint8_t inByte0, uint8_t inByte1, uint8_t inByte2){
   //we always send the first byte
	while ( !(UCSR0A & _BV(UDRE0)) ); // Wait for empty transmit buffer
	UDR0 = inByte0;
   //if cnt == 2 or 3 we send the send byte
   if(cnt > 1) {
      while ( !(UCSR0A & _BV(UDRE0)) ); // Wait for empty transmit buffer
      UDR0 = inByte1;
   }
   //if cnt == 3 we send the third byte
   if(cnt == 3) {
      while ( !(UCSR0A & _BV(UDRE0)) ); // Wait for empty transmit buffer
      UDR0 = inByte2;
   }
}

MidiDevice* serial_midi_device(void) {
   return &midi_device;
}

ISR(USART_RX_vect) {
   MidiDevice* device = &midi_device;
   uint8_t data = UDR0;
   midi_device_input(device, 1, &data);
}

void serial_get_midi(MidiDevice* device) {
   if ( UCSR0A & _BV(RXC0) ) {
      uint8_t data = UDR0;
      midi_device_input(device, 1, &data);
   }
}

MidiDevice* serial_midi_init(bool out, bool in){
   // set up the device
   midi_device_init(&midi_device);
   midi_device_set_send_func(&midi_device, serial_midi_send);
   //midi_device_set_pre_input_process_func(&midi_device, serial_get_midi);

	// Set baud rate
	UBRR0H = (uint8_t)(MIDI_CLOCK_RATE >> 8);
	UBRR0L = (uint8_t)(MIDI_CLOCK_RATE & 0xFF);
	// Enable transmitter
	if(out)
		UCSR0B |= _BV(TXEN0);
	if(in) {
		// Enable receiver
		// RX Complete Interrupt Enable  (user must provide routine)
		UCSR0B |= _BV(RXEN0) | _BV(RXCIE0);
      DDRD &= ~_BV(PD0);
	}
	// Set frame format: Async, 8data, 1 stop bit, 1 start bit, no parity
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

   return serial_midi_device();
}

