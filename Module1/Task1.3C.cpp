#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t ledState1 = LOW; // Stores the current state of the first LED
volatile uint8_t ledState2 = LOW; // Stores the current state of the second LED

void setup() {
  // Set PD2 (INT0) and PD3 (INT1) as input with internal pull-up resistors
  DDRD &= 0b00000000; // Clear bit 2 and 3 in DDRD to set PD2 and PD3 as input
  PORTD |= 0b0001100; // Set bit 2 and 3 in PORTD to enable internal pull-up resistors

  // Set PB5 (Digital Pin 13)and PB4(Digital Pin 12) as output for the first LED and second LED respectively
  DDRB |= 0b00110000;

  // Configure INT0 to trigger on any logical change
  EICRA |= 0b00000101; // Set bit 0 in EICRA (ISC00) and ISC10
  // Configure INT0 and INT1 to trigger on any logical change
  EICRA = (EICRA & 0b11110100) | 0b00000101; // Clear bits 1 and 3, set bits 0 and 2

  // Enable INT0 and INT1
  EIMSK |= 0b00000011; // Set bits 0 and 1 in EIMSK (INT0 and INT1)

  // Enable global interrupts
  sei();
}

// ISR for INT0
ISR(INT0_vect) {
  // Toggle the first LED state
  ledState1 = !ledState1;
  if (ledState1) {
  PORTB |= 0b00100000; // Turn first LED on (set bit 5 in PORTB)
} else {
  PORTB &= 0b11011111; // Turn first LED off (clear bit 5 in PORTB)
}

}

// ISR for INT1
ISR(INT1_vect) {
  // Toggle the second LED state
  ledState2 = !ledState2;
  if (ledState2) {
  PORTB |= 0b00010000; // Turn second LED on (set bit 4 in PORTB)
} else {
  PORTB &= 0b11101111; // Turn second LED off (clear bit 4 in PORTB)
}

}

void loop() {
  // Main loop does nothing, all work is done in ISRs
}

int main(void) {
  setup();
  while (1) {
    loop();
  }
}
