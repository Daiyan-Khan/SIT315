#include <avr/io.h>
#include <avr/interrupt.h>

volatile uint8_t ledState1 = LOW; // Stores the current state of the first LED
volatile uint8_t ledState2 = LOW; // Stores the current state of the second LED

const byte METER_PIN = A4; // Analog Pin A4 for the potentiometer

double state; // Current frequency set for the timer

// Define pin for Pin Change Interrupt
#define PCINT_PIN 7
#define PCINT_MODE CHANGE
#define PCINT_FUNCTION blinkLed
#define LED_BUILTIN 11

void setup() {
  // Initialize pins
  PORTD |= 0b10000000; //PCINT 7 WITH INPUT PULLUP

  // Set PB5 (Digital Pin 13), PB4 (Digital Pin 12) AND PB3(Digital Pin 11) as outputs
  DDRB |= 0b00111000; // Set bits 4, 5 and 6 in DDRB to set PB4 and PB5 as outputs

  // Configure INT0 and INT1 to trigger on any logical change
  EICRA = (EICRA & 0b11110100) | 0b00000101; // Clear bits 1 and 3, set bits 0 and 2

  // Enable INT0 and INT1
  EIMSK |= 0b00000011; // Set bits 0 and 1 in EIMSK (INT0 and INT1)

  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Attach the Pin Change Interrupt
  attachPinChangeInterrupt();

  // Start Timer with default frequency
  state = 2.0; // Default frequency
  startTimer(state);

  // Enable global interrupts
  sei();
}

void loop() {
  // Read potentiometer value and adjust timer frequency
  setTimerFromPotentiometer();
  delay(100); // Small delay to avoid constant polling
}

// Function to set the timer based on the frequency from the potentiometer
void setTimerFromPotentiometer() {
  // Read the analog value from METER_PIN
  double meterReading = analogRead(METER_PIN);

  // Map the analog value to a frequency range (1 to 4 Hz)
  double mappedFreq = map(meterReading, 0, 1023, 1, 4);

  // Update the timer only if the frequency has changed
  if (state != mappedFreq) {
    startTimer(mappedFreq);
    state = mappedFreq;
    Serial.print("Frequency set to: ");
    Serial.println(state);
  }
}

// Function to configure Timer1 with the specified frequency
void startTimer(double freq) {
  noInterrupts(); // Disable interrupts while configuring the timer

  // Calculate the compare match value for Timer1
  uint16_t cmpr = 16000000 / (1024 * freq) - 1; // Assuming 16 MHz clock and prescaler of 1024

  // Reset Timer1 Control Registers
  TCCR1A = 0; // Set to default mode (no output mode)
  TCCR1B = 0; // Set to default mode (no prescaler and no waveform generation)

  // Reset Timer1 counter
  TCNT1 = 0;

  // Configure Timer1 for CTC (Clear Timer on Compare Match) mode
  TCCR1B |= (1 << WGM12);

  // Set prescaler to 1024
  TCCR1B |= (1 << CS10);
  TCCR1B |= (1 << CS12);

  // Set the compare match register value
  OCR1A = cmpr;

  // Enable Timer1 Compare Match A interrupt
  TIMSK1 = (1 << OCIE1A);

  interrupts(); // Re-enable interrupts
}

// Timer1 Compare Match A interrupt service routine
ISR(TIMER1_COMPA_vect) {
  // Toggle the state of the LED
  if (PINB & (1 << PINB5)) {
    PORTB &= ~(1 << PORTB5); // Turn LED off
  } else {
    PORTB |= (1 << PORTB5); // Turn LED on
  }
}

// Pin Change Interrupt Function
void blinkLed(void) {
  // Switch LED state
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
}

//================================================================================
// PCINT Definitions
//================================================================================

#define PCMSK *digitalPinToPCMSK(PCINT_PIN)
#define PCINT digitalPinToPCMSKbit(PCINT_PIN)
#define PCIE  digitalPinToPCICRbit(PCINT_PIN)
#define PCPIN *portInputRegister(digitalPinToPort(PCINT_PIN))

#if (PCIE == 0)
#define PCINT_vect PCINT0_vect
#elif (PCIE == 1)
#define PCINT_vect PCINT1_vect
#elif (PCIE == 2)
#define PCINT_vect PCINT2_vect
#else
#error This board doesn't support PCINT ?
#endif

volatile uint8_t oldPort = 0x00;

void attachPinChangeInterrupt(void) {
  // Update the old state to the actual state
  oldPort = PCPIN;

  // Pin change mask registers decide which pins are enabled as triggers
  PCMSK |= (1 << PCINT);

  // PCICR: Pin Change Interrupt Control Register - enables interrupt vectors
  PCICR |= (1 << PCIE);
}

void detachPinChangeInterrupt(void) {
  // Disable the mask
  PCMSK &= ~(1 << PCINT);

  // If that's the last one, disable the interrupt
  if (PCMSK == 0)
    PCICR &= ~(0x01 << PCIE);
}

ISR(PCINT_vect) {
  // Get the new and old pin states for port
  uint8_t newPort = PCPIN;

  // Compare with the old value to detect a rising or falling
  uint8_t change = newPort ^ oldPort;

  // Check which pins are triggered, compared with the settings
  uint8_t trigger = change & (1 << PCINT);

  // Save the new state for next comparison
  oldPort = newPort;

  // If our needed pin has changed, call the interrupt function
  if (trigger)
    PCINT_FUNCTION();
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
