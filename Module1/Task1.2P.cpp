// C++ code
//
volatile uint8_t ledState = LOW; // Stores the current state of the LED


void setup()
{
  DDRD &= 0b00000000; // Clear bit 2 (pin 2) to set it as an input
  PORTD |= (0b0000100); // Enable internal pull-up resistor on pin 2
  
  DDRB |= 0b00100000 ; // Set Digital Pin 13 as output
  Serial.begin(9600);
  // Configure INT0 to trigger on any change
  EICRA |= 0b00000001; // Set bit 0 in EICRA to enable interrupt on any change
  EIMSK |= 0b00000001; // Enable INT0 in EIMSK (
    
  // Enable global interrupts
  sei();
}

ISR(INT0_vect)
{
    // Toggle the LED state
    ledState = !ledState;
    
    if (ledState) {
        PORTB |= 0b00100000;  // Set bit 5 in PORTB to turn on the LED (equivalent to PORTB |= (1 << PORTB5))
    } else {
        PORTB &= 0b11011111; // Clear bit 5 in PORTB to turn off the LED (equivalent to PORTB &= ~(1 << PORTB5))
    }
}

void loop()
{
  // empty
}