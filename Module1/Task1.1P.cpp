// C++ code
//
int sensorState = 0;

void setup()
{
  DDRD &= 0b00000000; // Clear bit 2 (pin 2) to set it as an input

  DDRB |= 0b00100000 ; // Set Digital Pin 13 as output
  Serial.begin(9600);
}

void loop()
{
  // read the state of the sensor/digital input
  sensorState = (PIND & 0b00000100) >> 2;
  // check if sensor pin is HIGH. if it is, set the
  // LED on.
  if (sensorState) {
    PORTB |= (1 << PORTB5);  // Set PB5 high to turn on the LED
    // For Serial output, additional UART setup and functions are needed
    // This example will not include Serial output
				} 
  else {
    PORTB &= ~(1 << PORTB5); // Set PB5 low to turn off the LED
		}
  delay(1); // Delay a little bit to improve simulation performance
}
