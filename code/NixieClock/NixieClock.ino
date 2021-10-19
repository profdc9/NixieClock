/*  NixieClock.ino */

/*
 * Copyright (c) 2020 Daniel Marks

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
 */

 #define DHT_SENSOR

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <DS3231.h>

#ifdef DHT_SENSOR
#include <DHT.h>
#endif

#define SERIAL_LATCH 10
#define SERIAL_DATA 11
#define SERIAL_DATAOUT 12
#define SERIAL_CLK 13
#define BUTTON1 14
#define BUTTON2 15
#define BUTTON3 16

#define ARDUINORX 1
#define ARDUINOTX 0

#define I2C_SDA 4
#define I2C_SCL 5

#define LED1 3
#define LED2 5
#define LED3 6

#define EEPROM_1224HR 0
#define EEPROM_TEMPCF 1

SPISettings spiA(4000000, MSBFIRST, SPI_MODE0);

#define NUM_DIGITS 6

//
// Display scanner control
//
// The display works by sweeping across the individual tubes, visiting each one
// briefly in round-robin fashion. During each visit, a tube's anode is powered
// while the cathode corresponding to the appropriate display elements (numeral
// and comma) are grounded. The microcontroller does this by shifting a bit
// string out to three cascaded shift registers whose outputs drive the the
// anodes and cathodes through high-voltage transistors. This is done in a
// four-phase refresh cycle driven asynchronously in the timer interrupt.
// 
// Bit string (Ax - anode X, Cx -- Cathode X, N -- not used)
//
// |        field 1        |        field 2        |        field 3        |
// |0 |1 |2 |3 |4 |5 |6 |7 |8 |9 |10|11|12|13|14|15|16|17|18|19|20|21|22|23|
// |      digit_code       |   number2/tick2_code  |   number1/tick1_code  |
// |A1|A2|A3|A4|A5|A6|N |N |C8|C9|C0|CB|N |N |N |N |CA|C7|C1|C2|C3|C4|C5|C6|
//

// Map between tube number (as index) and anode to activate in bit string field 1
const byte digit_code[] =   { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20 };

// Map between digit (as index) and cathode to activate in bit string field 1
const byte number1_code[] = { 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00 };

// Map between digit (as index) and cathode to activate in bit string field 2
const byte number2_code[] = { 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x08, 0x00 };

// Map between control value (as index) and comma cathode in bit string field 1
const byte ticks1_code[] = { 0x00, 0x00, 0x02, 0x02 };

// Map between control value (as index) and comma cathode in bit string field 2
const byte ticks2_code[] = { 0x00, 0x08, 0x00, 0x08 };

// NOTE: Comma control codes:
//   0 - No commas lit
//   1 - TODO: (left?) comma lit
//   2 - TODO: (right?) comma lit
//   3 - Both commas lit

//
// Display states and configuration
//

volatile byte digits[NUM_DIGITS]; // Current values of each display digit
volatile byte ticks[NUM_DIGITS];  // Current comma states for each display digit
byte displayrefreshcycles;        // Current refresh phase in display refresh cycle
byte current_digit;               // Which display digit is being refreshed
byte hour12;                      // 0 => 24 hour display, !0 => 12 hour display
byte tempcf;                      // 0 => Celcius, !0 => Fahrenheit

//
// Devices
//

DS3231 Clock; // Real-time clock interface

#ifdef DHT_SENSOR
DHT DHTSensor(4, DHT22, 1); // Humidity/temperature sensor interface
#endif

//
// Push button support
//

#define NUM_BUTTONS 3

// Button GPIO assignments
const byte buttonPin[NUM_BUTTONS] = {BUTTON1, BUTTON2, BUTTON3};

// How many read cycles each button remained in current state; for debouncing
static byte constStateButton[NUM_BUTTONS];

// Current debounced state of each button
volatile byte curStateButton[NUM_BUTTONS];

// True for each button when debounced state shows button is depressed. This
// acts like a latch so that button push is detected even after user stops
// pushing
volatile bool pushedButton[NUM_BUTTONS];

// Return true iff the button is depressed (LOW value)
bool isButtonDown(byte b)
{
  return !curStateButton[b];
}

// Wait until a button is no longer depressed by the user
void waitButtonUp(byte b)
{
  // Buttons are active-low
  while (curStateButton[b] == LOW);
  pushedButton[b] = false;
}

// Return true when a button has been pushed
bool getPushedButton(byte b)
{
  if (pushedButton[b])
  {
    // Reset the latch remembering the button was pushed
    pushedButton[b] = false;
    return true;
  }

  return false;
}

// Sample current button values and update debounced states
void debounce_buttons(void)
{
  for (byte b = 0; b < NUM_BUTTONS; b++)
  {
    // Sample present button value
    byte state = digitalRead(buttonPin[b]);

    // If state value differs from the current debounced state...
    if (curStateButton[b] != state)
    {
      // a new debounced state is reached after 50 consistent read cycles
      if ((++constStateButton[b]) >= 50)
      {
        curStateButton[b] = state;

        // If the new state is LOW, the button is depressed (active-low)
        if (state == LOW) pushedButton[b] = true;

        constStateButton[b] = 0;
      }
    }

    // If same as current debounced state, restart debounce counter
    else
      constStateButton[b] = 0;
  }
}

// Update nixie tube LED illumination states
void write_led_state(byte d1, byte d2, byte d3)
{
  analogWrite(LED1,d1);
  analogWrite(LED2,d2);
  analogWrite(LED3,d3);
}

// Interrupt on timer: Update display and button press states
ISR(TIMER1_COMPA_vect)
{
  // Prepare to write to display- shifting digit lead codes to scan display shift registers
  SPI.beginTransaction(spiA);

  // 4 display refresh cycles per tube visit: 1-3: light the tube cathodes; 0: "rest" period
  if (displayrefreshcycles > 0)
  {
    displayrefreshcycles--;

    // Shift out activation bit string to tube control shift registers
    //
    // Bit string field 1
    SPI.transfer(digit_code[current_digit]);
    
    // Bit string field 2
    SPI.transfer(number2_code[digits[current_digit]] | ticks2_code[ticks[current_digit]]);

    // Bit string field 3
    SPI.transfer(number1_code[digits[current_digit]] | ticks1_code[ticks[current_digit]]);
  }
  else
  {
    displayrefreshcycles = 4;

    // Shut off the tube
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);

    // Visit the next tube next cycle
    if ((++current_digit) >= NUM_DIGITS)
      current_digit = 0;
  }

  SPI.endTransaction();

  digitalWrite(SERIAL_LATCH, LOW);  // Low transition to set up for committing to display

  debounce_buttons();               // Update button press states; done here to give time
                                    // for serial latch signal to be effective

  digitalWrite(SERIAL_LATCH, HIGH); // Commit output to display
}

void setup() {
  // Initialize communication libraries, inputs, and outputs
  Serial.begin(9600);
  Wire.begin();
  current_digit = 0;
  pinMode(SERIAL_LATCH, OUTPUT);
  pinMode(LED1, OUTPUT);
  digitalWrite(LED1, LOW);
  pinMode(LED2, OUTPUT);
  digitalWrite(LED2, LOW);
  pinMode(LED3, OUTPUT);
  digitalWrite(LED3, LOW);
  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
  SPI.begin();

  // Set up MCU feature/mode registers
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 16;               // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

  Clock.setClockMode(false); // Set up real-time clock device

  // Fetch time and temperature display settings from EEPROM
  hour12 = (EEPROM.read(EEPROM_1224HR) == 1) ? 1 : 0;
  tempcf = (EEPROM.read(EEPROM_TEMPCF) == 1) ? 1 : 0;

#ifdef DHT_SENSOR
  DHTSensor.begin(); // Set up temp/humidity sensor device
#endif
}

// Set the current number shown by a particular pair of tubes
void display_1_number(byte ind, byte b)
{
  digits[ind] = b / 10;
  digits[ind + 1] = b % 10;
}

// Update the numbers shown by all three pairs of tubes
void display_three_numbers(byte n1, byte n2, byte n3)
{
  display_1_number(0, n1);
  display_1_number(2, n2);
  display_1_number(4, n3);
}

// Update the display numbers to be the current time
void display_current_time(void)
{
  bool h12, PM;
  byte nowHour = Clock.getHour(h12, PM);

  if (hour12)
  {
    if ((nowHour >= 12) && (nowHour <= 23)) ticks[0] = ticks[1] = 1;
    if (nowHour > 12) nowHour -= 12;
    if (nowHour == 0) nowHour = 12;
  }

  display_three_numbers(nowHour, Clock.getMinute(), Clock.getSecond());
}

// Update the display numbers to be the current datte
void display_current_date(void)
{
  bool Century;
  display_three_numbers(Clock.getYear(), Clock.getMonth(Century), Clock.getDate());
}

// Initialize each byte in a byte array to the same given value
void set_digs(byte *v, byte val)
{
  for (byte i = 0; i < NUM_DIGITS; i++) v[i] = val;
}

// Reset the display digits to invisible
void clear_display(void)
{
  // The number 12 maps to the activating none of the tube cathodes
  set_digs(digits, 12);
}

// Reset both commas for each digit to inivisible
void clear_ticks(void)
{
  set_digs(ticks, 0);
}

// Set both commas for each digit to visible
void all_ticks(void)
{
  set_digs(ticks, 3);
}

// Collect numeric user input ndig digits wide in a particular position on the display
// given an initial value and input range. NOTE: Timer interrupts are still enabled so
// that the display continues to update with digit/comma changes.
unsigned short getNewNumber(unsigned short n, byte dig, byte ndig, unsigned short minval, unsigned short maxval)
{
  clear_ticks();

  // Run a loop until the user is done (by pushing button 0)
  for (;;)
  {
    // Render the current value across the selected set of display tubes
    //
    unsigned short v = n;  // Copy current value for modification in loop below

    // Update each display digit with current value's digit in its decimal position 
    for (byte i = ndig; i > 0;)
    {
      i--;
      digits[dig + i] = v % 10;
      ticks[dig + i] = 3;
      v = v / 10;
    }

    // If button 2 is pushed, decrement the current value (wrapping around)
    if (getPushedButton(2))
    {
      if (n > minval) n--;
      else n = maxval;
    }

    // If button 1 is pushed, increment the current value (wrapping around)
    else if (getPushedButton(1))
    {
      if (n < maxval) n++;
      else n = minval;
    }

    // If button 0 is pushed, commit the current value
    else if (getPushedButton(0))
      break;
  }

  return n;
}

// Collect user input to set the clock and calendar
void set_time_and_date(void)
{
  bool h12, PM, Century;

  clear_display();
  delay(500);

  // First collect a number indicating 24-hour (0) or 12-hour (1) display
  hour12 = getNewNumber(hour12, 0, 1, 0, 1);
  EEPROM.write(EEPROM_1224HR, hour12); // Commit configuration to EEPROM

#ifdef DHT_SENSOR
  // Next collect a number indicating Celcius (0) or Fahrenheit (1) display
  clear_display();
  tempcf = getNewNumber(tempcf, 1, 2, 0, 1);
  EEPROM.write(EEPROM_TEMPCF, tempcf);
#endif

  // Show the current calendar (YYMMDD) and collect user changes
  display_three_numbers(Clock.getYear(), Clock.getMonth(Century), Clock.getDate());
  byte newYear = getNewNumber(Clock.getYear(), 0, 2, 0, 99);
  byte newMonth = getNewNumber(Clock.getMonth(Century), 2, 2, 1, 12);
  byte newDate = getNewNumber(Clock.getDate(), 4, 2, 1, 31);

  // Show the current time (HHMMSS) and collect user changtes
  display_current_time();
  byte newHour = Clock.getHour(h12, PM);
  newHour = getNewNumber(newHour, 0, 2, 0, 23);
  byte newMinute = getNewNumber(Clock.getMinute(), 2, 2, 0, 59);
  byte newSecond = getNewNumber(Clock.getSecond(), 4, 2, 0, 59);

  // Update the real-time clock device with new values
  Clock.setYear(newYear);
  Clock.setMonth(newMonth);
  Clock.setDate(newDate);
  Clock.setHour(newHour);
  Clock.setMinute(newMinute);
  Clock.setSecond(newSecond);

  // Affirm operation to user by showing all comma for a moment 
  all_ticks();
  delay(1000);
}

// Sweep through all the IN-14 nixie tube cathodes (display elements)
void cathode_condition()
{
  clear_ticks();

  // For each element (number and comma) individually
  for (byte i = 0; i < 12; i++)
  {
    // Set the element "hot"
    set_digs(digits, i);

    // Hold it that way for a while or until a signal from the user
    for (byte j = 0; j < 240; j++)
    {
      delay(250);
      if (getPushedButton(0)) break;  // button 0 -> go to next cathode
      if (getPushedButton(2)) return; // button 2 -> abort conditioning
    }
  }
}

#ifdef DHT_SENSOR
// Show the temperature and humidity
void display_dht(void)
{
  // Read the humidity/temperature values
  //
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = DHTSensor.readHumidity();
  float f = DHTSensor.readTemperature(tempcf ? true : false);

  // Update each digit directly to the appropriate decimal value
  digits[0] = ((byte)f)/10;
  digits[1] = ((byte)f)%10;
  digits[2] = ((byte)(f*10.0f))%10;
  ticks[1] = 1;
  digits[3] = 12;
  digits[4] = ((byte)h)/10;
  digits[5] = ((byte)h)%10;
}
#endif

byte mainlooptick; // Current "time tick" in main loop display rotation cycle
byte programtick;  // "Time tick" counter for activating time and date set
byte cathodetick;  // "Time tick" counter for activating cathode conditioning

// Define the length of the display rotation between calendar, DHT, and time
#ifdef DHT_SENSOR
#define TICK_CENTRAL_LOOP 297
#else
#define TICK_CENTRAL_LOOP 247
#endif

void loop() {
  delay(100); // Define the width of a "time tick"

  // Reset display rotation after it reaches the end
  if ((++mainlooptick) > TICK_CENTRAL_LOOP) mainlooptick = 0;

  // Show current time during first part of display rotation
  if (mainlooptick < 150)
  {
    clear_ticks();
    write_led_state(0,0,255);
    display_current_time();
  }

#ifdef DHT_SENSOR
  // Show temp and humidity during second part of display rotation
  else if (mainlooptick < 237)
  {
    clear_ticks();
    write_led_state(0,255,0);
    display_dht();
  }
#endif

  // Show time and date during third part of display rotation
  else
  {
    clear_ticks();
    write_led_state(255,0,0);
    display_current_date();
    ticks[0] = ticks[2] = ticks[4] = 2;
    ticks[1] = ticks[3] = ticks[5] = 1;
  }

  // If the user is holding down button 2
  if (isButtonDown(2))
  {
    write_led_state(0,0,0); // Affirm by turning off the LEDs

    // If they hold it down for 20 ticks, set time and date
    if ((++programtick) >= 20)
    {
      programtick = 0;
      waitButtonUp(2);
      set_time_and_date();
    }
  }
  else
    // If the user isn't holding it down or lets go, reset counter
    programtick = 0;

  // If the user is holding down button 1
  if (isButtonDown(1))
  {
    write_led_state(0,0,0); // Affirm by turning off the LEDs

    // If they hold it down for 20 ticks, run the cathode conditioner
    if ((++cathodetick) >= 20)
    {
      cathodetick = 0;
      waitButtonUp(1);
      cathode_condition();
    }
  }
  else
    // If the user isn't holding it down or lets go, reset counter
    cathodetick = 0;
}
