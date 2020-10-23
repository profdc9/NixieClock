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

const byte digit_code[] =   { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20 };
const byte number1_code[] = { 0x00, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00 };
const byte number2_code[] = { 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x08, 0x00 };

const byte ticks1_code[] = { 0x00, 0x00, 0x02, 0x02 };
const byte ticks2_code[] = { 0x00, 0x08, 0x00, 0x08 };

volatile byte digits[NUM_DIGITS];
volatile byte ticks[NUM_DIGITS];
byte displayrefreshcycles;
byte current_digit;
byte hour12;
byte tempcf;
DS3231 Clock;
#ifdef DHT_SENSOR
DHT DHTSensor(4, DHT22, 1);
#endif

#define NUM_BUTTONS 3

const byte buttonPin[NUM_BUTTONS] = {BUTTON1, BUTTON2, BUTTON3};
static byte constStateButton[NUM_BUTTONS];
volatile byte curStateButton[NUM_BUTTONS];
volatile bool pushedButton[NUM_BUTTONS];

void write_led_state(byte d1, byte d2, byte d3)
{
  analogWrite(LED1,d1);
  analogWrite(LED2,d2);
  analogWrite(LED3,d3);
}

bool isButtonDown(byte b)
{
  return !curStateButton[b];
}

void waitButtonUp(byte b)
{
  while (curStateButton[b] == LOW);
  pushedButton[b] = false;
}

bool getPushedButton(byte b)
{
  if (pushedButton[b])
  {
    pushedButton[b] = false;
    return true;
  }
  return false;
}

void debounce_buttons(void)
{
  for (byte b = 0; b < NUM_BUTTONS; b++)
  {
    byte state = digitalRead(buttonPin[b]);
    if (curStateButton[b] != state)
    {
      if ((++constStateButton[b]) >= 50)
      {
        curStateButton[b] = state;
        if (state == LOW) pushedButton[b] = true;
        constStateButton[b] = 0;
      }
    } else constStateButton[b] = 0;
  }
}

ISR(TIMER1_COMPA_vect)
{
  SPI.beginTransaction(spiA);
  if (displayrefreshcycles > 0)
  {
    displayrefreshcycles--;
    SPI.transfer(digit_code[current_digit]);
    SPI.transfer(number2_code[digits[current_digit]] | ticks2_code[ticks[current_digit]] );
    SPI.transfer(number1_code[digits[current_digit]] | ticks1_code[ticks[current_digit]] );
  }
  else
  {
    displayrefreshcycles = 4;
    SPI.transfer(0);
    SPI.transfer(0);
    SPI.transfer(0);
    if ((++current_digit) >= NUM_DIGITS)
      current_digit = 0;
  }
  SPI.endTransaction();
  digitalWrite(SERIAL_LATCH, LOW);
  debounce_buttons();
  digitalWrite(SERIAL_LATCH, HIGH);
}

void setup() {
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

  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 16;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= ((1 << CS11) | (1 << CS10));    // 256 prescaler
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts

  Clock.setClockMode(false);
  hour12 = (EEPROM.read(EEPROM_1224HR) == 1) ? 1 : 0;
  tempcf = (EEPROM.read(EEPROM_TEMPCF) == 1) ? 1 : 0;
#ifdef DHT_SENSOR
  DHTSensor.begin();
#endif
}

void display_1_number(byte ind, byte b)
{
  digits[ind] = b / 10;
  digits[ind + 1] = b % 10;
}

void display_three_numbers(byte n1, byte n2, byte n3)
{
  display_1_number(0, n1);
  display_1_number(2, n2);
  display_1_number(4, n3);
}

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

void display_current_date(void)
{
  bool Century;
  display_three_numbers(Clock.getYear(), Clock.getMonth(Century), Clock.getDate());
}

void set_digs(byte *v, byte val)
{
  for (byte i = 0; i < NUM_DIGITS; i++) v[i] = val;
}

void clear_display(void)
{
  set_digs(digits, 12);
}

void clear_ticks(void)
{
  set_digs(ticks, 0);
}

void all_ticks(void)
{
  set_digs(ticks, 3);
}

unsigned short getNewNumber(unsigned short n, byte dig, byte ndig, unsigned short minval, unsigned short maxval)
{
  clear_ticks();
  for (;;)
  {
    unsigned short v = n;
    for (byte i = ndig; i > 0;)
    {
      i--;
      digits[dig + i] = v % 10;
      ticks[dig + i] = 3;
      v = v / 10;
    }
    if (getPushedButton(2))
    {
      if (n > minval) n--;
      else n = maxval;
    } else if (getPushedButton(1))
    {
      if (n < maxval) n++;
      else n = minval;
    } else if (getPushedButton(0))
      break;
  }
  return n;
}

void set_time_and_date(void)
{
  bool h12, PM, Century;
  clear_display();
  delay(500);
  hour12 = getNewNumber(hour12, 0, 1, 0, 1);
  EEPROM.write(EEPROM_1224HR, hour12);
#ifdef DHT_SENSOR
  clear_display();
  tempcf = getNewNumber(tempcf, 1, 2, 0, 1);
  EEPROM.write(EEPROM_TEMPCF, tempcf);
#endif
  display_three_numbers(Clock.getYear(), Clock.getMonth(Century), Clock.getDate());
  byte newYear = getNewNumber(Clock.getYear(), 0, 2, 0, 99);
  byte newMonth = getNewNumber(Clock.getMonth(Century), 2, 2, 1, 12);
  byte newDate = getNewNumber(Clock.getDate(), 4, 2, 1, 31);
  display_current_time();
  byte newHour = Clock.getHour(h12, PM);
  newHour = getNewNumber(newHour, 0, 2, 0, 23);
  byte newMinute = getNewNumber(Clock.getMinute(), 2, 2, 0, 59);
  byte newSecond = getNewNumber(Clock.getSecond(), 4, 2, 0, 59);
  Clock.setYear(newYear);
  Clock.setMonth(newMonth);
  Clock.setDate(newDate);
  Clock.setHour(newHour);
  Clock.setMinute(newMinute);
  Clock.setSecond(newSecond);
  all_ticks();
  delay(1000);
}

void cathode_condition()
{
  clear_ticks();
  for (byte i = 0; i < 12; i++)
  {
    set_digs(digits, i);
    for (byte j = 0; j < 240; j++)
    {
      delay(250);
      if (getPushedButton(0)) break;
      if (getPushedButton(2)) return;
    }
  }
}

#ifdef DHT_SENSOR
void display_dht(void)
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = DHTSensor.readHumidity();
  float f = DHTSensor.readTemperature(tempcf ? true : false);

  digits[0] = ((byte)f)/10;
  digits[1] = ((byte)f)%10;
  digits[2] = ((byte)(f*10.0f))%10;
  ticks[1] = 1;
  digits[3] = 12;
  digits[4] = ((byte)h)/10;
  digits[5] = ((byte)h)%10;
}
#endif

byte mainlooptick;
byte programtick;
byte cathodetick;

#ifdef DHT_SENSOR
#define TICK_CENTRAL_LOOP 297
#else
#define TICK_CENTRAL_LOOP 247
#endif

void loop() {
 delay(100);
  if ((++mainlooptick) > TICK_CENTRAL_LOOP) mainlooptick = 0;
  if (mainlooptick < 150)
  {
    clear_ticks();
    write_led_state(0,0,255);
    display_current_time();
  } else
#ifdef DHT_SENSOR
  if (mainlooptick < 237)
  {
    clear_ticks();
    write_led_state(0,255,0);
    display_dht();
  } else
#endif
  {
    clear_ticks();
    write_led_state(255,0,0);
    display_current_date();
    ticks[0] = ticks[2] = ticks[4] = 2;
    ticks[1] = ticks[3] = ticks[5] = 1;
  }
  if (isButtonDown(2))
  {
    write_led_state(0,0,0);
    if ((++programtick) >= 20)
    {
      programtick = 0;
      waitButtonUp(2);
      set_time_and_date();
    }
  } else programtick = 0;
  if (isButtonDown(1))
  {
    write_led_state(0,0,0);
    if ((++cathodetick) >= 20)
    {
      cathodetick = 0;
      waitButtonUp(1);
      cathode_condition();
    }
  } else cathodetick = 0;
}
