/*********************

Example code for the Adafruit RGB Character LCD Shield and Library

This code displays text on the shield, and also reads the buttons on the keypad.
When a button is pressed, the backlight changes color.

**********************/

// include the library code:
#include "Arduino.h"
#include <EEPROM.h>
#include "global.h"
#include "Servo.h"
#include "LiquidCrystal_I2C.h"
#include "RotaryEncoder.h"

struct Config {
  float field1;
  byte field2;
  char name[10];
};

#define COLUMS           20
#define ROWS             4

#define LCD_SPACE_SYMBOL 0x20  //space symbol from the LCD ROM, see p.9 of GDM2004D datasheet

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

#define PIN_IN1 3
#define PIN_IN2 2
#define PIN_Enter A3

// This interrupt routine will be called on any change of one of the input signals
void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

// These #defines make it easy to set the backlight color
#define RED 0x1
#define YELLOW 0x3
#define GREEN 0x2
#define TEAL 0x6
#define BLUE 0x4
#define VIOLET 0x5
#define WHITE 0x7

Servo myservo;  // create servo object to control a servo
int potpin = A0;  // analog pin used to connect the potentiometer
int val=1500;    // variable to read the value from the analog pin
bool increment = true;
int function = 0; // witch function is active: 0 = manuell (Tasten), 1 = Sweep, 2 = Poti input for Servo
int min_us = 800;
int max_us = 2200;
int incval = 20;
char title[17] = "Servotester";

void printServoValue(char* title, int value){
  char valstr[17];

  int pc = int((long)((value - 1500) * 100 / 500));;

  snprintf(valstr, sizeof(valstr), "%4dms   %+4d%%  ", value, pc);

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(title);
  lcd.setCursor(0, 1);
  lcd.print(valstr);
}

void setup() {
  // Debugging output
  Serial.begin(9600);
  // set up the LCD's number of columns and rows: 
  while (lcd.begin(COLUMS, ROWS) != 1) //colums - 20, rows - 4
  {
    Serial.println(F("PCF8574 is not connected or lcd pins declaration is wrong. Only pins numbers: 4,5,6,16,11,12,13,14 are legal."));
    delay(2000);   
  }

  lcd.setCursor(0, 0);
  lcd.print(F("PCF8574 is OK..."));    //(F()) saves string to flash & keeps dynamic memory free
  Serial.println(F("PCF8574 is OK..."));
  delay(200);
  lcd.clear();

  // setup the rotary encoder functionality

  // use FOUR3 mode when PIN_IN1, PIN_IN2 signals are always HIGH in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // use FOUR0 mode when PIN_IN1, PIN_IN2 signals are always LOW in latch position.
  // encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR0);

  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

  pinMode(PIN_Enter,INPUT_PULLUP);

  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.writeMicroseconds(val);  // set servo to mid-point
}

uint8_t i=0;
void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  // lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  // lcd.print(millis()/1000);
  
  static int pos = 0;
  static boolean enter = 0;

  encoder->tick(); // just call tick() to check the state.

  boolean newenter = digitalRead(PIN_Enter);
  if(enter != newenter){
    Serial.println("enter: ");
    Serial.println(newenter);
    if(enter == 1){
      function++;
      if(function > 2) function = 0;
    }
    printServoValue(title, val);
    enter = newenter;
  }

  int newPos = encoder->getPosition();
  if (pos != newPos) {
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder->getDirection()));
    pos = newPos;
    printServoValue(title, val);
  } // if

  //Position
  if(function == 0){
    strncpy(title,"Position",sizeof(title));
    val += (int)(encoder->getDirection())*5;
    if(val > max_us) val=max_us;
    if(val < min_us) val=min_us;
    myservo.writeMicroseconds(val);                  // sets the servo position according to the scaled value
  }

  if(function == 1){
    strncpy(title,"Sweep",sizeof(title));
    if(val <= min_us) increment=true;
    if(val >= max_us) increment=false;
    
    incval += (int)(encoder->getDirection())*5;
    if(incval > 100) incval=100;
    if(incval < 5) incval=5;

    if(increment) val += incval;
    else val -= incval;

    printServoValue(title, val);
    myservo.writeMicroseconds(val);  // set servo to mid-point
  }
  
  if(function == 2){
    strncpy(title,"Poti",sizeof(title));
    val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
    val = map(val, 0, 1023, min_us, max_us);     // scale it to use it with the servo (value between 0 and 180)
    printServoValue(title, val);
    myservo.writeMicroseconds(val);                  // sets the servo position according to the scaled value
  }

}