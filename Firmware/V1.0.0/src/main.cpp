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

#define COLUMS           16
#define ROWS             2

#define LCD_SPACE_SYMBOL 0x20  //space symbol from the LCD ROM, see p.9 of GDM2004D datasheet

LiquidCrystal_I2C lcd(PCF8574_ADDR_A21_A11_A01, 4, 5, 6, 16, 11, 12, 13, 14, POSITIVE);

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

#define PIN_IN1 2
#define PIN_IN2 3
#define PIN_Enter A3

static long pos = 0;
static long lastpos = 0;

Servo servo0;  // create servo object to control a servo
Servo servo1;  // create servo object to control a servo
Servo servo2;  // create servo object to control a servo
Servo servo3;  // create servo object to control a servo
Servo servo4;  // create servo object to control a servo
int potpin = A0;  // analog pin used to connect the potentiometer
volatile int val=1500;     // variable to read the value from the analog pin
static int oldval=1499;  // force write- and printout
bool increment = true;
int min_us = 800;
int max_us = 2200;
volatile long incval = 20;
volatile char title[17] = "Servotester";

volatile int timer2Counter;  //Timer-Variable
volatile long timer2Cnt;  //Timer-Variable

enum efunction: int{
  fNeutral = 0,
  fPosition = 1,
  fMinPos = 2,
  fMaxPos = 3,
  fSweep = 4,
  fEnd = 4
};

efunction func = fNeutral;
efunction oldfunc = fEnd;  //force function init

void printServoValue(){
  char valstr[17];

  int pc = int((long)((val - 1500) * 100 / 500));;

  lcd.clear();
  lcd.setCursor(0,0);
  snprintf(valstr, sizeof(valstr), "%s", title);
  lcd.print(valstr);
  lcd.setCursor(0, 1);
  snprintf(valstr, sizeof(valstr), "%4dms   %+4d%%  ", val, pc);
  lcd.print(valstr);
  Serial.println(valstr);
}

void WriteServoPosition(){
  servo0.writeMicroseconds(val);  // set servo to mid-point
  servo1.writeMicroseconds(val);  // set servo to mid-point
  servo2.writeMicroseconds(val);  // set servo to mid-point
  servo3.writeMicroseconds(val);  // set servo to mid-point
  servo4.writeMicroseconds(val);  // set servo to mid-point
}

// This interrupt routine will be called on any change of one of the input signals
void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}

ISR(TIMER2_OVF_vect) {
  TCNT2 = timer2Counter;
  timer2Cnt--;
  if(timer2Cnt <= 0)
  {
    timer2Cnt = incval;
    if(func == fSweep){

      if(increment) val++;
      else val--;
    
      if(val <= min_us){
        increment=true;
        val = min_us;
      }
      
      if(val >= max_us){
        increment=false;
        val = max_us;
      }
    }

    WriteServoPosition();
  }
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
  encoder = new RotaryEncoder(PIN_IN1, PIN_IN2, RotaryEncoder::LatchMode::FOUR3);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(PIN_IN1), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_IN2), checkPosition, CHANGE);

  pinMode(PIN_Enter, INPUT_PULLUP);

  servo0.attach(9);  // attaches the servo on pin 9 to the servo object
  servo1.attach(8);  // attaches the servo on pin 9 to the servo object
  servo2.attach(7);  // attaches the servo on pin 9 to the servo object
  servo3.attach(6);  // attaches the servo on pin 9 to the servo object
  servo4.attach(5);  // attaches the servo on pin 9 to the servo object
  
  printServoValue();
  WriteServoPosition();

  // Setup Timer
  cli();//stop all interrupts
  TCCR2A = 0;  //delete all bits
  TCCR2B = 0;  //delete all bits
  timer2Counter = 200;
  timer2Cnt = incval;
  TCNT2 = timer2Counter;//preload timer
  
  TCCR2B |= (1 << CS10);
  TCCR2B |= (1 << CS12);//Sets Prescaler to 1024
  TIMSK2 |= (1 << TOIE2);// enables timer overflow interrups
  
  sei();//allow interrupts

  delay(1000);
}

uint8_t i=0;
void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  // lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  // lcd.print(millis()/1000);
  
  static boolean enter = 0;

  boolean newenter = digitalRead(PIN_Enter);
  if(enter != newenter){
    Serial.println("enter: ");
    Serial.println(newenter);
    if(enter == 1){
      func = (efunction)((int)func + 1);
      if(func > fEnd) func = fNeutral;
    }
    printServoValue();
    enter = newenter;
  }


  //Accelleration
  // Define some constants.

  // the maximum acceleration is 10 times.
  constexpr float m = 50;

  // at 200ms or slower, there should be no acceleration. (factor 1)
  constexpr float longCutoff = 200;

  // at 5 ms, we want to have maximum acceleration (factor m)
  constexpr float shortCutoff = 5;

  // To derive the calc. constants, compute as follows:
  // On an x(ms) - y(factor) plane resolve a linear formular factor(ms) = a * ms + b;
  // where  f(4)=10 and f(200)=1

  constexpr float a = (m - 1) / (shortCutoff - longCutoff);
  constexpr float b = 1 - longCutoff * a;
  
  //Read Encoder 
  lastpos = pos;
  encoder->tick(); // just call tick() to check the state.

  long newPos = encoder->getPosition();
  if (pos != newPos) {
    unsigned long ms = encoder->getMillisBetweenRotations();
    if(ms > longCutoff){
      Serial.print("Slow: ");
      Serial.println(ms);
    }
    else{
    // accelerate when there was a previous rotation in the same direction.

      // do some acceleration using factors a and b

      // limit to maximum acceleration
      if (ms < shortCutoff) {
        ms = shortCutoff;
      }

      float ticksActual_float = a * ms + b;
      Serial.print("  f= ");
      Serial.println(ticksActual_float);

      long deltaTicks = (long)ticksActual_float * (newPos - lastpos);
      Serial.print("  d= ");
      Serial.println(deltaTicks);

      newPos = newPos + deltaTicks;
      encoder->setPosition(newPos);

      Serial.print("Fast: ");
      Serial.println(ms);
    }
    Serial.print("pos:");
    Serial.print(newPos);
    Serial.print(" dir:");
    Serial.println((int)(encoder->getDirection()));
    pos = newPos;
    printServoValue();
  }

  if(func == fNeutral){
    if(func != oldfunc)
    {
      //Init function
      strncpy((char*)title,"Neutral",sizeof(title));
      val = 1500;
      printServoValue();

      oldfunc = func;
    }
  }

  //Position
  if(func == fPosition){
    if(func != oldfunc)
    {
      //Init function
      strncpy((char*)title,"Position",sizeof(title));
      
      val = 1500;
      pos = val;
      encoder->setPosition(pos);

      oldfunc = func;
    }
    
    val = pos;

    if(val > max_us){
      val=max_us;
      pos = val;
      encoder->setPosition(pos);      
    }

    if(val < min_us){
      val=min_us;
      pos = val;
      encoder->setPosition(pos);
    }
  }

  if(func == fMinPos){
    if(func != oldfunc)
    {
      //Init function
      strncpy((char*)title,"Min position",sizeof(title));
      val = min_us;
      pos = val;
      encoder->setPosition(pos);
      oldfunc = func;
    }

    min_us = encoder->getPosition();
    val = min_us;
  }

  if(func == fMaxPos){
    if(func != oldfunc)
    {
      //Init function
      strncpy((char*)title,"Max position",sizeof(title));
      val = max_us;
      pos = val;
      encoder->setPosition(pos);
      oldfunc = func;
    }

    max_us = encoder->getPosition();
    val = max_us;

  }

  if(func == fSweep){
    if(func != oldfunc)
    {
      //Init function
      incval = 20l;
      snprintf((char*)title,sizeof(title),"Sweep Speed: %2ld",incval);
      encoder->setPosition((long)incval*10);
      oldfunc = func;
    }
    
    //Get Speed
    long oldincval = incval;
    long tmp = encoder->getPosition();
    
    if(tmp > 600){
      Serial.print(F(">60: "));
      Serial.println(tmp);
      tmp=600l; 
      encoder->setPosition(tmp);
      pos = tmp;
    }
    
    if(tmp < 0){
      Serial.print(F("<0: "));
      Serial.println(tmp);
      tmp=0l; 
      encoder->setPosition(tmp);
      pos = tmp;
    }

    incval = tmp/10;

    if(incval != oldincval){
      snprintf((char*)title,sizeof(title),"Sweep Speed: %2ld",incval);
    }

  }
  
  if(val != oldval){
    printServoValue();
    delay(100);
    oldval = val;
  }

}