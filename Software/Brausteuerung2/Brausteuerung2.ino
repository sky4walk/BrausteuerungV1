// brausteuerung@AndreBetz.de
// hier gilt die Bierlizenz
///////////////////////////////////////////////
// includes
///////////////////////////////////////////////
#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <RCSwitch.h>
#include <PID_v1.h>
#include "DbgConsole.h"
#include "WaitTime.h"
#include "storage.h"
///////////////////////////////////////////////
// defines
///////////////////////////////////////////////
#define TEMPRESOLUTION      12
#define PIDOWINTERVAL       (1000 / (1 << (12 - TEMPRESOLUTION)))
#define PIDWINDOWSIZE       (PIDOWINTERVAL * 6)
#define PIDMINWINDOW        150 
#define MAXRAST             15
#define PINHEATER           13  //D13
#define PINTMPDS18B20       12  //D12
#define PINBUZZER           11  //D11
#define PINLCD              10
#define PINKEY              0
#define MAXBUTTONS          6
#define btnNONE             0
#define btnRIGHT            1
#define btnUP               2
#define btnDOWN             3
#define btnLEFT             4
#define btnSELECT           5
#define MINDIFF             0.5
///////////////////////////////////////////////////////////////////////////////
// data structure
///////////////////////////////////////////////////////////////////////////////
typedef struct
{
  unsigned long  time;
  int temp;
  bool active;
  bool wait;
  bool alarm;
} Braurast;
struct Rezept
{
  Braurast rasten[MAXRAST];
  float pidKp;
  float pidKi;
  float pidKd;
  unsigned long PidOWinterval;
  unsigned long PidWindowSize;
  unsigned long PidMinWindow;
  int SwitchProtocol;
  int SwitchPulseLength;
  int SwitchRepeat;
  int SwitchBits;
  unsigned long SwitchOn;
  unsigned long SwitchOff;
  int actRast;
  bool started;
  bool UseDefault;
} myRezept;
///////////////////////////////////////////////////////////////////////////////
// variablen
///////////////////////////////////////////////////////////////////////////////
double pidOutput                 = 0;
double isTmp                     = 25;
double sollTmp                   = 25;
bool   btnPressed[MAXBUTTONS]    = {false,false,false,false,false,false};
bool   heatState                 = false;
int    actMenuState              = 0;
int    subMenuState              = 0;
///////////////////////////////////////////////////////////////////////////////
// classes
///////////////////////////////////////////////////////////////////////////////
LiquidCrystal     lcd(8, 9, 4, 5, 6, 7);
OneWire           oneWire(PINTMPDS18B20);
DallasTemperature sensors(&oneWire);
DeviceAddress     tempDeviceAddress;
RCSwitch          mySwitch = RCSwitch();
PID myPID(&isTmp, &pidOutput, &sollTmp, myRezept.pidKp,myRezept.pidKi,myRezept.pidKd, DIRECT);
WaitTime          timerTempMeasure;
WaitTime          timerPidCompute;
WaitTime          timerSendHeatState;
StorageEEProm     store;
///////////////////////////////////////////////////////////////////////////////
// setDefaultValues
///////////////////////////////////////////////////////////////////////////////
void LoadValues()
{
  CONSOLELN("LoadValues");
  myRezept.UseDefault = false;
  store.load(0, sizeof(Rezept), (char*)&myRezept);
  //if (  false == myRezept.UseDefault )
  {
    // only go here after first installation
    memset((char*)&myRezept, 0, sizeof(Rezept));
    myRezept.pidKp             = 0.0f;
    myRezept.pidKi             = 0.0f;
    myRezept.pidKd             = 0.0f;
    myRezept.PidOWinterval     = PIDOWINTERVAL;
    myRezept.PidWindowSize     = PIDWINDOWSIZE;
    myRezept.PidMinWindow      = PIDMINWINDOW;
    myRezept.SwitchOn          = 1631343;
    myRezept.SwitchOff         = 1631342;
    myRezept.SwitchProtocol    = 1;
    myRezept.SwitchPulseLength = 315;
    myRezept.SwitchBits        = 24;
    myRezept.SwitchRepeat      = 15;
    myRezept.actRast           = 0;
    myRezept.started           = false;
    myRezept.UseDefault        = true;
    store.save(0, sizeof(Rezept), (char*)&myRezept);
  }
}
///////////////////////////////////////////////////////////////////////////////
// readKeyPad
///////////////////////////////////////////////////////////////////////////////
void setButton(int btnNr) 
{
  if ( MAXBUTTONS > btnNr  && 0 != btnNr ) 
  {
    if ( false == btnPressed[btnNr] )
    {
      btnPressed[btnNr] = true;
      CONSOLE("b");
      CONSOLELN(btnNr);
    }
  }
}
bool isButtonPressed(int btnNr)
{
  bool pressed = false;
  if ( MAXBUTTONS > btnNr )
  {
    pressed = btnPressed[btnNr];
    btnPressed[btnNr] = false;
  }
  return pressed;
}
int Keypad()
{
  int x;
  int y;
  int val = 0;
  int val_old = 0;

  do
  {
    val_old = val;
    x = analogRead(PINKEY);
    if      (x <  60) val = btnRIGHT;
    else if (x < 200) val = btnUP;
    else if (x < 400) val = btnDOWN;
    else if (x < 600) val = btnLEFT;
    else if (x < 800) val = btnSELECT;
    else              val = 0;
    delay(10);
  }
  while ( val != val_old );

  setButton(val);
  if ( 0 < val ) delay(200);
  return val;
}
///////////////////////////////////////////////////////////////////////////////
// print Logo
///////////////////////////////////////////////////////////////////////////////
void printLogo()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Brauverein.org"));
  delay(2000);
  lcd.clear();
}
///////////////////////////////////////////////////////////////////////////////
// menubuttons
///////////////////////////////////////////////////////////////////////////////
bool SubStateChange(int maxStates)
{
    if ( isButtonPressed (btnUP) ) 
    {
      CONSOLE(F("Up"));
      CONSOLELN(subMenuState);
      subMenuState++;
      if ( subMenuState == maxStates )
        subMenuState = 0;
    }
    else if ( isButtonPressed (btnDOWN) ) 
    { 
      CONSOLE(F("Down"));
      CONSOLELN(subMenuState);
      if ( 0 == subMenuState )
      {
        subMenuState = maxStates-1;
      }      
      else 
      {
        subMenuState--;
      }
    }
    else if ( isButtonPressed (btnLEFT) ) 
    { 
      return true;
    }
    return false;
}
///////////////////////////////////////////////////////////////////////////////
// menu
///////////////////////////////////////////////////////////////////////////////
void menu()
{
  lcd.setCursor(0, 0);
  switch(actMenuState)
  {
    case 0:
    {
      if ( true == SubStateChange(2) )
      {        
        if ( 0 == subMenuState )
          actMenuState = 1;
        else 
          actMenuState = 2;
        CONSOLE(F("S"));
        CONSOLELN(actMenuState);
      }
      lcd.setCursor(0, 0);
      lcd.print(F("Menu"));
      lcd.setCursor(0, 1);
      if ( 0 == subMenuState )
      {
        lcd.print(F("Brew"));
      } 
      else
      {
        lcd.print(F("Set "));
      }
      break;
    }
  }
}
///////////////////////////////////////////////////////////////////////////////
// setup
///////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  lcd.begin(16, 2);
  
  // load and set values
  LoadValues();

  // set some HW Pins  
  pinMode(PINTMPDS18B20,  INPUT);
  pinMode(PINBUZZER,      OUTPUT);
  pinMode(PINLCD,         OUTPUT);
  analogWrite (PINBUZZER, 0);
  digitalWrite(PINLCD, HIGH);

  // Start up DS18B20
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, TEMPRESOLUTION);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();
  
  // start 433MHz modul switch
  mySwitch.enableTransmit(PINHEATER); //D13
  mySwitch.setProtocol(myRezept.SwitchProtocol);
  mySwitch.setPulseLength(myRezept.SwitchPulseLength);
  mySwitch.setRepeatTransmit(myRezept.SwitchRepeat); 
  
  // configure timer
  timerTempMeasure.setTime(myRezept.PidOWinterval);
  timerPidCompute.setTime(myRezept.PidWindowSize);
  timerSendHeatState.setTime(myRezept.PidWindowSize);
  
  // logo
  printLogo();
}
///////////////////////////////////////////////////////////////////////////////
// loop
///////////////////////////////////////////////////////////////////////////////
void loop ()
{
  // read key pressed
  Keypad();
  // show menu
  menu();
  // temperatur meassure
  timerTempMeasure.start();
  if ( timerTempMeasure.timeOver() )
  {
    timerTempMeasure.restart();
    isTmp = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();

    if ( DEVICE_DISCONNECTED_C == isTmp)
    {
      CONSOLELN("DISCONNECTED");
    } 
    else
    {
      CONSOLELN(isTmp);
    }         
  }

  // PID compute 
  timerPidCompute.start();
  if ( timerPidCompute.timeOver() )
  {
    timerPidCompute.restart();
  }

  // always send state
  timerSendHeatState.start();
  if ( timerSendHeatState.timeOver() )
  {
    timerSendHeatState.restart();
    if ( heatState )
    {
        CONSOLELN("On");
        mySwitch.send(myRezept.SwitchOn,myRezept.SwitchBits);
    }
    else
    {
        CONSOLELN("Off");
        mySwitch.send(myRezept.SwitchOff,myRezept.SwitchBits);
    } 
  }
}
///////////////////////////////////////////////////////////////////////////////
