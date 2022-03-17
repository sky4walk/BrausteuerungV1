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
  int LastState;
  bool UseDefault;
} myRezept;
///////////////////////////////////////////////////////////////////////////////
// variablen
///////////////////////////////////////////////////////////////////////////////
double pidOutput                 = 0;
double isTmp                     = 25;
double sollTmp                   = 25;
bool   btnPressed[MAXBUTTONS]    = {false,false,false,false,false,false};
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
    myRezept.PidOWinterval     = PIDOWINTERVAL;
    myRezept.PidWindowSize     = PIDWINDOWSIZE;
    myRezept.PidMinWindow      = PIDMINWINDOW;
    myRezept.SwitchOn          = 1631343;//1631343;
    myRezept.SwitchOff         = 1631342;//1631342;
    myRezept.SwitchProtocol    = 1;
    myRezept.SwitchPulseLength = 315;
    myRezept.SwitchBits        = 24;
    myRezept.SwitchRepeat      = 15;
    myRezept.LastState         = 0;
    myRezept.UseDefault        = true;
    store.save(0, sizeof(Rezept), (char*)&myRezept);
  }
}
///////////////////////////////////////////////////////////////////////////////
// readKeyPad
///////////////////////////////////////////////////////////////////////////////
void setButton(int btnNr) 
{
  if ( MAXBUTTONS > btnNr ) 
    if ( false == btnPressed[btnNr] )
      btnPressed[btnNr] = true;
}
void resetButton(int btnNr)
{
  if ( MAXBUTTONS > btnNr )
    btnPressed[btnNr] = false;
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
  
  return val;
}
///////////////////////////////////////////////////////////////////////////////
// print Logo
///////////////////////////////////////////////////////////////////////////////
void printLogo()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Brauverein.org");
  delay(2000);
  lcd.clear();
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
    
  pinMode(PINTMPDS18B20,  INPUT);
  pinMode(PINBUZZER,      OUTPUT);
  pinMode(PINLCD,         OUTPUT);
  analogWrite (PINBUZZER, 0);

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
  
  digitalWrite(PINLCD, HIGH);

  timerTempMeasure.setTime(myRezept.PidOWinterval);
  timerPidCompute.setTime(myRezept.PidWindowSize); 
  delay(500);
  printLogo();
}
///////////////////////////////////////////////////////////////////////////////
// loop
///////////////////////////////////////////////////////////////////////////////
bool switchon = false;
void loop ()
{
  int keyState = Keypad();

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

    if ( switchon )
    {
        CONSOLELN("On");
        mySwitch.send(myRezept.SwitchOn,myRezept.SwitchBits);
        switchon = false;
    }
    else
    {
        CONSOLELN("Off");
        mySwitch.send(myRezept.SwitchOff,myRezept.SwitchBits);
        switchon = true;
    } 
  }
}
///////////////////////////////////////////////////////////////////////////////
