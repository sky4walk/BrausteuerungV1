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
#define EEPROM_HEADER_DATA  051215
#define EEPROM_HEADER_POS   0
#define EEPROM_HEADER_SIZE  4
#define MAXRAST             15
#define PINHEATER           13  //D3
#define PINTMPDS18B20       12  //D2
#define PINBUZZER           11  //D1
#define PINLCD              10
#define PINKEY              0
#define btnNONE             0
#define btnRIGHT            1
#define btnUP               2
#define btnDOWN             3
#define btnLEFT             4
#define btnSELECT           5
#define MINDIFF             0.5
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
  int PidOWinterval;
  int PidWindowSize;
  int PidMinWindow;
  int SwitchProtocol;
  int SwitchPulseLength;
  int SwitchRepeat;
  int SwitchBits;
  int SwitchOn;
  int SwitchOff;
} myRezept;
///////////////////////////////////////////////////////////////////////////////
// variablen
///////////////////////////////////////////////////////////////////////////////
double pidOutput        = 0;
double isTmp            = 25;
double sollTmp          = 25;
bool   btnSelectPressed = false;
bool   btnUpPressed     = false;
bool   btnDownPressed   = false;
///////////////////////////////////////////////////////////////////////////////
// classes
///////////////////////////////////////////////////////////////////////////////
LiquidCrystal     lcd(8, 9, 4, 5, 6, 7);
OneWire           oneWire(PINTMPDS18B20);
DallasTemperature sensors(&oneWire);
DeviceAddress     tempDeviceAddress;
RCSwitch          mySwitch = RCSwitch();
PID myPID(&isTmp, &pidOutput, &sollTmp, myRezept.pidKp,myRezept.pidKi,myRezept.pidKd, DIRECT);
WaitTime          timerTempMeasure(myRezept.PidOWinterval);
///////////////////////////////////////////////////////////////////////////////

void setup()
{
  myRezept.PidOWinterval  = PIDOWINTERVAL;
  myRezept.PidWindowSize  = PIDWINDOWSIZE;
  myRezept.PidMinWindow   = PIDMINWINDOW;
  
  Serial.begin(115200);
  
  pinMode(PINTMPDS18B20,  INPUT);
  pinMode(PINHEATER,      OUTPUT);
  pinMode(PINBUZZER,      OUTPUT);
  pinMode(PINLCD,         OUTPUT);

  analogWrite (PINBUZZER, 0);

  // Start up the DS18B20
  sensors.begin();
  sensors.getAddress(tempDeviceAddress, 0);
  sensors.setResolution(tempDeviceAddress, TEMPRESOLUTION);
  sensors.setWaitForConversion(false);
  sensors.requestTemperatures();

  mySwitch.enableTransmit(0); //D3
  mySwitch.setProtocol(myRezept.SwitchProtocol);
  mySwitch.setPulseLength(myRezept.SwitchPulseLength);
  mySwitch.setRepeatTransmit(myRezept.SwitchRepeat); 
  
  lcd.begin(16, 2);
  
}
void loop ()
{
  timerTempMeasure.start();
  if ( timerTempMeasure.timeOver() )
  {
    timerTempMeasure.init();
    isTmp = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();

    if ( DEVICE_DISCONNECTED_C == isTmp)
    {
      CONSOLELN("DISCONNECTED");
    } 
    else
    {
      CONSOLELN(isTmp);
      mySwitch.send(myRezept.SwitchOn,myRezept.SwitchBits);
    }
    
  }
}
