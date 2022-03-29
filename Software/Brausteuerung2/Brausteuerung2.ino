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
#define MAXRAST             16
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
// states
///////////////////////////////////////////////////////////////////////////////
#define MENU_START                    0
#define MENU_SETUP_PIDKP              1
#define MENU_SETUP_PIDKI              2
#define MENU_SETUP_PIDKD              3
#define MENU_SETUP_PidOWinterval      4
#define MENU_SETUP_PidWindowSize      5
#define MENU_SETUP_PidMinWindow       6
#define MENU_SETUP_SwitchProtocol     7
#define MENU_SETUP_SwitchPulseLength  8
#define MENU_SETUP_SwitchRepeat       9
#define MENU_SETUP_SwitchBits         10
#define MENU_SETUP_SwitchOn           11
#define MENU_SETUP_SwitchOff          12
#define MENU_SETUP_BREW_NR            13
#define MENU_SETUP_RAST_MENU          14
#define MENU_SETUP_RAST_NR            15
#define MENU_SETUP_RAST_TIME          16
#define MENU_SETUP_RAST_TEMP          17
#define MENU_SETUP_RAST_ACTIVE        18
#define MENU_SETUP_RAST_WAIT          19
#define MENU_SETUP_RAST_ALARM         20
#define MENU_BREW_RAST_NR             30
#define MENU_BREW_RAST_START          31
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
WaitTime          timerBrewTimer;
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
    myRezept.pidKp             = 5000.0f;
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
    bool getOK = false;
    if ( isButtonPressed (btnUP) ) 
    {
      CONSOLE(F("U"));
      subMenuState++;
      if ( subMenuState == maxStates )
        subMenuState = 0;
    }
    else if ( isButtonPressed (btnDOWN) ) 
    { 
      CONSOLE(F("D"));
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
      CONSOLE(F("O"));
      getOK = true;
    }
    return getOK;
}
///////////////////////////////////////////////////////////////////////////////
// next State
///////////////////////////////////////////////////////////////////////////////
void nextState(int nxtSt)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  actMenuState = nxtSt;
  subMenuState = 0;
}
///////////////////////////////////////////////////////////////////////////////
// menuInputVal
///////////////////////////////////////////////////////////////////////////////
long menuInputVal(long val, long min, long max)
{
  lcd.setCursor(0, 1);
  lcd.print(F("          "));
  if ( min != max ) 
  {
    if ( max <= val )
      return min;
    else if ( val < min )
      return max-1;
  }
  return val;
}
///////////////////////////////////////////////////////////////////////////////
// menuInputValBool
///////////////////////////////////////////////////////////////////////////////
void printBool(bool val)
{
  lcd.setCursor(0, 1);
  if ( val )
      lcd.print(F("Y"));
  else
      lcd.print(F("N"));  
}
bool menuInputValBool(bool val)
{
  val = ( val ? false : true );
  return val;
}
///////////////////////////////////////////////////////////////////////////////
// menu
///////////////////////////////////////////////////////////////////////////////
void menu()
{
  lcd.setCursor(0, 0);
  int rastNr = myRezept.actRast;
  switch(actMenuState)
  {
    case MENU_START:
    {
      if ( true == SubStateChange(2) )
      {
        switch ( subMenuState )
        {
          case 0:
            nextState(MENU_BREW_RAST_NR);
            break;        
          case 1:
            nextState(MENU_SETUP_PIDKP);
            break;        
        }
      }
      else
      {
        lcd.print(F("Menu"));
        lcd.setCursor(0, 1);
        switch ( subMenuState )
        {
          case 0: 
            lcd.print(F("Brew"));
            break;
          case 1: 
            lcd.print(F("Set "));
            break;
        }
      }
      break;
    }
    case MENU_SETUP_PIDKP:
    {
      lcd.print(F("PidKP"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.pidKp,2);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.pidKp += 0.01;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.pidKp -= 0.01;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_PIDKD);
        CONSOLELN(myRezept.pidKp );
      }
      break;
    }
    case MENU_SETUP_PIDKD:
    {
      lcd.print(F("PidKI"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.pidKi,2);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.pidKi += 0.01;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.pidKi -= 0.01;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_PIDKI);
        CONSOLELN(myRezept.pidKi );
      }
      break;
    }
    case MENU_SETUP_PIDKI:
    {
      lcd.print(F("PidKD"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.pidKd,2);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.pidKd += 0.01;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.pidKd -= 0.01;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_PidOWinterval);
        CONSOLELN(myRezept.pidKd );
      }
      break;
    }
    case MENU_SETUP_PidOWinterval:
    {
      lcd.print(F("PidOWInt"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.PidOWinterval);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.PidOWinterval++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.PidOWinterval--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_PidWindowSize);
        CONSOLELN(myRezept.PidOWinterval );
      }
      break;
    }
    case MENU_SETUP_PidWindowSize:
    {
      lcd.print(F("PidWSz"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.PidWindowSize);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.PidWindowSize++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.PidWindowSize--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_PidMinWindow);
        CONSOLELN(myRezept.PidWindowSize );
      }
      break;
    }
    case MENU_SETUP_PidMinWindow:
    {
      lcd.print(F("PidMinWnd"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.PidMinWindow);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.PidMinWindow++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.PidMinWindow--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_SwitchProtocol);
        CONSOLELN(myRezept.PidMinWindow );
      }
      break;
    }
    case MENU_SETUP_SwitchProtocol:
    {
      lcd.print(F("SwProt"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.SwitchProtocol);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.SwitchProtocol++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.SwitchProtocol--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_SwitchPulseLength);
        CONSOLELN(myRezept.SwitchProtocol );
      }
      break;
    }
    case MENU_SETUP_SwitchPulseLength:
    {
      lcd.print(F("SwPL"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.SwitchPulseLength);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.SwitchPulseLength++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.SwitchPulseLength--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_SwitchRepeat);
        CONSOLELN(myRezept.SwitchPulseLength );
      }
      break;
    }
    case MENU_SETUP_SwitchRepeat:
    {
      lcd.print(F("SwRpt"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.SwitchRepeat);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.SwitchRepeat++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.SwitchRepeat--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        nextState(MENU_SETUP_SwitchBits);
        CONSOLELN(myRezept.SwitchRepeat );
      }
      break;
    }
    case MENU_SETUP_SwitchBits:
    {
      lcd.print(F("SwBits"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.SwitchBits);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.SwitchBits++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.SwitchBits--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        CONSOLELN(myRezept.SwitchBits );
        nextState(MENU_SETUP_SwitchOn);
      }
      break;
    }
    case MENU_SETUP_SwitchOn:
    {
      lcd.print(F("SwOn"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.SwitchOn);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.SwitchOn++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.SwitchOn--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        CONSOLELN(myRezept.SwitchOn );
        nextState(MENU_SETUP_SwitchOff);
      }
      break;
    }
    case MENU_SETUP_SwitchOff:
    {
      lcd.print(F("SwOff"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.SwitchOff);
      
      if ( isButtonPressed ( btnUP ) )
        myRezept.SwitchOff++;
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.SwitchOff--;
      else if ( isButtonPressed ( btnLEFT ) )
      {
        CONSOLELN(myRezept.SwitchOff );
        nextState(MENU_SETUP_RAST_MENU);
      }
      break;
    }
    case MENU_SETUP_RAST_MENU:
    {
      if ( true == SubStateChange(2) )
      {
        switch ( subMenuState )
        {
          case 0:
            nextState(MENU_SETUP_RAST_NR);
            break;        
          case 1:
            store.save(0, sizeof(Rezept), (char*)&myRezept);
            CONSOLELN(F("SAVEVAL"));
            nextState(MENU_START);
            break;        
        }
      }
      else
      {
        lcd.print(F("RastMenu"));
        lcd.setCursor(0, 1);
        switch ( subMenuState )
        {
          case 0: 
            lcd.print(F("Rast"));
            break;
          case 1: 
            lcd.print(F("Menu"));
            break;
        }
      }
      break;
    }
    case MENU_SETUP_RAST_NR:
    {
      lcd.print(F("RastNr"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.actRast);      
      if ( isButtonPressed ( btnUP ) )
        myRezept.actRast = menuInputVal(myRezept.actRast+1,0,MAXRAST);
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.actRast = menuInputVal(myRezept.actRast-1,0,MAXRAST);
      else if ( isButtonPressed ( btnLEFT ) )
      {
        CONSOLELN( myRezept.actRast );
        nextState(MENU_SETUP_RAST_TEMP);
      }
      break;      
    }
    case MENU_SETUP_RAST_TEMP:
    {
      lcd.print(F("Temp[C] "));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.rasten[rastNr].temp);
      if ( isButtonPressed ( btnUP ) )
        myRezept.rasten[rastNr].temp = menuInputVal(myRezept.rasten[rastNr].temp+1,0,0);
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.rasten[rastNr].temp = menuInputVal(myRezept.rasten[rastNr].temp-1,0,0);
      else if ( isButtonPressed ( btnLEFT ) )
      {
        CONSOLELN(myRezept.rasten[rastNr].temp );
        nextState(MENU_SETUP_RAST_TIME);
      }
      break;
    }
    case MENU_SETUP_RAST_TIME:
    {
      lcd.print(F("Time[Min]"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.rasten[rastNr].time);
      if ( isButtonPressed ( btnUP ) )
        myRezept.rasten[rastNr].time = menuInputVal(myRezept.rasten[rastNr].time+1,0,0);
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.rasten[rastNr].time = menuInputVal(myRezept.rasten[rastNr].time-1,0,0);
      else if ( isButtonPressed ( btnLEFT ) )
      {
        CONSOLELN(myRezept.rasten[rastNr].time );
        nextState(MENU_SETUP_RAST_ACTIVE);
      }
      break;
    }
    case MENU_SETUP_RAST_ACTIVE:
    {
      lcd.print(F("Active  "));
      printBool( myRezept.rasten[rastNr].active );
      if ( isButtonPressed ( btnUP ) )
        myRezept.rasten[rastNr].active = menuInputValBool(myRezept.rasten[rastNr].active);
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.rasten[rastNr].active = menuInputValBool(myRezept.rasten[rastNr].active);
      else if ( isButtonPressed ( btnLEFT ) )
      {
        if ( myRezept.rasten[rastNr].active )
          CONSOLELN(F("On"));
        else
          CONSOLELN(F("Of"));
        nextState(MENU_SETUP_RAST_WAIT);
      }
      break;
    }
    case MENU_SETUP_RAST_WAIT:
    {
      lcd.print(F("Wait    "));
      printBool( myRezept.rasten[rastNr].wait );
      if ( isButtonPressed ( btnUP ) )
        myRezept.rasten[rastNr].wait = menuInputValBool(myRezept.rasten[rastNr].wait);
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.rasten[rastNr].wait = menuInputValBool(myRezept.rasten[rastNr].wait);
      else if ( isButtonPressed ( btnLEFT ) )
      {
        if ( myRezept.rasten[rastNr].wait )
          CONSOLELN(F("On"));
        else
          CONSOLELN(F("Of"));
        nextState(MENU_SETUP_RAST_ALARM);
      }
      break;
    }
    case MENU_SETUP_RAST_ALARM:
    {
      lcd.print(F("Alarm   "));
      printBool( myRezept.rasten[rastNr].alarm );
      if ( isButtonPressed ( btnUP ) )
        myRezept.rasten[rastNr].alarm = menuInputValBool(myRezept.rasten[rastNr].alarm);
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.rasten[rastNr].alarm = menuInputValBool(myRezept.rasten[rastNr].alarm);
      else if ( isButtonPressed ( btnLEFT ) )
      {
        if ( myRezept.rasten[rastNr].alarm )
          CONSOLELN(F("On"));
        else
          CONSOLELN(F("Of"));
        nextState(MENU_SETUP_RAST_MENU);
      }
      break;
    }
    case MENU_BREW_RAST_NR:
    {
      lcd.print(F("RastNr"));
      lcd.setCursor(0, 1);
      lcd.print(myRezept.actRast);      
      if ( isButtonPressed ( btnUP ) )
        myRezept.actRast = menuInputVal(myRezept.actRast+1,0,MAXRAST);
      else if ( isButtonPressed ( btnDOWN ) )
        myRezept.actRast = menuInputVal(myRezept.actRast-1,0,MAXRAST);
      else if ( isButtonPressed ( btnLEFT ) )
      {
        timerBrewTimer.setTime(myRezept.rasten[rastNr].time);
        nextState(MENU_BREW_RAST_START);
      }
      break;      
    }
    case MENU_BREW_RAST_START:
    {      
      if ( isButtonPressed ( btnUP ) ){}
      else if ( isButtonPressed ( btnDOWN ) ){}
      else if ( isButtonPressed ( btnLEFT ) )
        nextState(MENU_START);
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

  // wenn brausteuerung gestartet ist
  if ( MENU_BREW_RAST_START == actMenuState )
  {
    lcd.print(F("S:"));
    lcd.print(myRezept.actRast);
    lcd.print(F(" H:"));
    heatState ? lcd.print(F("1")) : lcd.print(F("0"));
    lcd.setCursor(0, 1);
    lcd.print(F("iT:"));
    lcd.print(isTmp);
    lcd.print(F(" sT:"));
    lcd.print(myRezept.rasten[myRezept.actRast].temp);

    CONSOLE(F("S:"));
    CONSOLE(myRezept.actRast);
    CONSOLE(F(" H:"));
    if ( heatState ) 
      CONSOLE(F("1")); 
    else
      CONSOLE(F("0"));
    CONSOLE(F("iT:"));
    CONSOLE(isTmp);
    CONSOLE(F(" sT:"));
    CONSOLELN(myRezept.rasten[myRezept.actRast].temp);
    
    // PID compute 
    timerPidCompute.start();
    if ( timerPidCompute.timeOver() )
    {
      timerPidCompute.restart();
    }
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
