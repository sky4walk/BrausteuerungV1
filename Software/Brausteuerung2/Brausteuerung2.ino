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
#define USEDEFAULTS
#define TEMPRESOLUTION      12
#define PIDOWINTERVAL       (1000 / (1 << (12 - TEMPRESOLUTION)))
#define PIDWINDOWSIZE       (PIDOWINTERVAL * 6)
#define PIDMINWINDOW        150
#define MAXRAST             16
#define PINTMPDS18B20       13  //D13
#define PINBUZZER           12  //D12
#define PINHEATER           11  //D11
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
#define MIL2MIN             60 * 1000
#define PRINTTIMELCD        250
#define PRINTTIMESER        1000
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
#define MENU_BREW_RAST_OVR            32
#define BREW_STATE_HEAT_UP            0
#define BREW_STATE_HEAT_TIMER         1
#define BREW_STATE_TIMER_FIN          2
#define BREW_STATE_NEXT_STATE         3
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
  bool heatState;
  bool tempReached;
  bool UseDefault;
} myRezept;
///////////////////////////////////////////////////////////////////////////////
// variablen
///////////////////////////////////////////////////////////////////////////////
double pidOutput                 = 0;
double actTmp                    = 25;
double readTmp                   = 25;
double sollTmp                   = 25;
double minDiffC                  = 0.2;
unsigned long relaisOnStartTime  = 0;
bool   btnPressed[MAXBUTTONS]    = {false, false, false, false, false, false};
int    actMenuState              = 0;
int    subMenuState              = 0;
int    actBrewStat               = 0;
bool   heatStateChanged          = false;
bool   buzzerActive              = false;
char   printBuf[16];
///////////////////////////////////////////////////////////////////////////////
// classes
///////////////////////////////////////////////////////////////////////////////
LiquidCrystal     lcd(8, 9, 4, 5, 6, 7);
OneWire           oneWire(PINTMPDS18B20);
DallasTemperature sensors(&oneWire);
DeviceAddress     tempDeviceAddress;
RCSwitch          mySwitch = RCSwitch();
PID myPID(&actTmp, &pidOutput, &sollTmp, myRezept.pidKp, myRezept.pidKi, myRezept.pidKd, DIRECT);
WaitTime          timerTempMeasure;
WaitTime          timerPidCompute;
WaitTime          timerSendHeatState;
WaitTime          timerBrewTimer;
WaitTime          lcdPrint;
WaitTime          serPrint;
WaitTime          pidRelaisTimer;
StorageEEProm     store;
///////////////////////////////////////////////////////////////////////////////
// setDefaultValues
///////////////////////////////////////////////////////////////////////////////
void LoadValues()
{
  CONSOLELN("LoadValues");
  myRezept.UseDefault = false;
  store.load(0, sizeof(Rezept), (char*)&myRezept);
#ifdef USEDEFAULTS  
  if (  false == myRezept.UseDefault )
#endif
  {
    // only go here after first installation
    memset((char*)&myRezept, 0, sizeof(Rezept));
    myRezept.pidKp             = 5000.0f;
    myRezept.pidKi             = 1000.0f;
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
    myRezept.heatState         = false;
    myRezept.UseDefault        = true;

    for ( int i = 0; i < MAXRAST; i++ )
    {
      myRezept.rasten[i].temp = 20;
      myRezept.rasten[i].time = 2;
    }

    store.save(0, sizeof(Rezept), (char*)&myRezept);
  }
}
///////////////////////////////////////////////////////////////////////////////
// resetPID
///////////////////////////////////////////////////////////////////////////////
void resetPID() {
  myPID.SetMode(MANUAL);
  pidOutput = 0;
  myPID.SetMode(AUTOMATIC);
  CONSOLELN(F("PIDres"));
}
///////////////////////////////////////////////////////////////////////////////
// setPID
///////////////////////////////////////////////////////////////////////////////
void setPID() {
  myPID.SetOutputLimits(myRezept.PidMinWindow, myRezept.PidWindowSize);
  myPID.SetTunings(myRezept.pidKp, myRezept.pidKi, myRezept.pidKd);
  myPID.SetSampleTime(myRezept.PidOWinterval);
  resetPID();
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
      subMenuState = maxStates - 1;
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
  relaisOnStartTime = 0;
  myRezept.started = false;
  myRezept.heatState = false;
  buzzerActive = false;
  BuzzerOnOff(buzzerActive);
  actBrewStat = BREW_STATE_NEXT_STATE;
  timerBrewTimer.init();
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
      return max - 1;
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
  switch (actMenuState)
  {
    case MENU_START:
      {
        if ( true == SubStateChange(4) )
        {
          switch ( subMenuState )
          {
            case 0:
              nextState(MENU_BREW_RAST_NR);
              break;
            case 1:
              nextState(MENU_SETUP_RAST_MENU);
              break;
            case 2:
              nextState(MENU_BREW_RAST_OVR);
              break;
            case 3:
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
              lcd.print(F("Rast"));
              break;
            case 2:
              lcd.print(F("OVR "));
              break;
            case 3:
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
        lcd.print(myRezept.pidKp, 2);

        if ( isButtonPressed ( btnUP ) )
          myRezept.pidKp += 10;
        else if ( isButtonPressed ( btnDOWN ) )
          myRezept.pidKp -= 10;
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
        lcd.print(myRezept.pidKi, 2);

        if ( isButtonPressed ( btnUP ) )
          myRezept.pidKi += 10;
        else if ( isButtonPressed ( btnDOWN ) )
          myRezept.pidKi -= 10;
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
        lcd.print(myRezept.pidKd, 2);

        if ( isButtonPressed ( btnUP ) )
          myRezept.pidKd += 10;
        else if ( isButtonPressed ( btnDOWN ) )
          myRezept.pidKd -= 10;
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
              setPID();
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
          myRezept.actRast = menuInputVal(myRezept.actRast + 1, 0, MAXRAST);
        else if ( isButtonPressed ( btnDOWN ) )
          myRezept.actRast = menuInputVal(myRezept.actRast - 1, 0, MAXRAST);
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
          myRezept.rasten[rastNr].temp = menuInputVal(myRezept.rasten[rastNr].temp + 1, 0, 0);
        else if ( isButtonPressed ( btnDOWN ) )
          myRezept.rasten[rastNr].temp = menuInputVal(myRezept.rasten[rastNr].temp - 1, 0, 0);
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
          myRezept.rasten[rastNr].time = menuInputVal(myRezept.rasten[rastNr].time + 1, 0, 0);
        else if ( isButtonPressed ( btnDOWN ) )
          myRezept.rasten[rastNr].time = menuInputVal(myRezept.rasten[rastNr].time - 1, 0, 0);
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
          myRezept.actRast = menuInputVal(myRezept.actRast + 1, 0, MAXRAST);
        else if ( isButtonPressed ( btnDOWN ) )
          myRezept.actRast = menuInputVal(myRezept.actRast - 1, 0, MAXRAST);
        else if ( isButtonPressed ( btnLEFT ) )
        {
          nextState(MENU_BREW_RAST_START);
          timerBrewTimer.setTime(myRezept.rasten[rastNr].time);
          sollTmp = myRezept.rasten[rastNr].temp;
          myRezept.started = true;
          actBrewStat = BREW_STATE_NEXT_STATE;
          resetPID();
        }
        break;
      }
    case MENU_BREW_RAST_START:
      {
        if ( isButtonPressed ( btnLEFT ) )
          nextState(MENU_START);
        break;
      }
    case MENU_BREW_RAST_OVR:
      {
        if ( isButtonPressed ( btnLEFT ) )
          nextState(MENU_START);
        break;
      }
  }
}
///////////////////////////////////////////////////////////////////////////////
// BuzzerOnOff
///////////////////////////////////////////////////////////////////////////////
void BuzzerOnOff(boolean onOff)
{
  if ( true == onOff )
  {
    analogWrite (PINBUZZER, 180);
  }
  else
  {
    analogWrite (PINBUZZER, 0);
  }
}
///////////////////////////////////////////////////////////////////////////////
// Relais
///////////////////////////////////////////////////////////////////////////////
void Relais(bool onOff)
{
  if ( onOff )
  {
    CONSOLELN("On");
    mySwitch.send(myRezept.SwitchOn, myRezept.SwitchBits);
  }
  else
  {
    CONSOLELN("Off");
    mySwitch.send(myRezept.SwitchOff, myRezept.SwitchBits);
  }
}
///////////////////////////////////////////////////////////////////////////////
// Relais
///////////////////////////////////////////////////////////////////////////////
void changeHeatState(bool onOff)
{
  if ( onOff != myRezept.heatState )
    heatStateChanged = true;
  myRezept.heatState = onOff;
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
  mySwitch.enableTransmit(PINHEATER);
  mySwitch.setProtocol(myRezept.SwitchProtocol);
  mySwitch.setPulseLength(myRezept.SwitchPulseLength);
  mySwitch.setRepeatTransmit(myRezept.SwitchRepeat);

  // configure timer
  timerTempMeasure.setTime(myRezept.PidOWinterval);
  timerPidCompute.setTime(myRezept.PidWindowSize);
  timerSendHeatState.setTime(myRezept.PidWindowSize);
  lcdPrint.setTime(PRINTTIMELCD);
  serPrint.setTime(PRINTTIMESER);

  //set PID values
  setPID();
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
    readTmp = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();

    if ( DEVICE_DISCONNECTED_C == readTmp)
    {
      CONSOLELN("DISCONNECTED");
    }
    else
    {
      actTmp = readTmp;
    }
  }

  // rast override
  if ( MENU_BREW_RAST_OVR == actMenuState)
  {
    if ( isButtonPressed ( btnUP ) )
    {
      changeHeatState(true);
    } 
    else if ( isButtonPressed ( btnDOWN ) )
    {
      changeHeatState(false);
    }
    
    lcdPrint.start();
    if ( lcdPrint.timeOver() )
    {
      lcdPrint.restart();

      lcd.setCursor(0, 0);
      lcd.print(F(" H:"));
      myRezept.heatState ? lcd.print(F("1")) : lcd.print(F("0"));
      lcd.print(F(" Ti:"));
      if ( DEVICE_DISCONNECTED_C == readTmp)
        lcd.print(F("--"));
      else
        lcd.print(actTmp);
    }

    serPrint.start();
    if ( serPrint.timeOver() )
    {
      serPrint.restart();
      CONSOLE(F("OVR H:"));
      if ( myRezept.heatState )
        CONSOLE(F("1"));
      else
        CONSOLE(F("0"));
      CONSOLE(F(" Ti:"));
      CONSOLE(actTmp);
    }
  }
  // wenn brausteuerung gestartet ist
  if ( myRezept.started )
  {
    // zustand suchen der aktiv ist
    if ( BREW_STATE_NEXT_STATE == actBrewStat )
    {
      while ( ( myRezept.actRast < MAXRAST ) &&
              ( false == myRezept.rasten[myRezept.actRast].active ) )
      {
        myRezept.actRast++;
      }
      CONSOLE(F("nxtSt:"));
      CONSOLELN(myRezept.actRast);
      if ( MAXRAST == myRezept.actRast )
      {
        myRezept.actRast = 0;
        relaisOnStartTime = 0;
        nextState(MENU_START);
      }
      else
      {
        actBrewStat = BREW_STATE_HEAT_UP;
      }
    }

    // ziel temperatur ueberschritten
    if ( BREW_STATE_HEAT_UP == actBrewStat )
    {
      if ( myRezept.rasten[myRezept.actRast].temp < actTmp )
      {
        actBrewStat = BREW_STATE_HEAT_TIMER;
        timerBrewTimer.setTime( myRezept.rasten[myRezept.actRast].time * MIL2MIN );
        timerBrewTimer.start();
        CONSOLELN(F("TmpReached"));
      }
    }

    // zeitmessung bis ende der rast
    if ( BREW_STATE_HEAT_TIMER == actBrewStat )
    {
      if ( timerBrewTimer.timeOver() )
      {
        CONSOLELN(F("TimeReached"));
        actBrewStat = BREW_STATE_TIMER_FIN;
        if ( myRezept.rasten[myRezept.actRast].alarm )
        {
          buzzerActive = true;
          BuzzerOnOff(buzzerActive);
        }
      }
    }


    // ende der rast erreicht
    if ( BREW_STATE_TIMER_FIN == actBrewStat )
    {
      if ( buzzerActive )
      {
        if ( isButtonPressed ( btnDOWN ) )
        {
          buzzerActive = false;
          BuzzerOnOff(buzzerActive);
        }
      }
      if ( myRezept.rasten[myRezept.actRast].wait )
      {
        if ( isButtonPressed ( btnUP ) )
        {
          actBrewStat = BREW_STATE_NEXT_STATE;
          buzzerActive = false;
          BuzzerOnOff(buzzerActive);
          myRezept.actRast++;
        }
      }
      else
      {
        actBrewStat = BREW_STATE_NEXT_STATE;
        myRezept.actRast++;
      }
    }

    if ( myRezept.started )
    {
      lcdPrint.start();
      if ( lcdPrint.timeOver() )
      {
        lcdPrint.restart();
        // LCD Show
        lcd.print(F("S:"));
        lcd.print(myRezept.actRast);
        lcd.print(F(" H:"));
        myRezept.heatState ? lcd.print(F("1")) : lcd.print(F("0"));
        if ( BREW_STATE_HEAT_TIMER == actBrewStat )
        {
          lcd.print(F(" R:"));
          unsigned long sT = timerBrewTimer.getDuration() / 1000;
          if ( sT > 119 )
            sT = sT / 60;
          sprintf (printBuf, "%03u", (int)sT);
          lcd.print( printBuf );
        }
        else if ( BREW_STATE_HEAT_UP == actBrewStat )
        {
          lcd.print(F(" T:"));
          lcd.print( myRezept.rasten[myRezept.actRast].time );
        }
        else if ( BREW_STATE_TIMER_FIN == actBrewStat )
        {
          lcd.print(F(" W:"));
        }
        lcd.setCursor(0, 1);
        lcd.print(F("Ti:"));
        if ( DEVICE_DISCONNECTED_C == readTmp)
          lcd.print(F("--"));
        else
          lcd.print(actTmp);
        lcd.print(F(" Ts:"));
        lcd.print(myRezept.rasten[myRezept.actRast].temp);
      }
    }
    serPrint.start();
    if ( serPrint.timeOver() )
    {
      serPrint.restart();
      // Debug show
      if ( buzzerActive )
        CONSOLE(F("A:"));
      else
        CONSOLE(F("B:"));
      CONSOLE(actBrewStat);
      CONSOLE(F(" S:"));
      CONSOLE(myRezept.actRast);
      CONSOLE(F(" H:"));
      if ( myRezept.heatState )
        CONSOLE(F("1"));
      else
        CONSOLE(F("0"));
      CONSOLE(F(" Ti:"));
      CONSOLE(actTmp);
      CONSOLE(F(" Ts:"));
      CONSOLE(myRezept.rasten[myRezept.actRast].temp);
      if ( BREW_STATE_HEAT_TIMER == actBrewStat )
      {
        CONSOLE(F(" R:"));
        CONSOLELN( timerBrewTimer.getDuration() / 1000 );
      }
      else if ( BREW_STATE_HEAT_UP == actBrewStat )
      {
        CONSOLE(F(" T:"));
        CONSOLELN( myRezept.rasten[myRezept.actRast].time  );
      }
      else if ( BREW_STATE_TIMER_FIN == actBrewStat )
      {
        CONSOLELN(F(" W:"));
      }
    }
    // PID compute
    timerPidCompute.start();
    if ( timerPidCompute.timeOver() )
    {
      timerPidCompute.restart();
      myPID.Compute();
      //relaisOnStartTime = millis();
      pidRelaisTimer.setTime(pidOutput);
      pidRelaisTimer.start();
      CONSOLE(F("PIDcomp:"));
      CONSOLELN(pidOutput);
    }
  }

  // wie lange geheizt werden soll
  if ( MENU_BREW_RAST_OVR != actMenuState )
  {
    if (   false == pidRelaisTimer.timeOver()    &&
           ( pidOutput > myRezept.PidMinWindow ) )
    {
      changeHeatState(true);
    }
    else
    {
      changeHeatState(false);
    }
  }
  
  // always send state
  timerSendHeatState.start();
  if ( timerSendHeatState.timeOver() || heatStateChanged )
  {
    timerSendHeatState.restart();
    heatStateChanged = false;
    Relais( myRezept.heatState );
  }
}
///////////////////////////////////////////////////////////////////////////////
