// brausteuerung@AndreBetz.de
// hier gilt die Bierlizenz
///////////////////////////////////////////////
// includes
///////////////////////////////////////////////

#include <OneWire.h>
#include <DallasTemperature.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>


///////////////////////////////////////////////
// defines
///////////////////////////////////////////////
#define EEPROM_HEADER_DATA 051215
#define EEPROM_HEADER_POS  0
#define EEPROM_HEADER_SIZE 4
#define MAX_STATES         15
#define MAX_RECIPES        3
#define SECPROMIN 	   60
#define MIL2SEC		   1000
#define btnNONE            0
#define btnRIGHT           1
#define btnUP              2
#define btnDOWN            3
#define btnLEFT            4
#define btnSELECT          5
///////////////////////////////////////////////
// hardware ein ausgaenge
///////////////////////////////////////////////
//
// DS18B20
// DALLES 18B50 CONNECTION
// Dallas     | waterproof | Arduino
// ----------------------------------
// PIN 1 GND  |  black     | GND
// PIN 2 Data |  yellow    | D12
// PIN 3 VCC  |  red       | 5V
//   _______
//  /  TOP  \
// /_________\
//    | | |
//    1 2 3
// 4.7KOhm zwischen PIN 2 und PIN 3
//
// 16x2 LCD Display
// ----------------
// A0: Buttons
//  4: DB4
//  5: DB5
//  6: DB6
//  7: DB7
//  8: RD (Data or Signal Display Selection
//  9: Enable
// 10: brightness
//
// RF-Link-Sender 434MHz Switch
//--------------
//  -------
//  |     |
//  | O   |
//  -------
//  | | | |
//  1 2 3 4
//
// 1: GND
// 2: Data in | D13
// 3: VCC
// 4: Antenne
//
/////////////////////////////////////////////////
//  Buzzer
//  ------------------------------
//  ------  - : GND
//  |    |
//  |    |  + : D11
//  ------
//   |  |
//   +  -
/////////////////////////////////////////////////
// Batteriespannungsmesser Spannungsteiler
// ---------------------------------------
//      ____       ____
// o---| R1 |--o--| R2 |--o    R1=R2=100KOhm
// |    ----   |   ----   |
// +9V         A1         GND
//
/////////////////////////////////////////////////
// Rotary Encoder Switch
// ---------------------------------------
//    1   2                RotEncSw | Arduino
//    o   o--R1--o +5V        1     | GND
//    -----                   2     | A2
//    | O |                   3     | D3
//    -----                   4     | GND
//    o o o                   5     | D2
//    3 4 5                   R1 = 2,2KOhm
//
/////////////////////////////////////////////////
// LCD                   R  E  D  D  D  D
// (HD44780-compatible)  S (N) 4  5  6  7  
// LiquidCrystal     lcd(8, 9, 4, 5, 6, 7);
// ---------------------------------------
//     V V V R R E D D D D D D D D L L
//     S D 0 S W   0 1 2 3 4 5 6 7 + -
//     S D
//                       1 1 1 1 1 1 1
//     1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6
//     o o o o o o o o o o o o o o o o
//         |                       |
//         |                       R3
//+5V o-R1-o-R2-o GND   R1=4,7KOhm |
//                      R2=330Ohm  o 
//                      R3=100Ohm  D10 Arduino
//    LCD  |  Arduino
//     1   |  GND
//     2   |  +5V
//     3   |  (siehe oben Spannungsteiler)
//     4   |  8
//     5   |  GND
//     6   |  9
//     7   |  GND
//     8   |  GND
//     9   |  GND
//    10   |  GND
//    11   |  4
//    12   |  5
//    13   |  6
//    14   |  7
//    15   |  (siehe oben)
//    16   |  GND
///////////////////////////////////////////////
// class
///////////////////////////////////////////////

/*
template <typename V, int N> class MeasureAverage
{
  public:
    MeasureAverage(V def = 0) : sum(0), p(0)
    {
      init(def);
    }
    void init(V def)
    {
      sum = 0;
      for (int i = 0; i < N; i++)
      {
        samples[i] = def;
        sum += samples[i];
      }
    }
    V add(V new_sample)
    {
      sum = sum - samples[p] + new_sample;
      samples[p++] = new_sample;
      if ( p >= N )  p = 0;
      return sum / N;
    }
  private:
    V samples[N];
    V sum;
    unsigned int p;
};
*/
class WaitTime
{
  public:
    WaitTime(unsigned long interval)
    {
      mWaitTime = interval;
      init();
    }
    void init()
    {
      mStartTime = 0;
      mLastStart = 0;
      mInitialized = false;
      mContinue    = false;
    }
    void init(unsigned long interval)
    {
      mStartTime = 0;
      mLastStart = interval;
      mInitialized = false;
      mContinue    = true;
    }
    void start(unsigned long interval)
    {
      if ( mContinue )
        mWaitTime = mLastStart;
      else
        mWaitTime = interval;
      start();
    }
    void start()
    {
      if ( false == mInitialized )
      {
        mStartTime = millis();
        mInitialized = true;
      }
    }
    boolean timeOver()
    {
      unsigned long actTime = millis();
      mDuration = actTime - mStartTime;
      if ( mDuration >= mWaitTime )
      {
        mDuration = mWaitTime;
        return true;
      }
      return false;
    }
    unsigned long getDuration()
    {
      return mWaitTime - mDuration;
    }
    boolean timerStarted()
    {
      mInitialized;
    }
  private:
    unsigned long mStartTime;
    unsigned long mWaitTime;
    unsigned long mDuration;
    unsigned long mLastStart;
    boolean mInitialized;
    boolean mContinue;
};

///////////////////////////////////////////////
// Hardware Ports
///////////////////////////////////////////////

int  pinEncoderA        = 2;   // interrupt 0
int  pinEncoderB        = 3;
int  pinHeizplatteOnOff	= 13; // digital Output: Heizplatt an 2 Kanaele RelaisModul Arduino PIC AVR DSP MCU
int  pinTemperaturDigit = 12; // digital Input:  Temperatursensor DB18B20
int  pinBuzzer		= 11; // need a PWM port
int  pinLcdLed		= 10; // helligkeits pin fuer LCD
int  pinKeyPad		= 0;  // Analog Keypad an A0
int  pinBatLow		= 1;  // Analog Keypad an A1
int  pinRotSwitch	= 2;  // Rotary Switch an A2


///////////////////////////////////////////////
// Hardware Classes
///////////////////////////////////////////////

LiquidCrystal     lcd(8, 9, 4, 5, 6, 7);
OneWire           oneWire(pinTemperaturDigit);
DallasTemperature sensors(&oneWire);
DeviceAddress     tempDeviceAddress;

///////////////////////////////////////////////
// interne variablen
///////////////////////////////////////////////
boolean weiterPressed	= false;	// weiter taste pressed
boolean weiterRequest   = false;        // warte auf weiter
boolean tempReached	= false;	// temperatur erreicht
boolean buzzerOn        = false;        // Wecker zustand
boolean heatingOn       = false;        // heizplatte geschaltet wird
boolean heizPlatteState = false;        // zustand heizplatte
boolean heizStateChange = false;        // flanke der heizsteuerung
boolean piezoPWM        = false;	// Piezzo mal an mal aus
boolean batLow          = false;	// Batterie ist schwach
boolean firstCall       = false;        // Braustufe erster aufruf
boolean tuning          = false;        // tuning active
volatile int rotaryCnt	= 0;	        // rotary encoder left
int rotaryCntLast	= 0;	        // rotary encoder left Last
boolean rotaryEncUsed	= false;	// rotary encoder used
int actState		= 0; 		// zustand brausteuerung
int menuState	        = 0;            // Zustand menu anzeige
int chooseState         = 0;            // Zustand ausgewaehlt
int resTemp             = 11;           // aufloesung temperatursensor
unsigned long pidPwmWindowStartTime;    // PID PWM window start time
float actTemp		= 0; 		// aktuelle Temperatur messung
double Setpoint;                        // angestrebte temperatur
double Input;                           // gemessee temperatur
double Output;                          // staerke der regelung
WaitTime timerTempMeasure(300);
WaitTime timerHeaterSwitch(5000);
WaitTime timerPiezzo(200);
WaitTime timerRasten(0);
WaitTime timerLCDBackLight(30000);
//MeasureAverage<float,3> temperaturAverage;
PID myPid(&Input, &Output, &Setpoint, 16.16, 0.14, 480.10, DIRECT);
PID_ATune aTune(&Input, &Output);

///////////////////////////////////////////////
// Braurezept
///////////////////////////////////////////////
typedef struct
{
  unsigned long  time;
  float temp;
  float dMinT;
  float dMaxT;
} Braustufe;

typedef struct
{
  Braustufe Einmaischen;
  Braustufe Ferularast;
  Braustufe Eiweisrast;
  Braustufe Maltoserast;
  Braustufe Zuckerrast;
  Braustufe Abmaischen;
  Braustufe Laeuterruhe;
  Braustufe Bitterhopfen;
  Braustufe Aromahopfen;
  Braustufe Nachkochen;
  Braustufe Whirlpool;
  byte Jodprobe;
} Rezept;

typedef struct
{
  int rezeptNr;
  byte controlerType;
  float kalM;
  float kalT;
  unsigned int WindowSize;
  int pidSampleTime;
  float DeltaPID;
  double Kp;
  double Ki;
  double Kd;
  double aTuneStep;
  double aTuneNoise;
  unsigned int aTuneLookBack;
  byte switchType;
  unsigned long  On;
  unsigned long  Off;
  byte repeats;
  unsigned int periodusec1;
  unsigned long  Adress;
  unsigned int periodusec2;
  byte unit;
  float batLowVal;
} Settings;


Settings defaultSetting =
{
  0,                     // Rezept Nummer
  0,                     // ControlerType
  1.01, 0.0,             // kalibrier M, kalibrier t
  1000.0,                // window size
  100.0,                 // PID sample time
  10.0,                  // deltaPID
  16.16, 0.14, 480.10,   // PID
  300, 1, 60000,         // PID Tune
  1,                     // switch type
  172772,                // steckdose off
  172770,                // steckdose off
  1,                     // repeats
  342,                   // periode
  1859584,               // adresse
  263,                   // periode
  1,                     // unit
  3.0   		 // bat Low Voltage 4,5 = 9V, 3.5 = 7V
};

Rezept BrauRezepte[MAX_RECIPES] = {
  {
    { 5, 50.0,  -0.1, 0.1}, // Einmaischen
    { 5, 51.0,  -0.1, 0.1}, // Ferularast
    {20, 57.0,  -0.1, 0.1}, // Eiweisrast
    {40, 63.0,  -0.1, 0.1}, // Maltoserast
    {45, 72.0,  -0.1, 0.1}, // Zuckerrast
    {35, 78.0,  -0.1, 0.1}, // Abmaischen
    {20, 0.0,   -0.1, 0.1}, // Laeuterruhe
    {10, 100.0, 0.5, 10.0}, // Bitterhopfen
    {70, 100.0, 0.5, 10.0}, // Aromahopfen
    {10, 100.0, 0.5, 10.0}, // Nachkochen
    {20, 0.0,   0.0, 0.0}, // Whirlpool
    1                      // Jodprobe
  },
  {
    {20, 50.0,  -0.1, 0.1}, // Einmaischen
    { 0,  0.0,  -0.1, 0.1}, // Ferularast
    {60, 63.0,  -0.1, 0.1}, // Eiweisrast
    { 6, 68.0,  -0.1, 0.1}, // Maltoserast
    {15, 72.0,  -0.1, 0.1}, // Zuckerrast
    {15, 76.0,  -0.1, 0.1}, // Abmaischen
    {20, 0.0,   -0.1, 0.1}, // Laeuterruhe
    {10, 100.0, 0.5, 10.0}, // Bitterhopfen
    {70, 100.0, 0.5, 10.0}, // Aromahopfen
    {10, 100.0, 0.5, 10.0}, // Nachkochen
    {20, 0.0,   0.0, 0.0}, // Whirlpool
    1                      // Jodprobe
  },
  { // Rezept zum Testen
    { 1, 50.0,  -0.1, 0.1}, // Einmaischen
    { 1, 55.0,  -0.1, 0.1}, // Ferularast
    { 1, 57.0,  -0.1, 0.1}, // Eiweisrast
    { 1, 63.0,  -0.1, 0.1}, // Maltoserast
    { 1, 72.0,  -0.1, 0.1}, // Zuckerrast
    { 1, 78.0,  -0.5, 0.1}, // Abmaischen
    { 1, 0.0,   -0.1, 0.1}, // Laeuterruhe
    { 1, 100.0, 0.5, 10.0}, // Bitterhopfen
    { 1, 100.0, 0.5, 10.0}, // Aromahopfen
    { 1, 100.0, 0.5, 10.0}, // Nachkochen
    { 1, 0.0,   0.0, 0.0}, // Whirlpool
    1                      // Jodprobe
  }
};

///////////////////////////////////////////////
// switch
///////////////////////////////////////////////

void sendCodeOld(
  int pin,
  unsigned long code,
  unsigned int periodusec,
  byte repeats)
{
  code &= 0xfffff; // Truncate to 20 bit ;
  // Convert the base3-code to base4, to avoid lengthy calculations when transmitting.. Messes op timings.
  // Also note this swaps endianess in the process. The MSB must be transmitted first, but is converted to
  // LSB here. This is easier when actually transmitting later on.
  unsigned long dataBase4 = 0;

  for (byte i = 0; i < 12; i++)
  {
    dataBase4 <<= 2;
    dataBase4 |= (code % 3);
    code /= 3;
  }

  repeats = 1 << (repeats & B111); // repeats := 2^repeats;

  for (byte j = 0; j < repeats; j++)
  {
    // Sent one telegram

    // Recycle code as working var to save memory
    code = dataBase4;
    for (byte i = 0; i < 12; i++)
    {
      switch (code & B11)
      {
        case 0:
          digitalWrite(pin, HIGH);
          delayMicroseconds(periodusec);
          digitalWrite(pin, LOW);
          delayMicroseconds(periodusec * 3);
          digitalWrite(pin, HIGH);
          delayMicroseconds(periodusec);
          digitalWrite(pin, LOW);
          delayMicroseconds(periodusec * 3);
          break;
        case 1:
          digitalWrite(pin, HIGH);
          delayMicroseconds(periodusec * 3);
          digitalWrite(pin, LOW);
          delayMicroseconds(periodusec);
          digitalWrite(pin, HIGH);
          delayMicroseconds(periodusec * 3);
          digitalWrite(pin, LOW);
          delayMicroseconds(periodusec);
          break;
        case 2: // KA: X or float
          digitalWrite(pin, HIGH);
          delayMicroseconds(periodusec);
          digitalWrite(pin, LOW);
          delayMicroseconds(periodusec * 3);
          digitalWrite(pin, HIGH);
          delayMicroseconds(periodusec * 3);
          digitalWrite(pin, LOW);
          delayMicroseconds(periodusec);
          break;
      }
      // Next trit
      code >>= 2;
    }

    // Send termination/synchronization-signal. Total length: 32 periods
    digitalWrite(pin, HIGH);
    delayMicroseconds(periodusec);
    digitalWrite(pin, LOW);
    delayMicroseconds(periodusec * 31);
  }
}

void sendCodeNewBit(
  int pin,
  unsigned int periodusec,
  boolean isBitOne)
{
  if (isBitOne)
  {
    // Send '1'
    digitalWrite(pin, HIGH);
    delayMicroseconds(periodusec);
    digitalWrite(pin, LOW);
    delayMicroseconds(periodusec * 5);
    digitalWrite(pin, HIGH);
    delayMicroseconds(periodusec);
    digitalWrite(pin, LOW);
    delayMicroseconds(periodusec);
  }
  else
  {
    // Send '0'
    digitalWrite(pin, HIGH);
    delayMicroseconds(periodusec);
    digitalWrite(pin, LOW);
    delayMicroseconds(periodusec);
    digitalWrite(pin, HIGH);
    delayMicroseconds(periodusec);
    digitalWrite(pin, LOW);
    delayMicroseconds(periodusec * 5);
  }
}

void sendCodeNew(
  int pin,
  unsigned long address,
  unsigned int periodusec,
  byte repeats,
  byte unit,
  boolean switchOn)
{
  for (int8_t i = repeats; i >= 0; i--)
  {
    unsigned long tempAdr = address;
    //start
    digitalWrite(pin, HIGH);
    delayMicroseconds(periodusec);
    digitalWrite(pin, LOW);
    // Actually 10.5T insteat of 10.44T. Close enough.
    delayMicroseconds(periodusec * 10 + (periodusec >> 1));

    // adresse
    for (int8_t i = 25; i >= 0; i--)
    {
      sendCodeNewBit(
        pin,
        periodusec,
        (tempAdr >> i) & 1 );
    }

    // No group bit
    sendCodeNewBit(
      pin,
      periodusec,
      false );

    // Switch on | off
    sendCodeNewBit(
      pin,
      periodusec,
      switchOn );

    //unit
    for (int8_t i = 3; i >= 0; i--)
    {
      sendCodeNewBit(
        pin,
        periodusec,
        unit & 1 << i );
    }

    //stop
    digitalWrite(pin, HIGH);
    delayMicroseconds(periodusec);
    digitalWrite(pin, LOW);
    delayMicroseconds(periodusec * 40);
  }
}

void sendRemote(bool onOff)
{
  if ( 0 == defaultSetting.switchType )
  {
    sendCodeOld(
      pinHeizplatteOnOff,
      onOff ? defaultSetting.On : defaultSetting.Off,
      defaultSetting.periodusec1,
      defaultSetting.repeats);
  }
  else
  {

    sendCodeNew(
      pinHeizplatteOnOff,
      defaultSetting.Adress,
      defaultSetting.periodusec2,
      defaultSetting.repeats,
      defaultSetting.unit,
      onOff);
  }
}

///////////////////////////////////////////////
// wecker
///////////////////////////////////////////////

void BuzzerOnOff(boolean onOff)
{
  if ( true == onOff )
  {
    analogWrite (pinBuzzer, 180);
  }
  else
  {
    analogWrite (pinBuzzer, 0);
  }
}

///////////////////////////////////////////////
// batterie spannung
///////////////////////////////////////////////
float getVoltage()
{
  int val = analogRead(pinBatLow);
  return (float)val * ( 4.7 / 1024.0 );
}

void checkBatterieVoltage()
{
  float val = getVoltage();
  if ( val < defaultSetting.batLowVal )
  {
    batLow = true;
  }
  else
    batLow = false;
}

///////////////////////////////////////////////
// tastatur eingaben
///////////////////////////////////////////////

boolean isWeiterPressed(boolean text, boolean buzzer)
{
  if ( false ==  weiterRequest )
    weiterRequest = true;

  int val = Keypad();
    
  if ( val == btnRIGHT )
  {
    if ( weiterPressed == false )
    {
      if ( true == buzzer )
        buzzerOn = false;
      weiterPressed = true;
      delay(200);
    }
  }
  else
  {
    weiterPressed = false;
    buzzerOn = buzzer;
  }

  if ( text ) printAccept();
  return weiterPressed;
}
void resetPressed()
{
  if ( false ==  weiterRequest )
  {
    int val = Keypad();
    if ( val == btnRIGHT )
    {
      actState = MAX_STATES + 1;
      delay(200);
    }
  }
}
boolean nextStatePressed()
{
  if ( nextPlusPressed() )
  {
    chooseState = chooseState + 1;
    if ( chooseState > MAX_STATES ) chooseState = 0;
    return true;
  }
  else if ( nextMinusPressed() )
  {
    if ( 0 == chooseState )
      chooseState = MAX_STATES;
    else
      chooseState = chooseState - 1;
    return true;
  }
  return false;
}
boolean nextPlusPressed()
{
  int key = Keypad();
  if ( key == btnSELECT ||
       ( rotaryCnt - rotaryCntLast > 0) ) 
  {
    rotaryCntLast = rotaryCnt;
    delay(100);
    return true;
  }
  return false;
}
boolean nextMinusPressed()
{
  int key = Keypad();
  if ( key == btnLEFT ||
       ( rotaryCnt - rotaryCntLast < 0) )
  {
    rotaryCntLast = rotaryCnt;
    delay(100);
    return true;
  }
  return false;
}

void doEncoder() 
{
  if (digitalRead(pinEncoderA) == 
      digitalRead(pinEncoderB) ) 
    rotaryCnt--;
  else 
    rotaryCnt++;
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
    x = analogRead(pinKeyPad);
    y = analogRead(pinRotSwitch);
    if (x < 60  || 
        y < 60)       val = btnRIGHT;
    else if (x < 200) val = btnUP;
    else if (x < 400) val = btnDOWN;
    else if (x < 600) val = btnLEFT;
    else if (x < 800) val = btnSELECT;
    else              val = 0;
    delay(10);
  }
  while ( val != val_old );

  return val;
}

///////////////////////////////////////////////
// temperatur device
///////////////////////////////////////////////
void setupDS18B20()
{
  sensors.begin();
  sensors.setResolution(tempDeviceAddress, resTemp);
  sensors.setWaitForConversion(false); // async mode
  sensors.requestTemperatures();
}

// holt temperaturwert digital
float getTemperaturDigital()
{
  float temperatur = sensors.getTempCByIndex(0);
  sensors.requestTemperatures(); // erneut wert anfragen
  temperatur = ( temperatur ) * defaultSetting.kalM + defaultSetting.kalT;
  return temperatur;
}


///////////////////////////////////////////////
// LCD Backlight
///////////////////////////////////////////////
void setLcdBacklight(boolean on)
{
  if ( on )
  {
    //lcd.display();
    digitalWrite(pinLcdLed, HIGH);
  }
  else
  {
    //lcd.noDisplay();
    digitalWrite(pinLcdLed, LOW);
  }
}
///////////////////////////////////////////////
// temperatur controller
///////////////////////////////////////////////
// heizplatte ein bzw ausschalten
void heizplatteOnOff()
{
  if ( true  == heatingOn &&
       false == batLow )
  {
    sendRemote(true);
    heizPlatteState = true;
  }
  else
  {
    sendRemote(false);
    heizPlatteState = false;
  }
}
boolean heatingStateChange(boolean heaterGetOn)
{
  if ( heatingOn != heaterGetOn ) heizStateChange = true;
  return heaterGetOn;
}
void heatingControllerZweipunkt(float temperatur, float min, float max)
{
  // zweipunktregler mit hysterese
  if ( heizPlatteState )
  {
    if ( actTemp > ( temperatur + max ) )
    {
      heatingOn = heatingStateChange(false);
    }
    else
    {
      heatingOn = heatingStateChange(true);
    }
  }
  else
  {
    if ( actTemp < ( temperatur + min ) )
    {
      heatingOn = heatingStateChange(true);
    }
    else
    {
      heatingOn = heatingStateChange(false);
    }
  }
}

void setPID()
{
  myPid.SetSampleTime(defaultSetting.pidSampleTime);
  myPid.SetTunings(
    defaultSetting.Kp,
    defaultSetting.Ki,
    defaultSetting.Kd);
  myPid.SetOutputLimits(0.0, defaultSetting.WindowSize);
  myPid.SetMode(AUTOMATIC);
}

void heatingControllerPID (float temperatur)
{
  Input    = actTemp;
  Setpoint = temperatur;

  unsigned long now = millis();
  unsigned long diff = now - pidPwmWindowStartTime;

  if ( tuning )
  {
    lcd.setCursor(0, 1);
    lcd.print(F("O:"));
    lcd.print(Output);
  }
  else
  {
    if ( (Setpoint - Input) < defaultSetting.DeltaPID )
    {
      myPid.Compute();
    }
    else
    {
      Output = defaultSetting.WindowSize;
    }
  }

  if ( diff > defaultSetting.WindowSize )
  {
    pidPwmWindowStartTime += defaultSetting.WindowSize;
    diff = 0;
  }

  if ( Output >= diff )
    heatingOn = heatingStateChange(true);
  else
    heatingOn = heatingStateChange(false);
}

boolean aufHeizen( float temp, float min, float max )
{
  if ( false == firstCall )
  {
    firstCall = true;
  }

  if ( 0 == defaultSetting.controlerType )
    heatingControllerZweipunkt( temp, min, max );
  else
    heatingControllerPID(temp);

  if ( tempReached == false &&
       actTemp     >=  temp )
  {
    tempReached = true;
  }

  printTemp(temp);

  return tempReached;
}

boolean isTimerReached(unsigned long duration)
{
  timerRasten.start(duration);
  if (timerRasten.timeOver())
  {
    return true;
  }
  else
  {
    printTimer(timerRasten.getDuration() / MIL2SEC);
  }
  return false;
}

boolean brausStufe(unsigned long time, float temp, float min, float max)
{
  if ( aufHeizen( temp, min, max ) )
  {
    if ( isTimerReached ( time * MIL2SEC * SECPROMIN ) )
    {
      return true;
    }
  }
  return false;
}

///////////////////////////////////////////////
// System
///////////////////////////////////////////////

void Reset()
{
  tempReached     = false;
  weiterPressed   = false;
  weiterRequest   = false;
  heatingOn       = false;
  buzzerOn        = false;
  heizPlatteState = false;
  batLow          = false;
  piezoPWM        = false;
  tuning          = false;
  actState 	  = 0;
  menuState	  = 0;
  chooseState     = 0;
  aTune.Cancel();

  setLcdBacklight(true);
  printLogo();
  delay(2000);
  lcd.clear();

  actTemp = getTemperaturDigital();

  timerRasten.init();
  heizplatteOnOff();
  setPID();
  BuzzerOnOff(piezoPWM);
  checkBatterieVoltage();
}

void nextState()
{
  if ( 0 != chooseState && 0 == actState)
  {
    actState = chooseState;
  }
  else
  {
    actState = actState + 1;
  }

  tempReached   = false;
  tuning        = false;
  firstCall     = false;
  weiterPressed = false;
  weiterRequest = false;
  timerRasten.init();
  lcd.clear();
  pidPwmWindowStartTime = millis();
  saveState(actState);
}

///////////////////////////////////////////////
// LCD ausgaben
///////////////////////////////////////////////
void setupDisplay()
{
  lcd.begin(16, 2);
}

void printTemp(float sollTemp)
{
  if ( true == batLow )
  {
    lcd.setCursor(0, 0);
    lcd.print("BatLow          ");
  }
  else
  {
    lcd.setCursor(0, 0);
    lcd.print(actState);
    lcd.print(F(" I"));
    lcd.print(actTemp);
    lcd.print(F(" S"));
    lcd.print(sollTemp);
  }
}

void printTimer(unsigned long countDown)
{
  lcd.setCursor(0, 1);
  lcd.print(F("Restzeit:      "));
  lcd.setCursor(9, 1);
  lcd.print(countDown);
}

void printLogo()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("mikroSikaru.de"));
}

void printState(int  state)
{
  lcd.setCursor(0, 0);
  lcd.print(F("  "));
  lcd.setCursor(0, 0);
  lcd.print(chooseState);
  lcd.setCursor(3, 0);
  switch (state)
  {
    case 0:    lcd.print(F("Setup       ")); break;
    case 1:    lcd.print(F("Aufheizen   ")); break;
    case 2:    lcd.print(F("Einmaischen ")); break;
    case 3:    lcd.print(F("Ferularast  ")); break;
    case 4:    lcd.print(F("Eiweisrast  ")); break;
    case 5:    lcd.print(F("Maltoserast ")); break;
    case 6:    lcd.print(F("Zuckerrast  ")); break;
    case 7:    lcd.print(F("Jodprobe    ")); break;
    case 8:    lcd.print(F("Abmaischen  ")); break;
    case 9:    lcd.print(F("Laeuterruhe ")); break;
    case 10:   lcd.print(F("Laeutern    ")); break;
    case 11:   lcd.print(F("Bitterhopfen")); break;
    case 12:   lcd.print(F("Aromahopfen ")); break;
    case 13:   lcd.print(F("Nachkochen  ")); break;
    case 14:   lcd.print(F("Whirlpool   ")); break;
    case 15:   lcd.print(F("Auto Tune   ")); break;
  }
}

void printMenu(String text, long val)
{
  lcd.setCursor(0, 0);
  lcd.print(text);
  lcd.setCursor(0, 1);
  lcd.print(F("          "));
  lcd.setCursor(0, 1);
  lcd.print(val);
}
void printMenuFloat(String text, float val)
{
  lcd.setCursor(0, 0);
  lcd.print(text);
  lcd.setCursor(0, 1);
  lcd.print(F("          "));
  lcd.setCursor(0, 1);
  lcd.print(val, 2);
}

void printAccept()
{
  lcd.setCursor(0, 1);
  lcd.print(F("Taste S druecken"));
}

///////////////////////////////////////////////
// Menu
///////////////////////////////////////////////
long menuInputVal(int state, String text, long val, long min, long max)
{
  printMenu(text, val);
  if ( nextPlusPressed() )
  {
    val += 1;
    if ( min != max && val > max ) val = max;
  }
  else if ( nextMinusPressed() )
  {
    val -= 1;
    if ( min != max && val < min ) val = min;
  }
  else if ( isWeiterPressed(false, false) )
  {
    lcd.clear();
    menuState++;
  }
  return val;
}
float menuInputValFloat(int state, String text, float val, float addSub)
{
  printMenuFloat(text, val);
  if ( nextPlusPressed() )       val += addSub;
  else if ( nextMinusPressed() ) val -= addSub;
  else if ( isWeiterPressed(false, false) )
  {
    lcd.clear();
    menuState++;
  }
  return val;
}
void menu()
{
  long val;
  float valf;
  switch ( menuState )
  {
    case 0:
      {
        printState(chooseState);
        nextStatePressed();
        // if ( nextMenuPressed() )
        if ( isWeiterPressed(true, false) )
        {
          lcd.clear();
          if ( 0 == chooseState )
            menuState++;
          else
            nextState();
        }
        break;
      }
    case 1:
      {
        val = (long) ( defaultSetting.rezeptNr);
        val = menuInputVal(0, F("Braurezept:  "), val, 0, MAX_RECIPES - 1);
        defaultSetting.rezeptNr = val;
        break;
      }
    case 2:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Einmaischen.time );
        val = menuInputVal(0, F("Einmaischen d:  "), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Einmaischen.time = val;
        break;
      }
    case 3:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Einmaischen.temp );
        val = menuInputVal(0, F("Einmaischen T:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Einmaischen.temp = (float)val;
        break;
      }
    case 4:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMinT;
        valf = menuInputValFloat(0, F("Einmaischen min:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMinT = valf;
        break;
      }
    case 5:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMaxT;
        valf = menuInputValFloat(0, F("Einmaischen max:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMaxT = valf;
        break;
      }
    case 6:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Ferularast.time );
        val = menuInputVal(0, F("Ferularast d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Ferularast.time = val;
        break;
      }
    case 7:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Ferularast.temp );
        val = menuInputVal(0, F("Ferularast T:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Ferularast.temp = (float)val;
        break;
      }
    case 8:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Ferularast.dMinT;
        valf = menuInputValFloat(0, F("Ferularast min:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Ferularast.dMinT = valf;
        break;
      }
    case 9:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Ferularast.dMaxT;
        valf = menuInputValFloat(0, F("Ferularast max:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Ferularast.dMaxT = valf;
        break;
      }
    case 10:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.time );
        val = menuInputVal(0, F("Eiweisrast d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.time = val;
        break;
      }
    case 11:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.temp );
        val = menuInputVal(0, F("Eiweisrast T:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.temp = (float)val;
        break;
      }
    case 12:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.dMinT;
        valf = menuInputValFloat(0, F("Eiweisrast min:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.dMinT = valf;
        break;
      }
    case 13:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.dMaxT;
        valf = menuInputValFloat(0, F("Eiweisrast max:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.dMaxT = valf;
        break;
      }
    case 14:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Maltoserast.time );
        val = menuInputVal(0, F("Maltoserast d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Maltoserast.time = val;
        break;
      }
    case 15:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Maltoserast.temp );
        val = menuInputVal(0, F("Maltoserast T:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Maltoserast.temp = (float)val;
        break;
      }
    case 16:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Maltoserast.dMinT;
        valf = menuInputValFloat(0, F("Maltoserast min:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Maltoserast.dMinT = valf;
        break;
      }
    case 17:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Maltoserast.dMaxT;
        valf = menuInputValFloat(0, F("Maltoserast max:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Maltoserast.dMaxT = valf;
        break;
      }
    case 18:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.time );
        val = menuInputVal(0, F("Zuckerrast d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.time = val;
        break;
      }
    case 19:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.temp );
        val = menuInputVal(0, F("Zuckerrast T:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.temp = (float)val;
        break;
      }
    case 20:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.dMinT;
        valf = menuInputValFloat(0, F("Zuckerrast min:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.dMinT = valf;
        break;
      }
    case 21:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.dMaxT;
        valf = menuInputValFloat(0, F("Zuckerrast max:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.dMaxT = valf;
        break;
      }
    case 22:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Jodprobe );
        val = menuInputVal(0, F("Jodprobe d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Jodprobe = (byte)val;
        break;
      }
    case 23:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Abmaischen.time );
        val = menuInputVal(0, F("Abmaischen d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Abmaischen.time = val;
        break;
      }
    case 24:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Abmaischen.temp );
        val = menuInputVal(0, F("Abmaischen T:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Abmaischen.temp = (float)val;
        break;
      }
    case 25:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Abmaischen.dMinT;
        valf = menuInputValFloat(0, F("Abmaischen min:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Abmaischen.dMinT = valf;
        break;
      }
    case 26:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Abmaischen.dMaxT;
        valf = menuInputValFloat(0, F("Abmaischen max:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Abmaischen.dMaxT = valf;
        break;
      }
    case 27:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.time );
        val = menuInputVal(0, F("Laeuterruhe d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.time = val;
        break;
      }
    case 28:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.temp );
        val = menuInputVal(0, F("Laeuterruhe T:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.temp = (float)val;
        break;
      }
    case 29:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.dMinT;
        valf = menuInputValFloat(0, F("Laeuterruhe min:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.dMinT = valf;
        break;
      }
    case 30:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.dMaxT;
        valf = menuInputValFloat(0, F("Laeuterruhe max:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.dMaxT = valf;
        break;
      }
    case 31:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.time );
        val = menuInputVal(0, F("Bitterhopfen d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.time = val;
        break;
      }
    case 32:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.time );
        val = menuInputVal(0, F("Aromahopfen d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.time = val;
        break;
      }
    case 33:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Nachkochen.time );
        val = menuInputVal(0, F("Nachkochen d:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Nachkochen.time = val ;
        break;
      }
    case 34:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Nachkochen.temp );
        val = menuInputVal(0, F("Kochen T:"), val, 0, 0);
        BrauRezepte[defaultSetting.rezeptNr].Nachkochen.temp   = (float)val;
        BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.temp = (float)val;
        BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.temp  = (float)val;
        break;
      }
    case 35:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Nachkochen.dMinT;
        valf = menuInputValFloat(0, F("Kochen min:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Nachkochen.dMinT   = valf;
        BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.dMinT = valf;
        BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.dMinT  = valf;
        break;
      }
    case 36:
      {
        valf = BrauRezepte[defaultSetting.rezeptNr].Nachkochen.dMaxT;
        valf = menuInputValFloat(0, F("Kochen max:"), valf, 0.01);
        BrauRezepte[defaultSetting.rezeptNr].Nachkochen.dMaxT   = valf;
        BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.dMaxT = valf;
        BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.dMaxT  = valf;
        break;
      }
    case 37:
      {
        val = (long) ( BrauRezepte[defaultSetting.rezeptNr].Whirlpool.time );
        val = menuInputVal(0, F("Whirlpool d:"), val, 0, 1);
        BrauRezepte[defaultSetting.rezeptNr].Whirlpool.time = val ;
        break;
      }
    case 38:
      {
        valf = defaultSetting.kalM;
        valf = menuInputValFloat(0, F("kal m:"), valf, 0.01);
        defaultSetting.kalM = valf;
        break;
      }
    case 39:
      {
        valf = defaultSetting.kalT;
        valf = menuInputValFloat(0, F("kal T:"), valf, 0.01);
        defaultSetting.kalT = valf;
        break;
      }
    case 40:
      {
        val = (long)defaultSetting.switchType ;
        val = menuInputVal(0, F("Schalter Typ:"), val, 0, 1);
        defaultSetting.switchType = (byte)val;
        break;
      }
    case 41:
      {
        if ( 0 == defaultSetting.switchType )
        {
          val = defaultSetting.On;
          val = menuInputVal(0, F("On Code:"), val, 0, 0);
          defaultSetting.On = val;
        }
        else
        {
          val = (long) defaultSetting.Adress;
          val = menuInputVal(0, F("Adresse:"), val, 0, 0);
          defaultSetting.Adress = (unsigned long)val;
        }
        break;
      }
    case 42:
      {
        if ( 0 == defaultSetting.switchType )
        {
          val = defaultSetting.Off;
          val = menuInputVal(0, F("Off Code:"), val, 0, 0);
          defaultSetting.Off = val;
        }
        else
        {
          val = (long)defaultSetting.unit;
          val = menuInputVal(0, F("Unit:"), val, 0, 10);
          defaultSetting.unit = (long)val;
        }
        break;
      }
    case 43:
      {
        if ( 0 == defaultSetting.switchType )
        {
          val = defaultSetting.periodusec1;
          val = menuInputVal(0, F("Periode:"), val, 0, 10000);
          defaultSetting.periodusec1 = val;
        }
        else
        {
          val = (long)defaultSetting.periodusec2;
          val = menuInputVal(0, F("Periode:"), val, 0, 10000);
          defaultSetting.periodusec2 = (unsigned int)val;
        }
        break;
      }
    case 44:
      {
        val = (long)defaultSetting.repeats;
        val = menuInputVal(0, F("Wiederholung:"), val, 0, 100);
        defaultSetting.repeats = (byte)val;
        break;
      }
    case 45:
      {
        val = (long)defaultSetting.controlerType;
        val = menuInputVal(0, F("PID:"), val, 0, 1);
        defaultSetting.controlerType = (byte)val;
        break;
      }
    case 46:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          val = (long)defaultSetting.WindowSize;
          val = menuInputVal(0, F("PWM Laenge:"), val, 0, 0);
          defaultSetting.WindowSize = (unsigned int)val;
        } else menuState++;
        break;
      }
    case 47:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          val = (long)defaultSetting.pidSampleTime;
          val = menuInputVal(0, F("Sample Zeit:"), val, 0, 0);
          defaultSetting.pidSampleTime = (int)val;
        } else menuState++;
        break;
      }
    case 48:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          valf = defaultSetting.DeltaPID;
          valf = menuInputValFloat(0, F("Delta:"), valf, 0.1);
          defaultSetting.DeltaPID = valf;
        } else menuState++;
        break;
      }
    case 49:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          valf = (float)defaultSetting.Kp;
          valf = menuInputValFloat(0, F("Kp:"), valf, 0.01);
          defaultSetting.Kp = (double)valf;
        } else menuState++;
        break;
      }
    case 50:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          valf = (float)defaultSetting.Ki;
          valf = menuInputValFloat(0, F("Ki:"), valf, 0.01);
          defaultSetting.Ki = (double)valf;
        } else menuState++;
        break;
      }
    case 51:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          valf = (float)defaultSetting.Kd;
          valf = menuInputValFloat(0, F("Kd:"), valf, 0.01);
          defaultSetting.Kd = (double)valf;
        } else menuState++;
        break;
      }
    case 52:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          valf = (float)defaultSetting.aTuneStep;
          valf = menuInputValFloat(0, F("Tune Step:"), valf, 1);
          defaultSetting.aTuneStep = (double)valf;
        } else menuState++;
        break;
      }
    case 53:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          valf = (float)defaultSetting.aTuneNoise;
          valf = menuInputValFloat(0, F("Tune Noise:"), valf, 0.01);
          defaultSetting.aTuneNoise = (double)valf;
        } else menuState++;
        break;
      }
    case 54:
      {
        if ( 0 != defaultSetting.controlerType )
        {
          val = (long)defaultSetting.aTuneLookBack;
          val = menuInputVal(0, F("Tune Look Back:"), val, 0, 0);
          defaultSetting.aTuneLookBack = (unsigned int)val;
        } else menuState++;
        break;
      }
    case 55:
      {
        valf = defaultSetting.batLowVal;
        valf = menuInputValFloat(0, F("BatLow:"), valf, 0.1);
        defaultSetting.batLowVal = valf;
        break;
      }
    default:
      {
        saveConfig();
        lcd.clear();
        setPID();
        menuState = 0;
      }
  }
}
///////////////////////////////////////////////
// brauablauf
///////////////////////////////////////////////
void brew()
{
  if ( 0 != actState )
  {
    timerTempMeasure.start();
    if ( timerTempMeasure.timeOver() )
    {
      timerTempMeasure.init();
      //      actTemp = temperaturAverage.add(getTemperaturDigital());
      actTemp = getTemperaturDigital();
      checkBatterieVoltage();
      BuzzerOnOff(batLow);
    }

    timerHeaterSwitch.start();
    if ( heizStateChange || timerHeaterSwitch.timeOver() )
    {
      timerHeaterSwitch.init();
      heizStateChange = false;
      heizplatteOnOff();
    }

    timerLCDBackLight.start();
    if ( timerLCDBackLight.timeOver() )
    {
      timerLCDBackLight.init();
      setLcdBacklight(false);
      saveState(actState);
    }
    if ( nextStatePressed() )
    {
      setLcdBacklight(true);
      timerLCDBackLight.init();
    }

    timerPiezzo.start();
    if ( timerPiezzo.timeOver() )
    {
      timerPiezzo.init();
      if ( true == buzzerOn )
      {
        piezoPWM = piezoPWM ? false : true;
      }
      else
      {
        piezoPWM = false;
      }
      BuzzerOnOff(piezoPWM);
    }

    resetPressed();
  }
  switch ( actState )
  {
    case 0: // warte auf start
      {
        menu();
        break;
      }
    case 1: // aufheizen
      {
        if ( aufHeizen ( BrauRezepte[defaultSetting.rezeptNr].Einmaischen.temp,
                         BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMinT,
                         BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMaxT ) )
        {
          if ( 0 != BrauRezepte[defaultSetting.rezeptNr].Einmaischen.time )
          {
            if ( isWeiterPressed(true, true) )
            {
              nextState();
            }
          }
          else nextState();
        }
        break;
      }
    case 2: // Einmaischen
      {
        if ( false == firstCall )
        {
          firstCall = true;
        }
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Einmaischen.time,
                          BrauRezepte[defaultSetting.rezeptNr].Einmaischen.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Einmaischen.time )
        {
          nextState();
        }
        break;
      }
    case 3: // Ferularast
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Ferularast.time,
                          BrauRezepte[defaultSetting.rezeptNr].Ferularast.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Ferularast.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Ferularast.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Ferularast.time )
        {
          nextState();
        }
        break;
      }
    case 4: // Eiweisrast
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.time,
                          BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Eiweisrast.time )
        {
          nextState();
        }
        break;
      }
    case 5: // Maltoserast
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Maltoserast.time,
                          BrauRezepte[defaultSetting.rezeptNr].Maltoserast.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Maltoserast.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Maltoserast.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Maltoserast.time )
        {
          nextState();
        }
        break;
      }
    case 6: // Zuckerrast
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.time,
                          BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.time )
        {
          if ( 1 == BrauRezepte[defaultSetting.rezeptNr].Jodprobe )
          {
            if ( isWeiterPressed(true, true) )
            {
              nextState();
            }
          }
          else nextState();
        }
        break;
      }
    case 7: // Jodprobe nehmen
      {
        aufHeizen(BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.temp,
                  BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.dMinT,
                  BrauRezepte[defaultSetting.rezeptNr].Zuckerrast.dMaxT);
        if ( 1 == BrauRezepte[defaultSetting.rezeptNr].Jodprobe )
        {
          if ( isWeiterPressed(true, false) )
          {
            nextState();
          }
        }
        else nextState();

        break;
      }
    case 8: // Abmaischen
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Abmaischen.time,
                          BrauRezepte[defaultSetting.rezeptNr].Abmaischen.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Abmaischen.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Abmaischen.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Abmaischen.time )
        {
          if ( isWeiterPressed(true, true) )
          {
            nextState();
          }
        }
        break;
      }
    case 9: // Laeuterruhe
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.time,
                          BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.time )
        {
          if ( isWeiterPressed(true, true) )
          {
            nextState();
          }
        }
        break;
      }
    case 10: // Laeutern
      {
        aufHeizen(BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.temp,
                  BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.dMinT,
                  BrauRezepte[defaultSetting.rezeptNr].Laeuterruhe.dMaxT);
        if ( isWeiterPressed(true, false) )
        {
          nextState();
        }
        break;
      }
    case 11: // Bitterhopfen
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.time,
                          BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.time )
        {
          if ( 0 == BrauRezepte[defaultSetting.rezeptNr].Bitterhopfen.time )
          {
            nextState();
          }
          else if ( isWeiterPressed(true, true) )
          {
            nextState();
          }
        }
        break;
      }
    case 12: // Aromahopfen
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.time,
                          BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.time)
        {
          if ( 0 == BrauRezepte[defaultSetting.rezeptNr].Aromahopfen.time )
          {
            nextState();
          }
          else if ( isWeiterPressed(true, true) )
          {
            nextState();
          }
        }
        break;
      }
    case 13: // kochen bis ende
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Nachkochen.time,
                          BrauRezepte[defaultSetting.rezeptNr].Nachkochen.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Nachkochen.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Nachkochen.dMaxT ) ||
             0 == BrauRezepte[defaultSetting.rezeptNr].Nachkochen.time )
        {
          if ( 0 == BrauRezepte[defaultSetting.rezeptNr].Nachkochen.time )
          {
            nextState();
          }
          else if ( isWeiterPressed(true, true) )
          {
            nextState();
          }
        }
        break;
      }
    case 14: // whirlpool
      {
        if ( brausStufe ( BrauRezepte[defaultSetting.rezeptNr].Whirlpool.time,
                          BrauRezepte[defaultSetting.rezeptNr].Whirlpool.temp,
                          BrauRezepte[defaultSetting.rezeptNr].Whirlpool.dMinT,
                          BrauRezepte[defaultSetting.rezeptNr].Whirlpool.dMaxT ) )
        {
          if ( 0 == BrauRezepte[defaultSetting.rezeptNr].Whirlpool.time )
          {
            nextState();
          }
          else if ( isWeiterPressed(true, true) )
          {
            nextState();
            actState++;
          }
        }
        break;
      }
    case 15: // auto tune
      {
        if ( 0 == defaultSetting.controlerType ) 
        	nextState();
        	
        if ( false == firstCall )
        {
          aTune.SetControlType(1); // PID
          aTune.SetNoiseBand(defaultSetting.aTuneNoise);
          aTune.SetOutputStep(defaultSetting.aTuneStep);
          aTune.SetLookbackSec((int)defaultSetting.aTuneLookBack);
          Output = defaultSetting.WindowSize;
          firstCall = true;
          tuning = false;
        }
        if ( aufHeizen(BrauRezepte[defaultSetting.rezeptNr].Einmaischen.temp,
                       BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMinT,
                       BrauRezepte[defaultSetting.rezeptNr].Einmaischen.dMaxT) )
        {
          tuning = true;
          if ( 0 != (aTune.Runtime() ) )
          {
            aTune.Cancel();
            defaultSetting.Kp = aTune.GetKp();
            defaultSetting.Ki = aTune.GetKi();
            defaultSetting.Kd = aTune.GetKd();
            tuning = false;
            nextState();
          }
        }
        break;
      }
    default:
      {
        saveState(0);
        Reset();
      }
  }

}

///////////////////////////////////////////////
// EEPROM store
///////////////////////////////////////////////
void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four  = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two   = ((value >> 16) & 0xFF);
  byte one   = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}
long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four  = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two   = EEPROM.read(address + 2);
  long one   = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four  << 0)  & 0xFF) +
         ((three << 8)  & 0xFFFF) +
         ((two   << 16) & 0xFFFFFF) +
         ((one   << 24) & 0xFFFFFFFF);
}
void loadConfig()
{
  for (unsigned int t = 0; t < sizeof(defaultSetting); t++)
  {
    *((char*)&defaultSetting + t) = EEPROM.read(EEPROM_HEADER_SIZE + t);
  }
  for (unsigned int t = 0; t < sizeof(BrauRezepte); t++)
  {
    *((char*)&BrauRezepte + t) = EEPROM.read(
                                   EEPROM_HEADER_SIZE + sizeof(defaultSetting) + t);
  }
}
void saveConfig()
{
  EEPROMWritelong(EEPROM_HEADER_POS, EEPROM_HEADER_DATA);
  for (unsigned int t = 0; t < sizeof(defaultSetting); t++)
    EEPROM.write(EEPROM_HEADER_SIZE + t, *((char*)&defaultSetting + t));
  for (unsigned int t = 0; t < sizeof(BrauRezepte); t++)
    EEPROM.write(EEPROM_HEADER_SIZE + sizeof(defaultSetting) + t, *((char*)&BrauRezepte + t));
}
void loadDataEEPROM()
{
  long head = EEPROMReadlong(EEPROM_HEADER_POS);
  if ( EEPROM_HEADER_DATA == head )
  {
    loadConfig();
  }
  else
  {
    saveState(0);
  }
}
void saveState(int state)
{
  EEPROMWritelong(
    EEPROM_HEADER_POS +
    EEPROM_HEADER_SIZE +
    sizeof(defaultSetting) +
    sizeof(BrauRezepte),
    (long)state);
  unsigned long lastDuration =
    (0 == state) ? 0 : (long)timerRasten.getDuration();
  EEPROMWritelong(
    EEPROM_HEADER_POS +
    EEPROM_HEADER_SIZE +
    sizeof(defaultSetting) +
    sizeof(BrauRezepte) +
    sizeof(long),
    lastDuration
  );
}
void loadState()
{
  actState = (int)EEPROMReadlong(
               EEPROM_HEADER_POS +
               EEPROM_HEADER_SIZE +
               sizeof(defaultSetting) +
               sizeof(BrauRezepte));
  unsigned long lastDuration = (unsigned long)EEPROMReadlong(
                                 EEPROM_HEADER_POS +
                                 EEPROM_HEADER_SIZE +
                                 sizeof(defaultSetting) +
                                 sizeof(BrauRezepte) +
                                 sizeof(long));
  timerRasten.init( lastDuration );
}
void ResetEEPROM()
{
  EEPROMWritelong(EEPROM_HEADER_POS, 0);
  EEPROMWritelong(
    EEPROM_HEADER_POS +
    EEPROM_HEADER_SIZE +
    sizeof(defaultSetting) +
    sizeof(BrauRezepte),
    (long)0);
}
///////////////////////////////////////////////
// Arduino special functions
///////////////////////////////////////////////
// system setup
void setup()
{
  pinMode(pinTemperaturDigit,   INPUT);
  pinMode(pinBatLow,		INPUT);
  pinMode(pinHeizplatteOnOff,   OUTPUT);
  pinMode(pinBuzzer, 		OUTPUT);
  pinMode(pinLcdLed,		OUTPUT);
  pinMode(pinEncoderA, INPUT); 
  digitalWrite(pinEncoderA, HIGH);       // turn on pullup resistor
  pinMode(pinEncoderB, INPUT); 
  digitalWrite(pinEncoderB, HIGH);       // turn on pullup resistor
  attachInterrupt(pinEncoderA-2, doEncoder, CHANGE);
  
  setupDS18B20();
  setupDisplay();
  loadDataEEPROM();
  Reset();
  loadState();
}
void loop ()
{
  brew();
}
///////////////////////////////////////////////

