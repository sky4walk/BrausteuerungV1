#include <LiquidCrystal.h>
#define PINKEY              0
#define MAXBUTTONS          6
#define PINLCD              10
#define BUFLEN              32
#define LCDSIZE             16
#define MannPOS             5
#define btnNONE             0
#define btnRIGHT            1
#define btnUP               2
#define btnDOWN             3
#define btnLEFT             4
#define btnSELECT           5
LiquidCrystal     lcd(8, 9, 4, 5, 6, 7);

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

  return val;
}
//                                   1         2         3
//                         01234567890123456789012345678901
char backBuffer[BUFLEN] = "abcdefghijklmnopqrstuvwxyzABCDEF";
char showBuffer[LCDSIZE+1];
int pos = 0;

void generiereLandschaft(int pos)
{
  for ( int i = 0; i < LCDSIZE; i+=2)
  {
    int bufPos = pos+i;
    int rNr = random(3);
    switch(rNr)
    {
      case 0:
        backBuffer[bufPos]   = '  ';
        backBuffer[bufPos+1] = '  ';
        break;
      case 1:
        backBuffer[bufPos]   = '  ';
        backBuffer[bufPos+1] = ' O';
        break;
      case 2:
        backBuffer[bufPos]   = '  ';
        backBuffer[bufPos+1] = 'O ';
        break;
    }
  }
}
void copyImage(int pos)
{
  int bufPos = 0;
  memset((char*)&showBuffer, 0, LCDSIZE+1);
  for ( int i = 0; i < LCDSIZE; i++)
  {
    bufPos = (pos + i) % BUFLEN;
    showBuffer[i] = backBuffer[bufPos];
  }
}
void showLandschaft()
{
  lcd.setCursor(0, 1);
  lcd.print(showBuffer);
}

void showMaennchen(bool jump)
{
  if ( true == jump )
  {
    lcd.setCursor(MannPOS, 0);
  }
  else
  {
    lcd.setCursor(MannPOS, 0);
    lcd.print(" ");  
    lcd.setCursor(MannPOS, 1);  
  }
  lcd.print("I");  
}
void setup() {
  Serial.begin(115200);
  lcd.begin(LCDSIZE, 2);
  pinMode(PINLCD,         OUTPUT);
  digitalWrite(PINLCD, HIGH);
  generiereLandschaft(0);
  generiereLandschaft(LCDSIZE);
}

void loop() {
  bool death = false;
  copyImage(pos);
  showLandschaft();
  int keyval = Keypad();
  if ( btnUP == keyval )
    showMaennchen(true);
  else
  {
    showMaennchen(false);
    if ( 'O' == showBuffer[MannPOS] )
    {
      lcd.setCursor(0, 0);
      lcd.print("Tod!!!");
      death = true;
    }
  }
  if (death)
  {
    lcd.setCursor(MannPOS, 1);
    lcd.print("X");
    if ( btnSELECT == keyval )
    {
      pos = 0;
      lcd.setCursor(0, 0);
      lcd.print("                ");
      death = false;
      generiereLandschaft(0);
    }
  }
  else
  {
    pos++; 
    if ( LCDSIZE == pos )
    {
       generiereLandschaft(0);
    }
    if ( BUFLEN == pos )
    {
      generiereLandschaft(LCDSIZE);
      pos = 0;
    }
    Serial.println(showBuffer);
    delay(500);
  }
}
