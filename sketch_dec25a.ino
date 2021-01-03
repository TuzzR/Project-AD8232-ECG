#include <TFT.h>
#include <SD.h>

//--------------LCD Display Pins--------------------------
//   LCD_RD   connects to Analog pin A0  
//   LCD_WR   connects to Analog pin A1  
//   LCD_RS   connects to Analog pin A2  
//   LCD_CS   connects to Analog pin A3  
//   LCD_RST  connects to Analog pin A4  
//   HEART SENSOR         Analog pin A5
//   LCD_D0   connects to digital pin 8  
//   LCD_D1   connects to digital pin 9  
//   LCD_D2   connects to digital pin 2
//   LCD_D3   connects to digital pin 3
//   LCD_D4   connects to digital pin 4
//   LCD_D5   connects to digital pin 5
//   LCD_D6   connects to digital pin 6
//   LCD_D7   connects to digital pin 7
//LO +        connects to digital pin 11
//LO -        connects to digital pin 10
//---------------------------------------------------------

 
#define LCD_CS A3 // Chip Select goes to Analog 3
#define LCD_CD A2 // Command/Data goes to Analog 2
#define LCD_WR A1 // LCD Write goes to Analog 1
#define LCD_RD A0 // LCD Read goes to Analog 0
#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

#include "Adafruit_GFX.h"
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv tft;

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

const int LO1Pin = 10; //LO -
const int LO2Pin = 11; //LO +
const int heartSensorPin = A5;
int LastTime=0;
int ThisTime;
bool BPMTiming=false;
bool BeatComplete=false;
int BPM=0;
int lastx=0;
int lasty=0;

#define UpperThreshold 560
#define LowerThreshold 530

int heartSensorValue;
int x, y; 

void setup()
{
  Serial.begin(9600);
  pinMode(LO1Pin, INPUT); // Setup for leads off detection LO -
  pinMode(LO2Pin, INPUT); // Setup for leads off detection LO +
  pinMode(heartSensorPin, INPUT);

  Serial.println(F("TFT LCD test"));
  tft.reset();  
  uint16_t identifier = tft.readID();
  if(identifier == 0x9325) {
    Serial.println(F("Found ILI9325 LCD driver"));
  } else if(identifier == 0x9328) {
    Serial.println(F("Found ILI9328 LCD driver"));
  } else if(identifier == 0x4535) {
    Serial.println(F("Found LGDP4535 LCD driver"));
  }else if(identifier == 0x7575) {
    Serial.println(F("Found HX8347G LCD driver"));
  } else if(identifier == 0x9595) {
    Serial.println(F("Found HX8347-I LCD driver"));
  } else if(identifier == 0x4747) {
    Serial.println(F("Found HX8347-D LCD driver"));
  } else if(identifier == 0x8347) {
    Serial.println(F("Found HX8347-A LCD driver"));
  }else if(identifier == 0x9341) {
    Serial.println(F("Found ILI9341 LCD driver"));
  }else if(identifier == 0x7783) {
    Serial.println(F("Found ST7781 LCD driver"));
  }else if(identifier == 0x8230) {
    Serial.println(F("Found UC8230 LCD driver"));  
  }else if(identifier == 0x8357) {
    Serial.println(F("Found HX8357D LCD driver"));
  } else if(identifier==0x0101){     
      identifier=0x9341;
      Serial.println(F("Found 0x9341 LCD driver"));
  }else if(identifier==0x7793){     
       Serial.println(F("Found ST7793 LCD driver"));
  }else if(identifier==0xB509){     
       Serial.println(F("Found R61509 LCD driver"));
  }else if(identifier==0x9486){     
       Serial.println(F("Found ILI9486 LCD driver"));
  }else if(identifier==0x9488){     
       Serial.println(F("Found ILI9488 LCD driver"));
  }else {
    Serial.print(F("Unknown LCD driver chip: "));
    Serial.println(identifier, HEX);
    Serial.println(F("If using the Adafruit 2.8\" TFT Arduino shield, the line:"));
    Serial.println(F("  #define USE_ADAFRUIT_SHIELD_PINOUT"));
    Serial.println(F("should appear in the library header (Adafruit_TFT.h)."));
    Serial.println(F("If using the breakout board, it should NOT be #defined!"));
    Serial.println(F("Also if using the breakout, double-check that all wiring"));
    Serial.println(F("matches the tutorial."));
    identifier=0x9486; 
  }
  tft.begin(identifier);
  Serial.print("TFT size is "); 
  Serial.print(tft.width()); 
  Serial.print("x"); 
  Serial.println(tft.height());

  tft.setRotation(1);
  tft.fillScreen(BLACK);
    
  delay (2000);
  x = 1;
}

void loop(){
  if((digitalRead(LO1Pin) == 1)||(digitalRead(LO2Pin) == 1)){
    Serial.println('!');
  }
  else{
    if(x > 319)  
    {
      x=0;
      lastx=x;
      tft.fillScreen(BLACK);
    }    
    heartSensorValue = analogRead(heartSensorPin);
    Serial.println(heartSensorValue);
    y = 180 - (int)(heartSensorValue/6.5);

    tft.setTextColor(WHITE);
    tft.drawLine(lastx,lasty,x,y,WHITE);
    lasty=y;
    lastx=x;
    x++;
    delay(1);

    //tft.drawLine(x, 239, x, y,BLUE);
    //tft.drawPixel(x,y,YELLOW);
    

    ThisTime=millis();
    if(heartSensorValue>UpperThreshold)
    {
      if(BeatComplete)
      {
        BPM=ThisTime-LastTime;
        BPM=int(60/(float(BPM)/1000));
        BPMTiming=false;
        BeatComplete=false;
      }
      if(BPMTiming==false)
      {   
          LastTime=millis();
          BPMTiming=true;
      }
    }
    if((heartSensorValue<LowerThreshold)&(BPMTiming))
      BeatComplete=true;
    tft.fillRect(0,190,320,240,BLACK);
    tft.setTextSize(2);
    tft.setCursor(125,200);    
    tft.print(BPM);
    tft.print(" BPM");
  }
}
