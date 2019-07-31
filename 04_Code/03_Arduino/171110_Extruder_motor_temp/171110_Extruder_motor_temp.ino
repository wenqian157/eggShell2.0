
#define SET_TEMP 210
#define MIN_EXTR_TEMP 205
#define STEP_ANGLE -10
#define STEP_FRACTION 0.15

#define TEMP_SENS_PIN 0
#define TEMP_CONTR_PIN 7
#define DIR_PIN 4
#define STEP_PIN 3

#include <math.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
Adafruit_SSD1306 display(4);

int ar[10];
double oldCount=0;
double newCount=0; 


 
//Stepper Motor Values
void rotateDeg(float deg, float speed){ 
  //rotate a specific number of degrees (negitive for reverse movement)
  //speed is any number from .01 -> 1 with 1 being fastest - Slower is stronger
  int dir = (deg < 0)? HIGH:LOW;
  digitalWrite(DIR_PIN,dir); 

  int steps = abs(deg)*(1/0.225);
  float usDelay = (1/speed) * 70;

  for(int i=0; i < steps; i++){ 
    digitalWrite(STEP_PIN, HIGH); 
    delayMicroseconds(usDelay); 

    digitalWrite(STEP_PIN, LOW); 
    delayMicroseconds(usDelay); 
  } 
}

//Temperature sensor _ START

double Thermistor(int RawADC) {
 double Temp;
 Temp = log(10000.0*((1024.0/RawADC-1))); 
//         =log(10000.0/(1024.0*5/5.35/RawADC-1)) // for pull-up configuration
 Temp = 1 / (0.000753224465084 + (0.000277898667672 * Temp) + (0.000000055387566 * Temp * Temp * Temp));
 Temp = Temp - 273.15;            // Convert Kelvin to Celcius
 //Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
 return Temp;
}

//Temperature sensor _ END


//Temperature control _ START
float av=0;
void TempContr(int SensorPin, int SetTemp){
  av=0;
  int NonZeroCount=0;
  int t = int(Thermistor(analogRead(SensorPin)));


//  if (timer=true){
//    t1=millis();
//    Serial.print(t); 
//    Serial.print("                 ");
//    Serial.print(av); 
//    Serial.print("                 ");
//    Serial.println(t1); 
//
//  }
//  if ((millis()-t1)>10){
//     timer=true;
//  }
//  else{timer=false;}
//  
// 
  for (int i=0; i<9; i++){
    ar[i]=ar[i+1];
    av=av+ar[i];
    if ((ar[i])!=0){
      NonZeroCount=NonZeroCount+1;
    }
  }
  NonZeroCount=NonZeroCount+1;
  ar[9]=t;
  av=(av+t)/NonZeroCount;
    Serial.print(t); 
    Serial.print("                 ");
    Serial.println(av); 
  if(av>SetTemp+3){
    digitalWrite(7,LOW);
  }
  if (av<SetTemp-3){
    digitalWrite(7,HIGH);
  }
  delay(0);
 }



void setup() {
 Serial.begin(9600);
 pinMode(DIR_PIN, OUTPUT); 
 pinMode(STEP_PIN, OUTPUT);
 pinMode(TEMP_CONTR_PIN,OUTPUT);
 digitalWrite(TEMP_CONTR_PIN,HIGH);

 //OLED Screen setup -- start
  pinMode(12,OUTPUT); // OLED Screen pover 5V pin
  digitalWrite(12,HIGH);// OLED Screen pover 5V pin
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); 
 

  //OLED Screen setup -- finish
 

}
void updateScreen(){ 
  display.clearDisplay();
  display.setTextColor(WHITE);
  display.setTextSize(0.5);
  display.setCursor(0,0);
  display.print("Temp.");

  display.setCursor(73, 0);
  display.print("Set.Temp.");
 
  display.setTextSize(1);
  display.setCursor(0,12);
  display.print((int)av);
  
  display.setTextSize(0.5);
  display.setCursor(37,16);
  display.print((char)247);
  display.print("C");
  
  display.setCursor(78, 12);
  display.setTextSize(1);
  display.print(SET_TEMP);
  
  display.setTextSize(0.5);
  display.setCursor(112,16);
  display.print((char)247);
  display.print("C");
 
  //Extruder OK!!
  if (av>=MIN_EXTR_TEMP){
    display.setCursor(20, 45);
    display.setTextSize(0.5);
    display.print("Extruder ");
    display.setTextSize(1);
    display.print("OK!");
  }else{
   newCount=millis();
   if ((newCount-oldCount)<400){
    display.fillRect(0,30,127,33,WHITE); 
    display.setTextSize(1);
    display.setTextColor(BLACK);
    display.setCursor(4, 35);
    display.print("LOW T");
    display.setCursor(90, 35);
    display.print("!!");
   }else if((newCount-oldCount)<800){ 
    display.fillRect(0,30,127,33,BLACK);
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(4, 35);
    display.print("LOW T");
    display.setCursor(90, 35);
    display.print("!!");
   }else{oldCount=newCount;}
  }
  rotateDeg(STEP_ANGLE, STEP_FRACTION);
   display.display();

//}
}

void loop() {
 TempContr(TEMP_SENS_PIN,SET_TEMP );
 //Serial.println(millis());
 rotateDeg(STEP_ANGLE, STEP_FRACTION); 
 
 updateScreen(); 
 
}
