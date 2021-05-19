//#include <Ethernet.h>
//#include "Talkie.h"
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
#include <Arduino.h>
//#include "Vocab_US_Large.h" //Vocab_US_Large.h is used to use the alerts
//#include "Vocab_Special.h" //Vocab_Special.h is used to use the pause
//#include "Vocab_US_Clock.h"
//#include <NewPing.h>        //for the Ultrasonic sensor function library.
//#include <Wire.h>
//Talkie voice; //define an object ‘value’ to use the commands
SoftwareSerial BT (9, 10);

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0


// change this to make the song slower or faster
int tempo = 160;

// change this to whichever pin you want to use
int buzzer = 8;

int melody[] = {

  REST, 1,
  REST, 1,
  NOTE_C4, 4, NOTE_E4, 4, NOTE_G4, 4, NOTE_E4, 4,
  NOTE_C4, 4, NOTE_E4, 8, NOTE_G4, -4, NOTE_E4, 4,
  NOTE_A3, 4, NOTE_C4, 4, NOTE_E4, 4, NOTE_C4, 4,
  NOTE_A3, 4, NOTE_C4, 8, NOTE_E4, -4, NOTE_C4, 4,
  NOTE_G3, 4, NOTE_B3, 4, NOTE_D4, 4, NOTE_B3, 4,
  NOTE_G3, 4, NOTE_B3, 8, NOTE_D4, -4, NOTE_B3, 4,

  NOTE_G3, 4, NOTE_G3, 8, NOTE_G3, -4, NOTE_G3, 8, NOTE_G3, 4,
  NOTE_G3, 4, NOTE_G3, 4, NOTE_G3, 8, NOTE_G3, 4,
  NOTE_C4, 4, NOTE_E4, 4, NOTE_G4, 4, NOTE_E4, 4,
  NOTE_C4, 4, NOTE_E4, 8, NOTE_G4, -4, NOTE_E4, 4,
  NOTE_A3, 4, NOTE_C4, 4, NOTE_E4, 4, NOTE_C4, 4,
  NOTE_A3, 4, NOTE_C4, 8, NOTE_E4, -4, NOTE_C4, 4,
  NOTE_G3, 4, NOTE_B3, 4, NOTE_D4, 4, NOTE_B3, 4,
  NOTE_G3, 4, NOTE_B3, 8, NOTE_D4, -4, NOTE_B3, 4,

  NOTE_G3, -1,

};


int notes = sizeof(melody) / sizeof(melody[0]) / 2;

int wholenote = (60000 * 4) / tempo;

int divider = 0, noteDuration = 0;

void music() {

  for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {

    if (analogRead(A0) > 300) {
      digitalWrite(6, HIGH);
      return;
    }

    divider = melody[thisNote + 1];
    if (divider > 0) {
      noteDuration = (wholenote) / divider;
    } else if (divider < 0) {
      noteDuration = (wholenote) / abs(divider);
      noteDuration *= 1.5;
    }
    tone(buzzer, melody[thisNote], noteDuration * 0.9);
    delay(noteDuration);
    noTone(buzzer);

  }
}

// If we using <Talkie.h>, it can speak out the current time.
//void podcast() {
//  digitalWrite(7, HIGH);
//  voice.say(spc_GOOD);
//  voice.say(spc_MORNING);
//  voice.say(spPAUSE1);
//  voice.say(spc_THE);
//  voice.say(spc_TIME);
//  voice.say(spc_IS);
//  voice.say(spc_ELEVEN);
//  voice.say(spc_THIRTY);
//  voice.say(spc_SIX);
//  voice.say(spc_A_M_);
//  voice.say(spPAUSE1);
//  digitalWrite(7, LOW);
//}

// we can reset at any given time
int h = 8;
int m = 59;
int s = 45;
int flag = 0; //AM
char c;
String weather = "cloudy";

LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

void setup() {
  lcd.begin(16, 2);
  Serial.begin (9600); 
  BT.begin(9600);
  pinMode(6, OUTPUT);
  Serial.println("start");
}
void loop() {
  Serial.println(analogRead(A0));
  if (h == 9 && m == 0 && s > 0) {
    while (analogRead(A0) < 300) {        //set pin = A0
      lcd.setCursor(0, 0);
      lcd.print("!!Already nine!!");
      music();
    }
  }
  if (h == 9 && m > 0) {
    lcd.print("NEXT:  MEETING  ");
  }
  if (Serial.available()) {
    c = Serial.read();
    BT.print(c);
  }
  if (BT.available()) {
    c = BT.read();
//    Serial.print(c);
    if (c == 'c') {
      weather = "cloudy";
    }
    if (c == 's') {
      weather = "sunny";
    }
    if (c == 'r') {
      weather = "rainy";
    }
  }
  lcd.setCursor(0, 0);
  lcd.print("Time ");
  if (h < 10)lcd.print("0");
  lcd.print(h);
  lcd.print(":");
  if (m < 10)lcd.print("0");
  lcd.print(m);
  lcd.print(":");
  if (s < 10)lcd.print("0");
  lcd.print(s);

  if (flag == 0) lcd.print(" AM");
  if (flag == 1) lcd.print(" PM");

  if (analogRead(A0) > 1000 && (s % 10 < 5)) {
    lcd.setCursor(0, 1);
    if (weather == "cloudy")
      lcd.print("Weather： cloudy ");
    if (weather == "sunny")
      lcd.print("Weather： sunny  ");
    if (weather == "rainy")
      lcd.print("Weather： rainy  ");
    delay(987);
  }
  else if (analogRead(A0) > 1000 && (s % 10 > 5)) {
    lcd.setCursor(0, 1);
    lcd.print("NEXT:  MEETING  "); // by using app we can modify the task
    delay(987);
  }
  else {
    lcd.setCursor(0, 1);
    lcd.print("2021/5/16 SUN");
    delay(987);
  }

  s = s + 1;

  if (s == 60) {
    s = 0;
    m = m + 1;
  }
  if (m == 60)
  {
    m = 0;
    h = h + 1;
  }
  if (h == 13)
  {
    h = 1;
    flag = flag + 1;
    if (flag == 2)flag = 0;
  }
}
