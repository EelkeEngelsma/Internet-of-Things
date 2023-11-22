#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define NUM_LEDS 6

const int ledPins[NUM_LEDS] = {2, 3, 4, 5, 6, 7};
#define ECHOPIN 11
#define TRIGPIN 12

LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Welcome.");

  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(ledPins[i], HIGH);
    delay(100);
    digitalWrite(ledPins[i], LOW);
    delay(200);
  }

  for (int i = 0; i < NUM_LEDS; i++) {
    digitalWrite(ledPins[i], HIGH);
  }
}

void loop() {
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  float distance = pulseIn(ECHOPIN, HIGH) / 58.0;

  Serial.println(distance);

  lcd.setCursor(0, 0);
  lcd.print("Altitude:");
  lcd.setCursor(0, 1);
  lcd.print(distance);
  delay(200);

  if (distance < 50) {
    for (int i = 0; i < NUM_LEDS; i++) {
      digitalWrite(ledPins[i], HIGH);
    }

    digitalWrite(ledPins[0], LOW);
    delay(200);
    digitalWrite(ledPins[0], HIGH);
    delay(300);

    digitalWrite(ledPins[3], HIGH);
    delay(1000);
    digitalWrite(ledPins[3], LOW);
    delay(500);

    digitalWrite(ledPins[3], HIGH);
    delay(100);
    digitalWrite(ledPins[3], LOW);
    delay(200);
  } else {
    for (int i = 0; i < NUM_LEDS; i++) {
      digitalWrite(ledPins[i], LOW);
    }
  }
}
