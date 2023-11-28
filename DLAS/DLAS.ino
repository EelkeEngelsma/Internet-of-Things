#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiS3.h>
#include "env.h"
#include <ArduinoMqttClient.h>

#define NUM_LEDS 6
WiFiClient wifi; // wifi aanmaken
MqttClient mqttClient(wifi); // initializatie van de mqtt client

const int ledPins[NUM_LEDS] = {2, 3, 4, 5, 6, 7};
#define ECHOPIN 11
#define TRIGPIN 12

LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  while (WiFi.begin(ssid, password) != WL_CONNECTED) // wifi ssid en wachtwoord ingeven verbinden met wifi
  {
    delay(5000);
  }

  mqttClient.setUsernamePassword(MQTTUsername, MQTTPassword); // mqtt wachtwoord en username geven


  bool MQTTconnected = false; // hier connect de mqtt broker de while loop houd in dat hij het blijft proberen tot hij verbinding heeft
  while (!MQTTconnected) {
    if (!mqttClient.connect(MQTTURL, MQTTPort))
      delay(1000);
    else
      MQTTconnected = true;
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

  mqttClient.beginMessage("eelke/altitude",true,0); // naam van de data, retention flag true geeft aan dat hij de laatste waarde onthoud, zekerheid van aankomen data
  mqttClient.print(distance); // versturen van ontvangen data
  mqttClient.endMessage(); // einde bericht

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
      delay(500);
        }
  }
}
