#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiS3.h>
#include "env.h"
#include <ArduinoMqttClient.h>

#define NUM_LEDS 6

WiFiClient wifi; // wifi aanmaken
MqttClient mqttClient(wifi); // initializatie van de mqtt client

String temperatuur;

const int ledPins[NUM_LEDS] = {2, 3, 4, 5, 6, 7};

#define ECHOPIN 11
#define TRIGPIN 12

LiquidCrystal_I2C lcd(0x3F, 16, 2);

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");


  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);
  }

  Serial.println("WiFi startup");
  
  while (WiFi.begin(ssid, password) != WL_CONNECTED) // wifi ssid en wachtwoord ingeven verbinden met wifi
  {
    Serial.println(".");
    delay(1000);
  }
  
Serial.println("WiFi startup done");

  mqttClient.setUsernamePassword(MQTTUsername, MQTTPassword); // mqtt wachtwoord en username geven

Serial.println("MQTT Connecting");

  bool MQTTconnected = false; // hier connect de mqtt broker de while loop houd in dat hij het blijft proberen tot hij verbinding heeft
  while (!MQTTconnected) {
    if (!mqttClient.connect(MQTTURL, MQTTPort))
    {
      delay(1000);
      Serial.println(".");
  }
    else
      MQTTconnected = true;
  }

Serial.println("Done connecting");

  mqttClient.onMessage(onMqttMessage);
  mqttClient.subscribe("Freark/outside/temperature"); // subscriben op de temp

  pinMode(ECHOPIN, INPUT);
  pinMode(TRIGPIN, OUTPUT);

  lcd.clear();
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

void onMqttMessage(int messageSize) {
Serial.print("Received temperature");
Serial.print(mqttClient.messageTopic());

temperatuur = "";
  while (mqttClient.available()){
    temperatuur += char(mqttClient.read());
    Serial.println(temperatuur);
  }
}
void loop() 
{
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  int distance = round(pulseIn(ECHOPIN, HIGH) / 58.0);
  
  mqttClient.poll();
  mqttClient.beginMessage("eelke/altitude",true,0); // naam van de data, retention flag true geeft aan dat hij de laatste waarde onthoud, zekerheid van aankomen data
  mqttClient.print(distance); // versturen van ontvangen data
  mqttClient.endMessage(); // einde bericht

  //Serial.println(distance);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Altitude:");
  lcd.setCursor(16 - String(distance).length(), 0);
  lcd.print(distance);
  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  lcd.setCursor(14 - temperatuur.length(), 1);
  lcd.print(temperatuur);
  lcd.setCursor(14, 1);
  lcd.print((char)223);
  lcd.setCursor(15, 1);
  lcd.print("C");

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
    
  } 
  
  else {
    for (int i = 0; i < NUM_LEDS; i++) {
      digitalWrite(ledPins[i], LOW);
        }
  delay(700);
  }
}
