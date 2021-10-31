#include <PCD8544.h>
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT22
#define RelayPin 9
#define LEDPin 8
#define ButtonPin 13



PCD8544 lcd;
DHT dht(DHTPIN, DHTTYPE);
unsigned long previousMillis = 0;        // will store last time LED was updated
unsigned long previousMillis2 = 0;
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
int ttemp = 0; //Set to Zero. Updated by potentiometer
// constants won't change:
const long interval = 5000;           // interval at which to blink (milliseconds)
const long interval2 = 250;  //interval at which to blink during temperature setup
int sensorValue = 0;        // value read from the pot
int outputValue = 0;        // value output to the PWM (analog out)
int buttonState = 0;  //variable for reading pushbutton status


void setup() {
  pinMode(LEDPin, OUTPUT);
  pinMode(RelayPin, OUTPUT);
  pinMode(ButtonPin, INPUT);
  digitalWrite(RelayPin, LOW);  // make sure it is off to start
  digitalWrite(LEDPin, LOW);
  Serial.begin(9600);
  lcd.begin(84, 48);
  dht.begin();
  lcd.setCursor(0, 0);
  lcd.print("Dehydrate!");
  // read the analog in value:

  delay(1000);
}
void loop() {
  float hum = dht.readHumidity();
  float fah = dht.readTemperature(true);
  Serial.println(fah);
  buttonState = digitalRead(ButtonPin);
  if (buttonState == LOW) {
    unsigned long currentMillis2 = millis();

    if (currentMillis2 - previousMillis2 >= interval2) {
      // save the last time you blinked the LED
      previousMillis2 = currentMillis2;
      // read the analog in value:
      sensorValue = analogRead(analogInPin);
      // map it to the range of the analog out:
      outputValue = map(sensorValue, 0, 1023, 90, 197);
      // change the analog out value:
      //  analogWrite(analogOutPin, outputValue);
      ttemp = outputValue;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Target:");
      lcd.print(ttemp);
    }
  } else {

    //drive output
    if (fah < ttemp) {
      digitalWrite(RelayPin, HIGH);
      digitalWrite(LEDPin, LOW);
    } else {
      digitalWrite(RelayPin, LOW);
      digitalWrite(LEDPin, HIGH);
    }

    // check to see if it's time to blink the LED; that is, if the difference
    // between the current time and last time you blinked the LED is bigger than
    // the interval at which you want to blink the LED.
    {
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis >= interval) {
        // save the last time you blinked the LED
        previousMillis = currentMillis;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Humi:");
        lcd.print(hum);
        lcd.print("%\t");
        lcd.setCursor(0, 1);
        lcd.print("Actual:");
        lcd.print(fah);
        lcd.setCursor(0, 2);
        lcd.print("Target:");
        lcd.print(ttemp);
      }
    }
  }
}
