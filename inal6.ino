#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>

#define DHTPIN 3
#define DHTTYPE DHT22
DHT_Unified dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2);

int LDRInput = A0;
int LED = 2;
int fans = 7;
int fan_so8ayara = 9;
int peltair = 10;
//temperature for green beans is maximum 29.5
 
long temperatureThreshold = 25;
unsigned long startTime = 0;
unsigned long currentTime = 0;
unsigned long elapsedTime = 0;
int thresholdValue = 400;
bool isLedOn = false;
bool ledOnFor3Hours = false;

unsigned long duration = 3 * 60 * 60* 1000;

void setup() {
  Serial.begin(9600);
  pinMode(LDRInput, INPUT);
  pinMode(LED, OUTPUT);
  pinMode(fans, OUTPUT);
  pinMode(fan_so8ayara, OUTPUT);
  pinMode(peltair, OUTPUT);

  lcd.init();
  lcd.backlight();

  dht.begin();
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
}

void loop() {
  int value = analogRead(LDRInput);

  currentTime = millis();

  if (!ledOnFor3Hours) {
    if (value < thresholdValue) {
      if (!isLedOn) {
        isLedOn = true;
        digitalWrite(LED, HIGH);
        startTime = currentTime;
      }
    } else {
      if (isLedOn) {
        digitalWrite(LED, LOW);
        isLedOn = false;
      }
    }

    if (isLedOn) {
      elapsedTime = currentTime - startTime;
      if (elapsedTime >= duration) {
        digitalWrite(LED, LOW);
        isLedOn = false;
        ledOnFor3Hours = true;
      }
    }
  }

  // Read temperature and humidity from DHT22 sensor
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  float temperature = event.temperature;
  dht.humidity().getEvent(&event);
  float humidity = event.relative_humidity;

  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C\t");
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  Serial.print("light sensor readings:");
  Serial.print(value);
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature);
  lcd.print("C");
  lcd.setCursor(0, 1);
  lcd.print("H:");
  lcd.print(humidity);
  lcd.print("%");


  if (temperature < temperatureThreshold) {
    digitalWrite(fans,HIGH);
    digitalWrite(fan_so8ayara, HIGH);
    digitalWrite(peltair, HIGH);
  } else {
    digitalWrite(fans,LOW);
    digitalWrite(fan_so8ayara, LOW);
    digitalWrite(peltair, LOW);
  }
  delay(1000);
}
