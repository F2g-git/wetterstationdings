#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Stepper.h>
// Stepper 1
const int stepsPerRevolution = 2048; // Schritte pro Umdrehung für den 28BYJ-48 Motor
Stepper stepper(stepsPerRevolution, 21, 19, 18, 5); // Pins für den Stepper Motor an ULN2003 Treiber
volatile float currentPosition = 0; // Aktuelle Position des Motors
const int hallSensorPin = 34;
//const float startPosition = 90; // Startposition des Zeigers (in Grad)
const float startPositionInSteps = 0; // step Position of minTemp (please adjust)
const float endPositionInSteps = 2048; // step position of maxTemp

const float minTemp = -15; // Mindesttemperatur auf der Skala
const float maxTemp = 40; // Maximale Temperatur auf der Skala
//const float tempPerStep = 5; // Erhöhung der Temperatur pro Schritt (in °C)
//const float degPerStep = 30; // Drehung des Zeigers pro Temperaturschritt (in Grad)

// Stepper 2

Stepper stepper2(stepsPerRevolution, 12, 14, 27, 26); // Pins für den zweiten Stepper Motor an ULN2003 Treiber
const int hallSensor2Pin = 35;
const float startWeatherPosition = 0; // Startposition des Weather-Zeigers (in Grad)
const float degPerStepWeather = 45; // Drehung des Zeigers pro Wetterbedingungsschritt (in Grad)
volatile float currentPositionWeather = 0; // Aktuelle Position des Weather-Zeigers


const int potPin = 32;

const char* ssid = "x";
const char* password = "x";

const char* server = "http://api.openweathermap.org";
const char* apiKey = "x";
const char* lat = "1";
const char* lon = "1";
const int cnt = 3;

double tempHeute = 0;
double tempMorgen = 0;
double tempUbermorgen = 0;
String witterungHeute = "";
String witterungMorgen = "";
String witterungUbermorgen = "";


int prevSelection = -1;
int potValue = 0;
unsigned long prevTime = 0;
const int updateInterval = 3600000; // 1 hour in milliseconds

void calibrateStepper() {
 Serial.print("Kalibriere Temperatur");
 pinMode(hallSensorPin, INPUT_PULLUP); // Konfiguration von Pin 13 als Eingang mit integriertem Pull-Down

 stepper.setSpeed(10); // Geschwindigkeit des Stepper Motors

 while (digitalRead(hallSensorPin) != LOW) {
  stepper.step(1); // Rotieren Sie den Motor um 1 Schritt im Uhrzeigersinn
  delay(5); // Kurze Pause für die Motorbewegung
 }

 stepper.step(0); // Stoppen Sie den Motor, wenn der Hall-Sensor aktiviert wird

 int currentPosition = 0; // Aktuelle Position des Motors
 Serial.print("Kalibrierung Temperatur abgeschlossen, Position: ");
 Serial.println(currentPosition);
}

void calibrateStepper2() {
 Serial.print("Kalibriere Wetter");
 pinMode(hallSensor2Pin, INPUT_PULLUP); // Konfiguration von Pin 13 als Eingang mit integriertem Pull-Down

 stepper2.setSpeed(10); // Geschwindigkeit des Stepper Motors

 while (digitalRead(hallSensor2Pin) != LOW) {
  stepper2.step(1); // Rotieren Sie den Motor um 1 Schritt im Uhrzeigersinn
  delay(5); // Kurze Pause für die Motorbewegung
 }

 stepper2.step(0); // Stoppen Sie den Motor, wenn der Hall-Sensor aktiviert wird

 int currentPositionWeather = 0; // Aktuelle Position des Motors
 Serial.print("Kalibrierung Wetter abgeschlossen, Position: ");
 Serial.println(currentPositionWeather);
}

void moveToTemperature(float temperature) {
 Serial.print("Bewege Temperaturzeiger");
 stepper.setSpeed(5);
 if (temperature < minTemp) {
  temperature = minTemp; // Begrenzen der Temperatur auf das Minimum
 } else if (temperature > maxTemp) {
  temperature = maxTemp; // Begrenzen der Temperatur auf das Maximum
 }

 int targetPosition = int(temperature/(maxTemp-minTemp)*(endPositionInSteps-startPositionInSteps))%endPositionInSteps;
 int stepsToMove = targetPosition-currentPosition;

stepper.step(stepsToMove);

 currentPosition = targetPosition; // Aktualisieren der aktuellen Position
 Serial.print("Temperatur: ");
 Serial.print(temperature);
 Serial.print(" °C, Zeigerposition: ");
 Serial.println(targetPosition);
}

void moveToWeather(String weatherCondition) {
 Serial.print("Bewege Wetterzeiger");
 stepper2.setSpeed(5);

 // Funktion zur Bewegung des Zeigers für die Wettersymbole
 float targetPosition = 0;

 if (weatherCondition == "Clear") {
  targetPosition = currentPositionWeather + 0 * degPerStepWeather;
 } else if (weatherCondition == "Clouds") {
  targetPosition = currentPositionWeather + 1 * degPerStepWeather;
 } else if (weatherCondition == "Drizzle") {
  targetPosition = currentPositionWeather + 2 * degPerStepWeather;
 } else if (weatherCondition == "Rain") {
  targetPosition = currentPositionWeather + 3 * degPerStepWeather;
 } else if (weatherCondition == "Thunderstorm") {
  targetPosition = currentPositionWeather + 4 * degPerStepWeather;
 } else if (weatherCondition == "Snow") {
  targetPosition = currentPositionWeather + 5 * degPerStepWeather;
 }

 int stepsToMove = abs(targetPosition - currentPositionWeather);

 if (targetPosition > currentPositionWeather) {
  stepper2.step(stepsToMove * (stepsPerRevolution / 360.0));
 } else {
  stepper2.step(-stepsToMove * (stepsPerRevolution / 360.0));
 }

 currentPositionWeather = targetPosition; // Aktualisieren der aktuellen Position
 Serial.print("Wetter: ");
 Serial.print(weatherCondition);
 Serial.print(", Zeigerposition: ");
 Serial.println(targetPosition);
}

void updateWeatherData() {
 Serial.print("Hole Wetterdaten");
 HTTPClient http;
 delay(5000);
 char url[150];
 strcat(url, server);
 strcat(url, "/data/2.5/forecast?lat=");
 strcat(url, lat);
 strcat(url, "&lon=");
 strcat(url, lon);
 strcat(url, "&cnt=");
 strcat(url, String(cnt).c_str());
 strcat(url, "&appid=");
 strcat(url, apiKey);
 strcat(url, "&units=metric");

 http.begin(url);

 int httpCode = http.GET();

 if (httpCode > 0) {
  String payload = http.getString();

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (error) {
   Serial.print("deserializeJson() failed: ");
   Serial.println(error.c_str());
   return;
  }

  tempHeute = doc["list"][0]["main"]["temp"];
  tempMorgen = doc["list"][1]["main"]["temp"];
  tempUbermorgen = doc["list"][2]["main"]["temp"];

  witterungHeute = doc["list"][0]["weather"][0]["main"].as<String>();
  witterungMorgen = doc["list"][1]["weather"][0]["main"].as<String>();
  witterungUbermorgen = doc["list"][2]["weather"][0]["main"].as<String>();

  Serial.print("Temperature for today: ");
  Serial.println(tempHeute);
  Serial.print("Weather for today: ");
  Serial.println(witterungHeute);
  Serial.print("Temperature for tomorrow: ");
  Serial.println(tempMorgen);
  Serial.print("Weather for tomorrow: ");
  Serial.println(witterungMorgen);
  Serial.print("Temperature for the day after tomorrow: ");
  Serial.println(tempUbermorgen);
  Serial.print("Weather for the day after tomorrow: ");
  Serial.println(witterungUbermorgen);
 } else {
  Serial.println("Error on HTTP request");
 }

 http.end();
}

void connectWifi() {
 WiFi.begin(ssid, password);
 Serial.println("Connecting");
 while(WiFi.status() != WL_CONNECTED) {
  delay(500);
  Serial.print(".");
 }
 Serial.println("");
 Serial.print("Connected to WiFi network with IP Address: ");
 Serial.println(WiFi.localIP());

 Serial.println("Timer set to 10 seconds (timerDelay variable), it will take 10 seconds before publishing the first reading.");
}

void setup() {
//Serial Monitor starten
 Serial.begin(9600);

 //WLAN verbinden
 connectWifi();
 //Nach boot Motoren ausrichten
 calibrateStepper();
 delay(10000);
 calibrateStepper2();
 //Nach boot Wetterdaten laden
 updateWeatherData();


}

void loop() {

 if (WiFi.status() != WL_CONNECTED) {
 connectWifi();
 }

 potValue = analogRead(potPin);
 int selection = -1;

 if (potValue >= 0 && potValue <= 750) {
  selection = 0;
 } else if (potValue > 751 && potValue <= 2500) {
  selection = 1;
 } else if (potValue > 2501 && potValue <= 5000) {
  selection = 2;
 }

 unsigned long currentTime = millis();

 if (selection != prevSelection || (currentTime - prevTime >= updateInterval)) {
  prevSelection = selection;
  prevTime = currentTime;
  updateWeatherData();

  switch (selection) {
   case 0:
    Serial.print("Today - Temperature: ");
    Serial.print(tempHeute);
    Serial.print(" °C, Weather: ");
    Serial.println(witterungHeute);
    moveToTemperature(tempHeute);
    moveToWeather(witterungHeute);
    break;
   case 1:
    Serial.print("Tomorrow - Temperature: ");
    Serial.print(tempMorgen);
    Serial.print(" °C, Weather: ");
    Serial.println(witterungMorgen);
    moveToTemperature(tempMorgen);
    moveToWeather(witterungMorgen);
    break;
   case 2:
    Serial.print("Day after tomorrow - Temperature: ");
    Serial.print(tempUbermorgen);
    Serial.print(" °C, Weather: ");
    Serial.println(witterungUbermorgen);
    moveToTemperature(tempUbermorgen);
    moveToWeather(witterungUbermorgen);
    break;
   default:
    break;
  }
 }
 Serial.println(potValue);
 delay(1000); // Optional delay between reading potentiometer
}
