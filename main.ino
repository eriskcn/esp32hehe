#define BLYNK_TEMPLATE_ID "TMPL6E__WXn0g"      // Template ID for Blynk project
#define BLYNK_TEMPLATE_NAME "Project"          // Project name
#define BLYNK_AUTH_TOKEN "JuIyJmviMcMp2BBaANKRVNAds4ka3UAc"  // Blynk authentication token

#include <LiquidCrystal_I2C.h>  // LCD library for displaying data
#include <DHT.h>                // Library for DHT sensor (temperature and humidity)
#include <BlynkSimpleEsp32.h>   // Library for Blynk with ESP32

// Sensor and actuator pin definitions
#define DHTPIN 4                
#define DHTTYPE DHT22           // DHT22 temperature/humidity sensor
#define SOIL_MOISTURE_PIN 34    // Soil moisture sensor
#define MQ2_PIN 35              // MQ2 gas sensor
#define LIGHT_SENSOR_PIN 32     // Light sensor
#define BUZZER_PIN 25           // Buzzer for alarms
#define LED_LIGHT_PIN 12        // LED for light indicator
#define LED_ALARM_PIN 14        // LED for alarm indicator
#define LED_HEATER_PIN 27       // LED for heater
#define MOSFET_PIN 26           // MOSFET for motor control

// Threshold values for sensors
const int SOIL_MOISTURE_THRESHOLD = 20;
const int LIGHT_THRESHOLD = 50;
const float TEMPERATURE_THRESHOLD = 18.0;

// Actuator states
bool motorState = false;
bool ledLightState = false;
bool ledHeaterState = false;

// Sensor objects and LCD display
DHT dht(DHTPIN, DHTTYPE);                  // Initialize DHT sensor
LiquidCrystal_I2C lcd(0x27, 20, 4);        // LCD with I2C address 0x27, 20x4 size

// Network credentials
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Huong's Galaxy A32";        // WiFi SSID
char pass[] = "neny6424";                  // WiFi password

// Blynk virtual pins for remote control and monitoring
const int VIRTUAL_PIN_AIR_DATA = V1;       // Temperature and humidity data
const int VIRTUAL_PIN_SOIL_MOISTURE = V2;  // Soil moisture data
const int VIRTUAL_PIN_MOTOR = V3;          // Motor control
const int VIRTUAL_PIN_LIGHT = V4;          // Light control
const int VIRTUAL_PIN_HEATER = V5;         // Heater control

// Flags for manual control from Blynk app
bool motorControlledByBlynk = false;
bool ledLightControlledByBlynk = false;
bool ledHeaterControlledByBlynk = false;

// Counters for controlling actuation frequency
int motorCounter = 0;
int lightCounter = 0;
int heaterCounter = 0;

void setup() {
  dht.begin();                    // Initialize DHT sensor
  lcd.init();                     // Initialize LCD
  lcd.backlight();                // Turn on LCD backlight
  Serial.begin(115200);           // Start serial communication

  Blynk.begin(auth, ssid, pass);  // Initialize Blynk connection

  setupPins();                    // Set up pin modes
  setupStates();                  // Initialize actuator states
  setupCounters();                // Initialize counters
}

void loop() {
  Blynk.run();                    // Run Blynk to handle server communication
  lcd.clear();                    // Clear LCD display for updates

  // Read sensor values
  int soilMoisture = map(analogRead(SOIL_MOISTURE_PIN), 0, 4095, 100, 0);
  int mq2Value = digitalRead(MQ2_PIN);      // Gas sensor reading
  int lightValue = map(analogRead(LIGHT_SENSOR_PIN), 0, 4095, 100, 0);
  float humidity = dht.readHumidity();      // Humidity reading from DHT
  float temperature = dht.readTemperature();// Temperature reading from DHT

  // Display sensor values on LCD and send to Blynk
  updateLCD(humidity, temperature, soilMoisture, lightValue);
  sendToBlynk(humidity, temperature, soilMoisture);

  // Control actuators based on sensor values
  controlWatering(soilMoisture);        // Water control based on soil moisture
  controlLighting(lightValue);          // Light control based on ambient light
  handleAlarm(mq2Value, temperature);   // Alarm handling for gas or temperature
  controlHeating(temperature);          // Heating control based on temperature

  delay(5000);                          // Delay before next sensor reading
}

void setupPins() {
  // Configure pins for actuators
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_LIGHT_PIN, OUTPUT);
  pinMode(LED_ALARM_PIN, OUTPUT);
  pinMode(LED_HEATER_PIN, OUTPUT);
  pinMode(MOSFET_PIN, OUTPUT);
}

void setupStates() {
  // Initialize actuator states
  motorState = false;
  ledLightState = false;
  ledHeaterState = false;
}

void setupCounters() {
  // Initialize counters for actuation
  motorCounter = 0;
  lightCounter = 0;
  heaterCounter = 0;
}

void updateLCD(float humidity, float temperature, int soilMoisture, int lightValue) {
  // Display sensor values on LCD
  lcd.setCursor(0, 0);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print("*C");

  lcd.setCursor(0, 2);
  lcd.print("Soil: ");
  lcd.print(soilMoisture);
  lcd.print("%");

  lcd.setCursor(0, 3);
  lcd.print("Light: ");
  lcd.print(lightValue);
  lcd.print("%");
}

void sendToBlynk(float humidity, float temperature, int soilMoisture) {
  // Send sensor data to Blynk server
  String airData = String(humidity) + "% - " + String(temperature) + "*C";
  Blynk.virtualWrite(V0, airData); 
  Blynk.virtualWrite(V1, String(soilMoisture) + "%");
}

void controlWatering(int soilMoisture) {
  if (motorCounter % 2 == 0) {  // Actuate every other loop iteration
    if (soilMoisture < SOIL_MOISTURE_THRESHOLD && !motorState) {
      digitalWrite(MOSFET_PIN, HIGH);  // Turn on motor
      motorState = true;
    } else if (soilMoisture >= SOIL_MOISTURE_THRESHOLD && motorState) {
      digitalWrite(MOSFET_PIN, LOW);   // Turn off motor
      motorState = false;
    }
    Blynk.virtualWrite(V2, motorState); // Update Blynk app
  }
}

void controlLighting(int lightValue) {
  if (lightCounter % 2 == 0) {  // Actuate every other loop iteration
    if (lightValue < LIGHT_THRESHOLD && !ledLightState) {
      digitalWrite(LED_LIGHT_PIN, HIGH);  // Turn on light
      ledLightState = true;
    } else if (lightValue >= LIGHT_THRESHOLD && ledLightState) {
      digitalWrite(LED_LIGHT_PIN, LOW);   // Turn off light
      ledLightState = false;
    }
    Blynk.virtualWrite(V3, ledLightState); // Update Blynk app
  }
}

void handleAlarm(int mq2Value, float temperature) {
  // Trigger alarm if gas detected or temperature is too high
  if (mq2Value == HIGH || temperature > 20) {
    digitalWrite(BUZZER_PIN, HIGH);      // Activate buzzer
    digitalWrite(LED_ALARM_PIN, HIGH);   // Activate alarm LED
  } else {
    digitalWrite(BUZZER_PIN, LOW);       // Deactivate buzzer
    digitalWrite(LED_ALARM_PIN, LOW);    // Deactivate alarm LED
  }
}

void controlHeating(float temperature) {
  if (heaterCounter % 2 == 0) {  // Actuate every other loop iteration
    if (temperature < TEMPERATURE_THRESHOLD && !ledHeaterState) {
      digitalWrite(LED_HEATER_PIN, HIGH);  // Turn on heater
      ledHeaterState = true;
    } else if (temperature >= TEMPERATURE_THRESHOLD && ledHeaterState) {
      digitalWrite(LED_HEATER_PIN, LOW);  // Turn off heater
      ledHeaterState = false;
    }
    Blynk.virtualWrite(V4, ledHeaterState); // Update Blynk app
  }
}

// Blynk virtual pin handlers for manual control
BLYNK_WRITE(V2) {
  motorState = param.asInt(); 
  digitalWrite(MOSFET_PIN, motorState ? HIGH : LOW);
  motorCounter++;
}

BLYNK_WRITE(V3) {
  ledLightState = param.asInt(); 
  digitalWrite(LED_LIGHT_PIN, ledLightState ? HIGH : LOW);
  lightCounter++;
}

BLYNK_WRITE(V4) {
  ledHeaterState = param.asInt(); 
  digitalWrite(LED_HEATER_PIN, ledHeaterState ? HIGH : LOW);
  heaterCounter++;
}
