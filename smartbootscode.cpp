#define BLYNK_TEMPLATE_ID "Blynk_ID"
#define BLYNK_TEMPLATE_NAME "Smart Shoes"
#define BLYNK_AUTH_TOKEN "Blynk_Auth_token"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Wi-Fi credentials
char ssid[] = "ssid";
char pass[] = "password";

// Blynk Timer and MPU6050
BlynkTimer timer;
Adafruit_MPU6050 mpu;

// Define pin connections
const int trigPin = 18;
const int echoPin = 19;
const int ldrPin = 33;
const int lightPin = 32;
const int rainPinDO = 35;
const int rainPinAO = 34;
const int sosButton = 26;
const int buzzerPin = 23;
const int SDA_PIN = 21;
const int SCL_PIN = 22;

// States and thresholds
bool terrainDetectorActive = false;
const int distanceThreshold = 50;  // cm
const int lightThreshold = 500;
const int moistureThreshold = 3700;
const float terrainThresholdLow = 2.5; // Lower threshold
const float terrainThresholdHigh = 8.5; // Upper threshold

// Ultrasonic Sensor Function
void checkCrevasse() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2;
  Blynk.virtualWrite(V0, distance);

  if (distance > distanceThreshold) {
    Blynk.logEvent("crevasse_ahead", "CREVASSE AHEAD"); // Notification for Crevasse
    Serial.println("Possible Crevasse Identified!");
  }
}

// LDR Function
void checkLight() {
  int lightValue = analogRead(ldrPin);
  Blynk.virtualWrite(V1, lightValue);

  if (lightValue > lightThreshold) { 
    digitalWrite(lightPin, HIGH);
  } else {
    digitalWrite(lightPin, LOW);
  }
}

// Rain Sensor Function
void checkSlippery() {
  int moistureValue = analogRead(rainPinAO);
  Blynk.virtualWrite(V2, moistureValue);

  if (moistureValue < moistureThreshold) {
    Blynk.logEvent("slippery_road", "SLIPPERY ROAD AHEAD"); // Noti for slippery path
    Serial.println("Possible Slippery Road ahead!");
  }
}

// Terrain Detector Activation
BLYNK_WRITE(V5) {
  terrainDetectorActive = param.asInt();
  if (terrainDetectorActive) {
    mpu.begin();
  }
}

// MPU6050 Function
void checkTerrain() {
  if (terrainDetectorActive) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Detect dangerous terrain if acceleration is outside safe range
    if (a.acceleration.z < terrainThresholdLow || a.acceleration.z > terrainThresholdHigh) {
      Blynk.logEvent("danger_ahead", "Danger Detected"); // Noti for danger terrain
      Serial.println("Possible dangerous terrain detected!");
      Blynk.virtualWrite(V6, "Dangerous Terrain!");
    } else {
      Blynk.virtualWrite(V6, "Terrain Safe");
    }

    // Send data to Blynk for monitoring
    Blynk.virtualWrite(V3, a.acceleration.x); // X-axis
    Blynk.virtualWrite(V4, a.acceleration.y); // Y-axis
    Blynk.virtualWrite(V5, a.acceleration.z); // Z-axis
  }
}

// SOS Button Function
void checkSOS() {
  if (digitalRead(sosButton) == LOW) { // Button pressed
    Blynk.logEvent("sos_alert", "SOS: HELP NEEDED"); // Noti SOS Event
    Serial.println("SOS Message Enabled!");
    tone(buzzerPin, 1000); 
    delay(10000);
    noTone(buzzerPin);
  }
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Initialize pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(lightPin, OUTPUT);
  pinMode(sosButton, INPUT_PULLUP);
  pinMode(buzzerPin, OUTPUT);

  // Initialize MPU6050 with explicit SDA/SCL
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }

  // Timer functions
  timer.setInterval(1000L, checkCrevasse);
  timer.setInterval(1000L, checkLight);
  timer.setInterval(1000L, checkSlippery);
  timer.setInterval(1000L, checkSOS);
}

void loop() {
  Blynk.run();
  timer.run();
  if (terrainDetectorActive) {
    checkTerrain();
  }
}