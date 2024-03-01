#include <Wire.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>
#include <TinyGPS.h>

#define VIBRATION_SENSOR_PIN A0
#define MQ3_SENSOR_PIN A1
#define EYE_BLINK_SENSOR_PIN A3
#define RELAY_MODULE_PIN 7
#define BUZZER_PIN 3
#define VOLTAGE_THRESHOLD 4 
#define TUMBLE_THRESHOLD 90 
#define VIBRATION_THRESHOLD 800 
#define GPS_TX_PIN 8 
#define GPS_RX_PIN 12
#define GSM_TX_PIN 10 
#define GSM_RX_PIN 11 

SoftwareSerial SIM900A(GSM_TX_PIN, GSM_RX_PIN);
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN);     
MPU6050 mpu;
TinyGPS gps; // GPS object

unsigned long lastTime = 0;
String phoneNumber = "+9779867522201";
String hospitalNumber = "+9779567575910"; 
float latitude, longitude;

void setup() {
  Serial.begin(9600);
  SIM900A.begin(9600);
  delay(2000); 

  Wire.begin();
  mpu.initialize();

  gpsSerial.begin(9600);

  pinMode(VIBRATION_SENSOR_PIN, INPUT);
  pinMode(EYE_BLINK_SENSOR_PIN, INPUT);
  pinMode(RELAY_MODULE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_MODULE_PIN, LOW);

  lastTime = millis();
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  int vibration = analogRead(VIBRATION_SENSOR_PIN);
  float vibrationPercentage = (vibration / 1023.0) * 100.0;

  if (vibrationPercentage > 70) {
    sendMessage("CRITICAL ACCIDENT ALERT. Current location: http://maps.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
  } else if (vibrationPercentage > 30 && vibrationPercentage <= 70) {
     sendMessageToHospital("MODERATE ACCIDENT ALERT. Current location: http://maps.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
  }

  int eyeState = digitalRead(EYE_BLINK_SENSOR_PIN);
  if (eyeState == HIGH) { 
    unsigned long blinkStartTime = millis(); 
    while (digitalRead(EYE_BLINK_SENSOR_PIN) == HIGH) {
      if (millis() - blinkStartTime >= 4000) { 
        Serial.println("Driver's eyes closed for too long");
        digitalWrite(BUZZER_PIN, HIGH);
        delay(3000);
        digitalWrite(BUZZER_PIN, LOW);
        if (eyeState == HIGH) {
          delay(1000);
          digitalWrite(RELAY_MODULE_PIN, HIGH);
          delay(1000);
          sendMessage("CRASH ALERT. Current location: http://maps.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
          digitalWrite(RELAY_MODULE_PIN, LOW);
        }
        break;
      }
    }
  }

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      gps.f_get_position(&latitude, &longitude);
      sendMessage("Current location: http://maps.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
    }
  }
  delay(100);
}

void sendMessage(String message) {
  SIM900A.println("AT+CMGF=1");
  delay(1000);
  SIM900A.print("AT+CMGS=\"" + phoneNumber + "\"\r");
  delay(1000);
  SIM900A.println(message);
  delay(1000);
  SIM900A.println((char)26);
  delay(1000);
}

void sendMessageToHospital(String message) {
  SIM900A.println("AT+CMGF=1");
  delay(1000);
  SIM900A.print("AT+CMGS=\"" + hospitalNumber + "\"\r");
  delay(1000);
  SIM900A.println(message);
  delay(1000);
  SIM900A.println((char)26);
  delay(1000);
}
