#include <Wire.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>// for gyro sensor
#include <TinyGPS.h>// for gps module

#define VIBRATION_SENSOR_PIN A0
#define MQ3_SENSOR_PIN A1
#define EYE_BLINK_SENSOR_PIN A3
#define RELAY_MODULE_PIN 7
#define BUZZER_PIN 3
#define VOLTAGE_THRESHOLD 4 // Voltage threshold in volts
#define TUMBLE_THRESHOLD 90 // Tumble angle threshold in degrees
#define VIBRATION_THRESHOLD 800 // Vibration threshold for ignoring pitch and roll angles
#define GPS_TX_PIN 8 // in arduino 8 pin
#define GPS_RX_PIN 12// in arduino 12 pin
#define GSM_TX_PIN 10 // in ardunio 10 pin
#define GSM_RX_PIN 11 // in ardunio 11 pin

SoftwareSerial SIM900A(GSM_TX_PIN, GSM_RX_PIN);
SoftwareSerial gpsSerial(GPS_TX_PIN, GPS_RX_PIN); // GPS connection
MPU6050 mpu;
TinyGPS gps; // GPS object

float gyroData, accelData;
unsigned long lastTime = 0;
float dt = 0.0; // Time interval between sensor readings in seconds
bool rollExceeded = false;
bool pitchExceeded = false;
String phoneNumber = "+9779867522201";
float latitude, longitude;
void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Initialize SIM900A module
  SIM900A.begin(9600);
  delay(2000); // Wait for the module to respond

  // Initialize MPU6050 sensor
  Wire.begin();
  mpu.initialize();

  // Initialize GPS module
  gpsSerial.begin(9600);

  pinMode(VIBRATION_SENSOR_PIN, INPUT);
  pinMode(MQ3_SENSOR_PIN, INPUT);
  pinMode(EYE_BLINK_SENSOR_PIN, INPUT);
  pinMode(RELAY_MODULE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RELAY_MODULE_PIN, LOW);

  // Set initial value of lastTime
  lastTime = millis();
}

void loop() {
  // Calculate time interval since last reading
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0; // Convert milliseconds to seconds
  lastTime = now;

  // Read vibration sensor data if sensor is connected and returning valid values
  int vibration = analogRead(VIBRATION_SENSOR_PIN);

  // Read gyro sensor data if sensor is connected and returning valid values
  int16_t gyroX, gyroY, gyroZ;
  mpu.getRotation(&gyroX, &gyroY, &gyroZ); {
    gyroData = gyroY; // Use gyroY data as an example, adjust for your sensor orientation if needed

  // Read accelerometer data if sensor is connected and returning valid values
  int16_t accelX, accelY, accelZ;
  mpu.getAcceleration(&accelX, &accelY, &accelZ); {
    // Calculate roll and pitch angles
    float roll = atan2(accelY, accelZ) * 180.0 / PI;
    float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

    // Check if vehicle has rolled over
    if (vibration < VIBRATION_THRESHOLD && (abs(roll) > TUMBLE_THRESHOLD || abs(pitch) > TUMBLE_THRESHOLD)) {
      Serial.println("Vehicle pitch or roll angle exceeds 90 degrees");
      delay(1000);
      // Activate buzzer and relay
      digitalWrite(RELAY_MODULE_PIN, HIGH);
      delay(1000);
      sendMessage("Vehicle tumbled");
      digitalWrite(RELAY_MODULE_PIN, LOW);
    }
  }



  // Read eye blink sensor data if sensor is connected and returning valid values
  int eyeState = digitalRead(EYE_BLINK_SENSOR_PIN);
  if (eyeState == HIGH) { // Eyes are closed
    unsigned long blinkStartTime = millis(); // Start counting blink duration
    while (digitalRead(EYE_BLINK_SENSOR_PIN) == HIGH) {
      if (millis() - blinkStartTime >= 4000) { // Adjusted blink duration threshold
        Serial.println("Driver's eyes closed for too long");
        digitalWrite(BUZZER_PIN, HIGH);
        delay(3000); // Buzzer on for 3 seconds
        digitalWrite(BUZZER_PIN, LOW);
        if (eyestate==HIGH){
          delay(1000);
          digitalWrite(RELAY_MODULE_PIN, HIGH);
          delay(1000);
          sendMessage("CRASH ALERT .Current location: http://maps.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
          digitalwrite(RELAY_MODULE_PIN,LOW);
        }
        break;
      }
    }
  }

  // Read voltage from vibration sensor pin
  float voltage = analogRead(VIBRATION_SENSOR_PIN) * (5.0 / 1023.0); // Convert analog reading to voltage
  if (voltage > VOLTAGE_THRESHOLD) {
    Serial.println("Vibration detected");
    delay(1000); // Buzzer on for 3 seconds

    // Activate buzzer and relay
    digitalWrite(RELAY_MODULE_PIN, HIGH);
    delay(3000);
    sendMessage("CRASH ALERT .Current location: http://maps.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
    delay(1000);
    digitalWrite(RELAY_MODULE_PIN, LOW);
  }

  // Read GPS data if available
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      gps.f_get_position(&latitude, &longitude, &fix_age);
      sendMessage("Current location: http://maps.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6));
    }
  }

  // Add a delay to control the loop rate
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
