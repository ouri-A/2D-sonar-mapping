#include <Servo.h>
#include <SoftwareSerial.h>

// Pin definitions
const int SERVO_PIN = 9;
const int SENSOR_RX_PIN = 8;      // Sensor TX (White)
const int SENSOR_TX_PIN = 7;      // Sensor RX (Yellow)

// Serial settings
const long PC_SERIAL_BAUD = 115200;
const long SENSOR_SERIAL_BAUD = 115200;

// Sensor protocol
const byte SENSOR_TRIGGER_CMD = 0x55;
const byte SENSOR_FRAME_HEADER = 0xFF;

// Servo parameters
const int SERVO_MIN_ANGLE = 30;     // Start angle of sweep
const int SERVO_MAX_ANGLE = 150;   // End angle of sweep (120-degree range)
const int SERVO_ANGLE_STEP = 5;    // Increment for each reading
const int SERVO_STABILIZE_MS = 200;
const int SENSOR_TIMEOUT_MS = 100;

Servo scannerServo;
SoftwareSerial sensorCom(SENSOR_RX_PIN, SENSOR_TX_PIN);

void setup() {
  Serial.begin(PC_SERIAL_BAUD);
  sensorCom.begin(SENSOR_SERIAL_BAUD);
  scannerServo.attach(SERVO_PIN);
  delay(100);
}

void loop() {
  for (int angle = SERVO_MIN_ANGLE; angle <= SERVO_MAX_ANGLE; angle += SERVO_ANGLE_STEP) {
    performScan(angle);
  }
  for (int angle = SERVO_MAX_ANGLE - SERVO_ANGLE_STEP; angle >= SERVO_MIN_ANGLE; angle -= SERVO_ANGLE_STEP) {
    performScan(angle);
  }
}

void performScan(int angle) {
  scannerServo.write(angle);
  delay(SERVO_STABILIZE_MS);

  int distance = getDistance();
  Serial.print(angle);
  Serial.print(",");
  Serial.println(distance);
}

int getDistance() {
  byte buffer[4];
  sensorCom.write(SENSOR_TRIGGER_CMD);
  delay(30); 

  unsigned long startTime = millis();
  int bytesRead = 0;
  while (bytesRead < 4 && (millis() - startTime) < SENSOR_TIMEOUT_MS) {
    if (sensorCom.available()) {
      buffer[bytesRead++] = sensorCom.read();
    }
  }

  if (bytesRead < 4) { // Timeout
    while(sensorCom.available()) sensorCom.read();
    return -1; 
  }
  if (buffer[0] != SENSOR_FRAME_HEADER) { // Invalid header
    return -1; 
  }

  byte checksum = (buffer[0] + buffer[1] + buffer[2]) & 0xFF;
  if (buffer[3] != checksum) { // Checksum error
    return -1;
  }
  return (buffer[1] << 8) | buffer[2]; // Distance
}
