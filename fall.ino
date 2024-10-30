#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>

// Define pins
const int buttonPin = 2; // Push button connected to digital pin 2
SoftwareSerial sim900(7, 8); // RX and TX for SIM900A (connected to Arduino Pins 7 and 8)

// Create an instance for MPU6050
Adafruit_MPU6050 mpu;

// Phone number to call and send SMS
String phoneNumber = "7780706770"; // Replace with the target phone number

// Variables for fall detection
float ax, ay, az;
bool fallDetected = false;
const float accelerationThreshold = 15.0; // Adjust based on testing

void setup() {
  Serial.begin(115200); // Initialize serial for debugging
  sim900.begin(9600);   // Start SIM900 communication
  pinMode(buttonPin, INPUT_PULLUP); // Set button pin as input with pull-up resistor

  
  // Initialize MPU6050 sensor
  if (mpu.begin()) { // Fixed the logic here
    Serial.println("Failed to initialize MPU6050!");
    while (1); // Stay in an infinite loop if initialization fails
  }

  // Set accelerometer range
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  Serial.println("System initialized. Monitoring for falls...");
}

void loop() {
  // Check if push button is pressed
  if (digitalRead(buttonPin) == LOW) {
    Serial.println("Manual Alert: Button pressed.");
    sendAlert("Manual Alert: Button pressed!");
    makeCall();
    delay(500); // Debounce delay
  }

  // Read accelerometer data from MPU6050
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Store acceleration data
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  // Calculate magnitude of acceleration
  float accelerationMagnitude = sqrt(ax * ax + ay * ay + az * az);

  // Fall detection logic
  if (accelerationMagnitude > accelerationThreshold) {
    if (fallDetected) { // Changed from if (fallDetected) to if (!fallDetected)
      fallDetected = true;
      Serial.println("Fall detected!");
      sendAlert("Emergency: Fall detected! Please assist.");
      makeCall();
    }
  } else {
    fallDetected = false; // Reset fall detection
  }

  delay(1000); // Check every second
}

// Function to send SMS alert using SIM900A
void sendAlert(String message) {
  sim900.println("AT+CMGF=1"); // Set SMS to text mode
  delay(100);
  sim900.print("AT+CMGS=\"");
  sim900.print(phoneNumber);
  sim900.println("\"");
  delay(100);
  sim900.println(message); // Send the message
  delay(100);
  sim900.write(26); // Send Ctrl+Z to end the SMS
  Serial.println("SMS Sent!");
}

// Function to make a call using SIM900A
void makeCall() {
  sim900.print("ATD"); // Dial command without a newline
  sim900.print(phoneNumber); // Append the phone number
  sim900.println(";"); // End with a semicolon
  Serial.println("Calling...");
  delay(10000); // Call duration, adjust as needed
  sim900.println("ATH"); // Hang up the call
  Serial.println("Call ended.");
}