#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// WiFi settings
const char* ssid = "IAAC-WIFI";        // Replace with your WiFi SSID
const char* password = "password"; // Replace with your WiFi Password

WiFiServer server(80);  // Start a web server on port 80

Adafruit_MPU6050 mpu;

// RGB LED Pins
const int redPin = 19;
const int greenPin = 18;
const int bluePin = 17;

void setup() {
  // Start Serial for debugging
  Serial.begin(115200);

  // Initialize RGB LED pins as output
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Connect to WiFi
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  // Try connecting to Wi-Fi until success or timeout after 30 attempts
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    attempts++;

    // Timeout after 30 attempts (15 seconds)
    if (attempts >= 30) {
      Serial.println("\nFailed to connect to WiFi. Please check your credentials.");
      digitalWrite(redPin, HIGH);  // Red LED to indicate failure
      digitalWrite(greenPin, LOW);
      digitalWrite(bluePin, LOW);
      while (1) {
        delay(1000); // Stop execution on failure
      }
    }
  }

  // Once connected, print IP address and indicate success with green LED
  Serial.println("\nConnected to WiFi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());  // Print the IP address to Serial Monitor
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, HIGH);  // Green LED to indicate success
  digitalWrite(bluePin, LOW);

  // Start the server
  server.begin();
  Serial.println("Server started!");

  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    digitalWrite(redPin, HIGH);  // Red LED to indicate sensor failure
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
    while (1) { delay(10); } // Stop if sensor is not found
  }
  Serial.println("MPU6050 initialized!");

  // Set the sensor ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Indicate successful startup with Green LED
  digitalWrite(greenPin, HIGH);
  delay(2000);
  digitalWrite(greenPin, LOW);
}

void loop() {
  WiFiClient client = server.available();

  // Check WiFi status and control LED
  if (WiFi.status() != WL_CONNECTED) {
    // WiFi not connected, set Red LED
    digitalWrite(redPin, HIGH);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, LOW);
  }

  if (client) {
    // Wait for client to send data
    while (client.connected()) {
      if (client.available()) {
        // Read the accelerometer, gyro, and temperature data
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        // Check if sensor data is valid (non-zero)
        if (isnan(accel.acceleration.x) || isnan(accel.acceleration.y) || isnan(accel.acceleration.z)) {
          digitalWrite(bluePin, HIGH);  // Blue LED indicates sensor error
          digitalWrite(redPin, LOW);
          digitalWrite(greenPin, LOW);
          Serial.println("Sensor data is invalid!");
        } else {
          // Sensor data is valid, send it in JSON format
          StaticJsonDocument<512> doc;
          doc["accel_x"] = accel.acceleration.x;
          doc["accel_y"] = accel.acceleration.y;
          doc["accel_z"] = accel.acceleration.z;
          doc["gyro_x"] = gyro.gyro.x;
          doc["gyro_y"] = gyro.gyro.y;
          doc["gyro_z"] = gyro.gyro.z;
          doc["temp"] = temp.temperature;

          // Serialize the JSON object to a string and send it to the client
          String output;
          serializeJson(doc, output);
          client.println(output); // Send the JSON data as a response

          // Set Green LED as everything is functioning normally
          digitalWrite(redPin, LOW);
          digitalWrite(greenPin, HIGH);  // Green LED for normal operation
          digitalWrite(bluePin, LOW);
        }

        // Delay to avoid overwhelming the client
        delay(1000);
      }
    }

    // Close the client connection
    client.stop();
    // Turn off all LEDs when client disconnects (Blue LED - IDLE)
    digitalWrite(redPin, LOW);
    digitalWrite(greenPin, LOW);
    digitalWrite(bluePin, HIGH);
  }
}
