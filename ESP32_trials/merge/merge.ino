#include <Wire.h>
#include <WiFi.h>
#include <MPU6050.h>
#include <ESP32WebServer.h>

// Define RGB LED pins
#define RED_PIN 19
#define GREEN_PIN 18
#define BLUE_PIN 17

// Create an MPU6050 object
MPU6050 mpu;

// Wi-Fi credentials
const char* ssid = "IAAC-WIFI";
const char* password = "password";

// Server for handling HTTP requests
ESP32WebServer server(80);  // The server listens on port 80

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(1000);

  // Initialize LED pins as output
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Connect to Wi-Fi
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  // Initialize MPU6050
  Wire.begin();
  mpu.initialize();

  // Check for MPU6050 sensor connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful!");
  } else {
    Serial.println("MPU6050 connection failed. Red LED ON.");
    // Turn on Red LED to indicate sensor connection error
    digitalWrite(RED_PIN, HIGH);
  }

  // Initialize Web Server Routes
  server.on("/", HTTP_GET, handleSensorData);

  // Start the server
  server.begin();
  Serial.println("Server started.");
}

void loop() {
  // Handle incoming HTTP requests
  server.handleClient();

  // Check if the sensor is still sending data
  if (mpu.testConnection()) {
    // Read data from the MPU6050 sensor
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getAcceleration(&ax, &ay, &az);
    mpu.getRotation(&gx, &gy, &gz);

    // Print the sensor data to the serial monitor
    Serial.print("Accelerometer X: "); Serial.print(ax);
    Serial.print(", Y: "); Serial.print(ay);
    Serial.print(", Z: "); Serial.println(az);
    Serial.print("Gyroscope X: "); Serial.print(gx);
    Serial.print(", Y: "); Serial.print(gy);
    Serial.print(", Z: "); Serial.println(gz);

    // Turn on blue LED when data is being sent
    digitalWrite(BLUE_PIN, HIGH);

    // After sending data, turn off blue LED
    digitalWrite(BLUE_PIN, LOW);
  } else {
    // Sensor is not responding, turn on red LED to indicate error
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
  }
}

void handleSensorData() {
  // Read data from the MPU6050 sensor
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Prepare JSON response
  String json = "{\"accel_x\": " + String(ax) + ", \"accel_y\": " + String(ay) + ", \"accel_z\": " + String(az) +
                ", \"gyro_x\": " + String(gx) + ", \"gyro_y\": " + String(gy) + ", \"gyro_z\": " + String(gz) + "}";

  // Send the JSON data as the HTTP response
  server.send(200, "application/json", json);
}
