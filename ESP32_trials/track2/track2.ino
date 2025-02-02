#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <ArduinoJson.h>

// WiFi settings
const char* ssid = "intellection";        // Replace with your WiFi SSID
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
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi");

  // Start the server
  server.begin();

  // Initialize the MPU6050 sensor
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
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

  if (client) {
    // Wait for client to send data
    while (client.connected()) {
      if (client.available()) {
        // Read the accelerometer, gyro, and temperature data
        sensors_event_t accel, gyro, temp;
        mpu.getEvent(&accel, &gyro, &temp);

        // Create a JSON object to store the data
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

        // Check sensor readings and update LED color accordingly
        if (abs(accel.acceleration.x) > 2.0 || abs(accel.acceleration.y) > 2.0 || abs(accel.acceleration.z) > 2.0) {
          // If significant movement detected -> Red LED (error)
          digitalWrite(redPin, HIGH);
          digitalWrite(greenPin, LOW);
          digitalWrite(bluePin, LOW);
        } else {
          // If stable sensor values -> Green LED (normal)
          digitalWrite(redPin, LOW);
          digitalWrite(greenPin, HIGH);
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
