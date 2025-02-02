#include <Wire.h>
#include <WiFi.h>
#include <MPU6050.h>
#include <WebServer.h> // Updated for ESP32-WROOM-DA compatibility

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
WebServer server(80);  // Use WebSderver class for ESP32

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  delay(1000);

  // Initialize LED pins as output
  pinMode(RED_PIN, OUTPUT);
  pinMode(GREEN_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  // Turn on red LED to indicate setup
  digitalWrite(RED_PIN, HIGH);

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
    while (1); // Stop execution
  }

  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");
  Serial.println(WiFi.localIP()); // Print the IP address

  // Indicate successful connection
  digitalWrite(RED_PIN, LOW);
  digitalWrite(GREEN_PIN, HIGH);

  // Initialize Web Server Routes
  server.on("/", HTTP_GET, handleSensorData);

  // Start the server
  server.begin();
  Serial.println("Server started.");
}

void loop() {
  // Handle incoming HTTP requests
  server.handleClient();
}

void handleSensorData() {
  // Read data from the MPU6050 sensor
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getAcceleration(&ax, &ay, &az);
  mpu.getRotation(&gx, &gy, &gz);

  // Prepare CSV-style response
  String data = String(ax) + "," + String(ay) + "," + String(az) + "," +
                String(gx) + "," + String(gy) + "," + String(gz) + "\n";

  // Turn on blue LED to indicate data transmission
  digitalWrite(BLUE_PIN, HIGH);

  // Send the data to the client
  server.send(200, "text/plain", data);

  // Turn off blue LED after sending
  digitalWrite(BLUE_PIN, LOW);
}