#include <ESP8266WiFi.h> // Include the ESP8266 Wi-Fi library

// Replace with your network credentials
const char* ssid = "IAAC-WIFI";          // Your Wi-Fi SSID
const char* password = "password";  // Your Wi-Fi password

void setup() {
  // Start the Serial communication
  Serial.begin(115200);  
  delay(10);

  // Connecting to Wi-Fi
  Serial.println();
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  // Wait for the Wi-Fi to connect
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);  // Wait 1 second
    Serial.print(".");
  }

  // Once connected, print the details
  Serial.println();
  Serial.println("Connected to Wi-Fi!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP()); // Print the IP address assigned to the ESP8266
}

void loop() {
  // You can add further functionality here if you like
  // For now, we will just keep this empty
}
