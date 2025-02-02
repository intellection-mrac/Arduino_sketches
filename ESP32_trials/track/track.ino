#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Create an instance of the MPU6050
Adafruit_MPU6050 mpu;

// I2C Pins for ESP32 (Default I2C pins for ESP32)
#define SDA_PIN 21  // ESP32 GPIO 21 (SDA)
#define SCL_PIN 22  // ESP32 GPIO 22 (SCL)

// RGB LED Pins
#define RED_PIN 19   // Red color of RGB LED
#define GREEN_PIN 18 // Green color of RGB LED
#define BLUE_PIN 17  // Blue color of RGB LED

void setup() {
    // Start Serial communication
    Serial.begin(115200);
    
    // Initialize I2C communication
    Wire.begin(SDA_PIN, SCL_PIN);
    
    // Initialize the RGB LED pins as OUTPUT
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    pinMode(BLUE_PIN, OUTPUT);
    
    // Initialize the MPU6050 sensor
    if (!mpu.begin()) {
        Serial.println("Failed to initialize MPU6050!");
        // If sensor fails, turn LED to blue (error state)
        setLEDColor(0, 0, 255);  // Blue LED for error
        while (1) { delay(10); }  // Stop the program if MPU6050 is not found
    }
    Serial.println("MPU6050 Initialized!");
    
    // Set the accelerometer and gyroscope ranges
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    
    // Turn LED to green to indicate normal operation
    setLEDColor(0, 255, 0);  // Green LED for normal operation
}

void loop() {
    sensors_event_t accel, gyro, temp;

    // Get accelerometer, gyroscope, and temperature data
    mpu.getEvent(&accel, &gyro, &temp);

    // Output the raw accelerometer data (in m/s^2)
    Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" m/s^2, ");
    Serial.print("Accel Y: "); Serial.print(accel.acceleration.y); Serial.print(" m/s^2, ");
    Serial.print("Accel Z: "); Serial.print(accel.acceleration.z); Serial.println(" m/s^2");

    // Output the raw gyroscope data (in rad/s)
    Serial.print("Gyro X: "); Serial.print(gyro.gyro.x); Serial.print(" rad/s, ");
    Serial.print("Gyro Y: "); Serial.print(gyro.gyro.y); Serial.print(" rad/s, ");
    Serial.print("Gyro Z: "); Serial.print(gyro.gyro.z); Serial.println(" rad/s");

    // Output the temperature data (in °C)
    Serial.print("Temperature: "); Serial.print(temp.temperature); Serial.println(" °C");

    // Add some logic to simulate battery percentage or errors (optional)
    // For example, simulate an error or bad data to turn the LED blue
    if (temp.temperature < 20) {
        // Error condition: turn LED blue
        setLEDColor(0, 0, 255);  // Blue LED for error
    }
    
    delay(500);  // Delay to slow down the output (500 ms)
}

// Function to set RGB LED color (values from 0 to 255 for each color)
void setLEDColor(int red, int green, int blue) {
    analogWrite(RED_PIN, red);
    analogWrite(GREEN_PIN, green);
    analogWrite(BLUE_PIN, blue);
}
