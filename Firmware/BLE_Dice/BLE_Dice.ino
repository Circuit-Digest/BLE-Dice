/*
 * Project Name: BLE Dice
 * Project Brief: nRF52840 based Bluetooth LE Dice.
 * Author: Jobit Joseph @ https://github.com/jobitjoseph
 * IDE: Arduino IDE 2.x.x
 * Arduino Core: ESP32 Arduino Core V 3.0.7
 * Dependencies : ArduinoBLE Library V 1.3.7 @ https://github.com/arduino-libraries/ArduinoBLE
 *                Adafruit MPU6050 Library V 2.2.6 @ https://github.com/adafruit/Adafruit_MPU6050
 * Copyright © Jobit Joseph
 * Copyright © Semicon Media Pvt Ltd
 * Copyright © Circuitdigest.com
 * 
 * This code is licensed under the following conditions:
 *
 * 1. Non-Commercial Use:
 * This program is free software: you can redistribute it and/or modify it
 * for personal or educational purposes under the condition that credit is given 
 * to the original author. Attribution is required, and the original author 
 * must be credited in any derivative works or distributions.
 *
 * 2. Commercial Use:
 * For any commercial use of this software, you must obtain a separate license
 * from the original author. Contact the author for permissions or licensing
 * options before using this software for commercial purposes.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE, AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT, OR OTHERWISE, ARISING 
 * FROM, OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 *
 * Author: Jobit Joseph
 * Date: 18 October 2024
 *
 * For commercial use or licensing requests, please contact [jobitjoseph1@gmail.com].
 */
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <ArduinoBLE.h>

// Pin definitions
#define rgbRed A7
#define rgbGreen A6
#define rgbBlue 3
#define leftLEDs 4
#define rightLEDs 9
#define backLEDs 10
#define bottomLEDs 6
#define topLEDs 5

Adafruit_MPU6050 mpu;

// Variables for shake and stationary detection
float shakeThreshold = 15.0; // Adjust for shake sensitivity
float stationaryThreshold = 15.0; // Threshold for being stationary
unsigned long stationaryDuration = 1500; // Duration to confirm stationary (in ms)

// BLE Characteristics
BLEService diceService("180A");//0000180A-0000-1000-8000-00805f9b34fb
BLEByteCharacteristic statusCharacteristic("2A57", BLERead | BLEWrite | BLENotify);//00002A58-0000-1000-8000-00805f9b34fb
BLEByteCharacteristic faceCharacteristic("2A58", BLERead | BLENotify);//00002A57-0000-1000-8000-00805f9b34fb

// Other Variables
int currentFace = -1; // Tracks the current face
unsigned long lastStationaryTime = 0;
unsigned long lastRGBBlinkTime = 0;
bool BLEStatus = false;
int LEDPins[7] = {-1,A6, 4, 5, 6, 9,10};

enum DiceState {
  WAIT_FOR_SHAKE,
  WAIT_FOR_STATIONARY,
  UPDATE_FACE,
  WAIT_FOR_BLE_RESET
};

DiceState currentState = WAIT_FOR_SHAKE; // Start in the WAIT_FOR_SHAKE state
unsigned long lastFlashTime = 0; // For non-blocking sequential LED flashing
int flashIndex = 0; // Current LED index for sequential flashing
bool shakeDetected = false; // To track shake detection
unsigned long stationaryStartTime = 0; // Track time for stationary detection


void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize LED pins
  pinMode(rgbRed, OUTPUT);
  pinMode(rgbGreen, OUTPUT);
  pinMode(rgbBlue, OUTPUT);
  pinMode(leftLEDs, OUTPUT);
  pinMode(rightLEDs, OUTPUT);
  pinMode(backLEDs, OUTPUT);
  pinMode(bottomLEDs, OUTPUT);
  pinMode(topLEDs, OUTPUT);

  turnOffAllLEDs(); // Ensure all LEDs are off at boot
  digitalWrite(rgbRed, LOW);
  // Initialize BLE
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }
  BLE.setLocalName("BLE_Dice");
  BLE.setAdvertisedService(diceService);

  diceService.addCharacteristic(statusCharacteristic);
  diceService.addCharacteristic(faceCharacteristic);

  BLE.addService(diceService);

  // Set characteristics to 0 at boot
  statusCharacteristic.writeValue(0);
  faceCharacteristic.writeValue(0);

  BLE.advertise();
  Serial.println("BLE_Dice is ready!");
  
}

void loop() {
  // Handle BLE
  BLEDevice central = BLE.central();
  if (central) {
    BLEStatus = true;
    Serial.print("Connected to: ");
    Serial.println(central.address());
    currentState = WAIT_FOR_SHAKE;
    statusCharacteristic.writeValue(0); // Dice ready
    faceCharacteristic.writeValue(0); // Detected face
    turnOffAllLEDs();
    while (central.connected()) {
      handleDiceLogic();
    }
    Serial.println("Disconnected from central");
    turnOffAllLEDs();
    BLEStatus = false;
    digitalWrite(rgbRed, LOW);
  }
  if(BLEStatus == false) {
  blinkRGB();
  }
}

void handleDiceLogic() {
  switch (currentState) {
    case WAIT_FOR_SHAKE:
      // Blink blue LED until shake is detected
      blinkBlueLED();
      if (detectShake()) {
        shakeDetected = true;
        digitalWrite(rgbBlue, HIGH); // Turn off blue LED
        currentState = WAIT_FOR_STATIONARY; // Move to next state
        Serial.println("Shake detected! Moving to WAIT_FOR_STATIONARY.");
      }
      break;

    case WAIT_FOR_STATIONARY:
      // Flash LEDs sequentially until stationary
      if (millis() - lastFlashTime >= 100) { // Non-blocking delay for LED flashing
        flashSequentialLEDs();
        lastFlashTime = millis();
      }
      if (detectStationary()) {
        currentState = UPDATE_FACE; // Move to next state
        Serial.println("Stationary detected! Moving to UPDATE_FACE.");
      }
      break;

    case UPDATE_FACE:
      // Determine face and update LEDs and BLE characteristics
      currentFace = determineFace();
      if(currentFace < 0)
      {
        return;
      }
      updateLEDs(currentFace);

      // Set BLE characteristics
      statusCharacteristic.writeValue(1); // Dice ready
      faceCharacteristic.writeValue(currentFace); // Detected face

      Serial.println("Face updated! Waiting for BLE reset.");
      currentState = WAIT_FOR_BLE_RESET; // Move to next state
      break;

    case WAIT_FOR_BLE_RESET:
      // Wait for user to set statusCharacteristic to 0
      if (statusCharacteristic.value() == 0) {
        turnOffAllLEDs(); // Reset LEDs before next cycle
        currentState = WAIT_FOR_SHAKE; // Go back to initial state
        Serial.println("BLE reset! Returning to WAIT_FOR_SHAKE.");
      }
      break;
  }
}

bool detectShake() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float magnitude = sqrt(a.acceleration.x * a.acceleration.x +
                         a.acceleration.y * a.acceleration.y +
                         a.acceleration.z * a.acceleration.z);

  if (magnitude > shakeThreshold) {
    Serial.println("Shake detected!");
    return true;
  }
  return false;
}

bool detectStationary() {
  static float accelSum = 0;         // Sum of acceleration magnitudes
  static int sampleCount = 0;        // Number of samples in the averaging window

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float magnitude = sqrt(a.acceleration.x * a.acceleration.x +
                         a.acceleration.y * a.acceleration.y +
                         a.acceleration.z * a.acceleration.z);

  // Add current magnitude to the sum
  accelSum += magnitude;
  sampleCount++;

  // Average the acceleration magnitude over the sample window
  float averageMagnitude = accelSum / sampleCount;

  if (averageMagnitude < stationaryThreshold) {
    if (millis() - lastStationaryTime > stationaryDuration) {
      // Reset for the next cycle
      accelSum = 0;
      sampleCount = 0;
      Serial.println("Stationary detected!");
      return true;
    }
  } else {
    // Reset stationary timer if movement is above the threshold
    lastStationaryTime = millis();
    accelSum = 0;
    sampleCount = 0;
  }

  return false;
}

int determineFace() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  if (a.acceleration.y > 8.0) return 3; // Top
  if (a.acceleration.y < -8.0) return 4; // Bottom
  if (a.acceleration.z > 8.0) return 5; // Right
  if (a.acceleration.z < -8.0) return 2; // Left
  if (a.acceleration.x > 8.0) return 6; // Back
  if (a.acceleration.x < -8.0) return 1; // Front
  return -1; // Unknown
}

void flashSequentialLEDs() {
  const int leds[] = {rgbGreen,leftLEDs, rightLEDs, backLEDs, bottomLEDs, topLEDs};
  for (int i = 0; i < 6; i++) {
    turnOffAllLEDs();
    if(i == 0)
    {
        digitalWrite(leds[i], LOW);
    }
    else {
        digitalWrite(leds[i], HIGH);
    }
    delay(100);
  }
}

void blinkBlueLED() {
  static unsigned long lastBlinkTime = 0;
  static bool ledState = false;

  if (millis() - lastBlinkTime >= 500) {
    ledState = !ledState;
    digitalWrite(LEDPins[currentFace], ledState ? LOW : HIGH);
    lastBlinkTime = millis();
  }
}

void blinkRGB() {

  if (millis() - lastRGBBlinkTime >= 200) {
    if(digitalRead(rgbRed) == 0)
    {
      digitalWrite(rgbRed, HIGH);
      digitalWrite(rgbBlue, HIGH);
      digitalWrite(rgbGreen, LOW);
    }
    else if(digitalRead(rgbGreen) == 0)
    {
      digitalWrite(rgbRed, HIGH);
      digitalWrite(rgbGreen, HIGH);
      digitalWrite(rgbBlue, LOW);
    }
    else if(digitalRead(rgbBlue) == 0)
    {
      digitalWrite(rgbBlue, HIGH);
      digitalWrite(rgbGreen, HIGH);
      digitalWrite(rgbRed, LOW);
    }
    lastRGBBlinkTime = millis();
  }
}

void updateLEDs(int face) {
  turnOffAllLEDs();
  if(face == 1)
  {
    digitalWrite(LEDPins[face], LOW);
  }
  else
  {
    digitalWrite(LEDPins[face], HIGH);
  }

}

void turnOffAllLEDs() {
  digitalWrite(rgbRed, HIGH); // Fully off for common anode
  digitalWrite(rgbGreen, HIGH);
  digitalWrite(rgbBlue, HIGH);
  digitalWrite(leftLEDs, LOW);
  digitalWrite(rightLEDs, LOW);
  digitalWrite(backLEDs, LOW);
  digitalWrite(bottomLEDs, LOW);
  digitalWrite(topLEDs, LOW);
}