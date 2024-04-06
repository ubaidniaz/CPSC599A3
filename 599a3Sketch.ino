#include <Servo.h>

Servo servo1;  // First servo for wind direction
Servo servo2;  // Second servo for wind speed
int servoPin1 = 13;
int servoPin2 = 11; // Pin for the second servo
int currentTemperature = -1;
int currentWindDirection = -1;
int currentWindSpeed = -1; // Initialize with an invalid wind speed
int currentServo2Position = 0; // Track the current position of the second servo

void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  Serial.begin(9600);
  servo1.write(92); // Stop the first servo upon setup
  servo2.write(180);  // Move second servo to initial position (0 degrees)
  currentServo2Position = 0; // Initialize the current position of the second servo

  // Initialize digital pins 2 through 8 as outputs for LEDs.
  for (int pin = 22; pin <= 38; pin++) {
    pinMode(pin, OUTPUT);
  }

}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    Serial.println(data);

    int tempIndex = data.indexOf('T');
    int dirIndex = data.indexOf('D');
    int windIndex = data.indexOf('W', dirIndex + 1); // Find the start of the wind speed data

    if (tempIndex != -1 && dirIndex != -1 && windIndex != -1) {
      String tempStr = data.substring(tempIndex + 1, dirIndex);
      int temperature = tempStr.toInt();
      String dirStr = data.substring(dirIndex + 1, windIndex);
      int windDirection = dirStr.toInt();
      String windStr = data.substring(windIndex + 1);
      int windSpeed = windStr.toInt();

      if (currentTemperature != temperature) {
        updateLEDs(temperature);
        currentTemperature = temperature;
      }

      if (currentWindDirection != windDirection) {
        pointToWindDirection(windDirection);
        currentWindDirection = windDirection;
      }

      if (currentWindSpeed != windSpeed) {
        moveWindSpeedServo(windSpeed);
        currentWindSpeed = windSpeed;
      }
    }
  }
}

void moveWindSpeedServo(int windSpeed) {
    int targetPosition = map(windSpeed, 0, 40, 180, 0); // Map the wind speed to the servo angle
    if (targetPosition != currentServo2Position) {
        servo2.write(targetPosition);
        currentServo2Position = targetPosition; // Update the current position
    }
}


void updateLEDs(int newTemperature) {
    static int lastTemperature = 0; // Keep track of the last temperature
    const int zeroTempPin = 28; // Pin representing 0°C
    const int firstLedPin = 22; // First LED pin number (for most negative value)
    const int maxLedPin = 38; // Last LED pin number (for highest positive value)
    const int tempPerLed = 2; // Degrees represented per LED

    // Calculate new LED pin based on the temperature
    int newLedPin = zeroTempPin + (newTemperature / tempPerLed);
    int lastLedPin = zeroTempPin + (lastTemperature / tempPerLed);

    // Ensure the LED pins are within the available range
    newLedPin = constrain(newLedPin, firstLedPin, maxLedPin);
    lastLedPin = constrain(lastLedPin, firstLedPin, maxLedPin);

    if (newTemperature >= 0) {
        // Clear LEDs for negative temperatures if moving from negative to positive
        if (lastTemperature < 0) {
            for (int pin = firstLedPin; pin < zeroTempPin; pin++) {
                digitalWrite(pin, LOW);
            }
        }
        for (int pin = max(lastLedPin, zeroTempPin); pin <= newLedPin; pin++) {
            digitalWrite(pin, HIGH);
            delay(100); // Visual effect
        }
    } else {
        // If the temperature is negative, ensure positive LEDs are turned off
        if (lastTemperature >= 0) {
            for (int pin = zeroTempPin; pin <= maxLedPin; pin++) {
                digitalWrite(pin, LOW);
            }
        }
        // Light up LEDs for negative temperatures
        for (int pin = zeroTempPin - 1; pin >= newLedPin; pin--) {
            digitalWrite(pin, HIGH);
            delay(100); // Visual effect
        }
    }

    lastTemperature = newTemperature; // Update the last known temperature
}




void pointToWindDirection(int windDirection) {
    float rotationFraction = windDirection / 360.0;
    int rotationTimeToDirection = 1280 * rotationFraction;

    // Rotate to simulate pointing in the wind direction
    servo1.write(0); // Start rotation
    delay(rotationTimeToDirection); // Delay for calculated duration
    servo1.write(92); // Stop the servo

    delay(1500); // Wait for a brief moment

    // Slightly reduce the time for return rotation to prevent overshooting
    int adjustmentValue = 7; // Start with a small adjustment, increase if necessary
    int rotationTimeToReturn = rotationTimeToDirection - adjustmentValue;

    // Rotate back to the original position ("North")
    servo1.write(180); // Reverse rotation direction
    delay(rotationTimeToReturn); // Adjusted delay for return rotation
    servo1.write(92); // Stop the servo, aiming for a more precise stop at the starting position
}







