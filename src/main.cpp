#include <Arduino.h>

// Define the GPIO pins for the IR sensors
const int greenSensorPin = 25; // Sensor 1 (Green)
const int redSensorPin = 26;   // Sensor 2 (Red)

  // Variables to store the state of the sensors
int greenSensorState = HIGH;
int lastGreenSensorState = HIGH; // Stores previous state of green sensor
int redSensorState = HIGH;
int lastRedSensorState = HIGH; // Stores previous state of red sensor

  // Timeout for sensor activation (in milliseconds)
const unsigned long sensorTimeout = 3;
unsigned long greenDetectedTime = 0;
unsigned long redDetectedTime = 0;


// Define the GPIO pins for leds
const int greenLedPin = 18;
const int redLedPin = 19;

  // LED control variables
bool greenLedOn = false;
bool redLedOn = false;
unsigned long greenLedStartTime = 0;
unsigned long redLedStartTime = 0;


// People stats
int peopleInside = 0;

void setup() {
  Serial.begin(115200);

  pinMode(greenSensorPin, INPUT_PULLUP);
  pinMode(redSensorPin, INPUT_PULLUP);

  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);

  Serial.println("People Counter Started");
  Serial.print("Initial People Inside: ");
  Serial.println(peopleInside);
}

void loop() {
  // Read the debounced states of the sensors
  int newGreenSensorState = digitalRead(greenSensorPin);
  int newRedSensorState = digitalRead(redSensorPin);
  unsigned long currentTime = millis();

  static bool greenDetectedFirst = false;
  static bool redDetectedFirst = false;

  // Green sensor logic (edge-triggered)
  if (newGreenSensorState == LOW && lastGreenSensorState == HIGH) {
    Serial.println("Green!");
    if (!redDetectedFirst) {
      Serial.println("Green first!");
      greenDetectedFirst = true;
      greenDetectedTime = currentTime; // Store the time of detection
      redDetectedFirst = false;
    }
  }
  lastGreenSensorState = newGreenSensorState;


  // Red sensor logic (edge-triggered)
  if (newRedSensorState == LOW && lastRedSensorState == HIGH) {
    Serial.println("Red!");
    if (!greenDetectedFirst) {
      Serial.println("Red first!");
      redDetectedFirst = true;
      redDetectedTime = currentTime; // Store the time of detection
    }
  }
  lastRedSensorState = newRedSensorState;


  // Check for entry (Green then Red)
  if (greenDetectedFirst && !newRedSensorState) {
      peopleInside++;
      Serial.println("Someone went inside!");
      Serial.print("People Inside: ");
      Serial.println(peopleInside);
      greenDetectedFirst = false;
      redDetectedFirst = false;

      // Turn on green LED non-blocking
    digitalWrite(greenLedPin, HIGH);
    greenLedOn = true;
    greenLedStartTime = currentTime;
  }

  // Check for exit (Red then Green)
  if (redDetectedFirst && !newGreenSensorState && peopleInside > 0) {
    peopleInside--;
    Serial.println("Someone went outside!");
    Serial.print("People Inside: ");
    Serial.println(peopleInside);
    greenDetectedFirst = false;
    redDetectedFirst = false;

    // Turn on red LED non-blocking
    digitalWrite(redLedPin, HIGH);
    redLedOn = true;
    redLedStartTime = currentTime;

  } else if (redDetectedFirst && !newGreenSensorState && peopleInside == 0) {
        Serial.println("Huh..?");
        greenDetectedFirst = false;
        redDetectedFirst = false;
  }

  // Timeout logic to clear flags
  if (greenDetectedFirst && (currentTime - greenDetectedTime > sensorTimeout)) {
    greenDetectedFirst = false;
    Serial.println("Green sensor timed out. Resetting.");
  }
  if (redDetectedFirst && (currentTime - redDetectedTime > sensorTimeout)) {
    redDetectedFirst = false;
    Serial.println("Red sensor timed out. Resetting.");
  }

  // Turn off green LED after 1 second
  if (greenLedOn && (currentTime - greenLedStartTime >= 1000)) {
    digitalWrite(greenLedPin, LOW);
    greenLedOn = false;
  }

  // Turn off red LED after 1 second
  if (redLedOn && (currentTime - redLedStartTime >= 1000)) {
    digitalWrite(redLedPin, LOW);
    redLedOn = false;
  }

  delay(10);
}