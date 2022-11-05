// This Arduino sketch manages 3 hardware components, connected to an Arduino Nano 33 BLE:
// - LED strip
// - EMG muscle sensor
// - accelerometer (built-in)
// Updatetd values, either for the contraction or the rotation exercises, are sent via BLE to a mobile app as a BLE characteristic

#include <Arduino.h>
#include <Wire.h>               // allows to communicate to the LED strip via I2C protocol
#include <Adafruit_NeoPixel.h>  // allows to control the LED strip
#include <Arduino_LSM9DS1.h>    // allows to read from the built-in accelerometer
#include <ArduinoBLE.h>         // allows to use the built-in BLE/WiFi module

#define PIN 6  // define digital pin for writing to the LED strip

// --- BLUETOOTH
// refs.: https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
const int UPDATE_FREQUENCY = 500;
long previousMillis = 0;

BLEService exerciseService("0ccc7966-1399-4c67-9ede-9b05dbea1ba2");                                            // create service "Physical Activity Monitor"
BLEIntCharacteristic exerciseCharacteristic("b964a50a-20fa-4d37-97eb-971bf5233a98", BLERead | BLENotify);    // create characteristic "Physical Activity Monitor Features"
BLEBoolCharacteristic exerciseTypeCharacteristic("7fc27348-4794-4edf-af03-9e13078ef99b", BLEWrite | BLERead);  // create characteristic "Physical Activity Session Descriptor"
// ---

// Analog pins to read from the muscle sensors
#define EMG_SENSOR_FRONT A0
#define EMG_SENSOR_REAR A1

float current_EMG_value = 0;
float MAX_CONTRACTION = 900;
float MUSCLE_THRESHOLD = 350;
float GYRO_THRESHOLD = 0.25;
float old_gyro_sign = 0;
bool old_EMG_threshold = true;

int contractionCounter = 0;
int rotationCounter = 0;

// maximum values recorded for one exercise repetition in %
float gyro_max_perc = 0;
float EMG_max_perc = 0;

// LED strip object declaration
Adafruit_NeoPixel strip = Adafruit_NeoPixel(8, PIN, NEO_GRB + NEO_KHZ800);

// custom colours and colours array
uint16_t maxIndex = 0;
uint32_t Red = strip.Color(255, 0, 0);
uint32_t Yellow = strip.Color(255, 255, 0);
uint32_t Green = strip.Color(0, 255, 0);
uint32_t Blue = strip.Color(0, 0, 255);
uint32_t colours[8] = { Red, Red, Yellow, Yellow, Yellow, Green, Green, Blue };

// accelerometer variables
float ax, ay, az;

bool isMuscleExercise = true;  // a.k.a. 1: false means only gyroscope => rotation exercise

// --- SETUP
void setup() {
  Serial.begin(115200);

  strip.setBrightness(10);
  strip.begin();
  strip.show();  // initialize all pixels to 'off'

  // --- ACCELEROMETER
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  // --- BLUETOOTH
  if (!BLE.begin()) {
    Serial.println("Failed to start BLE!");
    while (1)
      ;
  }
  // set the local name peripheral advertises
  BLE.setLocalName("RehTracker - Console");
  // set the service this peripheral advertises
  BLE.setAdvertisedService(exerciseService);

  // add the characteristics to the service
  exerciseService.addCharacteristic(exerciseCharacteristic);
  exerciseService.addCharacteristic(exerciseTypeCharacteristic);
  BLE.addService(exerciseService);

  // initialise chars. and advertise service
  exerciseCharacteristic.writeValue(0);
  exerciseTypeCharacteristic.writeValue(isMuscleExercise);
  BLE.advertise();
}

void loop() {
  // --- BLUETOOTH
  BLEDevice central = BLE.central();  // Wait for a BLE central to connect

  if (central) {
    Serial.println("BLE - Central connected");
    while (central.connected()) {

      isMuscleExercise = exerciseTypeCharacteristic.value();

      // --- MUSCLE EXERCISE
      if (isMuscleExercise) {
        muscleSensor();
        if (current_EMG_value > EMG_max_perc) {
          EMG_max_perc = current_EMG_value;
        }
        // send value
        if (old_EMG_threshold == true && (current_EMG_value < MUSCLE_THRESHOLD)) {
          //          exerciseCharacteristic.writeValue(EMG_max_perc / MAX_CONTRACTION);
//          exerciseCharacteristic.writeValue((int) EMG_max_perc);

contractionCounter++;
  exerciseCharacteristic.writeValue(contractionCounter);

          Serial.print(contractionCounter);
          Serial.println(" - new! sent via ble!");
          EMG_max_perc = 0;
        }
        old_EMG_threshold = (current_EMG_value >= MUSCLE_THRESHOLD);
      }

      // --- GYRO EXERCISE
      else {
        IMU.readAcceleration(ax, ay, az);  // ay
        float current_gyro_sign = (ay > 0) ? 1 : ((ay < 0) ? -1 : 0);

        if (abs(ay) >= GYRO_THRESHOLD) {
          if (abs(ay) > gyro_max_perc) {
            gyro_max_perc = abs(ay);
          }
        }
        // send value
        if ((old_gyro_sign != current_gyro_sign) && (gyro_max_perc >= GYRO_THRESHOLD)) {
//          exerciseCharacteristic.writeValue((int)(100 * gyro_max_perc));
rotationCounter++;
exerciseCharacteristic.writeValue(rotationCounter);

          Serial.print((int)(100 * gyro_max_perc));
          Serial.println(" - new! sent via ble!");
          gyro_max_perc = 0;
        }
        old_gyro_sign = current_gyro_sign;
      }

      // --- LED STRIP
      updateMaxIndex();
      setColours();
      delay(50);  // delay to make LED-to-LED transition slower
    }
  }
}

// Updates 'maxIndex' to the new last LED to light up, according to the contraction intensity
void updateMaxIndex() {
  maxIndex = (uint16_t)1.0;  // keeping the first red LED always on

  // --- MUSCLE EXERCISE
  if (isMuscleExercise) {
    if (current_EMG_value / MAX_CONTRACTION > (1.0 / 8.0)) // not the first LED
      maxIndex = (uint16_t)round(current_EMG_value / MAX_CONTRACTION * 8.0);  // getting proper index in (1,8]
  }

  // --- GYRO EXERCISE
  else {
    if (abs(ay) > 1.0) maxIndex = (uint16_t)8.0;
    else if (abs(ay) > (1.0 / 8.0))
      maxIndex = (uint16_t)round(abs(ay) * 8.0);
  }
}

// Switches off all the LEDs from 'maxIndex' to the last one (blue/max contraction intensity)
void resetToMaxInput() {
  for (uint16_t i = maxIndex; i < 8; i++) {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
}

// Sets the colours of the LEDs that should light up, according to the contraction intensity
void setColours() {
  resetToMaxInput();
  for (uint16_t i = 0; i < maxIndex; i++) {
    uint32_t c = colours[i];
    strip.setPixelColor(i, c);
  }
  strip.show();
}

// Reads and updates 'current_EMG_value' with the current raw value from the muscle sensor
void muscleSensor() {
  current_EMG_value = analogRead(EMG_SENSOR_FRONT);
  //Serial.print("--------------------");Serial.println(current_EMG_value);
  current_EMG_value = (current_EMG_value > MAX_CONTRACTION ? MAX_CONTRACTION : current_EMG_value);
}

