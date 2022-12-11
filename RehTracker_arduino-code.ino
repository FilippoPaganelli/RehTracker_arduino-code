// This Arduino sketch manages 3 hardware components, connected to an Arduino Nano 33 BLE:
// - LED strip
// - EMG muscle sensor
// - accelerometer (built-in)
// Updated values are sent via BLE as values of BLE characteristics

#include <Wire.h>              // allows to communicate to the LED strip via I2C protocol
#include <Adafruit_NeoPixel.h> // allows to control the LED strip
#include <Arduino_LSM9DS1.h>   // allows to read from the built-in accelerometer
#include <ArduinoBLE.h>        // allows to use the built-in BLE/WiFi module

// define digital pin for writing to the LED strip
#define PIN 6
// analog pins to read from the muscle sensors
#define EMG_SENSOR_FRONT A0

// general constants
const float MAX_CONTRACTION = 900;
const float MUSCLE_THRESHOLD = 350;
const float GYRO_THRESHOLD = 0.25;

float current_EMG_value = 0;
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
const uint32_t Red = strip.Color(255, 0, 0);
const uint32_t Yellow = strip.Color(255, 255, 0);
const uint32_t Green = strip.Color(0, 255, 0);
const uint32_t Blue = strip.Color(0, 0, 255);
const uint32_t colours[8] = {Red, Red, Yellow, Yellow, Yellow, Green, Green, Blue};

// accelerometer variables
float ax, ay, az;

// false: gyro exercise, true: muscle exercise
bool isMuscleExercise = true;

// false: RehTracker behaviour, true: robotics demo
bool isRoboticsDemo = false;

// BLE objects declaration
// refs.: https://btprodspecificationrefs.blob.core.windows.net/assigned-values/16-bit%20UUID%20Numbers%20Document.pdf
BLEService exerciseService("0ccc7966-1399-4c67-9ede-9b05dbea1ba2");                                            // create service "Physical Activity Monitor"
BLEIntCharacteristic exerciseValueCharacteristic("b964a50a-0001-4d37-97eb-971bf5233a98", BLERead | BLENotify); // create characteristic "Physical Activity Monitor Features"
BLEBoolCharacteristic exerciseTypeCharacteristic("b964a50a-0002-4d37-97eb-971bf5233a98", BLEWrite | BLERead);  // create characteristic "Physical Activity Session Descriptor"
BLEBoolCharacteristic isRoboticsDemoCharacteristic("b964a50a-0003-4d37-97eb-971bf5233a98", BLEWrite | BLERead);

// --- SETUP
void setup()
{
  // --- LED STRIP
  // initialise all pixels to 'off'
  strip.setBrightness(10);
  strip.begin();
  strip.show();

  // --- ACCELEROMETER
  if (!IMU.begin())
  {
    Serial.println("Failed to initialise IMU!");
    while (1)
      ;
  }

  // --- BLUETOOTH
  if (!BLE.begin())
  {
    Serial.println("Failed to start BLE!");
    while (1)
      ;
  }

  // set the local name peripheral advertises
  BLE.setLocalName("RehTracker - Console");

  // set the service this peripheral advertises
  BLE.setAdvertisedService(exerciseService);

  // add the characteristics to the service
  exerciseService.addCharacteristic(exerciseValueCharacteristic);
  exerciseService.addCharacteristic(exerciseTypeCharacteristic);
  exerciseService.addCharacteristic(isRoboticsDemoCharacteristic);
  BLE.addService(exerciseService);

  // initialise characteristics and advertise the service
  exerciseValueCharacteristic.writeValue(0);
  exerciseTypeCharacteristic.writeValue(isMuscleExercise);
  isRoboticsDemoCharacteristic.writeValue(0);
  BLE.advertise();
}

void loop()
{
  // --- BLUETOOTH
  BLEDevice central = BLE.central(); // wait for a BLE central to connect

  if (central)
  {
    while (central.connected())
    {
      bool isRoboticsDemoNewValue = isRoboticsDemoCharacteristic.value();

      // deals with the mode change
      if (isRoboticsDemo != isRoboticsDemoNewValue)
      {
        old_EMG_threshold = false;
      }

      isRoboticsDemo = isRoboticsDemoNewValue;

      // --- ROBOTICS DEMO
      if (isRoboticsDemo)
      {
        handleRoboticsDemo();
      }
      else
      {
        bool isMuscleExerciseNewValue = exerciseTypeCharacteristic.value();

        // deals with the exercise type change
        if (isMuscleExercise != isMuscleExerciseNewValue)
        {
          contractionCounter = 0;
          rotationCounter = 0;
        }

        isMuscleExercise = isMuscleExerciseNewValue;

        // --- MUSCLE EXERCISE
        if (isMuscleExercise)
        {
          handleMuscleExercise();
        }

        // --- GYRO EXERCISE
        else
        {
          handleGyroExercise();
        }
      }

      // --- LED STRIP
      updateMaxIndex();
      setColours();
      delay(50); // delay to make LED-to-LED transition slower
    }
  }
}

void handleRoboticsDemo()
{
  // readFromMuscleSensor();

  // send value
  exerciseValueCharacteristic.writeValue(random(0, 100));

  delay(150);
}

void handleMuscleExercise()
{
  readFromMuscleSensor();

  if (current_EMG_value > EMG_max_perc)
  {
    EMG_max_perc = current_EMG_value;
  }

  // send value
  if (old_EMG_threshold == true && (current_EMG_value < MUSCLE_THRESHOLD))
  {

    contractionCounter++;
    exerciseValueCharacteristic.writeValue(contractionCounter);
    EMG_max_perc = 0;
  }

  old_EMG_threshold = (current_EMG_value >= MUSCLE_THRESHOLD);
}

void handleGyroExercise()
{
  readFromGyroSensor();
  float current_gyro_sign = (ay > 0) ? 1 : ((ay < 0) ? -1 : 0);

  if (abs(ay) >= GYRO_THRESHOLD)
  {
    if (abs(ay) > gyro_max_perc)
    {
      gyro_max_perc = abs(ay);
    }
  }

  // send value
  if ((old_gyro_sign != current_gyro_sign) && (gyro_max_perc >= GYRO_THRESHOLD))
  {
    rotationCounter++;
    exerciseValueCharacteristic.writeValue(rotationCounter);
    gyro_max_perc = 0;
  }

  old_gyro_sign = current_gyro_sign;
}

// Updates 'maxIndex' to the new last LED to light up, according to the contraction intensity
void updateMaxIndex()
{
  maxIndex = (uint16_t)1.0; // keeping the first red LED always on

  // --- MUSCLE EXERCISE
  if (isMuscleExercise)
  {
    if (current_EMG_value / MAX_CONTRACTION > (1.0 / 8.0)) // not the first LED
    {
      // getting proper index in the interval (1,8]
      maxIndex = (uint16_t)round(current_EMG_value / MAX_CONTRACTION * 8.0);
    }
  }

  // --- GYRO EXERCISE
  else
  {
    if (abs(ay) > 1.0)
    {
      maxIndex = (uint16_t)8.0;
    }
    else if (abs(ay) > (1.0 / 8.0))
    {
      maxIndex = (uint16_t)round(abs(ay) * 8.0);
    }
  }
}

// Switches off all the LEDs from 'maxIndex' to the last one (blue/max contraction intensity)
void resetToMaxInput()
{
  for (uint16_t i = maxIndex; i < 8; i++)
  {
    strip.setPixelColor(i, strip.Color(0, 0, 0));
  }
}

// Sets the colours of the LEDs that should light up, according to the contraction intensity
void setColours()
{
  resetToMaxInput();

  for (uint16_t i = 0; i < maxIndex; i++)
  {
    uint32_t c = colours[i];
    strip.setPixelColor(i, c);
  }

  strip.show();
}

// Reads and updates 'current_EMG_value' with the current value from the muscle sensor
void readFromMuscleSensor()
{
  current_EMG_value = analogRead(EMG_SENSOR_FRONT);
  current_EMG_value = (current_EMG_value > MAX_CONTRACTION ? MAX_CONTRACTION : current_EMG_value);
}

// Reads and updates 'ay' with the current value from the built-in accelerometer
void readFromGyroSensor()
{
  IMU.readAcceleration(ax, ay, az); // only 'ay' is the needed value
}