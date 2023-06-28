#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RCSwitch.h>
#include <AceButton.h>
#include <esp_now.h>

// Includes needed for OTA Updates (Over-The-Air updates)
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

String  VERSION = "v2.63";
String  DEVICE_NAME = "BKO-DMZ-CTL1";

#define DEBUG_LOG_ENABLED false
#define DEBUG_LOG if(DEBUG_LOG_ENABLED)Serial

/// PIN Usage for this project
//
//  +----+------------+------------------------------------------------+
//  |  2 | OUTPUT     | ESP32 Built-in LED                             |
//  |  4 | INPUT      | Button: Inc/Options                            |
//  |  5 | INPUT      | 433Mhz device (Receiver)                       |
//  | 12 | OUTPUT     | Pump Public Klong LED                          |
//  | 14 | OUTPUT     | Pump Public Klong RELAY                        |
//  | 15 | INPUT      | Button: Dec/Options                            |
//  | 18 | INPUT      | Button: Confirm/Cancel                         |
//  | 19 | OUTPUT     | Meter Sensor Trigger pin (Public Klong)        |
//  | 21 | <-->       | I2C Clock  (Used for LCD display)              |
//  | 22 | <-->       | I2C Data   (Used for LCD display)              |
//  | 25 | OUTPUT     | 433Mhz device (Transmitter)                    |
//  | 26 | INPUT      | Button: Menu                                   |
//  | 27 | OUTPUT     | Relay (Pump South Klong)  NOT IMPLEMENTED      |
//  | 34 | INPUT      | Meter Sensor Echo pin (Public Klong)           |
//  | -- |            | RTC Real-time Clock - Clock Pin                |
//  | -- |            | RTC Real-time Clock - Data Pin                 |
//  | -- |            | RTC Real-time Clock - Reset Pin                |
//  | -- | OUTPUT     | Needed for South Clong Meter Sensor (Trig)     |
//  | -- | IMPUT      | Needed for South Clong Meter Sensor (Echo)     |
//  |    |            |                                                |
//  +----+------------+------------------------------------------------+

// Board Connectors
// Distance Sensor:
//    [o][o][o][o]  GND-TRIG(19)-ECHO(34)-VCC5
// Pump Relay
//    [o][o][o]     



// REMOTE OPERATIONS
//
// http://192.18.4.1/update            --> OTA - Over The Air firmware update
// http://192.18.4.1/setmin?lvl=30     --> Set Sensor-To-Pump distance (cm) as threshold to enable the pump

// BUTTON OPERATIONS
//
//  [o] [o] [o] [o]
//   |   |   |   |
//   |   |   |   +----- OK / CONFIRM / CANCEL
//   |   |   +--------- INCREASE / OPTION B
//   |   +------------- DECREASE / OPTION A
//   +----------------- MENU / SCREEN SELECTION 


// DATA PACKETS Structure: ID.T.XXXXXX
//     ID : Device ID
//      T : Data Type
// XXXXXX : Value of the data
// DEVICE INFORMATION used for 433Mhz data packets
#define DATA_PACKET_DEVICE_ID_CTL1    10  // This device DMZ_CTL1
#define DATA_PACKET_DEVICE_ID_PK      11  // Public Klong water distance
// DATA TYPES used for 433Mhz data packets
#define DATA_PACKET_DATATYPE_PWR      1   // Power Status
#define DATA_PACKET_DATATYPE_WLVL     2   // Water Level
// DATA VALUES used for 433Mhz data packets
// These are variables but some are basic such as 1: power is ON, 0: power is off


// LCD Screen Display settings
// ---------------------------
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
#define SSD_L1 0    // Define lines SSD_Lx for the SSD Display (which pixel position to address for a given line)
#define SSD_L2 9
#define SSD_L3 18
#define SSD_L4 27
#define SSD_L5 36
#define SSD_L6 45
#define SSD_L7 57
#define PIXELS_PER_CHAR 7
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// WIFI settings
#define DEV_WIFI_MODE_AP   // DEV_WIFI_MODE_AP or DEV_WIFI_MODE_STATION
#ifdef DEV_WIFI_MODE_AP
  const char* ssid = "BKO-GARDEN-DMZ-DEV1";
  const char* password = "BKOGARDEN123*";
  const char* wifimode = "AP";
#else
  const char* ssid = "Toumim-1st-Floor";
  const char* password = "GFUCNEE4";
  const char* wifimode = "STATION";
#endif
IPAddress IP;

// DISPLAY MODE settings
#define NUMBER_OF_DISPLAY_MODES         4
#define DISPLAY_MODE_OPERATIONS         0
#define DISPLAY_MODE_WIFI               1
#define DISPLAY_MODE_FREQUENCY_SETUP    2
#define DISPLAY_MODE_REBOOT             3
int DISPLAY_MODE = 0;

// WATER MEASURES settings
// - "Virtual Zero Height" represents the water level when the klong is at open pipe level (target full capacity)
// - "Negative Measures" means the water level is below the virtual zero height
// - "Positive Measures" means the water level is above the virtual zero height
int   southKlongSensor_VirtualZero = 15;      // Water distance from the sensor to the water when at target full capacity
float southKlong_SensorWaterDistance = 0;     // In centimeters, the distance between the sensor and the water
float southKlong_CurrentWaterLevel = 0.0;     // In centimeters, the current relative water level from the virtual zero level. 
                                              // n > 0 = water is above target, n < 0 = water is below target
int   publicKlongSensor_VirtualZero = 25;     // Water distance from the sensor to the water when public klong is the same as south klong target full capacity
float publicKlong_RawDataWaterDistance = 0.0; // In centimeters, the distance between the sensor and the water exactly as received
float publicKlong_SensorWaterDistance = 0.0;  // In centimeters, the distance between the sensor and the water after applying smart dataset noise reduction logic
float publicKlong_CurrentWaterLevel = 0.0;    // In centimeters, the current relative water level from the virtual zero level. 
                                              // n > 0 = water is above target, n < 0 = water is below target
int   publicKlong_PumpMinimumWaterLevel = 30; // Distance from Sensor to water level needed to start the pump safely
int   publicKlong_PumpDecision = 0;
String systemAnalysis = "";
// Provide options for how often we check water and make decisions
// This helps for debug (more frequently in matter of seconds) or for production (every N minutes)
int   waterSensorsReadFrequencyOptions[] = { 1, 5, 10, 15, 30, 60, 2 * 60, 5 * 60, 10 * 60, 15 * 60, 30 * 60, 45 * 60, 60 * 60, 120 * 60 };  // Intervals in seconds
int   waterSensorsReadFrequencySelected = 5;  // The INDEX of the option selected (default index 5: 1 minute)
int   waterSensorsReadFrequencySelection = waterSensorsReadFrequencySelected;  // The INDEX of the option selected
unsigned long waterSensorsLastReadTickerMS = 0;
unsigned long waterSensorsPublicKlongLastDataReceivedMS = 0;
bool  needRevaluationOfSituation = false;
int publicKlong_overheat_protection_max_runtime_minutes = 4 * 60;  // Pump should not run for more than that, otherwise might overheat
int publicKlong_overheat_protection_time_off_minutes = 20;         // How long should the pump be stay off for overheat protection
bool publicKlong_overheat_protection_activated = false;            // Tracks if overheat protection is activated
unsigned long publicKlong_overheat_protection_kickoff_ms = 0;      // Track time since the pump was forced off for overheat protection

#define master_mode_automated "AUTO"
#define master_mode_forced_off "FORCED OFF"
#define master_mode_forced_on   "FORCED ON"
String master_operations_mode = master_mode_automated;


#define MASTER_MODE_PUBLIC_PUMP_MANUAL_OFF
#define MASTER_MODE_PUBLIC_PUMP_MANUAL_ON

// MEASURE SENSORS settings (Ultrasonic SR04 Distance Sensor)
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
const int sensorPublicKlong_trigPin = 19;
const int sensorPublicKlong_echoPin = 34;
const int sensorSouthKlong_trigPin = 19;    // #TODO Change to new sensor Trig Pin number
const int sensorSouthKlong_echoPin = 34;    // #TODO Change to new sensor Echo Pin number
long duration;
float distanceCm;

// RELAY / PUMP Control settings
#define PUBLIC_KLONG_RELAY_PIN      14
#define PUBLIC_KLONG_RELAY_LED_PIN  12
bool  publicKlong_powered = 0;
float publicKlong_operating_time_min = 0.0; // Total minutes since the pump started
unsigned long publicKlong_operating_time_sec = 0; // Total Seconds since the pump started
unsigned long publicKlong_operating_start = 0;    // Current milliseconds when the pump started

#define SOUTH_KLONG_RELAY_PIN  27
#define SOUTH_KLONG_RELAY_LED_PIN  0
bool  southKlong_powered = 0;
float southKlong_operating_time_min = 0.0;  // Total minutes since the pump started
unsigned long southKlong_operating_time_sec = 0;  // Total Seconds since the pump started
unsigned long southKlong_operating_start = 0;     // Current milliseconds when the pump started

// Constants for Remotes On/Off commands (RM1: Remote 1, RM2: Remote 2)
#define RM2_A_ON  4195665
#define RM2_A_OFF 4195668

#define LIGHT_ON true
#define LIGHT_OFF false

int GPIO_RF_PIN = 5;
int GPIO_TRANSMIT_PIN = 25;
int mode = 1;   // (1: Learn,  2: Display)
int lastRFvalue = 0;
int previousRFvalue = -1;
unsigned long previousMS = 0;
bool displayUpdateMark = false;
String lastCommand = "?";
bool virtualPlug2A = false;
int onboardLEDStatus = 0;

RCSwitch myRadioSignalSwitch = RCSwitch();

// Forward Declarations
void displayStatus(void);
void displayDeviceStatus(void);
void updateWifiStatus(void);
void updateDisplay(void);
void displaySplashScreen(void);
void sendRF433MhzPowerInfo(int);
void sendRF433MhzCode(int, int, int);

// Dynamic Preferences for all appropriate settings
#define prefNameWaterSensorReadFrequency "waterSensReadFq"
#define prefRequiredDistancePublicKlong  "waterSensMinLvl"
#define prefMasterOperationsMode         "masterOpsMode"
#define prefRequiredDistancePublicKlong_default  30
Preferences preferences;

using namespace ace_button;
// Initialize all smart buttons
#define BUTTON_MENU_PIN         26
#define BUTTON_DEC_OPTIONS_PIN  15
#define BUTTON_INC_OPTIONS_PIN   4
#define BUTTON_CONFIRM_PIN      18
AceButton btnMenu(BUTTON_MENU_PIN);
AceButton btnDecOptions(BUTTON_DEC_OPTIONS_PIN);
AceButton btnIncOptions(BUTTON_INC_OPTIONS_PIN);
AceButton btnConfirm(BUTTON_CONFIRM_PIN);
void btnHandleEvent(AceButton*, uint8_t, uint8_t);

// Initialize Async Web Server used by OTA (and maybe other stuff)
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


// MEASUREMENTS DATASET CLEANUP
// Smooths data to eliminate noise
const int maxMeasures = 30;
const int maxHighestMeasures = 20;
float measures[maxMeasures];
int measureIndex = -1;
int highestMeasureIndex = -1;
int measureCollectionRound = 1;
// Simply return the average of all measurements
float getMeasurementsAvg() {
  float total = 0.0;
  for (int i=0; i < maxMeasures; i++) {
    total += measures[i];
  }
  float avg = total / 30.0;
  return avg;
}
// Look at one full dataset and collect the N lowest points
// then calculate the averate and replace all other poinst (higher points)
// with the average.
float applyHighestMeasuresAsAverage() {
  float workMeasures[maxMeasures];            // We are aletering data so need to work on a copy
  float originalMeasures[maxMeasures];        // This array will contain the original datapoints for DEBUG ONLY!
  float lastHighestLevel = 0.0;
  int lastHighestIndex = -1;
  int highestMeasuresCount = 0;
  float highestMeasuresSum = 0.0;
  float highestMeasuresAvg = 0.0;
  // Step 1: Copy the raw data before we alter it
  //         to both work array and debug log array
  for (int i=0; i < maxMeasures; i++) {
    workMeasures[i] = measures[i];
    originalMeasures[i] = measures[i];
  }
  // Step 2: - Find the next highest value for as many 
  //           times needed to find out the maxHighestMeasues needed
  //         - Calculate the average of those highest measures on the fly
  DEBUG_LOG.println("Finding the top N highest measures...");
  for (int m=0; m < maxHighestMeasures; m++) { 
    DEBUG_LOG.print("Measures Scan #");
    DEBUG_LOG.println(m + 1);
    lastHighestLevel = 0.0;
    // Find the highest value and its index
    for (int i=0; i < maxMeasures; i++) { 
      DEBUG_LOG.print("   ");
      DEBUG_LOG.print(i);
      DEBUG_LOG.print(" --> ");
      DEBUG_LOG.print(workMeasures[i]);
      if (workMeasures[i] > lastHighestLevel) {
        lastHighestLevel = workMeasures[i];
        lastHighestIndex = i;
        DEBUG_LOG.print(" ! (last highest :");
        DEBUG_LOG.print(lastHighestLevel);
        DEBUG_LOG.print(" @");
        DEBUG_LOG.print(lastHighestIndex);
      }
      DEBUG_LOG.println();
    }
    DEBUG_LOG.print("Last Highest Measure: ");
    DEBUG_LOG.print(lastHighestLevel);
    DEBUG_LOG.print(" at index ");
    DEBUG_LOG.println(lastHighestIndex);
    // Track the max values average
    // Reset to -1 the highest value identified
    highestMeasuresSum += workMeasures[lastHighestIndex];
    highestMeasuresCount += 1;
    workMeasures[lastHighestIndex] = -1;
  }
  // Calculate average of selected highest values
  highestMeasuresAvg = highestMeasuresSum / (highestMeasuresCount * 1.0);
  // Now we have identified the N highest measures
  // We can replace all out-of-scope measures with the average
  for (int i=0; i < maxMeasures; i++) {
    if (workMeasures[i] != -1) {
      measures[i] = highestMeasuresAvg; 
    }
  }

  // Debug Serial output of data processing
  DEBUG_LOG.print("Highest Average: ");
  DEBUG_LOG.println(highestMeasuresAvg);
  DEBUG_LOG.println();
  DEBUG_LOG.println("Original  Highest  Outcome");
  for (int i=0; i < maxMeasures; i++) {
    DEBUG_LOG.print(originalMeasures[i]);
    DEBUG_LOG.print("   ");
    DEBUG_LOG.print(workMeasures[i]);
    DEBUG_LOG.print("   ");
    DEBUG_LOG.print(measures[i]);
    DEBUG_LOG.println("   ");
  }
  return highestMeasuresAvg;
}
// Process raw data by adding processing received measures
float addMeasureToMeasurements(float lastMeasure) {
  float applicableMeasure = 0.0;
  DEBUG_LOG.print("Round: ");
  DEBUG_LOG.print(measureCollectionRound);
  DEBUG_LOG.print("  Last index: ");
  DEBUG_LOG.println(measureIndex);

  // Process the first data collection as raw data
  if (measureCollectionRound == 1) {
    if (measureIndex < maxMeasures -1) {       // We are still filling up the first dataset
      measureIndex += 1;              
      measures[measureIndex] = lastMeasure;    // Add latest measure to it
      applicableMeasure = 666;
    } else {    // We have a full first dataset
      measureIndex = 0;  // Prepare index for the next round
      measureCollectionRound += 1;
      applicableMeasure = applyHighestMeasuresAsAverage();
      measures[measureIndex] = applicableMeasure;    // Add latest measure to it
    }
  } else {
    if (measureIndex < maxMeasures -1) {
      measureIndex += 1;              
    } else {    // We have a full first dataset
      measureIndex = 0;  // Prepare index for the next round
      measureCollectionRound += 1;
    }
    // At this point we have our baseline, we can decide
    // what to do with new value if within x% of the average of highest measures:
    // - Accept the last measurement, add to measurements and return it as latest value
    // - Reject the last measurement, add the average into measurements instead and return the average value
    float measurementsAvg = getMeasurementsAvg();
    if (   (lastMeasure - measurementsAvg) / measurementsAvg < -0.03 
        || (lastMeasure - measurementsAvg) / measurementsAvg > 0.03 ) {
      applicableMeasure = measurementsAvg;
      measures[measureIndex] = applicableMeasure;
      DEBUG_LOG.print("New Measure: OUT OF BOUND. Measure: ");
      DEBUG_LOG.print(lastMeasure);
      DEBUG_LOG.print("  Applicable: ");
      DEBUG_LOG.print(applicableMeasure);
      DEBUG_LOG.print("  Average: ");
      DEBUG_LOG.println(measurementsAvg);
    } else {
      // applicableMeasure = lastMeasure;
      measures[measureIndex] = lastMeasure;
      applicableMeasure = measurementsAvg;
      DEBUG_LOG.print("New Measure: ACCEPTED. Measure: ");
      DEBUG_LOG.print(lastMeasure);
      DEBUG_LOG.print("  Average: ");
      DEBUG_LOG.println(measurementsAvg);
    }
  }
  return applicableMeasure;
}


// ESP-NOW Klong Sensor Data Structure
typedef struct klong_sensor_data_message {
  char deviceId[16];
  char version[5];
  unsigned long millis;
  float waterDistance;
} klong_sensor_data_message;
klong_sensor_data_message klongData;
unsigned long publicKlong_last_received_message_ms = 0;
unsigned long publicKlong_time_since_last_message_ms = 0;

// Callback function that will be executed when data is received from Sensors via ESP-NOW
void onKlongDataReciever(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Serial.print(len);
  // Serial.print(" Bytes received from ");
  //  for (int i = 0; i < 6; i++) {
  //    Serial.print("0x");
  //    Serial.print(mac[i], HEX);
  //    Serial.print(":");
  //  }
  // Serial.println();
  memcpy(&klongData, incomingData, sizeof(klongData));
  publicKlong_time_since_last_message_ms = klongData.millis - publicKlong_last_received_message_ms;
  publicKlong_last_received_message_ms = klongData.millis;
  publicKlong_RawDataWaterDistance = klongData.waterDistance;
  publicKlong_SensorWaterDistance = addMeasureToMeasurements(klongData.waterDistance);   // TODO: Plug data cleanup call here
  Serial.print("From ");
  Serial.print(klongData.deviceId);
  Serial.print(" (");
  Serial.print(klongData.version);
  Serial.print("): ");
  Serial.print(klongData.waterDistance);
  Serial.print("cm, Timer: ");
  Serial.print(klongData.millis);
  Serial.print(" Elapse: ");
  Serial.print(publicKlong_time_since_last_message_ms / 1000);
  Serial.println(" sec");
  waterSensorsPublicKlongLastDataReceivedMS = millis();
}

int getPreferredSensorRefreshFrequencyInSeconds() {
  return waterSensorsReadFrequencyOptions[waterSensorsReadFrequencySelected];
}

int getPreferredSensorRefreshFrequencyInSeconds(int selectedIndex) {
  return waterSensorsReadFrequencyOptions[selectedIndex];
}

char strBuff[20];
char * getPreferredSensorRefreshFrequencyAsString() {
  int freq = getPreferredSensorRefreshFrequencyInSeconds();
  if (freq < 60) {
    snprintf(strBuff, sizeof(strBuff), "%i sec", freq);
  } else {
    snprintf(strBuff, sizeof(strBuff), "%i min", freq / 60);
  }
  return strBuff;
}

char * getPreferredSensorRefreshFrequencyAsString(int selectedIndex) {
  int freq = getPreferredSensorRefreshFrequencyInSeconds(selectedIndex);
  if (freq < 60) {
    snprintf(strBuff, sizeof(strBuff), "%i sec", freq);
  } else {
    snprintf(strBuff, sizeof(strBuff), "%i min", freq / 60);
  }
  return strBuff;
}


#pragma region Pump Controllers

void startPump1() {
  int previousState = publicKlong_powered;
  publicKlong_powered = 1;
  sendRF433MhzPowerInfo(publicKlong_powered);
  digitalWrite(PUBLIC_KLONG_RELAY_PIN, LOW);
  digitalWrite(PUBLIC_KLONG_RELAY_LED_PIN, !digitalRead(PUBLIC_KLONG_RELAY_PIN));
  if (previousState == 1) return;  // Do nothing if it was already in required state
  publicKlong_operating_start = millis();
  publicKlong_operating_time_sec = 0;
}

void stopPump1() {
  int previousState = publicKlong_powered;
  publicKlong_powered = 0;
  sendRF433MhzPowerInfo(publicKlong_powered);
  digitalWrite(PUBLIC_KLONG_RELAY_PIN, HIGH);
  digitalWrite(PUBLIC_KLONG_RELAY_LED_PIN, !digitalRead(PUBLIC_KLONG_RELAY_PIN));
  if (previousState == 0) return;  // Do nothing if already required state
  publicKlong_operating_start = 0;
  publicKlong_operating_time_sec = 0;
}

void startPump2() {
  if (southKlong_powered) return;  // Do nothing if already required state
  southKlong_powered = 1;
  southKlong_operating_start = millis();
  southKlong_operating_time_sec = 0;
  digitalWrite(SOUTH_KLONG_RELAY_PIN, LOW);
  digitalWrite(SOUTH_KLONG_RELAY_LED_PIN, !digitalRead(SOUTH_KLONG_RELAY_PIN));
}

void stopPump2() {
  if (!southKlong_powered) return;  // Do nothing if already required state
  southKlong_powered = 0;
  southKlong_operating_start = 0;
  southKlong_operating_time_sec = 0;
  digitalWrite(SOUTH_KLONG_RELAY_PIN, HIGH);
  digitalWrite(SOUTH_KLONG_RELAY_LED_PIN, !digitalRead(SOUTH_KLONG_RELAY_PIN));
}

#pragma endregion

void calculateWaterLevels() {
  publicKlong_CurrentWaterLevel = publicKlongSensor_VirtualZero - publicKlong_SensorWaterDistance;
  southKlong_CurrentWaterLevel = southKlongSensor_VirtualZero - southKlong_SensorWaterDistance;
}

void updatePumpTimers() {
  // If either timers data show a CPU roll-over, reset timers
  if (millis() < publicKlong_operating_start || millis() < southKlong_operating_start) {
    publicKlong_operating_start = 0;
    publicKlong_operating_time_sec = 0;
    publicKlong_operating_time_min = 0.0;
    return;
  } 

  // Public Klong
  unsigned long publicKlong_ElapseMS = millis() - publicKlong_operating_start;
  publicKlong_operating_time_sec = publicKlong_ElapseMS / 1000;
  publicKlong_operating_time_min = (publicKlong_operating_time_sec / 60) + ((publicKlong_operating_time_sec % 60) / 100.0);
  
  // South Klong
  unsigned long southKlong_ElapseMS = millis() - southKlong_operating_start;
  southKlong_operating_time_sec = southKlong_ElapseMS / 1000;
  southKlong_operating_time_min = (southKlong_operating_time_sec / 60) + ((southKlong_operating_time_sec % 60) / 100.0);
}

void analyzeWaterLevels() {
  // Analyze Water Levels to conclude what to do
  systemAnalysis = "Checking...";

  updatePumpTimers();

  Serial.print("DEBUG ANALYSIS STARTED: ");

  // Check water levels to make decisions:
  // - If South Klong is full, no need to pump
  // - If South Klong needs wather, check if there is enough water in Public Klong to operate the pump
  // Override any decision if operations mode has been set to ON or OFF
  if (master_operations_mode ==  master_mode_forced_on) {
    // If Mode is FORCED ON then just turn the pump ON
    publicKlong_PumpDecision = 1;
    Serial.print("A");
  } else if (master_operations_mode ==  master_mode_forced_off) {
    // If Mode is FORCED OFF then just turn the pump OFF
    publicKlong_PumpDecision = 0;
    Serial.print("B");
  } else if ( publicKlong_SensorWaterDistance > 300 || publicKlong_SensorWaterDistance < 5) {
    // Cannot make decisions if the sensors returns unrealistic numbers
    // For protection, turn the pump OFF because we just don't know if there is water 
    systemAnalysis = "Invalid measure!";
    publicKlong_PumpDecision = 0;
    Serial.print("C");
  } else if (publicKlong_PumpMinimumWaterLevel > publicKlong_SensorWaterDistance) {
    // We have enough water to activate the PUMP, but there are more conditions...
    Serial.print("D");
    if ( (publicKlong_operating_time_min > publicKlong_overheat_protection_max_runtime_minutes) && !publicKlong_overheat_protection_activated) {
      // The Pump has been running for more than acceptable time
      // so we enable the overheat protection
      publicKlong_operating_time_min = 0;
      publicKlong_overheat_protection_kickoff_ms = millis();
      publicKlong_overheat_protection_activated = true;
      publicKlong_PumpDecision = 0;
      systemAnalysis = "Overheat protection activated";
      Serial.print("E");
    } else {
      // We have enough water, just need to check if we are in overheat protection or not
      Serial.print("F");
      if (publicKlong_overheat_protection_activated) {
        Serial.print("G");
        // As we are under overheat protection, check if it has been off long enough
        if (((millis() - publicKlong_overheat_protection_kickoff_ms) / 1000 / 60) >= publicKlong_overheat_protection_time_off_minutes) {
          // It's been OFF long enough for overheat protection, we can restart the pump
          publicKlong_overheat_protection_activated = false;
          publicKlong_PumpDecision = 1;
          systemAnalysis = "Overheat protection ended";
          Serial.print("H");
        } else {
          publicKlong_PumpDecision = 0;
          systemAnalysis = "Overheat protection activated";
          Serial.print("I");
        }
      } else {
        publicKlong_PumpDecision = 1;
        systemAnalysis = "OK to pump!";
        Serial.print("J");
      }
    }
  } else {
    publicKlong_PumpDecision = 0;
    systemAnalysis = "Water TOO LOW to pump";
    Serial.print("K");
  }

  Serial.println(); // End of Debug line

  if (publicKlong_PumpDecision == 1) {
    startPump1();
  } else {
    stopPump1();
  }

  // Send Water data over 433Mhz for statistics
  if (publicKlong_SensorWaterDistance > 999) {
    sendRF433MhzCode(DATA_PACKET_DEVICE_ID_PK, DATA_PACKET_DATATYPE_WLVL, 999 * 100);
  } else {
    sendRF433MhzCode(DATA_PACKET_DEVICE_ID_PK, DATA_PACKET_DATATYPE_WLVL, publicKlong_SensorWaterDistance * 100);
  }

}

void handleMasterOperationsChoice(String opsmode) {
  if (opsmode == master_mode_forced_on) {
    Serial.println("Switch the system to FORCED ON mode");
    startPump1();
  } else if (opsmode == master_mode_forced_off) {
    Serial.println("Switch the system to FORCED OFF mode");
    stopPump1();
  } else {
    Serial.println("Switch the system to AUTO mode");
    // opsmode == master_mode_automated 
    // Do nothing here, the next Loop cycle will take care of everything
  }
}


const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>BKO Garden System</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.0rem;}
    p {font-size: 3.0rem;}
    body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    table {border-collapse: collapse;margin-left:auto;margin-right:auto;}
    td, th {border: 1px solid #dddddd; text-align: left; padding: 8px;}
    tr:nth-child(even) { background-color: #dddddd; }
  </style>
</head>
<body>
  <h2>BKO Garden System</h2>
  %OPERATINGMODEPLACEHOLDER%
  <br/>
  %PUMPSTATUSPLACEHOLDER%
  <br/>
  %SYSTEMSTATUSPLACEHOLDER%
  <br/>
  %WATERLIMITPLACEHOLDER%
  <br/>
  %VERSIONPLACEHOLDER%
  <br/>
  %FIRMWAREPLACEHOLDER%
<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  window.addEventListener('load', onLoad);
  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    websocket = new WebSocket(gateway);
    websocket.onopen    = onOpen;
    websocket.onclose   = onClose;
    websocket.onmessage = onMessage; // <-- add this line
  }
  function onOpen(event) {
    console.log('Connection opened');
  }
  function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
  }
  function onMessage(event) {
    console.log('WS On Message:' + event.data);
    msg = JSON.parse(event.data)
    var sensorLevel = msg.sensor;
    var rawsensorlevel = msg.rawsensor;
    var lastpkespnowmsg = msg.lastpkespnowmsg;
    var frequency = msg.frequency;
    var minlvl = msg.minlvl;
    var opsmode = msg.opsmode;
    var sysanalysis = msg.sysanalysis;
    var powerpk = msg.powerpk;
    var powerpktimer = msg.powerpktimer;
    var timetoanalysis = msg.timetoanalysis;
    var overheattimeleft = msg.overheattimeleft;
    var overheatprotectiontime = msg.overheatprotectiontime;
    var overheatprotectionactivated = msg.overheatprotectionactivated;
    var overheatprotectionmaxruntime = msg.overheatprotectionmaxruntime;
    document.getElementById('sensor').innerHTML = sensorLevel + "&nbsp;cm (" + lastpkespnowmsg + " sec&nbsp;ago)<br/>Raw: " + rawsensorlevel + "cm";
    document.getElementById('frequency').innerHTML = frequency;
    document.getElementById('minlvl').innerHTML = minlvl + " cm";
    document.getElementById('opsmode').innerHTML = opsmode;
    document.getElementById('sysanalysis').innerHTML = sysanalysis;
    document.getElementById('timetoanalysis').innerHTML = timetoanalysis;
    elempumpstatus = document.getElementById('pumpstatus');
    elempumpstatusbar = document.getElementById('pumpstatusbar');
    if (opsmode == "AUTO" && powerpk == 1) {
      elempumpstatus.innerHTML = "ON (" + powerpktimer + " min)";
      elempumpstatusbar.innerHTML = "<div style='width:280px;margin:auto;background:green;color:white;padding:5px 5px 5px 5px;'>Pumping</div><br/>";
    } else if (opsmode == "AUTO" && powerpk == 0) {
      elempumpstatus.innerHTML = "OFF";
      elempumpstatusbar.innerHTML = "<div style='width:280px;margin:auto;background:red;color:white;padding:5px 5px 5px 5px;'>Not pumping</div><br/>";
    } else if (opsmode == "FORCED OFF") {
      elempumpstatus.innerHTML = "Forced OFF";
      elempumpstatusbar.innerHTML = "<div style='width:280px;margin:auto;background:grey;color:white;padding:5px 5px 5px 5px;'>Forced Offline</div><br/>";
    } else if (opsmode == "FORCED ON") {
      elempumpstatus.innerHTML = "Forced ON (" + powerpktimer + " min)";
      elempumpstatusbar.innerHTML = "<div style='width:280px;margin:auto;background:green;color:white;padding:5px 5px 5px 5px;'>Pumping (forced)</div><br/>";
    } else {
      elempumpstatus.innerHTML = "UNKNOWN MODE!!!";
      elempumpstatusbar.innerHTML = "<div style='width:280px;margin:auto;background:green;color:white;padding:5px 5px 5px 5px;'>Forced Pumping</div><br/>";
    }
    var overheatprotectioninfo = "<span style='text-decoration: underline;'>Configuration:</span><br/>";
    overheatprotectioninfo += "\u2022 " + overheatprotectionmaxruntime + " minutes on,<br/>";
    overheatprotectioninfo += "\u2022 " + overheatprotectiontime + " minutes off.<br/>";
    if (overheatprotectionactivated == 1) {
      document.getElementById('overheatmaxtime').innerHTML = "" + overheatprotectioninfo + "<span style='color:red'>Active Overheat Protection<br/> time left: " + overheattimeleft + " sec.</span>";
    } else {
      document.getElementById('overheatmaxtime').innerHTML = "" + overheatprotectioninfo;
    }
  }

  function onLoad(event) {
    initWebSocket();
    initButton();
  }
  function initButton() {
    //document.getElementById('button').addEventListener('click', toggle);
  }
  function toggle(){
    websocket.send('toggle');
  }
</script>  
</body>
</html>
)rawliteral";

// Replaces placeholder with button section in your web page
String htmlProcessor(const String& var){
  // Serial.print("HTML Processor for: ");
  // Serial.println(var);

  if (var == "VERSIONPLACEHOLDER"){
    String version = "";
    version += DEVICE_NAME + " (" + VERSION + ")<br/>";
    return version;
  }
  
  if (var == "OPERATINGMODEPLACEHOLDER"){
    String html = "";
    html += "Operating Mode: <span id='opsmode'>" + master_operations_mode + "</span><br/>";
    html += "<a href='/mastermode?mode=on'>Force ON</a> ";
    html += "<a href='/mastermode?mode=off'>Force OFF</a> ";
    html += "<a href='/mastermode?mode=auto'>AUTO</a><br/>";
    html += "<br/>";
    return html;
  }
  
  if (var == "WATERLIMITPLACEHOLDER"){
    String html = "";
    html += "<form action='/setmin'>";
    html += "<label for='lvl'>Min Level:</label> ";
    html += "<input type='text' style='width:30px' id='flvl' name='lvl' value='" + String(publicKlong_PumpMinimumWaterLevel) + "'> cm ";
    html += "<input type='submit' value='Update'>";
    html += "</form>";
    return html;
  }
  
  if (var == "PUMPSTATUSPLACEHOLDER"){
    String html = "<span id='pumpstatusbar'>";
    if  (publicKlong_PumpDecision == 1) {
      html += "<div style='width:280px;margin:auto;background:green;color:white;padding:5px 5px 5px 5px;'>Pumping</div><br/>";
    } else {
      if ( master_operations_mode == master_mode_forced_off) {
        html += "<div style='width:280px;margin:auto;background:grey;color:white;padding:5px 5px 5px 5px;'>SYSTEM OFFLINE</div><br/>";
      } else {
        html += "<div style='width:280px;margin:auto;background:red;color:white;padding:5px 5px 5px 5px;'>Not pumping</div><br/>";
      }
    }
    html += "</span>";
    return html;
  }

  if (var == "SYSTEMSTATUSPLACEHOLDER"){
    String html = "";
    html += "<table>";
    html += "  <tr>";
    html += "    <td>Frequency:</td>";
    html += "    <td><span id='frequency'>" + String(getPreferredSensorRefreshFrequencyAsString()) + "</span> ";
    html += "       <form action='/frequency'>";
    html += "         <select name='freq' id='freq'>";
    html += "           <option value='0'>1 sec</option>";
    html += "           <option value='1'>5 sec</option>";
    html += "           <option value='2'>10 sec</option>";
    html += "           <option value='3'>15 sec</option>";
    html += "           <option value='4'>30 sec</option>";
    html += "           <option value='5'>1 min</option>";
    html += "           <option value='6'>2 min</option>";
    html += "           <option value='7'>5 min</option>";
    html += "           <option value='8'>10 min</option>";
    html += "           <option value='9'>15 min</option>";
    html += "           <option value='10'>30 min</option>";
    html += "           <option value='11'>45 min</option>";
    html += "           <option value='12'>1 hr</option>";
    html += "           <option value='13'>2 hrs</option>";
    html += "         </select>";
    html += "       <input type='submit' value='Set'>";
    html += "     </form>";
    html += "    </td>";
    html += "  </tr>";
    html += "  <tr>";
    html += "    <td>Required:</td>";
    html += "    <td id='minlvl'>" + String(publicKlong_PumpMinimumWaterLevel) + " cm</td>";
    html += "  </tr>";
    html += "  <tr>";
    html += "    <td>Sensor:</td>";
    html += "    <td id='sensor'>" + String(publicKlong_SensorWaterDistance) + "</td>";
    html += "  </tr>";

    html += "  <tr>";
    html += "  <td>Pump:</td>";
    html += "  <td id='pumpstatus'>";
    if (publicKlong_PumpDecision == 1) {
      if (master_operations_mode == master_mode_forced_on) {
        html += "Forced ON";
      } else {
        html += "ON";
      }
      html += " (";
      html += String(publicKlong_operating_time_min);
      html += " min)";
      html += "</td>";
    } else {
      if (master_operations_mode == master_mode_forced_off) {
        html += "Forced OFF";
      } else {
        html += "OFF";
      }
      html += "</td>";
    }
    html += "  </tr>";

    html += "  <tr>";
    html += "    <td>Status:</td>";
    html += "    <td id='sysanalysis'>" + systemAnalysis + "</td>";
    html += "  </tr>";

    html += "  <tr>";
    html += "    <td>Overheat<br/>Protection:</td>";
    html += "    <td id='overheatmaxtime'></td>";
    html += "  </tr>";

    html += "</table><br/>";

    html += "Next Evaluation: <span id='timetoanalysis'></span><br/>";
    return html;
  }
  
  if (var == "FIRMWAREPLACEHOLDER") {
    String html = "";
    html += "Firmware: " + String(__DATE__) + " " + String(__TIME__) + "<br/>";
    html += "<a href='/update'>Update Firmware</a><br/>";
    html += "<a href='/reboot'>Reboot</a>";
    return html;
  }
  
  return String();
}

// WEBSOCKET functions
// -------------------
void notifyClients() {
  // Serial.println("WebSocket: Notify Clients");
  String json = "";
  json += "{";
  json += "\"sensor\":" + String(publicKlong_SensorWaterDistance);
  json += ",\"rawsensor\":" + String(publicKlong_RawDataWaterDistance);
  json += ",\"frequency\":\"" + String(getPreferredSensorRefreshFrequencyAsString()) + "\"";
  json += ",\"minlvl\":\"" + String(publicKlong_PumpMinimumWaterLevel) + "\"";
  json += ",\"opsmode\":\"" + master_operations_mode + "\"";
  json += ",\"powerpk\":\"" + String(publicKlong_powered) + "\"";
  json += ",\"powerpktimer\":\"" + String(publicKlong_operating_time_min) + "\"";
  json += ",\"sysanalysis\":\"" + systemAnalysis + "\"";
  json += ",\"timetoanalysis\":\"" + String(getPreferredSensorRefreshFrequencyInSeconds() - (millis() - waterSensorsLastReadTickerMS) / 1000) + "s\"";
  json += ",\"lastpkespnowmsg\":\"" + String((millis() - waterSensorsPublicKlongLastDataReceivedMS) / 1000.0) + "\"";
  if (publicKlong_overheat_protection_activated) {
    json += ",\"overheattimeleft\":\"" + String((publicKlong_overheat_protection_time_off_minutes * 60) - ((millis() - publicKlong_overheat_protection_kickoff_ms) / 1000)) + "\"";
  } else {
    json += ",\"overheattimeleft\":\"0\"";
  }
  json += ",\"overheatprotectionactivated\":\"" + String(publicKlong_overheat_protection_activated) + "\"";
  json += ",\"overheatprotectiontime\":\"" + String(publicKlong_overheat_protection_time_off_minutes) + "\"";
  json += ",\"overheatprotectionmaxruntime\":\"" + String(publicKlong_overheat_protection_max_runtime_minutes) + "\"";
  json += "}";
  ws.textAll(json);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    if (strcmp((char*)data, "toggle") == 0) {
      // ledState = !ledState;
      notifyClients();
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void initWebSocket() {
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

bool handleSetOpsMode(String opsMode) {
  if (opsMode == "on") master_operations_mode = master_mode_forced_on;
  else if (opsMode == "off") master_operations_mode = master_mode_forced_off;
  else if (opsMode == "auto") master_operations_mode = master_mode_automated;
  else {
    master_operations_mode = master_mode_forced_off;
    return false;
  }
  int results = preferences.putString(prefMasterOperationsMode, master_operations_mode);
  if (results == 0) {
    Serial.println("Preferences: An error occured while saving Master Operations Mode");
  } else {
    Serial.print("Preferences: Master Operations Mode saved: ");
    Serial.print(opsMode);
  }      
  return true;
}

// ***********************
// *****  S E T U P  *****
// ***********************
void setup() {

  // Basic setup stuff for ESP communications
  Serial.begin(115200);
  Serial.println();
  Serial.println("Initialize...");

  // IMMEDIATELY configure the PINs that control relays so they
  // are not turned ON too long while booting... (will flicker)
  // Configure Pump Relay and LED PINs
  pinMode(PUBLIC_KLONG_RELAY_PIN, OUTPUT);
  pinMode(PUBLIC_KLONG_RELAY_LED_PIN, OUTPUT);
  digitalWrite(PUBLIC_KLONG_RELAY_PIN, HIGH);
  digitalWrite(PUBLIC_KLONG_RELAY_LED_PIN, !digitalRead(PUBLIC_KLONG_RELAY_PIN));

  pinMode(SOUTH_KLONG_RELAY_PIN, OUTPUT);
  pinMode(SOUTH_KLONG_RELAY_LED_PIN, OUTPUT);
  digitalWrite(SOUTH_KLONG_RELAY_PIN, HIGH);
  digitalWrite(SOUTH_KLONG_RELAY_LED_PIN, !digitalRead(SOUTH_KLONG_RELAY_PIN));

  // Onboard LED off by default
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  displaySplashScreen();

  #ifdef DEV_WIFI_MODE_AP
    // Inisitalize WIFI as an ACCESS POINT (AP)
    Serial.println("WIFI operating as ACCESS POINT (AP) mode.");
    WiFi.softAP(ssid, password);
    IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    Serial.print("Channel: ");
    Serial.println(WiFi.channel());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
    Serial.print("Soft AP MAC address: ");
    Serial.println(WiFi.softAPmacAddress());
  #else
    // Initialize WIFI as a STATION (Client) and wait for connection
    Serial.println("WIFI operating as STATION mode.");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    IP = WiFi.localIP();
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("Channel: ");
    Serial.println(WiFi.channel());
    Serial.print("IP address: ");
    Serial.println(IP);
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
    Serial.print("Soft AP MAC address: ");
    Serial.println(WiFi.softAPmacAddress());
  #endif

  // Initialize WebSocket support
  initWebSocket();

  // Initialize OTA (Over-The-Air ElegantOTA system)
  // Start default webserver
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", index_html, htmlProcessor);
    // request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  // Support Reboot ESP32 from web page
  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->redirect("/");
    delay(1000);
    ESP.restart();
  });

  // Handle Set Frequency
  server.on("/frequency", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("Web request /frequency");
    needRevaluationOfSituation = true;  // Always re-evaluate everything after a web page request
    String fq;
    if (request->hasParam("freq")) {
      fq = request->getParam("freq")->value();
      waterSensorsReadFrequencySelected = fq.toInt();
      int results = preferences.putInt(prefNameWaterSensorReadFrequency, waterSensorsReadFrequencySelected);
      if (results == 0) {
        Serial.println("Preferences: An error occured while saving Frequency settings");
      } else {
        Serial.println("Preferences: Frequency settings saved");
      }     
    }
    else {
      String msg = "Invalid request: ?freq=xx parameter is required!";
      request->send(200, "text/plain", msg);
      return;
    }
    Serial.print("Web request /frequency: New frequency set to (index) ");
    Serial.println(waterSensorsReadFrequencySelected);
    request->redirect("/");
  });

  // Handle Set Master Operations Mode
  server.on("/mastermode", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("Web request /mastermode");
    needRevaluationOfSituation = true;  // Always re-evaluate everything after a web page request
    String opsMode;
    if (request->hasParam("mode")) {
      opsMode = request->getParam("mode")->value();
      Serial.print("Web request /mastermode: ");
      Serial.println(opsMode);
      if (!handleSetOpsMode(opsMode)) {
        String msg = "INVALID REQUEST:  /mastermode?mode=xx Valid values are on / off / auto";
        request->send(200, "text/plain", msg);
        return;
      };
    }
    else {
      String msg = "Invalid request, ?mode=xx parameter is required!";
      request->send(200, "text/plain", msg);
      return;
    }
    analyzeWaterLevels();
    request->redirect("/");
  });

  // Handle Set Minimum Water level required
  server.on("/setmin", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("Web request /setmin");
    needRevaluationOfSituation = true;  // Always re-evaluate everything after a web page request
    String minLvl;
    if (request->hasParam("lvl")) {
      minLvl = request->getParam("lvl")->value();
      publicKlong_PumpMinimumWaterLevel = minLvl.toInt();
      int results = preferences.putInt(prefRequiredDistancePublicKlong, publicKlong_PumpMinimumWaterLevel);
      if (results == 0) {
        Serial.println("Preferences: An error occured while saving Minimum Water Level");
      } else {
        Serial.print("Preferences: Minimum Water Level saved: ");
        Serial.println(minLvl);
        Serial.print(" cm.");
      }      
    }
    else {
      String msg = "Invalid request: ?lvl=xx parameter is required!";
      request->send(200, "text/plain", msg);
      return;
    }
    Serial.print("Web request /setmin: New level set to ");
    Serial.println(minLvl);
    request->redirect("/");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

 
  // CONFIGURE ALL SMART BUTTONS
  pinMode(BUTTON_MENU_PIN, INPUT_PULLUP);
  pinMode(BUTTON_DEC_OPTIONS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_INC_OPTIONS_PIN, INPUT_PULLUP);
  pinMode(BUTTON_CONFIRM_PIN, INPUT_PULLUP);
  ButtonConfig* buttonConfig = ButtonConfig::getSystemButtonConfig();
  buttonConfig->setEventHandler(btnHandleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);

  myRadioSignalSwitch.enableReceive(GPIO_RF_PIN);  // Receiver input on interrupt 0 (D2) OR D3???
  myRadioSignalSwitch.enableTransmit(GPIO_TRANSMIT_PIN);  

  // Confirgure Ultrasounic Distance/Meter Sensors Pins
  pinMode(sensorPublicKlong_trigPin, OUTPUT); // Sets the sensorPublicKlong_trigPin as an Output
  pinMode(sensorPublicKlong_echoPin, INPUT); // Sets the sensorPublicKlong_echoPin as an Input
  pinMode(sensorSouthKlong_trigPin, OUTPUT); // Sets the sensorPublicKlong_trigPin as an Output
  pinMode(sensorSouthKlong_echoPin, INPUT); // Sets the sensorPublicKlong_echoPin as an Input

  // Handle Preferences
  preferences.begin("bko_dmz_dev1", false);
  waterSensorsReadFrequencySelected = preferences.getInt(prefNameWaterSensorReadFrequency, waterSensorsReadFrequencySelected);
  publicKlong_PumpMinimumWaterLevel = preferences.getInt(prefRequiredDistancePublicKlong, prefRequiredDistancePublicKlong_default);
  master_operations_mode = preferences.getString(prefMasterOperationsMode, master_operations_mode);

  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  // esp_now_set_self_role((ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(onKlongDataReciever);

  delay(2000); // Wait a bit more here as we show the splash screen
  Serial.println("Initialized!");
}

// Send RF Smart codes over 433Mhz
// - Device ID from 10 to 99,  e.g. device id 27
// - Data Type from 0 to 9     e.g. data type 1 (distance measure)
// - Data value xxxx.xx        e.g. 124.78cm 
// The value for this example will be:
//   271012478
// Data Types are:
//  1: Distance Measure (klong water level for example)
//  2: Pump status
void sendRF433MhzCode(int deviceid, int datatype, int value) {
  // Check if parameters are acceptable
  if (    deviceid < 10 || deviceid > 99
       || datatype < 0  || datatype > 9
       || value < 0     || value > 999999 ) {
    Serial.print("433Mhz: Out of range value! ");
    Serial.print("DeviceID: ");
    Serial.print(deviceid);
    Serial.print("DataType: ");
    Serial.print(datatype);
    Serial.print("Value: ");
    Serial.println(value);
  }
  // Build the binary string for the final code
  uint32_t intValue = deviceid * 10000000 + datatype * 1000000 + value;
  Serial.print("Sending 433Mhz code: ");
  Serial.println(intValue);
  myRadioSignalSwitch.setPulseLength(325);
  // myRadioSignalSwitch.setProtocol(1);
  myRadioSignalSwitch.setRepeatTransmit(3);
  myRadioSignalSwitch.send(intValue, 32); 
  delay(200);
}

// Send RF Code over 433Mhz
void sendRF433MhzPowerInfo(int on) {
  if (on) { 
    sendRF433MhzCode(DATA_PACKET_DEVICE_ID_PK, DATA_PACKET_DATATYPE_PWR, 2);
  } else { 
    if (publicKlong_overheat_protection_activated) {
      sendRF433MhzCode(DATA_PACKET_DEVICE_ID_PK, DATA_PACKET_DATATYPE_PWR, 1);
    } else {
      sendRF433MhzCode(DATA_PACKET_DEVICE_ID_PK, DATA_PACKET_DATATYPE_PWR, 0);
    }
  }    
}

// Based on 433Mhz received code, get the name matching the command (code)
String getCommandName(int v) {
  // Phenix RM 2  (DIP: 01111)
  if (v == 4195665) { return "PH2-A-ON"; }  else 
  if (v == 4195668) { return "PH2-A-OFF"; } else
  if (v == 4198737) { return "PH2-B-ON"; }  else
  if (v == 4198740) { return "PH2-B-OFF"; } else
  if (v == 4199505) { return "PH2-C-ON"; }  else
  if (v == 4199508) { return "PH2-C-OFF"; } else
  if (v == 4199697) { return "PH2-D-ON"; }  else
  if (v == 4199700) { return "PH2-D-OFF"; } else
  // Phenix RM 1  (DIP: 00000)
  if (v == 5588305) { return "PH1-A-ON"; }  else 
  if (v == 5588308) { return "PH1-A-OFF"; } else
  if (v == 5591377) { return "PH1-B-ON"; }  else
  if (v == 5591380) { return "PH1-B-OFF"; } else
  if (v == 5592145) { return "PH1-C-ON"; }  else
  if (v == 5592148) { return "PH1-C-OFF"; } else
  if (v == 5592337) { return "PH1-D-ON"; }  else
  if (v == 5592340) { return "PH1-D-OFF"; } else
  { return "N/A"; }
}

void displayDeviceStatus(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // Display Line 1: Device Identification and firmware version
  display.setCursor(128 - (5 * PIXELS_PER_CHAR), SSD_L1);
  display.print(VERSION);
  display.setCursor(0, SSD_L1);
  display.print("#");
  display.print(DEVICE_NAME);

  display.setCursor(0, SSD_L2);
  display.print("MAC:");
  display.print(WiFi.softAPmacAddress());

  // Display Line 4: Water Levels (Headers)
  display.setCursor(0, SSD_L3);
  display.print("PK:");
  char buff[5];
  sprintf_P(buff, PSTR("%.02f"), (millis() - waterSensorsPublicKlongLastDataReceivedMS) / 1000.0); //results like 23.500001
  display.print(String(buff));
  display.print("s");
  display.setCursor(67, SSD_L3);
  display.print("Required");

  // Display Line 5: Water Levels (Values)
  display.setCursor(0, SSD_L4);
  // display.print(southKlong_CurrentWaterLevel);
  display.print(publicKlong_SensorWaterDistance);
  display.print("cm");
  display.setCursor(67, SSD_L4);
  display.print(publicKlong_PumpMinimumWaterLevel);
  display.print("cm");
  
  // Display Line 6: Time in seconds until next measure/action
  display.setCursor(0, SSD_L5);
  display.print("Fq: ");
  display.print(getPreferredSensorRefreshFrequencyAsString());
  display.setCursor(67, SSD_L5);
  display.print("Next: ");
  display.print(getPreferredSensorRefreshFrequencyInSeconds() - (millis() - waterSensorsLastReadTickerMS) / 1000);
  display.print("s");

  // Display Line 7: Pump Status
  display.setCursor(0, SSD_L6);
  display.print("Pump:");
  if (publicKlong_PumpDecision == 1) {
    if (master_operations_mode == master_mode_forced_on) {
      display.print("F-ON");
      systemAnalysis = "*** Pump FORCED ON";
    } else {
      display.print("ON (A)");
    }
    display.setCursor(64, SSD_L6);
    display.print(" ");
    display.print(publicKlong_operating_time_min);
    display.print(" min");
  } else {
    if (master_operations_mode == master_mode_forced_off) {
      display.print("F-OFF");
      systemAnalysis = "*** FORCE OFFLINE";
    } else {
      display.print("OFF (AUTO)");
    }
  }

  // display.print("Mode:");
  // display.print("AUTO");
  
  // Display Line 8: Timers & Operating Mode / Menu
  display.setCursor(0, SSD_L7);
  display.print(systemAnalysis);
  display.setCursor(64, SSD_L7);
  // display.print("254 min");

  // Refresh the display
  display.display();

}

void displayWifiStatus(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(128 - (5 * PIXELS_PER_CHAR), SSD_L1);
  display.print(VERSION);
  display.setCursor(0, SSD_L1);
  display.print("#");
  display.print(DEVICE_NAME);

  display.setCursor(0, SSD_L2);
  display.print("WIFI Mode: ");
  display.println(wifimode);
  display.println("Name:");
  display.println(ssid);
  #ifdef DEV_WIFI_MODE_AP
    display.println("Pwd:");
    display.println(password);
  #endif
  display.print("IP:");
  display.println(IP.toString());

  display.display();
}

void displayFrequencySetup(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, SSD_L1);
  display.println("Frequency Setup:");
  display.println("");
  display.print("Current: ");
  display.println(getPreferredSensorRefreshFrequencyAsString());
  display.print("New:     ");
  display.println(getPreferredSensorRefreshFrequencyAsString(waterSensorsReadFrequencySelection));
  display.println("");
  display.println("Change: -/+ buttons");
  display.println("Save: CONFIRM button");

  display.display();
}

void displayInvalidDisplayMode(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, SSD_L1);
  display.println("INVALID");
  display.println("DISPLAY");
  display.println("MODE!");
  display.println(" ");
  display.println("Press button again!");

  display.display();
}

void displayRebootMenu(void) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, SSD_L1);
  display.println("System Reboot");
  display.println("");
  display.println("");
  display.println("CONFIRM to reboot");
  
  display.display();
}

void displaySplashScreen(void) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.setTextSize(2);
  display.println("* B K O  *");
  display.println("* GARDEN *");
  display.println("* SYSTEM *");
  display.setTextSize(1);
  display.println();
  display.print(VERSION);

  display.display();
  Serial.println("Waiting for Slash Screen on LCD display");
}

void updateDisplay(void) {
  if (DISPLAY_MODE == DISPLAY_MODE_OPERATIONS) displayDeviceStatus();
  else if (DISPLAY_MODE == DISPLAY_MODE_WIFI) displayWifiStatus();
  else if (DISPLAY_MODE == DISPLAY_MODE_FREQUENCY_SETUP) displayFrequencySetup();
  else if (DISPLAY_MODE == DISPLAY_MODE_REBOOT) displayRebootMenu();
  else displayInvalidDisplayMode();
}

void switchToNextDisplayMode(void) {
  if (DISPLAY_MODE == DISPLAY_MODE_OPERATIONS) DISPLAY_MODE = DISPLAY_MODE_WIFI;
  else if (DISPLAY_MODE == DISPLAY_MODE_WIFI) DISPLAY_MODE = DISPLAY_MODE_FREQUENCY_SETUP;
  else if (DISPLAY_MODE == DISPLAY_MODE_FREQUENCY_SETUP) DISPLAY_MODE = DISPLAY_MODE_REBOOT;
  else if (DISPLAY_MODE == DISPLAY_MODE_REBOOT) DISPLAY_MODE = -1;
  else if (DISPLAY_MODE == -1) DISPLAY_MODE = DISPLAY_MODE_OPERATIONS;
  // Special actions when switching display mode
  if (DISPLAY_MODE == DISPLAY_MODE_FREQUENCY_SETUP) {
    waterSensorsReadFrequencySelection = waterSensorsReadFrequencySelected;
  }
  updateDisplay();
}

// Output all information to serial console (RX/TX)
void printAllDeviceDataToSerial() {
  return;
  if (DISPLAY_MODE != DISPLAY_MODE_OPERATIONS) return;
  Serial.print("#");
  Serial.print(DEVICE_NAME);
  Serial.print(" Wifi: ");
  Serial.print(wifimode);
  Serial.print(" Ssid: ");
  Serial.print(ssid);
  Serial.print(" ");
  Serial.print(IP.toString());
  Serial.print(" Sth Kl: ");
  Serial.print(southKlong_CurrentWaterLevel);
  Serial.print("cm");
  Serial.print(" Public Klong: ");
  Serial.print(" Sensor: ");
  Serial.print(publicKlong_SensorWaterDistance);
  Serial.print(" Raw Sensor: ");
  Serial.print(publicKlong_RawDataWaterDistance);
  Serial.print(" Level: ");
  Serial.print(publicKlong_CurrentWaterLevel);
  Serial.print(" Pump: ");
  if (publicKlong_PumpDecision == 1) {
    Serial.print("ON");
    Serial.print(" ");
    Serial.print(publicKlong_operating_time_min);
    Serial.print(" min");
  } else {
    Serial.print("OFF");
  }
  Serial.print(" Mode:");
  Serial.print("AUTO");
  Serial.print(" Analysis: ");
  Serial.print(systemAnalysis);
  Serial.print(" Timer: ");
  Serial.print("254min");
  Serial.print(" Time to update: ");
  Serial.print(getPreferredSensorRefreshFrequencyInSeconds() - (millis() - waterSensorsLastReadTickerMS) / 1000);
  Serial.println("");
}

static const char* bin2tristate(const char* bin) {
  static char returnValue[50];
  int pos = 0;
  int pos2 = 0;
  while (bin[pos]!='\0' && bin[pos+1]!='\0') {
    if (bin[pos]=='0' && bin[pos+1]=='0') {
      returnValue[pos2] = '0';
    } else if (bin[pos]=='1' && bin[pos+1]=='1') {
      returnValue[pos2] = '1';
    } else if (bin[pos]=='0' && bin[pos+1]=='1') {
      returnValue[pos2] = 'F';
    } else {
      return "not applicable";
    }
    pos = pos+2;
    pos2++;
  }
  returnValue[pos2] = '\0';
  return returnValue;
}

static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength) {
  static char bin[64]; 
  unsigned int i=0;

  while (Dec > 0) {
    bin[32+i++] = ((Dec & 1) > 0) ? '1' : '0';
    Dec = Dec >> 1;
  }

  for (unsigned int j = 0; j< bitLength; j++) {
    if (j >= bitLength - i) {
      bin[j] = bin[ 31 + i - (j - (bitLength - i)) ];
    } else {
      bin[j] = '0';
    }
  }
  bin[bitLength] = '\0';
  
  return bin;
}

void debugRF433MhzOutput(unsigned long decimal, unsigned int length, unsigned int delay, unsigned int* raw, unsigned int protocol) {
  const char* b = dec2binWzerofill(decimal, length);
  Serial.print("Decimal: ");
  Serial.print(decimal);
  Serial.print(" (");
  Serial.print( length );
  Serial.print("Bit) Binary: ");
  Serial.print( b );
  Serial.print(" Tri-State: ");
  Serial.print( bin2tristate( b) );
  Serial.print(" PulseLength: ");
  Serial.print(delay);
  Serial.print(" microseconds");
  Serial.print(" Protocol: ");
  Serial.println(protocol);
  
  Serial.print("Raw data: ");
  for (unsigned int i=0; i<= length*2; i++) {
    Serial.print(raw[i]);
    Serial.print(",");
  }
  Serial.println();
  Serial.println();
}


void getWaterSensorsData() {

  // If the last esp-now message with sensor data is too old,
  // return zero to make it invalid by default

  if ((millis() - waterSensorsPublicKlongLastDataReceivedMS) / 1000 > 10) {
    publicKlong_SensorWaterDistance = 0.0;
    publicKlong_RawDataWaterDistance = 0.0;
  }

  // CODE BELOW IS OBSOLETE SINCE WE RECEIVE THIS INFO VIA ESP-NOW CONTROLLER
  // // PUBLIC KLONG
  // // ------------
  // // Trigger the sensor
  // digitalWrite(sensorPublicKlong_trigPin, LOW);
  // delayMicroseconds(2);
  // digitalWrite(sensorPublicKlong_trigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(sensorPublicKlong_trigPin, LOW);
  // // Reads the echoPin, returns the sound wave travel time in microseconds
  // duration = pulseIn(sensorPublicKlong_echoPin, HIGH);
  // publicKlong_SensorWaterDistance = duration * SOUND_SPEED/2;

  // // SOUTH KLONG
  // // ------------
  // // Trigger the sensor
  // digitalWrite(sensorSouthKlong_trigPin, LOW);
  // delayMicroseconds(2);
  // digitalWrite(sensorSouthKlong_trigPin, HIGH);
  // delayMicroseconds(10);
  // digitalWrite(sensorSouthKlong_trigPin, LOW);
  // // Reads the echoPin, returns the sound wave travel time in microseconds
  // duration = pulseIn(sensorSouthKlong_echoPin, HIGH);
  // southKlong_SensorWaterDistance = duration * SOUND_SPEED/2;
}

void loop() {
  btnMenu.check();
  btnDecOptions.check();
  btnIncOptions.check();
  btnConfirm.check();
  ws.cleanupClients();

  // Handle 433Mhz Communication Events
  if (myRadioSignalSwitch.available()) {
    lastRFvalue = myRadioSignalSwitch.getReceivedValue();
    debugRF433MhzOutput(myRadioSignalSwitch.getReceivedValue(), myRadioSignalSwitch.getReceivedBitlength(), myRadioSignalSwitch.getReceivedDelay(), myRadioSignalSwitch.getReceivedRawdata(), myRadioSignalSwitch.getReceivedProtocol());
    // if (lastRFvalue == RM2_A_ON)  { delay(800); digitalWrite(LED_BUILTIN, HIGH); sendRF433MhzPowerInfo(true); }
    // if (lastRFvalue == RM2_A_OFF) { delay(800); digitalWrite(LED_BUILTIN, LOW);  sendRF433MhzPowerInfo(false); }
    delay(1);
    myRadioSignalSwitch.resetAvailable();
  }

  if (lastRFvalue != previousRFvalue) {
    previousRFvalue = lastRFvalue;
  }

  // Do watever needed every second
  // ------------------------------
  if ((millis() - previousMS) > 1000) {
    previousMS = millis();
    printAllDeviceDataToSerial();
    // Gather sensors data but do not action pumps, this is a 1 second frequency section only
    getWaterSensorsData();
    calculateWaterLevels();
    updatePumpTimers();
    updateDisplay();
    notifyClients();
  }
  
  // Handle the Timer for Water Sensor analysis & Decision
  // based on preferred frequency settings
  // -----------------------------------------------------
  if ((millis() - waterSensorsLastReadTickerMS) > getPreferredSensorRefreshFrequencyInSeconds() * 1000
      || needRevaluationOfSituation) {
    waterSensorsLastReadTickerMS = millis();
    needRevaluationOfSituation = false;
    Serial.print("Time for analysis! ");
    Serial.print("Frequency: ");
    Serial.print(getPreferredSensorRefreshFrequencyAsString());
    Serial.print("  Elasped: ");
    Serial.print((millis() - waterSensorsLastReadTickerMS));
    Serial.println();

    // Distance Sensor measurement
    // ***************************
    getWaterSensorsData();
    calculateWaterLevels();
    analyzeWaterLevels();
    updateDisplay();

    // notifyClients();
  }

}



// MENU button has some event to look into...
void btnMenuEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  Serial.println("MENU: ");
  switch (eventType) {
    case AceButton::kEventClicked:
      Serial.println("CLICK");
      switchToNextDisplayMode();
      break;
  }
}

// DEC/OPTIONS button has some event to look into...
void btnDecOptionsEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  Serial.print("DEC_OPTIONS: ");
  switch (eventType) {
    case AceButton::kEventClicked:
      Serial.println("CLICK");
      if (DISPLAY_MODE == DISPLAY_MODE_FREQUENCY_SETUP) {
        Serial.print("SizeOf[]: ");
        Serial.println(sizeof(waterSensorsReadFrequencyOptions) / sizeof(waterSensorsReadFrequencyOptions[0]));
        if (waterSensorsReadFrequencySelection > 0) {
          waterSensorsReadFrequencySelection--;
        } else {
          waterSensorsReadFrequencySelection = sizeof(waterSensorsReadFrequencyOptions) / sizeof(waterSensorsReadFrequencyOptions[0])-1;
        }
        Serial.print(waterSensorsReadFrequencySelected);
        Serial.print("->");
        Serial.println(waterSensorsReadFrequencySelection);
        updateDisplay();
      } 
      if (DISPLAY_MODE == DISPLAY_MODE_OPERATIONS) {
          if (!handleSetOpsMode("off")) {
          Serial.println("Something went wrong. Valid values are on / off / auto");
          }
      }
      break;
  }
}

// INC/OPTIONS button has some event to look into...
void btnIncOptionsEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  Serial.print("INC_OPTIONS:");
  switch (eventType) {
    case AceButton::kEventClicked:
      Serial.println("CLICK");
      if (DISPLAY_MODE == DISPLAY_MODE_FREQUENCY_SETUP) {
        if (waterSensorsReadFrequencySelection < (sizeof(waterSensorsReadFrequencyOptions) / sizeof(waterSensorsReadFrequencyOptions[0]))-1) {
          waterSensorsReadFrequencySelection++;
        } else {
          waterSensorsReadFrequencySelection = 0;
        }
        updateDisplay();
      }
      if (DISPLAY_MODE == DISPLAY_MODE_OPERATIONS) {
          if (!handleSetOpsMode("on")) {
          Serial.println("Something went wrong. Valid values are on / off / auto");
          }
      }
      break;
  }
}

// CONFIRM/CANCEL button clicked, action depends on current screen displayed
void btnConfirmEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  Serial.print("CONFIRM/CANCEL: ");
  switch (eventType) {
    case AceButton::kEventClicked:
      Serial.println("CLICK");
      if (DISPLAY_MODE == DISPLAY_MODE_FREQUENCY_SETUP) {
        waterSensorsReadFrequencySelected = waterSensorsReadFrequencySelection;
        int results = preferences.putInt(prefNameWaterSensorReadFrequency, waterSensorsReadFrequencySelected);
        if (results == 0) {
          Serial.println("Preferences: An error occured while saving Frequency settings");
        } else {
          Serial.println("Preferences: Frequency settings saved");
        }
        DISPLAY_MODE = DISPLAY_MODE_OPERATIONS;
      }
      if (DISPLAY_MODE == DISPLAY_MODE_REBOOT) {
        ESP.restart();
      }
      if (DISPLAY_MODE == DISPLAY_MODE_OPERATIONS) {
          if (!handleSetOpsMode("auto")) {
          Serial.println("Something went wrong. Valid values are on / off / auto");
          }
      }
      updateDisplay();
      break;
  }
}

// ACE BUTTONs event handler
void btnHandleEvent(AceButton* button, uint8_t eventType, uint8_t buttonState) {
  uint8_t btnPIN = button->getPin();
  // uint8_t btnID = button->getId();
  // Print out a message for all events.
  // Serial.print(F("AceButton Event! "));
  // Serial.print(F(" ID: "));
  // Serial.print(btnID);
  // Serial.print(F(" PIN: "));
  // Serial.print(btnPIN);
  // Serial.print(F(" EventType: "));
  // Serial.print(eventType);
  // Serial.print(F(" buttonState: "));
  // Serial.println(buttonState);

  // Call the right Button Handler based on PIN
  switch (btnPIN) {
    case BUTTON_MENU_PIN:
      // Serial.println("MENU button event");
      btnMenuEvent(button, eventType, buttonState);
      break;
    case BUTTON_DEC_OPTIONS_PIN:
      // Serial.println("DEC_OPTIONS button event");
      btnDecOptionsEvent(button, eventType, buttonState);
      break;
    case BUTTON_INC_OPTIONS_PIN:
      // Serial.println("INC_OPTIONS button event");
      btnIncOptionsEvent(button, eventType, buttonState);
      break;
    case BUTTON_CONFIRM_PIN:
      // Serial.println("CONFIRM/CANCEL button event");
      btnConfirmEvent(button, eventType, buttonState);
      break;
  }
}

