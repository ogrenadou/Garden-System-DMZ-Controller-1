#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RCSwitch.h>
#include <AceButton.h>

// Includes needed for OTA Updates (Over-The-Air updates)
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define VERSION "v2.16"
#define DEVICE_NAME "BKO-DMZ-DEV1"

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
#define SSD_L7 54
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
float publicKlong_SensorWaterDistance = 0;    // In centimeters, the distance between the sensor and the water
float publicKlong_CurrentWaterLevel = 0.0;    // In centimeters, the current relative water level from the virtual zero level. 
                                              // n > 0 = water is above target, n < 0 = water is below target
int   publicKlong_PumpMinimumWaterLevel = 30;  // Distance from Sensor to water level needed to start the pump safely
int   publicKlong_PumpDecision = 0;
String systemAnalysis = "";
// Provide options for how often we check water and make decisions
// This helps for debug (more frequently in matter of seconds) or for production (every N minutes)
int   waterSensorsReadFrequencyOptions[] = { 1, 5, 10, 15, 30, 60, 2 * 60, 5 * 60, 10 * 60, 15 * 60, 30 * 60, 45 * 60, 60 * 60, 120 * 60 };  // Intervals in seconds
int   waterSensorsReadFrequencySelected = 5;  // The INDEX of the option selected (default index 5: 1 minute)
int   waterSensorsReadFrequencySelection = waterSensorsReadFrequencySelected;  // The INDEX of the option selected
unsigned long waterSensorsLastReadTickerMS = 0;

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

// Dynamic Preferences for all appropriate settings
#define prefNameWaterSensorReadFrequency "waterSensReadFq"
#define prefRequiredDistancePublicKlong  "waterSensMinLvl"
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
  if (publicKlong_powered) return;  // Do nothing if already required state
  publicKlong_powered = 1;
  publicKlong_operating_start = millis();
  publicKlong_operating_time_sec = 0;
  digitalWrite(PUBLIC_KLONG_RELAY_PIN, LOW);
  digitalWrite(PUBLIC_KLONG_RELAY_LED_PIN, !digitalRead(PUBLIC_KLONG_RELAY_PIN));
}

void stopPump1() {
  if (!publicKlong_powered) return;  // Do nothing if already required state
  publicKlong_powered = 0;
  publicKlong_operating_start = 0;
  publicKlong_operating_time_sec = 0;
  digitalWrite(PUBLIC_KLONG_RELAY_PIN, HIGH);
  digitalWrite(PUBLIC_KLONG_RELAY_LED_PIN, !digitalRead(PUBLIC_KLONG_RELAY_PIN));
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

  // Cannot make decisions if the sensors returns unrealistic numbers
  // If so, turn off the pump for safety
  if ( publicKlong_SensorWaterDistance > 300 || publicKlong_SensorWaterDistance < 5) {
    // stopPump1();
    // stopPump2();
    systemAnalysis = "Invalid measure!";
    // Keep the pump as it was
    return;
  }

  // Check water levels to make decisions:
  // - If South Klong is full, no need to pump
  // - If South Klong needs wather, check if there is enough water in Public Klong to operate the pump

  if (publicKlong_PumpMinimumWaterLevel > publicKlong_SensorWaterDistance) {
    publicKlong_PumpDecision = 1;
    systemAnalysis = "OK to pump!";
    startPump1();
    return;
  } else {
    publicKlong_PumpDecision = 0;
    systemAnalysis = "Water TOO LOW to pump";
    stopPump1();
    return;
  }
  
  // Fancy version when I can measure South Klong to avoid overflow
  // if (southKlong_CurrentWaterLevel >= 0) {
  //   publicKlong_PumpDecision = 0;
  //   systemAnalysis = "All set!";
  //   stopPump1();
  // } else { 
  //   if (publicKlong_SensorWaterDistance < publicKlong_PumpMinimumWaterLevel) {
  //     publicKlong_PumpDecision = 1;
  //     systemAnalysis = "Pump!";
  //     startPump1();
  //   } else {
  //     publicKlong_PumpDecision = -1;
  //     systemAnalysis = "Cant Pump";
  //     stopPump1();
  //   }
  // }

  // Nothing has been decided at this point (BUG!!!)
  // Let's turn the pump OFF to be safe
  publicKlong_PumpDecision = 0;
  systemAnalysis = "!!! Cannot decide !!!";
  stopPump1();
}

// ***********************
// *****  S E T U P  *****
// ***********************
void setup() {

  // Basic setup stuff for ESP communications
  Serial.begin(115200);
  Serial.println();
  Serial.println("Initialize...");
  
  #ifdef DEV_WIFI_MODE_STATION
    // Initialize WIFI as a STATION (Client) and wait for connection
    Serial.println("WIFI operating as STATION mode.");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.println("");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  #else
  // Inisitalize WIFI as an ACCESS POINT (AP)
    Serial.println("WIFI operating as ACCESS POINT (AP) mode.");
    WiFi.softAP(ssid, password);
    IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
  #endif


  // Initialize OTA (Over-The-Air ElegantOTA system)
  // Start default webserver
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });
  // Handle Set Minimum Water level required
  server.on("/setmin", HTTP_GET, [] (AsyncWebServerRequest *request) {
    Serial.print("Web request /setmin");
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
      minLvl = "Invalid request";
    }
    Serial.print("Web request /setmin: ");
    Serial.println(minLvl);
    request->send(200, "text/plain", "OK");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

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

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

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

  displaySplashScreen();

  Serial.println("Initialized!");
}

void sendCode(bool on) {
  myRadioSignalSwitch.setPulseLength(325);
  myRadioSignalSwitch.setProtocol(1);
  myRadioSignalSwitch.setRepeatTransmit(5);

  Serial.print("Sending Code ");
  Serial.println(on);
  
  if (on) 
  { // RM2-B-ON
    myRadioSignalSwitch.send("010000000001000101010001"); 
    virtualPlug2A = true;
    digitalWrite(LED_BUILTIN, HIGH);
  }     
  else 
  { // RM2-B-OFF
    myRadioSignalSwitch.send("010000000001000101010100"); 
    virtualPlug2A = false;
    digitalWrite(LED_BUILTIN, LOW);
  }    

  updateDisplay();
}

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

  // Display Line 4: Water Levels (Headers)
  display.setCursor(0, SSD_L3);
  display.print("Sensor");
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
    display.print("ON");
    display.setCursor(64, SSD_L6);
    display.print(" ");
    display.print(publicKlong_operating_time_min);
    display.print(" min");
  } else {
    display.print("OFF");
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
  delay(4000);
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

void output(unsigned long decimal, unsigned int length, unsigned int delay, unsigned int* raw, unsigned int protocol) {
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
  // PUBLIC KLONG
  // ------------
  // Trigger the sensor
  digitalWrite(sensorPublicKlong_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorPublicKlong_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorPublicKlong_trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(sensorPublicKlong_echoPin, HIGH);
  publicKlong_SensorWaterDistance = duration * SOUND_SPEED/2;

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

  // Handle 433Mhz Communication Events
  if (myRadioSignalSwitch.available()) {
    lastRFvalue = myRadioSignalSwitch.getReceivedValue();
    output(myRadioSignalSwitch.getReceivedValue(), myRadioSignalSwitch.getReceivedBitlength(), myRadioSignalSwitch.getReceivedDelay(), myRadioSignalSwitch.getReceivedRawdata(), myRadioSignalSwitch.getReceivedProtocol());
    // if (lastRFvalue == RM2_A_ON)  { delay(800); digitalWrite(LED_BUILTIN, HIGH); sendCode(true); }
    // if (lastRFvalue == RM2_A_OFF) { delay(800); digitalWrite(LED_BUILTIN, LOW);  sendCode(false); }
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
  }
  
  // Handle the Timer for Water Sensor analysis & Decision
  // based on preferred frequency settings
  // -----------------------------------------------------
  if ((millis() - waterSensorsLastReadTickerMS) > getPreferredSensorRefreshFrequencyInSeconds() * 1000) {
    waterSensorsLastReadTickerMS = millis();
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

