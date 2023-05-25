#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <RCSwitch.h>
#include <AceButton.h>

// Includes needed for OTA Updates (Over-The-Air updates)
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

#define VERSION "v1.09"
#define DEVICE_NAME "BKO-DMZ-DEV1"

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
#define DISPLAY_MODE_OPERATIONS 0
#define DISPLAY_MODE_WIFI       1
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
int   publicKlong_PumpMinimumWaterLevel = 30;  // Distance from Sensor to water level needed to start tPUBLIC_KLONG_RELAY_PINhe pump safely
int   publicKlong_PumpDecision = 0;
String systemAnalysis = "";

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
#define PUBLIC_KLONG_RELAY_PIN  14
bool  publicKlong_powered = 0;
float publicKlong_operating_time_min = 0.0; // Total minutes since the pump started
unsigned long publicKlong_operating_time_sec = 0; // Total Seconds since the pump started
unsigned long publicKlong_operating_start = 0;    // Current milliseconds when the pump started

#define SOUTH_KLONG_RELAY_PIN  27
bool  southKlong_powered = 0;
float southKlong_operating_time_min = 0.0;  // Total minutes since the pump started
unsigned long southKlong_operating_time_sec = 0;  // Total Seconds since the pump started
unsigned long southKlong_operating_start = 0;     // Current milliseconds when the pump started

// Constants for Remotes On/Off commands (RM1: Remote 1, RM2: Remote 2)
#define RM2_A_ON  4195665
#define RM2_A_OFF 4195668

#define LIGHT_ON true
#define LIGHT_OFF false

int mode = 1;   // (1: Learn,  2: Display)
int lastRFvalue = 0;
int previousRFvalue = -1;
int GPIO_RF = 5;
int BUTTON_PIN = 18;
int previousMS = 0;
bool displayUpdateMark = false;
String lastCommand = "?";
bool virtualPlug2A = false;
int onboardLEDStatus = 0;

RCSwitch mySwitch = RCSwitch();

// Forward Declarations
void displayStatus(void);
void displayDeviceStatus(void);
void updateWifiStatus(void);
void updateDisplay(void);

using namespace ace_button;

// Initialize the smart BUTTON object
AceButton button(BUTTON_PIN);

// Initialize Async Web Server used by OTA (and maybe other stuff)
AsyncWebServer server(80);

// Forward reference to prevent Arduino compiler becoming confused.
void handleEvent(AceButton*, uint8_t, uint8_t);

#pragma region Pump Controllers

void startPump1() {
  if (publicKlong_powered) return;  // Do nothing if already required state
  publicKlong_powered = 1;
  publicKlong_operating_start = millis();
  publicKlong_operating_time_sec = 0;
  digitalWrite(PUBLIC_KLONG_RELAY_PIN, publicKlong_powered);
}

void stopPump1() {
  if (!publicKlong_powered) return;  // Do nothing if already required state
  publicKlong_powered = 0;
  publicKlong_operating_start = 0;
  publicKlong_operating_time_sec = 0;
  digitalWrite(PUBLIC_KLONG_RELAY_PIN, publicKlong_powered);
}

void startPump2() {
  if (southKlong_powered) return;  // Do nothing if already required state
  southKlong_powered = 1;
  southKlong_operating_start = millis();
  southKlong_operating_time_sec = 0;
  digitalWrite(SOUTH_KLONG_RELAY_PIN, southKlong_powered);
}

void stopPump2() {
  if (!southKlong_powered) return;  // Do nothing if already required state
  southKlong_powered = 0;
  southKlong_operating_start = 0;
  southKlong_operating_time_sec = 0;
  digitalWrite(SOUTH_KLONG_RELAY_PIN, southKlong_powered);
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
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });
  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");

  // Configure Pump Relay PINs
  pinMode(PUBLIC_KLONG_RELAY_PIN, OUTPUT);
  pinMode(SOUTH_KLONG_RELAY_PIN, OUTPUT);
  digitalWrite(PUBLIC_KLONG_RELAY_PIN, LOW);
  digitalWrite(SOUTH_KLONG_RELAY_PIN, LOW);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Configure the ButtonConfig with the event handler, and enable all higher level events.
  ButtonConfig* buttonConfig = button.getButtonConfig();
  buttonConfig->setEventHandler(handleEvent);
  buttonConfig->setFeature(ButtonConfig::kFeatureClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureDoubleClick);
  buttonConfig->setFeature(ButtonConfig::kFeatureLongPress);
  buttonConfig->setFeature(ButtonConfig::kFeatureRepeatPress);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  mySwitch.enableReceive(GPIO_RF);  // Receiver input on interrupt 0 (D2) OR D3???
  mySwitch.enableTransmit(25);

  // Confirgure Ultrasounic Distance/Meter Sensors Pins
  pinMode(sensorPublicKlong_trigPin, OUTPUT); // Sets the sensorPublicKlong_trigPin as an Output
  pinMode(sensorPublicKlong_echoPin, INPUT); // Sets the sensorPublicKlong_echoPin as an Input
  pinMode(sensorSouthKlong_trigPin, OUTPUT); // Sets the sensorPublicKlong_trigPin as an Output
  pinMode(sensorSouthKlong_echoPin, INPUT); // Sets the sensorPublicKlong_echoPin as an Input

  display.clearDisplay();
  display.display();

  Serial.println("Initialized!");
}

void sendCode(bool on) {
  mySwitch.setPulseLength(325);
  mySwitch.setProtocol(1);
  mySwitch.setRepeatTransmit(5);

  Serial.print("Sending Code ");
  Serial.println(on);
  
  if (on) 
  { // RM2-B-ON
    mySwitch.send("010000000001000101010001"); 
    virtualPlug2A = true;
    digitalWrite(LED_BUILTIN, HIGH);
  }     
  else 
  { // RM2-B-OFF
    mySwitch.send("010000000001000101010100"); 
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
  display.setCursor(75, SSD_L3);
  display.print("Required");

  // Display Line 5: Water Levels (Values)
  display.setCursor(0, SSD_L4);
  // display.print(southKlong_CurrentWaterLevel);
  display.print(publicKlong_SensorWaterDistance);
  display.print("cm");
  display.setCursor(75, SSD_L4);
  display.print(publicKlong_PumpMinimumWaterLevel);
  display.print("cm");
  
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

void updateDisplay(void) {
  if (DISPLAY_MODE == DISPLAY_MODE_OPERATIONS) displayDeviceStatus();
  else if (DISPLAY_MODE == DISPLAY_MODE_WIFI) displayWifiStatus();
  else displayInvalidDisplayMode();
}

void switchToNextDisplayMode(void) {
  if (DISPLAY_MODE == DISPLAY_MODE_OPERATIONS) DISPLAY_MODE = DISPLAY_MODE_WIFI;
  else if (DISPLAY_MODE == DISPLAY_MODE_WIFI) DISPLAY_MODE = -1;
  else if (DISPLAY_MODE == -1) DISPLAY_MODE = DISPLAY_MODE_OPERATIONS;
  updateDisplay();
}

// Output all information to serial console (RX/TX)
void printAllDeviceDataToSerial() {
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
  Serial.println("");
}

static const char* bin2tristate(const char* bin);
static char * dec2binWzerofill(unsigned long Dec, unsigned int bitLength);

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

  delay(300);
  // SOUTH KLONG
  // ------------
  // Trigger the sensor
  digitalWrite(sensorSouthKlong_trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensorSouthKlong_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensorSouthKlong_trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(sensorSouthKlong_echoPin, HIGH);
  southKlong_SensorWaterDistance = duration * SOUND_SPEED/2;
}

void loop() {
  button.check();

  // Handle 433Mhz Communication Events
  if (mySwitch.available()) {
    lastRFvalue = mySwitch.getReceivedValue();
    output(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(), mySwitch.getReceivedProtocol());
    // if (lastRFvalue == RM2_A_ON)  { delay(800); digitalWrite(LED_BUILTIN, HIGH); sendCode(true); }
    // if (lastRFvalue == RM2_A_OFF) { delay(800); digitalWrite(LED_BUILTIN, LOW);  sendCode(false); }
    delay(1);
    mySwitch.resetAvailable();
  }

  if (lastRFvalue != previousRFvalue) {
    previousRFvalue = lastRFvalue;
  }

  // Do watever needed every second
  // ------------------------------
  if ((millis() - previousMS) > 1000) {
    previousMS = millis();

    // Debug all data
    printAllDeviceDataToSerial();

    // Distance Sensor measurement
    // ***************************
    getWaterSensorsData();
    calculateWaterLevels();
    analyzeWaterLevels();
   
    updateDisplay();
  }
  
}



// The event handler for the button.
void handleEvent(AceButton* /* button */, uint8_t eventType, uint8_t buttonState) {

  // Print out a message for all events.
  Serial.print(F("handleEvent(): eventType: "));
  Serial.print(eventType);
  Serial.print(F("; buttonState: "));
  Serial.println(buttonState);

  switch (eventType) {
    case AceButton::kEventPressed:
      switchToNextDisplayMode();
      break;
    case AceButton::kEventReleased:
      break;
    case AceButton::kEventLongPressed:
      break;
    case AceButton::kEventClicked:
      break;
  }
}