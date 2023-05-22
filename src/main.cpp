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

// Configure WIFI settings
#define DEV_WIFI_MODE_AP   // DEV_WIFI_MODE_AP or DEV_WIFI_MODE_STATION
#ifdef DEV_WIFI_MODE_AP
  const char* ssid = "BKO-GARDEN-DMZ-DEV1";
  const char* password = "BKOGARDEN123*";
#else
  const char* ssid = "Toumim-1st-Floor";
  const char* password = "GFUCNEE4";
#endif
IPAddress IP;

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
int interuptCounter = 0;
int previousInteruptCounter = -1;
int previousMS = 0;
bool displayUpdateMark = false;
String lastCommand = "?";
bool virtualPlug2A = false;
int onboardLEDStatus = 0;


// Constants and variables to run the Ultrasonic SR04 Distance Sensor
const int trigPin = 19;
const int echoPin = 34;
#define SOUND_SPEED 0.034
#define CM_TO_INCH 0.393701
long duration;
float distanceCm;
float distanceInch;


RCSwitch mySwitch = RCSwitch();
void IRAM_ATTR myISR();

// Forward Declarations
void displayStatus(void);
void displayDeviceStatus(void);

using namespace ace_button;

// Initialize the smart BUTTON object
AceButton button(BUTTON_PIN);

// Initialize Async Web Server used by OTA (and maybe other stuff)
AsyncWebServer server(80);

// Forward reference to prevent Arduino compiler becoming confused.
void handleEvent(AceButton*, uint8_t, uint8_t);

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


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Button uses the built-in pull up register.
//  pinMode(BUTTON_PIN, INPUT_PULLUP);
//  attachInterrupt(BUTTON_PIN, myISR, RISING); // FALLING);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Configure the ButtonConfig with the event handler, and enable all higher
  // level events.

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

  display.clearDisplay();
  display.display();

  mySwitch.enableReceive(GPIO_RF);  // Receiver input on interrupt 0 (D2) OR D3???
  mySwitch.enableTransmit(25);

  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT); // Sets the echoPin as an Input

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

  displayDeviceStatus();
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

  // Display Line 1
  display.setCursor(128 - (5 * PIXELS_PER_CHAR), SSD_L1);
  display.println(F("v1.00"));
  display.setCursor(0, SSD_L1);
  display.println(F("BKO-DMZ-DEV1"));

  // display.setCursor(0, SSD_L2);
  // display.println(F("SSID:"));
  display.setCursor(0 * PIXELS_PER_CHAR, SSD_L2);
  display.println(F("BKO-GARDEN-DMZ-DEV1"));

  // display.setCursor(0, SSD_L3);
  // display.println(F("IP:"));
  display.setCursor(0 * PIXELS_PER_CHAR, SSD_L3);
  display.println(F("xxx.xxx.xx.xx"));

  display.setCursor(0, SSD_L4);
  display.println(F("IP:"));
  display.setCursor(0 * PIXELS_PER_CHAR, SSD_L4);
  display.println(IP.toString());

  // Refresh the display
  display.display();
}

void displayStatus(void) {
  // Serial.println("Display Status...");
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, SSD_L1);
  display.println(F("BKO *OTA* v3.00"));

  display.setCursor(128 - 10, SSD_L1);
  if (displayUpdateMark) {
    display.print("/");
    displayUpdateMark = !displayUpdateMark;
  } else {
    display.print("\\");
    displayUpdateMark = !displayUpdateMark;
  }

  // Display MODE info
  display.setCursor(0, SSD_L2);
  display.print(F("Mode: "));

  if (mode == 1) {
    display.print("LEARN   ");
    // mode = 2;
  } else {
    display.print("LISTEN  ");
    // mode = 1;
  }

  // display H/L levels
  if (digitalRead(GPIO_RF) == HIGH) {
    display.print("H/");
  } else {
    display.print("L/");
  }

  if (digitalRead(BUTTON_PIN) == HIGH) {
    display.print("H");
  } else {
    display.print("L");
  }

  // Display Received Data + Data PIN number
  display.setCursor(0, SSD_L3);
  display.print(F("Last code: "));
  display.setCursor(0, SSD_L4);
  display.print("RF:  ");
  display.print(lastRFvalue);
  display.setCursor(128 - 20, SSD_L4);
  display.print(GPIO_RF);
  // Display COMMAND name
  display.setCursor(0, SSD_L5);
  display.print(F("CMD: "));
  display.print(getCommandName(lastRFvalue));

  // Display INTERRUPT Data + Interrupt PIN number
  display.setCursor(0, SSD_L7);
  display.print("INT: ");
  display.print(interuptCounter);
  display.setCursor(128 - 20, SSD_L7);
  display.print(BUTTON_PIN);

  // Display LIGHT ON/OFF status
  display.setCursor(0,SSD_L6);
  display.print("Plug: ");
  if (virtualPlug2A) {
    display.print("ON");
  } else {
    display.print("OFF");
  }

  // Display Distance
  display.setCursor(50, SSD_L7);
  display.print(distanceCm);
  display.print("cm");

  // Refresh the display
  display.display();
}


void IRAM_ATTR myISR() {
    interuptCounter += 1;
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


void loop() {
  button.check();

  if (mySwitch.available()) {
    lastRFvalue = mySwitch.getReceivedValue();
    output(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(), mySwitch.getReceivedProtocol());

    if (lastRFvalue == RM2_A_ON)  { delay(800); digitalWrite(LED_BUILTIN, HIGH); sendCode(true); }
    if (lastRFvalue == RM2_A_OFF) { delay(800); digitalWrite(LED_BUILTIN, LOW);  sendCode(false); }

    delay(1);
    mySwitch.resetAvailable();
  }

  if (lastRFvalue != previousRFvalue) {
    previousRFvalue = lastRFvalue;
  }

  if (interuptCounter != previousInteruptCounter) {
    previousInteruptCounter = interuptCounter;
    displayDeviceStatus();
  }  

  if ((millis() - previousMS) > 1000) {
    previousMS = millis();

    // Distance Sensor measurement
    // ***************************
    // Clears the trigPin
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin, HIGH);
    distanceCm = duration * SOUND_SPEED/2;
    distanceInch = distanceCm * CM_TO_INCH;

    Serial.print("Distance: ");
    Serial.print(distanceCm);
    Serial.print("cm   (");
    Serial.print(distanceInch);
    Serial.println(" inches) ");
    
    displayDeviceStatus();

    // if (distanceCm > 28.0 && distanceCm < 32.00) {
    //     Serial.println("Switch Plug ON based on distance");
    //     sendCode(LIGHT_ON);
    //     virtualPlug2A = LIGHT_ON;
    // } else     if (distanceCm > 38.0 && distanceCm < 45.00) {
    //     Serial.println("Switch Plug OFF based on distance");
    //     sendCode(LIGHT_OFF);
    //     virtualPlug2A = LIGHT_OFF;      
    // }
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
    // case AceButton::kEventPressed:
    //   digitalWrite(LED_PIN, LED_ON);
    //   break;
    case AceButton::kEventReleased:
      Serial.println("Button Event: kEventReleased");
      if (virtualPlug2A) {
        Serial.println("Switch Plug OFF");
        sendCode(LIGHT_OFF);
        virtualPlug2A = LIGHT_OFF;
      } else {
        Serial.println("Switch Plug ON");
        sendCode(LIGHT_ON);
        virtualPlug2A = LIGHT_ON;
      }
      break;
  }
}