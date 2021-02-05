// VESC Setting:
// APP Settings
//  General
//    App to Use: UART
//  UART
//    Baudrate 19200
// Motor Settings
//   General
//     Current
//       Motor Current Max 105
//       Absolute Maximum Current 110
//     RPM 
//       MAX RPM 10500
//       MAX RPM reverse -105000
//  FOC
//    Encoder
//      Sensorless ERPM 4000
// To Do: test with empty battery - adjust duty cycles based on battery status


#include <dummy.h>
#include <WiFi.h>
//#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
//#include "TLV493D.h"
//#include "Wire.h"

//#include <IotWebConf.h>

#include "RemoteDebug.h"  //https://github.com/JoaoLopesF/RemoteDebug
RemoteDebug Debug;

//TLV493D sensor1;
//TLV493D sensor2;

#include "FS.h"
#include "SPIFFS.h"

#include <ESP8266FtpServer.h>

#include <VescUart.h>

/** Initiate VescUart class */
VescUart UART;

#include <Wire.h>
#include <AMS5600.h>
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
  #define SERIAL SerialUSB
  #define SYS_VOL   3.3
#else
  #define SERIAL Serial
  #define SYS_VOL   5
#endif

AMS_5600 as5600;

int ang, lang = 0;

FtpServer ftpSrv;   //set #define FTP_DEBUG in ESP8266FtpServer.h to see ftp verbose on serial

#define FORMAT_SPIFFS_IF_FAILED true

const byte number_of_poles = 14;
const byte triggerPin = 12;
const int max_speed = 1000*number_of_poles;
const int min_speed = 50*number_of_poles;
const float max_duty = 0.9;
const float min_duty = 0.08;
const float duty_step = 0.03;
const float min_current = 3;
const float max_current = 64;
const float boost_current = 90;
const int boostMillis_max = 15000; // 15 seconds max
const float max_current_step = 2;
const int startup_max = 30;
const int startup_delay_value = 3;
int startup_delay;

float target_current, current_step, current; /** Motor current*/
float target_duty, duty;

int RPM = 0;
int target_RPM;

int speed, target_speed;

int gears[] = {0,450*number_of_poles, 500*number_of_poles, 550*number_of_poles, 600*number_of_poles, 650*number_of_poles};
byte current_gear=5;

boolean triggered = false;
boolean boosted = false;
unsigned long boostMillis;

int cruise_control = 0;
int startup = startup_max;

const char* ssid = "experimental";
const char* password = "experience";

unsigned long wifi_waitCount = 0;            // counter to reconnect wifi
uint8_t conn_stat = 0;                       // Connection status for WiFi

                                             //
                                             // status |   WiFi   |    
                                             // -------+----------+------------
                                             //      0 |   down   |    
                                             //      1 | starting |    
                                             //      2 |    up    |    

const char * path = "/Log.txt";

#define HOST_NAME "DPV0001"

//const byte sensor1_pwr_pin = 18;
//const byte sensor2_pwr_pin = 19;
//const byte i2c_sda = 21;  //sda 21, scl 22

//defines for power control

// defines for spi lcd display
  #define TFT_CS         5
  #define TFT_RST        22 
  #define TFT_DC         21

const int sleeptime = 900; // in seconds, how long of inactivity before going to sleep

enum directions_enum {
  forward,
  reverse
};

directions_enum directions = forward;

// using two cores of the esp32 independantly

TaskHandle_t Task1;
TaskHandle_t Task2;

//variables for blinking an LED with Millis
const byte led = 2; // ESP32 Pin to which onboard LED is connected
unsigned long previousLEDMillis = 0;  // will store last time LED was updated
unsigned long currentLEDMillis;
const long interval = 1000;  // interval at which to blink (milliseconds)
int ledState = LOW;  // ledState used to set the LED

int rpm = 0;

String myString;

#define DEBUG_ENABLED

void appendFile(fs::FS &fs, const char * path, String message){
//    Serial.printf("Appending to file: %s ", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("- failed to open file for appending");
        return;
    }
    if(file.print(message)){
//        Serial.println("- message appended");
    } else {
        Serial.println("- append failed");
    }
}

// read the speed dial
float convertRawAngleToDegrees(word newAngle)
{
  /* Raw data reports 0 - 4095 segments, which is 0.087 of a degree */
  float retVal = newAngle * 0.087;
  ang = retVal;
  return ang;
}

void setup() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  
// Initialize the server (telnet or web socket) of RemoteDebug
//  Debug.begin(HOST_NAME);

  pinMode(led, OUTPUT);  
//  pinMode(triggerPin, INPUT);
  pinMode(triggerPin, INPUT_PULLDOWN);

  /** Setup Serial port to display data */
  Serial.begin(115200);

  /** Setup UART port (Serial1 on Atmega32u4) */
  Serial2.begin(19200);
  
  while (!Serial) {;}

  /** Define which ports to use as UART */
  UART.setSerialPort(&Serial2);

  duty = 0.0;
  target_duty = 0.26;
  target_current = 10;
//  current_gear = 2;
//  target_RPM = gears[current_gear];//550*number_of_poles;

  current_step = 0.4;
//  duty = 0.00;
  current = min_current;

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
//  WiFi.begin(ssid, password);     // Wifi.begin handled in Task2 om core 1
//  ftpSrv.begin("esp32","esp32");    //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)

  Wire.begin();

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("Gavin DPV");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

//  ArduinoOTA.begin();

  //create a task that will be executed in the Task1code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(
//  xTaskCreate(
                    Task1code,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1       /* Task handle to keep track of created task */
                    ,0           /* pin task to core 0 */      
                    );    

  //create a task that will be executed in the Task2code() function, with priority 1 and executed on core 1
  xTaskCreatePinnedToCore(
//  xTaskCreate(
                    Task2code,   // Task function. 
                    "Task2",     // name of task. 
                    10000,       // Stack size of task 
                    NULL,        // parameter of the task 
                    1,           // priority of the task 
                    &Task2      // Task handle to keep track of created task 
                    ,1          // pin task to core 1 
                    );

    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }
}

void clickEvent() {
   if (boosted == true) {
    boosted = false;
    if (target_current>max_current) target_current = max_current;
    if (current>target_current) current=target_current;
    cruise_control = 0; // just in case
    Serial.println("cruise off");
   }
   else {
     if (cruise_control >0){
      cruise_control = 0;
      Serial.println("cruise off");
     }
     else {
      cruise_control = 2;
      Serial.println("cruise on");
     }
   }
}
void doubleClickEvent() {
//   Serial.println("double-click");
   Serial.println("boost");
   boosted = true;
   boostMillis = millis();
}
void releaseEvent() {
   Serial.println("released");
   if (boosted == true) 
     Serial.println("stop boost");
   boosted = false;
   if (cruise_control>0) cruise_control -= 1;
   //trigger was released
//   else //if (triggered)
  if (cruise_control<=0)
     { // only do this once, if trigger just released
        UART.setDuty(0);
        UART.setCurrent(0);
        UART.setBrakeCurrent(0);
        current_step = 0.4;
        current = min_current;
//        Serial.println("trigger released");
        triggered = false;
      }
}

//Task1code: set power to motor
void Task1code( void * pvParameters ){
  Serial.print("Task1: speed control running on core ");
  Serial.println(xPortGetCoreID());
//  startup_delay = startup_delay_value;

  for(;;){
    UART.getVescValues();
    RPM = int(UART.data.rpm/number_of_poles);

   // Get button event and act accordingly
   int b = checkButton();
   if (b == 1) clickEvent();
   if (b == 2) doubleClickEvent();
   if (b == 3) releaseEvent();

    if ((digitalRead(triggerPin)==HIGH) or (cruise_control >0)) {
      if (digitalRead(triggerPin)==HIGH) 
        if (triggered== false) {
          Serial.println("trigger pressed");
          triggered = true;
          cruise_control -=1;
            if (cruise_control <= 0) {
              cruise_control = 0;
              Serial.println("cruise off");
            }
        }
      target_current = map(convertRawAngleToDegrees(as5600.getRawAngle()), 5,322,20,max_current);
        if (target_current>max_current) target_current = max_current;
        if (boosted == true) {
          if (millis() - boostMillis >= boostMillis_max ) {
            current = target_current;
            boosted = false;
            Serial.println("stop boost");
          }
          else
            target_current = boost_current;
        }
        if (current<target_current) {
          current+= current_step;
          if (current_step<max_current_step)
            current_step+=0.05;
          else
            current_step=max_current_step;
        }
        else {
          current = target_current;
        } 
      Serial.print("current aim: ");
      Serial.println(current);
      UART.setCurrent(current);
    }
/*    else { //trigger was released
      if (triggered) { // only do this once, if trigger just released
        UART.setDuty(0);
        UART.setCurrent(0);
        UART.setBrakeCurrent(0);
        current_step = 0.4;
        current = min_current;
//        Serial.println("tigger released");
        triggered = false;
        boosted = false;
      }
    }
*/

    // service the log file if needed
    currentLEDMillis = millis();
    if (currentLEDMillis - previousLEDMillis >= interval) {
    // save the last time you blinked the LED
      previousLEDMillis = currentLEDMillis;
      if (WiFi.status() == WL_CONNECTED){
        // if the LED is off turn it on and vice-versa:
        ledState = not(ledState);
        // set the LED with the ledState of the variable:
      }
      else {
        ledState = false;
      }
      digitalWrite(led,  ledState);
      if ((digitalRead(triggerPin)==HIGH)  or (cruise_control >0)){
        myString = String(int(currentLEDMillis/1000)) + ", " + String(RPM) + ", " + String(UART.data.inpVoltage)  + ", " + String(UART.data.avgInputCurrent)  + ", " + String(UART.data.avgMotorCurrent)  + ", " + String(UART.data.dutyCycleNow) + ", " + String(UART.data.tempFET) + ", " + String(UART.data.tempMotor) + "\r\n";
        appendFile(SPIFFS, "/Log.txt", myString);
      }
  }
  delay(20);
  } 
}

//Task2code: handle OTA update, blinks an LED
void Task2code( void * pvParameters ){

  Serial.print("Task2: OTA update running on core ");
  Serial.println(xPortGetCoreID());
//  if (1==2) { 
  for(;;){
//    debugD("* This is a message of debug level DEBUG");
    if ((WiFi.status() != WL_CONNECTED) && (conn_stat != 1)) { conn_stat = 0; }
    if ((WiFi.status() == WL_CONNECTED) && (conn_stat != 3)) { conn_stat = 2; }
    switch (conn_stat) {
      case 0:                                                       // WiFi down: start WiFi
        Serial.println("WiFi down: start WiFi");
        WiFi.begin(ssid, password);
        conn_stat = 1;
        break;
      case 1:                                                       // WiFi starting, do nothing here
        wifi_waitCount++;
          if (wifi_waitCount>300) {
            WiFi.disconnect();
            WiFi.mode(WIFI_AP_STA);
            wifi_waitCount = 0;
            conn_stat = 0;
          }
        break;
      case 2:                                                       // WiFi up
        Serial.println("WiFi up");
        Serial.print("IP address: ");
        Serial.println(WiFi.localIP());
        ArduinoOTA.begin();
        ftpSrv.begin("esp32","esp32");    //username, password for ftp.  set ports in ESP8266FtpServer.h  (default 21, 50009 for PASV)
        Debug.begin(HOST_NAME); // Initialize the WiFi server

        Debug.setResetCmdEnabled(true); // Enable the reset command

        Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
        Debug.showColors(true); // Colors
        conn_stat = 3;
        wifi_waitCount = 0;
        break;
      case 3:                                                       // WiFi up, handle all web related activities
        ArduinoOTA.handle();  
        ftpSrv.handleFTP();        //make sure in loop you call handleFTP()!!  
        debugHandle(); // Equal to SerialDebug
//        server.handleClient();   //example if running a webserver you still need to call .handleClient();
        break;
      }
    delay(20);
  }
}

void loop() {
}

//=================================================
//  MULTI-CLICK:  One Button, Multiple Events
// stolen from https://forum.arduino.cc/index.php?topic=14479.0

// Button timing variables
const int debounce = 5;          // ms debounce period to prevent flickering when pressing or releasing the button
const int DCgap = 250;            // max ms between clicks for a double click event
const int holdTime = 200;        // ms hold period: how long to wait for press+hold event
//const int longHoldTime = 5000;    // ms long hold period: how long to wait for press+hold event

// Button variables
boolean buttonVal = HIGH;   // value read from button
boolean buttonLast = HIGH;  // buffered value of the button's previous state
boolean DCwaiting = false;  // whether we're waiting for a double click (down)
boolean DConUp = false;     // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;    // whether it's OK to do a single click
volatile long downTime = -1;         // time the button was pressed down
volatile long upTime = -1;           // time the button was released
boolean ignoreUp = false;   // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;        // when held, whether to wait for the up event
boolean holdEventPast = false;    // whether or not the hold event happened already
boolean longHoldEventPast = false;// whether or not the long hold event happened already

int checkButton() {    
   int event = 0;
   buttonVal = digitalRead(triggerPin);
   // Button pressed down
   if (buttonVal == LOW && buttonLast == HIGH && (millis() - upTime) > debounce)
   {
       downTime = millis();
       ignoreUp = false;
       waitForUp = false;
       singleOK = true;
       holdEventPast = false;
       longHoldEventPast = false;
       if ((millis()-upTime) < DCgap && DConUp == false && DCwaiting == true)  DConUp = true;
       else  DConUp = false;
       DCwaiting = false;
   }
   // Button released
   else if (buttonVal == HIGH && buttonLast == LOW && (millis() - downTime) > debounce)
   {        
       if (not ignoreUp)
       {
           upTime = millis();
           if (DConUp == false) DCwaiting = true;
           else
           {
               event = 2;
               DConUp = false;
               DCwaiting = false;
               singleOK = false;
           }
       }
   }
   // Test for normal click event: DCgap expired
   if ( buttonVal == HIGH && (millis()-upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true && event != 2)
   {
       event = 1;
       DCwaiting = false;
   }
   // Test for hold
   if (buttonVal == LOW && (millis() - downTime) >= holdTime) {
       // Trigger "normal" hold
       if (not holdEventPast)
       {
           event = 3;
           waitForUp = true;
           ignoreUp = true;
           DConUp = false;
           DCwaiting = false;
           //downTime = millis();
           holdEventPast = true;
           longHoldEventPast = true;
       }
       // Trigger "long" hold
/*       if ((millis() - downTime) >= longHoldTime)
       {
           if (not longHoldEventPast)
           {
               event = 3;
               longHoldEventPast = true;
           }
       }
       */
   }
   buttonLast = buttonVal;
   return event;
}
