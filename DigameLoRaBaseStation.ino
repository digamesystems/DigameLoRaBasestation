/* DigameLoRaBaseStation
 *  
 *  A simple LoRa listener for events from our Vehicle Counters. 
 *  Builds up a JSON message for each event and routes them our server 
 *  over WiFi.
 *  
 *  The program provides a web interface for configuration of LoRa
 *  and WiFi parameters.
 *  
 *  Copyright 2022, Digame systems. All rights reserved. 
 */
 

#define USE_LORA true
String model             = "DS-STN-LoRa-WiFi-1";
String model_description = "(LoRa-WiFi Base Station)";  

bool showDataStream = false;

//for Over-the-Air updates...
#include <WiFi.h>

#include <ArduinoJson.h>   
#include <CircularBuffer.h>   // Adafruit library. Pretty small!


#include <digameDebug.h>
#include "digameVersion.h"
#include <digameJSONConfig.h> // Program parameters from config file on SD card
Config config;                // Singleton data structure of program parameters (defined in digameJSONConfig.h)
                              //   used by most Digame libraries.
#include <digameTime.h>       // Time Functions - RTC, NTP Synch etc
#include <digameNetwork.h>    // Network Functions - Login, MAC addr
#include <digamePowerMgt.h>   // Power management modes 
#include <digameDisplay.h>    // eInk Display Functions
#include <digameLoRa.h>       // Reyax LoRa module control functions
#include <digameCounterWebServer.h>  // Handles parmater tweaks through a web page

#define debugUART Serial
#define CTR_RESET 32          // Reset Button Input

// We have 3 different displays in the UI
#define SPLASH 1   // Splash screen
#define NETWORK 2  // Base station IP address screen
#define COUNTERS 3 // Counter summary screen


// GLOBALS
int displayMode = SPLASH ; // Wake up showing the splash screen


bool   accessPointMode = false;

// Heartbeat managment variables
int    heartbeatMinute;    // Issue Heartbeat message once an hour. Holds the current Minute.
int    oldheartbeatMinute; // Value the last time we looked.
int    bootMinute;         // The minute (0-59) within the hour we woke up at boot. 
bool   heartbeatMessageNeeded = true;
bool   bootMessageNeeded = true;

unsigned long lastHeartbeatMillis = 0; 

unsigned long bootMillis=0;

String myMACAddress; // Grab at boot time

// Multi-Tasking
SemaphoreHandle_t mutex_v; // Mutex used to protect variables across RTOS tasks. 
TaskHandle_t messageManagerTask;
TaskHandle_t eventDisplayManagerTask;

String strDisplay=""; // contents of the Summary screen.

const int samples = 200;
CircularBuffer<String, samples> loraMsgBuffer; // A buffer containing JSON messages to be 
                                               // sent to the Server

// FUNCTION DECLARATIONS

void   initPorts();
void   splash();
String buildJSONHeader(String);
void   processLoRaMessage(String);
void   messageManager(void *);


//****************************************************************************************
// Configure the IO pins and set up the UARTs
void initPorts(){
  
  pinMode(CTR_RESET,INPUT_PULLUP);
  initLoRa();
  debugUART.begin(115200); 
  delay(1000);
  Wire.begin();

}


//****************************************************************************************
// A pretty(?) splash notification to debugUART
void splash(){
  
  String compileDate = F(__DATE__);
  String compileTime = F(__TIME__);
  
  debugUART.println();
  debugUART.println("*****************************************************");
  debugUART.println("         HEIMDALL Vehicle Counting System");
  debugUART.println("         - WiFi-LoRa BASE STATION UNIT -");
  debugUART.println();
  debugUART.println("Model: DS-VC-BASE-LOR-1 (WiFi/LoRa Base Station)");
  debugUART.println("Version: " + SW_VERSION);
  debugUART.println("Copyright 2022, Digame Systems. All rights reserved.");
  debugUART.println();
  debugUART.print("Compiled on ");
  debugUART.print(compileDate);
  debugUART.print(" at ");
  debugUART.println(compileTime); 
  debugUART.println("*****************************************************");
  debugUART.println();
  debugUART.println("HARDWARE INITIALIZATION");
  debugUART.println();

  // first update should be full screen refresh
  initDisplay();
  showWhite();
  displaySplashScreen("(Base Station)",SW_VERSION);

}

//****************************************************************************************
// Save a JSON log file to the server
void postDatalog(){
  if (initSDCard()){
    String jsonPayload="";

    debugUART.println("POSTING DATALOG.TXT to server...");
    
    File dataFile = SD.open("/DATALOG.TXT");
  
    // If the file is available, read from it:
    if (dataFile) {
       while (dataFile.available()) {
          jsonPayload = dataFile.readStringUntil('\r');
          jsonPayload.trim();
          postJSON(jsonPayload, config);
       }
       debugUART.println("Done.");
       dataFile.close();
       SD.remove("/DATALOG.TXT"); // Delete the log after upload   
    } else {
      // No file.
      debugUART.println("No DATALOG.TXT present.");  
    }
  } else {
    debugUART.println("ERROR! No SD card present.");
  }
}


//*****************************************************************************
// Append a message to the DATALOG.TXT file on the SD card.
// TODO: Error handling if card is full, etc.
void appendDatalog(String jsonPayload){
  if (initSDCard()){
    File dataFile = SD.open("/DATALOG.TXT", FILE_APPEND);
  
    // If the file is available, write to it:
    if (dataFile) {
      debugUART.println("Appending data file on SD:");
      debugUART.println(jsonPayload);
      
      dataFile.println(jsonPayload);
      dataFile.close();
    }
    // If the file isn't open, pop up an error:
    else {
      debugUART.println("ERROR! Trouble opening datalog.txt");
    }
  } else {
    Serial.println("ERROR! SD Card not present.");  
  }
}




// COUNTER LORA MESSAGE HANDLING FUNCTIONS 

    String spacer(String s1){
      s1 = s1 + "                       ";
      return s1.substring(0,6);
    }
    
    
    bool isDuplicateMessage(String msg){
      static String lastMessage="";
      if (msg.equals(lastMessage)){ //If we've processed this message previously, but the counter 
                                   // missed the ACK, it will resend. Don't process it twice. 
        debugUART.println("We've seen this message before.");
        return true;  
      }
      lastMessage = msg;
      return false;
    }
    
    String getDeviceAddress(String msg){
      // Get the device's Address.
      int idxstart = msg.indexOf('=')+1;
      int idxstop  = msg.indexOf(','); 
      String strAddress = msg.substring(idxstart,idxstop);  
      return strAddress; 
    }
    
    String getPayload(String msg){
    
      // Start and end of the JSON payload in the msg.
      int idxstart = msg.indexOf('{');
      int idxstop  = msg.lastIndexOf('}')+1; // The close of the JSON message payload.
                                        // Using lastIndexOf since we are nesting JSON structs in some messages and 
                                        // can have multiple {{}} situations. 
      // The message contains a JSON payload extract to the char array json
      String payload = msg.substring(idxstart,idxstop); 
      return payload;    
    }
    
    String getRSSI(String msg){
      // Start and end of the JSON payload in the msg.
      int idxstart = msg.indexOf('{');
      int idxstop  = msg.lastIndexOf('}')+1; // The close of the JSON message payload.
                                        // Using lastIndexOf since we are nesting JSON structs in some messages and 
                                        // can have multiple {{}} situations. 
      
      // After the payload comes the RSSI and SNR values;
      String trailer = msg.substring(idxstop +1);
      //debugUART.println(trailer);
      idxstop = trailer.indexOf(',');
      
      String strRSSI = trailer.substring(0,idxstop);
      //debugUART.println(strRSSI);
      strRSSI.trim(); 
      return strRSSI; 
    }
    
    String getSNR(String msg){
        // Start and end of the JSON payload in the msg.
      int idxstart = msg.indexOf('{');
      int idxstop  = msg.lastIndexOf('}')+1; // The close of the JSON message payload.
                                        // Using lastIndexOf since we are nesting JSON structs in some messages and 
                                        // can have multiple {{}} situations. 
      // After the payload comes the RSSI and SNR values;
      String trailer = msg.substring(idxstop + 1);
      //debugUART.println(trailer);
      idxstop = trailer.indexOf(',');
      String strSNR = trailer.substring(idxstop + 1);
      //debugUART.println(strSNR);  
      strSNR.trim();
      return strSNR;  
    }

// END COUNTER LORA MESSAGE HANDLING FUNCTIONS 



//****************************************************************************************
String loraMsgToJSON(String msg){

  if (isDuplicateMessage(msg)) return "IGNORE"; 

  // LoRa address
  String strAddress = getDeviceAddress(msg);
  // JSON message payload
  String payload    = getPayload(msg);
  // Signal strength
  String strRSSI    = getRSSI(msg); 
  // Data Quality 
  String strSNR     = getSNR(msg);
    
  // Deserialize the JSON document in the payload
  StaticJsonDocument<512> doc;
  char json[512] = {};
  payload.toCharArray(json,payload.length()+1);
  DeserializationError error = deserializeJson(doc, json);
  // Test if parsing succeeds.
  if (error) {
    debugUART.print(F("deserializeJson() failed: "));
    debugUART.println(error.f_str());
    return "IGNORE";
  }
  
  // Fetch values from the doc object.  
  
  // Grab the Event Type
  String strEventType;
  String et = doc["et"];
  if (et=="b"){
      strEventType = "Boot";
  } else if (et == "hb"){
      strEventType = "Heartbeat";
  } else if (et == "v"){
      strEventType = "Vehicle";     
  } else { 
      strEventType = "Unknown";
      debugUART.println("ERROR: Unknown Message Type!");
      return "IGNORE";  // If we don't know what this is, don't bother the server with it.
  }

  // MAC address
  String strMACAddr = doc["ma"];

  // Firmware Version
  String strVersion = doc["v"];
 
  // Timestamp
  String strTime = doc["ts"];
   
  // Count
  String strCount = doc["c"];
  
  // Lane for the event
  String strLane = doc["l"];
  
  // Detection Algorithm
  String strDetAlg;
  String da = doc["da"];
  if (da == "t"){
    strDetAlg = "Threshold";
  } else if (da == "c") {
    strDetAlg = "Correlation";
  } else {
    strDetAlg = "Unknown";
  }

  // Counter RTC temperature
  String strTemperature = doc["t"];

  // Number of send attempts the counter took to get an ACK
  String strRetries = doc["r"];

  // Settings -- reported in Boot and HB events. 
  String strSettings = doc["s"];
  
  String strDeviceName = "Unknown Device";
 
    // TODO: move to a look up function and come up with a better storage scheme.
    if (strAddress == config.sens1Addr){
      strDeviceName = config.sens1Name; 
      if (strLane == "1"){config.sens1Zone1 = strCount;}else{config.sens1Zone2 = strCount;};  
      str1Count = spacer(config.sens1Zone1) + config.sens1Zone2;
      
    } else if (strAddress == config.sens2Addr){
      strDeviceName = config.sens2Name;
      if (strLane == "1"){config.sens2Zone1 = strCount;}else{config.sens2Zone2 = strCount;};  
      str2Count = spacer(config.sens2Zone1) + config.sens2Zone2;
      
    } else if (strAddress == config.sens3Addr){
      strDeviceName = config.sens3Name;
      if (strLane == "1"){config.sens3Zone1 = strCount;}else{config.sens3Zone2 = strCount;};  
      str3Count = spacer(config.sens3Zone1) + config.sens3Zone2;
      
    } else if (strAddress == config.sens4Addr){
      strDeviceName = config.sens4Name;
      if (strLane == "1"){config.sens4Zone1 = strCount;}else{config.sens4Zone2 = strCount;};  
      str4Count = spacer(config.sens4Zone1) + config.sens4Zone2;
    }
  

  if (displayMode == COUNTERS) { // If strDisplay changes, the screen will update. See: displayManager task.
    strDisplay = msg; }

  // Build the WiFi JSON message
  String jsonPayload;
  jsonPayload = "{\"deviceName\":\""       + strDeviceName + 
                 "\",\"deviceMAC\":\""     + strMACAddr  + 
                 "\",\"firmwareVer\":\""   + strVersion  + 
                 "\",\"timeStamp\":\""     + strTime + 
                 "\",\"linkMode\":\""      + "LoRa" +
                 "\",\"eventType\":\""     + strEventType +
                 "\",\"detAlgorithm\":\""  + strDetAlg +
                 "\",\"count\":\""         + strCount + 
                 "\",\"rssi\":\""          + strRSSI + 
                 "\",\"snr\":\""           + strSNR +    
                 "\",\"temp\":\""          + strTemperature +  
                 "\",\"retries\":\""       + strRetries; 

  if (et=="v"){
    jsonPayload = jsonPayload + "\",\"lane\":\"" + strLane +"\"";
  }
                 
  if ((et=="b")||(et=="hb")){
    jsonPayload = jsonPayload + "\",\"settings\":" + strSettings;                  
  }
  
  jsonPayload = jsonPayload + "}";
  
  return jsonPayload;
  
}

//****************************************************************************************
// Parse a LoRa message from a vehicle counter. Format as a JSON message
// and POST it to the server if WiFi is available. If not, save to the SD card. 
void processLoRaMessage(String msg){
  String jsonPayload;

  jsonPayload = loraMsgToJSON(msg);

  if (jsonPayload == "IGNORE"){
    return;
  }
  
  if(WiFi.status()== WL_CONNECTED){
      // We have a WiFi connection. -- Upload the data to the the server. 
      postJSON(jsonPayload, config);
      
  } else {
      // No WiFi -- Save locally.
      appendDatalog(jsonPayload);
  
      // Try connecting every five minutes 
      // msLastConnectionAttempt set in enableWiFi
      if ((millis() - msLastConnectionAttempt) > (5*60*1000)){ 

        enableWiFi(config);
   
        if (WiFi.status() == WL_CONNECTED){
          postDatalog();  // POSTS and clears out the log.
        }
     }
   } 
}


//****************************************************************************************
// JSON messages to the server all have a similar format. 
String buildJSONHeader(String eventType){
  String jsonHeader;

  jsonHeader = "{\"deviceName\":\""      + config.deviceName + 
                 "\",\"deviceMAC\":\""   + myMACAddress + // Read at boot
                 "\",\"firmwareVer\":\"" + TERSE_SW_VERSION  + 
                 "\",\"timeStamp\":\""   + getRTCTime() +  // Updated in main loop from RTC
                 "\",\"eventType\":\""   + eventType +
                 "\",\"temp\":\""        + getRTCTemperature() + 
                 "\""; 
                   
  return jsonHeader;
}


//****************************************************************************************
void handleModeButtonPress(){
  
  if (digitalRead(CTR_RESET)== LOW) {
    displayMode++;
    if (displayMode > COUNTERS) displayMode = SPLASH;
    switch (displayMode) {
      case SPLASH:
        displaySplashScreen("(Base Station)",SW_VERSION);
        break;
      case NETWORK:
        displayIPScreen(WiFi.localIP().toString()); 
        break;
      case COUNTERS:
        displayCountersSummaryScreen("Counter Values",getCounterSummary());
        break;     
    }
  }   
  
}


//****************************************************************************************
// Experimenting with using a circular buffer and multi-tasking to enqueue 
// messages to the server...
void messageManager(void *parameter){
  String activeMessage;
  String jsonPayload;

  debugUART.print("Message Manager Running on Core #: ");
  debugUART.println(xPortGetCoreID());
  
  for(;;){  

    //Serial.println("messageManager TICK");
    //**********************************************
    // Check if we need to send a boot message
    //**********************************************      
    if (bootMessageNeeded){

      jsonPayload = buildJSONHeader("Boot");
      jsonPayload = jsonPayload + "}";

      //debugUART.println(jsonPayload);
      postJSON(jsonPayload, config);

      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        bootMessageNeeded = false;
      xSemaphoreGive(mutex_v);
      
    }
    

    //**********************************************
    // Check if we need to send a heartbeat message
    //**********************************************
    unsigned long deltaT = (millis() - lastHeartbeatMillis);
    unsigned long slippedMilliSeconds; 
    if ( (deltaT) >= config.heartbeatInterval.toInt() * 1000 ){
      debugUART.println(deltaT);
      slippedMilliSeconds = deltaT - config.heartbeatInterval.toInt() *1000; // Since this Task is on a 100 msec schedule, we'll always be a little late...
      debugUART.println(slippedMilliSeconds);
      heartbeatMessageNeeded = true;
    }  
    
    if (heartbeatMessageNeeded){
      debugUART.println("Heartbeat needed.");
      
      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        lastHeartbeatMillis = millis() - slippedMilliSeconds; // Small tweak to time to reflect when we should have fired the event. 
        heartbeatMessageNeeded = false;
      xSemaphoreGive(mutex_v);
      
      jsonPayload = buildJSONHeader("Heartbeat");
      jsonPayload = jsonPayload + "}";

      //debugUART.println(jsonPayload);
      postJSON(jsonPayload, config);
      
    }
       
    //********************************************
    // Check for Messages on the Queue and Process
    //********************************************
    if (loraMsgBuffer.size()>0){

      xSemaphoreTake(mutex_v, portMAX_DELAY); 
        activeMessage=loraMsgBuffer.shift();
      xSemaphoreGive(mutex_v);

      processLoRaMessage(activeMessage);
    }

    upTimeMillis = millis() - bootMillis; 
    vTaskDelay(10 / portTICK_PERIOD_MS);   
  }   
}

void eventDisplayManager(void *parameter){
  int eventDisplayUpdateRate = 20;
  static unsigned long updates = 0; 
  String oldStrDisplay;

  #if !(SHOW_DATA_STREAM)
    debugUART.print("Display Manager Running on Core #: ");
    debugUART.println(xPortGetCoreID());
  #endif 
  
  for(;;){  

    if (strDisplay != oldStrDisplay){
        oldStrDisplay = strDisplay;        
        //displayEventScreen(strDisplay);
        if (updates % 10 == 0){ showWhite();};
        
        displayCountersSummaryScreen("Counter Values",getCounterSummary());
        updates ++;
      }       
  
    vTaskDelay(eventDisplayUpdateRate / portTICK_PERIOD_MS);
  }
}
    

//****************************************************************************************
// Returns a string containing a summary of counts seen from each VC. 
String getCounterSummary(){
  String retVal  = "";
  //retVal += " #  L1    L2\n";
  retVal += " #  1/In  2/Out\n";
  retVal += " ---------------\n";
  retVal += " 1  " + str1Count + "\n";
  retVal += " 2  " + str2Count + "\n";
  retVal += " 3  " + str3Count + "\n";
  retVal += " 4  " + str4Count + "\n";
  return retVal;
}


bool bootToAPMode(){
  bool retVal = false;
  String foo = "Digame-STN-" + getShortMACAddress();
  const char* ssid = foo.c_str();

  if ((config.ssid == "YOUR_SSID") || (digitalRead(CTR_RESET)== LOW)){
    
    // -- Enter Access Point Mode to configure.
    accessPointMode = true; 
    debugUART.println("*******************************");
    debugUART.println("Launching in Access Point Mode!");  
    debugUART.println("*******************************");
    debugUART.println("Setting AP (Access Point)…");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("  AP IP address: ");
    Serial.println(IP);
    displayAPScreen(ssid, WiFi.softAPIP().toString());  
   
  }
  
  return retVal;  
}




//****************************************************************************************
// Setup
//****************************************************************************************
void setup() {
  
  initPorts();                      // Set up serial ports and GPIO
  splash();                         // Title, copyright, etc.
  initJSONConfig(filename, config); // Load the program config parameters 

  // Check for an unconfigured base station or RESET button pressed at boot. 
  if (bootToAPMode()==false) { 
      
    // -- Running in Normal Mode.
    debugUART.println("    Device Name: " + config.deviceName);
    debugUART.println("    SSID: " + config.ssid);
    debugUART.println("    ServerURL: " + config.serverURL);
    
    setFullPowerMode(); //Run at full power and max speed with WiFi enabled by default
    delay(500);
    enableWiFi(config);
    
    myMACAddress = getMACAddress();
    debugUART.println("    MAC Address: " + myMACAddress);
      
    configureLoRa(config);
    
    initRTC();

    if (wifiConnected){ // Attempt to synch ESP32 clock with NTP Server...
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      synchTimesToNTP(); // This crashes the system when running with Bailee's phone as an AP. Why? 
      //!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
      
      displayIPScreen(WiFi.localIP().toString());
      delay(5000);
    }

    bootMinute = getRTCMinute();
    heartbeatMinute = bootMinute;
    oldheartbeatMinute = heartbeatMinute; 
       
    mutex_v = xSemaphoreCreateMutex();  //The mutex we will use to protect the jsonMsgBuffer
    
    // Create a task that will be executed in the messageManager() function, 
    //   with priority 0 and executed on core 0
    xTaskCreatePinnedToCore(
      messageManager,      /* Task function. */
      "Message Manager",   /* name of task. */
      10000,               /* Stack size of task */
      NULL,                /* parameter of the task */
      0,                   /* priority of the task */
      &messageManagerTask, /* Task handle to keep track of created task */
      0);                  /* pin task to core 0 */ 

    // Create a task that will be executed in the CountDisplayManager() function, 
    // with priority 0 and executed on core 0
    xTaskCreatePinnedToCore(
      eventDisplayManager, /* Task function. */
      "Display Manager",   /* name of task. */
      10000,               /* Stack size of task */
      NULL,                /* parameter of the task */
      0,                   /* priority of the task */
      &eventDisplayManagerTask, /* Task handle to keep track of created task */
      0);

    displayMode=COUNTERS;
    //displayTextScreen("STATUS","    Listening\n\n       ...");
    displayCountersSummaryScreen("Counter Values",getCounterSummary());
  }

  upTimeMillis = millis() - bootMillis; 
  initWebServer();

  DEBUG_PRINTLN("WAITING 5 SECONDS...");
  delay(5000);
  DEBUG_PRINTLN("RESTARTING WEB SERVER");
  restartWebServer();
      
  debugUART.println();
  debugUART.println("RUNNING\n");
  
}


//****************************************************************************************
// Main Loop
//****************************************************************************************
void loop() {
  String loraMsg;
  
  if (resetFlag){
    debugUART.println("Reset flag has been flipped. Rebooting the processor.");
    delay(2000);  
    ESP.restart();
  }


  
  //**************************************************************************************
  //Access Point Operation
  //**************************************************************************************
  if (accessPointMode){
 
    //Nothing to do here... 
    
  //**************************************************************************************
  //Standard Operation  
  //**************************************************************************************
  } else {
       
    // Check for display mode button being pressed and switch display
      handleModeButtonPress();
    
    // Handle what the LoRa module has to say. 
    // If it's a message from another module, add it to the queue so the manager  
    // function can handle it. Otherwise, just echo to the debugUART.
    
    if (LoRaUART.available()) {
      
      loraMsg = LoRaUART.readStringUntil('\n');
      
      debugUART.println("LoRa Message Received: ");  
      debugUART.println(loraMsg);
           
      //Messages received by the Module start with '+RCV'
      if (loraMsg.indexOf("+RCV")>=0){
        // Send an acknowlegement to the sender.

        // Grab the address of the sender
        String senderAddress = getDeviceAddress(loraMsg); 
      
        // Let the sender know we got the message.
        debugUART.println("Sending ACK. ");
        debugUART.println();
        sendReceiveReyax("AT+SEND=" + senderAddress + ",3,ACK"); 
       
        // Put the message we received on the queue to process
        xSemaphoreTake(mutex_v, portMAX_DELAY);      
          loraMsgBuffer.push(loraMsg);
        xSemaphoreGive(mutex_v);
    
      } else {
        debugUART.println(loraMsg);      
      }         
    }
  } 
}
