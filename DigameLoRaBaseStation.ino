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
#define USE_MQTT true

String model             = "DS-STN-LoRa-WiFi-1";
String model_description = "(LoRa-WiFi Base Station)";  

bool showDataStream = false;

//for Over-the-Air updates...
#include <WiFi.h>

#if USE_MQTT
  #include <MQTT.h>
  MQTTClient client(512);
  WiFiClient net;
#endif

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

#include "globals.h"

// GLOBALS
int displayMode = SPLASH ; // Wake up showing the splash screen

bool   accessPointMode = false;

// Heartbeat managment variables
int    heartbeatMinute;    // Issue Heartbeat message once an hour. Holds the current Minute.
int    oldheartbeatMinute; // Value the last time we looked.
int    bootMinute;         // The minute (0-59) within the hour we woke up at boot. 
bool   heartbeatMessageNeeded = false;
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
bool   processLoRaMessage(String);
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
  
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("*****************************************************");
  DEBUG_PRINTLN("         HEIMDALL Vehicle Counting System");
  DEBUG_PRINTLN("         - WiFi-LoRa BASE STATION UNIT -");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("Model: DS-VC-BASE-LOR-1 (WiFi/LoRa Base Station)");
  DEBUG_PRINTLN("Version: " + SW_VERSION);
  DEBUG_PRINTLN("Copyright 2022, Digame Systems. All rights reserved.");
  DEBUG_PRINTLN();
  DEBUG_PRINT("Compiled on ");
  DEBUG_PRINT(compileDate);
  DEBUG_PRINT(" at ");
  DEBUG_PRINTLN(compileTime); 
  DEBUG_PRINTLN("*****************************************************");
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("HARDWARE INITIALIZATION");
  DEBUG_PRINTLN();

  // first update should be full screen refresh
  initDisplay();
  showWhite();
  displayCenteredText("HEIMDALL", "(Base Station)","","Vehicle","Counting System","Version", SW_VERSION);
  displayCopyright();

}


//****************************************************************************************
// Save a JSON log file to the server
void postDatalog(){
  if (initSDCard()){
    String jsonPayload="";

    DEBUG_PRINTLN("POSTING DATALOG.TXT to server...");
    
    File dataFile = SD.open("/DATALOG.TXT");
  
    // If the file is available, read from it:
    if (dataFile) {
       while (dataFile.available()) {
          jsonPayload = dataFile.readStringUntil('\r');
          jsonPayload.trim();
          postJSON(jsonPayload, config);
       }
       DEBUG_PRINTLN("Done.");
       dataFile.close();
       SD.remove("/DATALOG.TXT"); // Delete the log after upload   
    } else {
      // No file.
      DEBUG_PRINTLN("No DATALOG.TXT present.");  
    }
  } else {
    DEBUG_PRINTLN("ERROR! No SD card present.");
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
      DEBUG_PRINTLN("Appending data file on SD:");
      DEBUG_PRINTLN(jsonPayload);
      
      dataFile.println(jsonPayload);
      dataFile.close();
    }
    // If the file isn't open, pop up an error:
    else {
      DEBUG_PRINTLN("ERROR! Trouble opening datalog.txt");
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
        DEBUG_PRINTLN("We've seen this message before.");
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
      //DEBUG_PRINTLN(trailer);
      idxstop = trailer.indexOf(',');
      
      String strRSSI = trailer.substring(0,idxstop);
      //DEBUG_PRINTLN(strRSSI);
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
      //DEBUG_PRINTLN(trailer);
      idxstop = trailer.indexOf(',');
      String strSNR = trailer.substring(idxstop + 1);
      //DEBUG_PRINTLN(strSNR);  
      strSNR.trim();
      return strSNR;  
    }

// END COUNTER LORA MESSAGE HANDLING FUNCTIONS 



//****************************************************************************************
String loraMsgToJSON(String msg){

  // FIRST, check if we have a well-formed payload. 
  String payload    = getPayload(msg);
  // Deserialize the JSON document in the payload
  StaticJsonDocument<512> doc;
  char json[512] = {};
  payload.toCharArray(json,payload.length()+1);
  DeserializationError error = deserializeJson(doc, json);
  // Test if parsing succeeds.
  if (error) {
    DEBUG_PRINT(F("deserializeJson() failed: "));
    DEBUG_PRINTLN(error.f_str());
    return "IGNORE";
  }
  

  // SECOND, We have a well-formed payload, grab other bits from the message.
  // LoRa address
  String strAddress = getDeviceAddress(msg);
  // Signal strength
  String strRSSI    = getRSSI(msg); 
  // Data Quality 
  String strSNR     = getSNR(msg);
  //TODO: the above calls might fail with corruption of other parts of the message than the payload. Put in a check and return IGNORE if any of these fail. 

    
  // FINALLY, Fetch values from the doc object and build a JSON Message for the server.  
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
      DEBUG_PRINTLN("ERROR: Unknown Message Type!");
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

  if ((displayMode == COUNTERS)) { // If strDisplay changes, the screen will update. See: displayManager task.
                                   // Don't update on non-vehicle events like heartbeat.
    strDisplay = msg; 
  }
  
  return jsonPayload;
  
}

//****************************************************************************************
// Parse a LoRa message from a vehicle counter. Format as a JSON message
// and POST it to the server if WiFi is available. If not, save to the SD card. 
bool processLoRaMessage(String msg){
  String jsonPayload;

  jsonPayload = loraMsgToJSON(msg);

  if (jsonPayload == "IGNORE"){ 
    return false;
  }

  bool result = postJSON(jsonPayload, config); // http POST of data to Jared's server
  
  #if USE_MQTT
    if (config.useMQTT == "checked"){
      if ( (result) && (client.connected()) )
        bool publishSuccessful = client.publish("/digame/event", jsonPayload); // mqqt PUBLISH to broker

        //TODO: think about something like: result = (result && publishSuccessful);
        //...
    }
  #endif
      

  //} else {
    /* TEST TEST TEST - Turn this off temporarily
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
    */
   //} 

   return result;
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
        displayCenteredText("HEIMDALL", "(LIDAR Counter)","","Vehicle","Counting System","Version", SW_VERSION);
        displayCopyright();
        break;
      case NETWORK:
        displayCenteredText("NETWORK","(Station Mode)","","IP Address","",WiFi.localIP().toString());
        displayCopyright();
        break;
      case COUNTERS:
        //displayCountersSummaryScreen("Counter Values",getCounterSummary());
        displayRawText("SUMMARY", getCounterSummary());
        displayCopyright(); 
        break;     
    }
  }   
  
}


bool networkHealthy(){
  //TODO: Test for network being up. -- If not, restart WiFi/MQTT properly.  
  
  if(WiFi.status()!= WL_CONNECTED){ //No WiFi connection?      
    enableWiFi(config); // Attempt to enable it... 
  }
    
  #if USE_MQTT
    if (WiFi.status() == WL_CONNECTED){ // We have a WiFi connection...
      if (config.useMQTT == "checked"){
        if (!client.connected()) {      // Check if we're connected to the MQTT server...
          DEBUG_PRINTLN("Reconnnecting to MQTT server...");
          mqttConnect();                // No? Reconnect. 
        }
        return ( (WiFi.status() == WL_CONNECTED) && (client.connected()) ); //"healthy" = WiFi and MQTT are connected. 
      }
    }
  #endif 
  
  return (WiFi.status() == WL_CONNECTED); // "healthy" = WiFi is connected.
 
}


//****************************************************************************************
// Given a string, can we find a well-formed JSON payload in it?
bool messageIsWellFormed(String msg){
  
  bool retval = true;
  
  String payload = getPayload(msg);
  
  // Deserialize the JSON document in the payload
  StaticJsonDocument<512> doc;
  char json[512] = {};
  payload.toCharArray(json,payload.length()+1);
  DeserializationError error = deserializeJson(doc, json);
 
  // Test if parsing succeeds.
  if (error) {
    DEBUG_PRINTLN("ERROR: Message is not well-formed.");
    DEBUG_PRINT(F("deserializeJson() failed: "));
    DEBUG_PRINTLN(error.f_str());
    retval = false;
  }

  return retval;
  
}

//****************************************************************************************
// Experimenting with using a circular buffer and multi-tasking to enqueue 
// messages to the server...
void messageManager(void *parameter){
  String activeMessage;
  String jsonPayload;

  DEBUG_PRINT("Message Manager Running on Core #: ");
  DEBUG_PRINTLN(xPortGetCoreID());
  
  for(;;){  

    if ( networkHealthy() ) // TEST TEST TEST 
    { 
      
      #if USE_MQTT 
        if (config.useMQTT=="checked")client.loop();  
      #endif
  
      //**********************************************
      // Check if we need to send a boot message
      //**********************************************      
      if (bootMessageNeeded){
        jsonPayload = buildJSONHeader("Boot");
        jsonPayload = jsonPayload + "}";
  
        //DEBUG_PRINTLN(jsonPayload);
        postJSON(jsonPayload, config);
        
        #if USE_MQTT
          if (config.useMQTT == "checked") {
            client.publish("/digame/boot",jsonPayload);
          }
        #endif
  
        xSemaphoreTake(mutex_v, portMAX_DELAY); 
          bootMessageNeeded = false;
        xSemaphoreGive(mutex_v);
        
      }
      
  
      //**********************************************
      // Check if we need to send a heartbeat message
      //**********************************************
      if (lastHeartbeatMillis == 0){ //lastHeartbeatMillis is un-initialized.
        DEBUG_PRINTLN("lastHeartbeatMillis = 0");
        lastHeartbeatMillis = millis();
      }
      
      unsigned long deltaT = (millis() - lastHeartbeatMillis);
      
      unsigned long slippedMilliSeconds; 
      if ( (deltaT) >= config.heartbeatInterval.toInt() * 1000 ){
        DEBUG_PRINT("deltaT: ");
        DEBUG_PRINTLN(deltaT);
        slippedMilliSeconds = deltaT - config.heartbeatInterval.toInt() *1000; // Since this Task is on a 100 msec schedule, we'll always be a little late...
        DEBUG_PRINTLN(slippedMilliSeconds);
        heartbeatMessageNeeded = true;
      }  
      
      if (heartbeatMessageNeeded){
        DEBUG_PRINTLN("Heartbeat needed.");
        
        xSemaphoreTake(mutex_v, portMAX_DELAY); 
          lastHeartbeatMillis = millis() - slippedMilliSeconds; // Small tweak to time to reflect when we should have fired the event. 
          heartbeatMessageNeeded = false;
        xSemaphoreGive(mutex_v);
        
        jsonPayload = buildJSONHeader("Heartbeat");
        jsonPayload = jsonPayload + "}";
  
        //DEBUG_PRINTLN(jsonPayload);
        postJSON(jsonPayload, config);  
        
        #if USE_MQTT
          if (config.useMQTT=="checked"){
            client.publish("/digame/heartbeat", jsonPayload);
          }
        #endif
      }
         
      //********************************************
      // Check for Messages on the Queue and Process
      //********************************************
      if (loraMsgBuffer.size()>0) { // We have a message to send.

        bool postSuccessful = false;
        
        activeMessage = String(loraMsgBuffer.first().c_str()); // Read from the buffer without removing the data from it.
        
        DEBUG_PRINTLN("Active Message: " + activeMessage);
        DEBUG_PRINT("WiFi Connected: ");
        DEBUG_PRINTLN((WiFi.status() == WL_CONNECTED));
        #if USE_MQTT
          if (config.useMQTT == "checked"){
            DEBUG_PRINT("MQTT Connected: ");
          }
        #endif
        DEBUG_PRINTLN(client.connected());


        if (isDuplicateMessage(activeMessage)){ // The counter may not have heard our "ACK" and is sending the same msg again. Ignore it.
          xSemaphoreTake(mutex_v, portMAX_DELAY); 
            DEBUG_PRINTLN("Shifting Duplicate Message...\n");
            activeMessage=loraMsgBuffer.shift();  // Take duplicate messages off the queue.
          xSemaphoreGive(mutex_v);
          postSuccessful = false;
        }else {
          postSuccessful = processLoRaMessage(activeMessage);
        }

        DEBUG_PRINT("POST Successful: ");
        DEBUG_PRINTLN(postSuccessful);
         
        if(postSuccessful){ // HTTP POST worked. If using MQTT, Send the MQTT message as well.
          
          #if USE_MQTT
            if (config.useMQTT == "checked"){
              bool result = client.publish("/digame/lora", String(millis()) + ": " + activeMessage);
              DEBUG_PRINT("MQTT Publish Result: ");
              DEBUG_PRINTLN(result);
            }
          #endif
          
          //Pull data off of the queue after successful transmission.
          // Q: Do we want to require that both HTTP and MQTT worked? 
          //    Currently only need HTTP POST success...
          xSemaphoreTake(mutex_v, portMAX_DELAY); 
            DEBUG_PRINTLN("Shifting Active Message...\n");
            activeMessage=loraMsgBuffer.shift();
          xSemaphoreGive(mutex_v);
 
        }
      
      }
    
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
    DEBUG_PRINT("Display Manager Running on Core #: ");
    DEBUG_PRINTLN(xPortGetCoreID());
  #endif 
  
  for(;;){  

    if (strDisplay != oldStrDisplay){
        oldStrDisplay = strDisplay;        
        //displayEventScreen(strDisplay);
        if (updates % 10 == 0){ showWhite();};
        
        displayRawText("SUMMARY",getCounterSummary());
        displayCopyright();
        updates ++;
      }       
  
    vTaskDelay(eventDisplayUpdateRate / portTICK_PERIOD_MS);
  }
}
    

//****************************************************************************************
// Returns a string containing a summary of counts seen from each VC. 
String getCounterSummary(){
  String retVal  = " Counter  Values\n\n";
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
    DEBUG_PRINTLN("*******************************");
    DEBUG_PRINTLN("Launching in Access Point Mode!");  
    DEBUG_PRINTLN("*******************************");
    DEBUG_PRINTLN("Setting AP (Access Point)…");
    
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("  AP IP address: ");
    Serial.println(IP);
    displayCenteredText("NETWORK","(AP Mode)","","SSID",ssid,"","IP Address", WiFi.softAPIP().toString());
    displayCopyright();
    useOTAFlag = true;
    retVal = true;
 
  }
  
  return retVal;  
}


//MQTT Message Received event
void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
  
  // Note: Do not use the client in the callback to publish, subscribe or
  // unsubscribe as it may cause deadlocks when other things arrive while
  // sending and receiving acknowledgments. Instead, change a global variable,
  // or push to a queue and handle it in the loop after calling `client.loop()`.
}

#if USE_MQTT
  void mqttConnect(){
    unsigned long T1, T2;
    T1 = millis();
    T2 = T1;
    bool timeout = false;
    DEBUG_PRINT("    Connecting to MQTT Broker...");

    while (!client.connect("digame") && (!timeout)){
      Serial.print(".");
      vTaskDelay(100 / portTICK_PERIOD_MS);
      T2 = millis();
      timeout = ((T2-T1) > 6000);
    }
    
    if (client.connected()){ Serial.println("    Connected!"); }else{ Serial.println("    Connect failed.");}   
  }
#endif

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
    DEBUG_PRINTLN("    Device Name: " + config.deviceName);
    DEBUG_PRINTLN("    SSID: " + config.ssid);
    DEBUG_PRINTLN("    Server URL: " + config.serverURL);
    
    setFullPowerMode(); //Run at full power and max speed with WiFi enabled by default
    delay(500);
    enableWiFi(config);
    
    myMACAddress = getMACAddress();
    DEBUG_PRINTLN("    MAC Address: " + myMACAddress);

    #if USE_MQTT
      if(config.useMQTT =="checked"){
        DEBUG_PRINTLN("    MQTT URL: " + config.mqttURL);
        DEBUG_PRINTLN("    MQTT Port: " + config.mqttPort);
        client.begin(config.mqttURL.c_str(), config.mqttPort.toInt(), net);
        client.onMessage(messageReceived);
        mqttConnect();
      }
    #endif 
      
    configureLoRa(config);
    
    initRTC();

    if (wifiConnected){ // Attempt to synch ESP32 clock with NTP Server...
      synchTimesToNTP();  
      displayCenteredText("NETWORK","(Station Mode)","","IP Address","",WiFi.localIP().toString());
      displayCopyright();
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

    displayRawText("SUMMARY",getCounterSummary());
    displayCopyright();
  
  } else {
  
    initWebServer(); // TEST TEST TEST 
  
  }

  upTimeMillis = millis() - bootMillis; 
      
  DEBUG_PRINTLN();
  DEBUG_PRINTLN("RUNNING\n");
  
}


//****************************************************************************************
// Main Loop
//****************************************************************************************
void loop() {
  String loraMsg;
  
  if (resetFlag){
    DEBUG_PRINTLN("Reset flag has been flipped. Rebooting the processor.");
    delay(2000);  
    ESP.restart();
  }
  //**************************************************************************************
  //Standard Operation  
  //**************************************************************************************
  if (!accessPointMode){
    // Check for display mode button being pressed and switch display
      handleModeButtonPress();
    
    // Handle what the LoRa module has to say. 
    // If it's a message from another module, add it to the queue so the manager  
    // function can handle it. Otherwise, just echo to the debugUART.
    if (LoRaUART.available()) {    
      loraMsg = LoRaUART.readStringUntil('\n');  
      DEBUG_PRINTLN("LoRa Message Received: ");  
      DEBUG_PRINTLN(loraMsg);

      if (messageIsWellFormed(loraMsg)) {  // Looks like one of our messages. 
        
        DEBUG_PRINTLN("Message is well-formed.");
        
        //Messages received by the Module start with '+RCV'
        if (loraMsg.indexOf("+RCV")>=0){
          // Grab the address of the sender
          String senderAddress = getDeviceAddress(loraMsg); 
  
          vTaskDelay(50 / portTICK_PERIOD_MS);  // TESTING: Give the counter a little bit to get ready for the ACK. 
          
          // Let the sender know we got the message.
          DEBUG_PRINTLN("Sending ACK. ");
          DEBUG_PRINTLN();
          sendReceiveReyax("AT+SEND=" + senderAddress + ",3,ACK"); 
          
          // Put the message we received on the queue to process
          xSemaphoreTake(mutex_v, portMAX_DELAY);      
            loraMsgBuffer.push(loraMsg);
          xSemaphoreGive(mutex_v);
      
        } else {
          
          DEBUG_PRINTLN(loraMsg);     
           
        } 
                
      }
    }
  }

  // In standard operation, we'll restart the ESP when a /restart url is hit. 
  // This is a silly work around for a bug where the unit has issues when the web UI is inactive for 
  // some time. TODO: Resolve!
  if (restartWebServerFlag){
    DEBUG_PRINTLN("RESTARTING.");
    resetFlag = true;
 }

  upTimeMillis = millis() - bootMillis;
  vTaskDelay(10 / portTICK_PERIOD_MS); 
}
