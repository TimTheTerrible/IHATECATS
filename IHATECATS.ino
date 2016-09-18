#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VC0706.h>
#include <Adafruit_FeatherOLED.h>
#include <Adafruit_FeatherOLED_WiFi.h>
#include <adafruit_feather.h>
#include <adafruit_mqtt.h>
#include <adafruit_aio.h>
#include <SD.h>
#include <SPI.h>
#include <Ethernet2.h>
#include <IniFile.h>
#include <debugprint.h>

#define RELAY_PIN                 PA0
#define VBAT_PIN                  PA1
#define SDCARD_PIN                PB4

const size_t bufferlen = 80;
char wlanSSID[bufferlen];
char wlanKey[bufferlen];
char aioEndpoint[bufferlen];
char aioUsername[bufferlen];
char aioKey[bufferlen];

#define FEED_VBAT                 "vbat"
#define FEED_RSSI                 "rssi"

int updateWait = 10000;  // wait 10 seconds between updates
int updateTime;

AdafruitAIO                        aio("none","none");
AdafruitAIOFeedGauge<float>        feedVBAT(&aio, FEED_VBAT);
AdafruitAIOFeedGauge<float>        feedRSSI (&aio, FEED_RSSI);

Adafruit_FeatherOLED_WiFi  oled = Adafruit_FeatherOLED_WiFi();

Adafruit_VC0706 cam = Adafruit_VC0706(&Serial2);

/**************************************************************************/
/*!
    @brief  Read the WiFi and IOT info
*/
/**************************************************************************/
bool readConfig()
{
  const size_t bufferLen = 80;
  char buffer[bufferLen];

  IniFile ini("/config.ini");

  debugprint(DEBUG_TRACE, "Reading config file...");

  // Open the config file
  if ( ! ini.open() ) {
    debugprint(DEBUG_ERROR, "Failed to open config file!");
    return false;
  }

  // Validate the config file
  if ( ! ini.validate(buffer, bufferLen) ) {
    debugprint(DEBUG_ERROR, "Invalid config file!");
    return false;
  }

  if ( ! ini.getValue("wifi", "ssid", buffer, bufferlen) ) {
      debugprint(DEBUG_ERROR, "Failed to read WiFi SSID from config file!");
      return false;
  }

  // Copy the buffer to the WiFi SSID
  strncpy(wlanSSID, buffer, bufferlen);
  
  if ( ! ini.getValue("wifi", "key", buffer, bufferlen) ) {
    debugprint(DEBUG_ERROR, "Failed to read WiFi key from config file!");
    return false;
  }

  // Copy the buffer to the WiFi Key
  strncpy(wlanKey, buffer, bufferlen);

  if ( ! ini.getValue("aio", "endpoint", buffer, bufferlen) ) {
    debugprint(DEBUG_ERROR, "Failed to read AIO endpoint from config file!");
    return false;
  }

  // Copy the buffer to the AIO endpoint
  strncpy(aioEndpoint, buffer, bufferlen);

  if ( ! ini.getValue("aio", "username", buffer, bufferlen) ) {
    debugprint(DEBUG_ERROR, "Failed to read AIO username from config file!");
    return false;
  }

  // Copy the buffer to the AIO username
  strncpy(aioUsername, buffer, bufferlen);

  if ( ! ini.getValue("aio", "key", buffer, bufferlen) ) {
    debugprint(DEBUG_ERROR, "Failed to read AIO key from config file!");
    return false;
  }

  // Copy the buffer to the AIO Key
  strncpy(aioKey, buffer, bufferlen);

  debugprint(DEBUG_TRACE, "Config file read");
  return true;
}

/**************************************************************************/
/*!
    @brief  Connect to the AP
    @return Error flag
*/
/**************************************************************************/
bool connectAP()
{   
  debugprint(DEBUG_TRACE, "Connecting to SSID %s...", wlanSSID);

  oled.refreshIcons();
  oled.clearMsgArea();
  oled.print("SSID: ");
  oled.println(wlanSSID);
  oled.println("Connecting...");
  oled.display();

  // Attempt to connect to the AP
  if ( Feather.connect(wlanSSID, wlanKey) )
  {
    debugprint(DEBUG_TRACE, "Connected");

    int8_t rssi = Feather.RSSI();
    uint32_t ipAddress = Feather.localIP();
    oled.setConnected(true);
    oled.setRSSI(rssi);
    oled.setIPAddress(ipAddress);
    oled.refreshIcons();
    oled.clearMsgArea();
    oled.println("Connected");
    oled.display();
  }
  else
  {
    // Display the error message
    err_t err = Feather.errno();
    oled.setConnected(false);
    oled.refreshIcons();
    oled.clearMsgArea();
    oled.println("Connection Error:");
    switch (err)
    {
      case ERROR_WWD_ACCESS_POINT_NOT_FOUND:
        // SSID wasn't found when scanning for APs
        oled.println("Invalid SSID");
        break;
      case ERROR_WWD_INVALID_KEY:
        // Invalid SSID passkey
        oled.println("Invalid Password");
        break;
      default:
        // The most likely cause of errors at this point is that
        // you are just out of the device/AP operating range
        oled.print(Feather.errno());
        oled.print(":");
        oled.println(Feather.errstr());
        oled.refreshIcons(); // Refresh icons in case the text ran over
        break;
    }    
    oled.display();

    debugprint(DEBUG_ERROR, "Failed to connect to AP!");
    return false;
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Connect to the IOT broker
    @return Error flag
*/
/**************************************************************************/
bool connectIOT()
{   
  debugprint(DEBUG_TRACE, "Connecting to IOT broker...");

  // Set the AIO credentials
  aio.setCredentials(aioUsername, aioKey);

  // Attempt to connect to a Broker
  oled.clearMsgArea();
  oled.print("AIO: ");
  oled.println(aioEndpoint);
  oled.println("Connecting...");
  oled.display();

  // Connect to AIO server
  if ( aio.connect() )
  {
    debugprint(DEBUG_TRACE, "Connected");
    oled.clearMsgArea();
    oled.println("Connected");
    oled.display();
    delay(3000);
    return true;
  }
  else
  {
    debugprint(DEBUG_ERROR, "Failed to connect!");
    oled.clearMsgArea();
    oled.print("Failed! Error: ");
    oled.println(aio.errno(), HEX);
    oled.display();
    delay(3000);
    return false;
  }
}  

/**************************************************************************/
/*!
    @brief  Attempt to recconect the wifi
*/
/**************************************************************************/
void reconnect()
{
  // Stop the camera momentarily...
  cam.setMotionDetect(false);

  // The connection was lost ... reset the status icons
  oled.setConnected(false);
  oled.setRSSI(0);
  oled.setIPAddress(0);
  oled.refreshIcons();
  oled.clearMsgArea();
  oled.println("Disconnected");
  oled.println("Re-connecting...");
  oled.display();

  // Try to re-connect to WiFi
  if ( connectAP() && connectIOT() ) {
    oled.clearMsgArea();
    oled.print("SSID: ");
    oled.println(wlanSSID);
    oled.print("AIO: ");
    oled.println(aioEndpoint);
    oled.display();
  }
  else {
    debugprint(DEBUG_ERROR, "Connection failed!");
  }
  // Turn the camera back on
  cam.setMotionDetect(true);
}

/**************************************************************************/
/*!
    @brief Update the battery voltage
*/
/**************************************************************************/
void updateVbat() 
{
  int   vbatADC   = 0;         // The raw ADC value off the voltage div
  float vbatFloat = 0.0F;      // The ADC equivalent in millivolts
  float vbatLSB   = 0.80566F;  // mV per LSB

  // Read the analog in value:
  vbatADC = analogRead(VBAT_PIN);
  vbatADC = analogRead(VBAT_PIN);

  // Multiply the ADC by mV per LSB, and then
  // double the output to compensate for the
  // 10K+10K voltage divider
  vbatFloat = ((float)vbatADC * vbatLSB) * 2.0F;

  oled.setBattery(vbatFloat/1000);
  
  // Push VBAT out to MQTT if possible
  if (aio.connected())
  {
    feedVBAT = vbatFloat/1000;
  }
}

/**************************************************************************/
/*!
    @brief Update the RSSI
*/
/**************************************************************************/
void updateRSSI() 
{
  // Update the RSSI value
  int8_t rssi = Feather.RSSI();
  oled.setRSSI(rssi);

  // Push RSSI out to MQTT if possible
  if (aio.connected())
  {
    feedRSSI = (float)rssi;
  }
}

/**************************************************************************/
/*!
    @brief AIO callback when there is new value with Feed VBAT
*/
/**************************************************************************/
void aio_vbat_callback(float value)
{
  //oled.println("AIO VBAT: ");
  //oled.display();
}

/**************************************************************************/
/*!
    @brief AIO callback when there is new value with Feed RSSI
*/
/**************************************************************************/
void aio_rssi_callback(float value)
{
  //oled.println("AIO RSSI:");
  //oled.display();
}

/**************************************************************************/
/*!
    @briefCheck for motion, capture and store an image, and toggle the relay
*/
/**************************************************************************/
void checkCamera() {

  if (cam.motionDetected()) {

    debugprint(DEBUG_TRACE, "Motion!");
    oled.clearMsgArea();
    oled.println("Motion detected!");
    oled.display();

    // Stop motion detection while we work with the camera...
    cam.setMotionDetect(false);

    // Click the relay to spray some air...
    digitalWrite(RELAY_PIN, HIGH);
    delay(100);
    digitalWrite(RELAY_PIN, LOW);

    // How fast does a cat react to being sprayed with air?
    // delay(100);

    // Then take a picture...
    if ( ! cam.takePicture() ) {
      debugprint(DEBUG_ERROR, "Failed to snap!");
      oled.clearMsgArea();
      oled.println("Failed to snap!");
      oled.println("Check the camera?");
      oled.display();
      delay(5000);
    }
    else {
      debugprint(DEBUG_TRACE, "Picture taken");
  
      int count = 0;
      char filename[15];

      do {
        sprintf(filename, "IMG_%4.4d.JPG", ++count);        
      }  while ( SD.exists(filename) );

      char msg[24];
      sprintf(msg, "Saving %s", filename);
      oled.println(msg);
      oled.display();

      File imgFile = SD.open(filename, FILE_WRITE);
      
      uint16_t jpglen = cam.frameLength();
      debugprint(DEBUG_TRACE, "%d byte image", jpglen);
      debugprint(DEBUG_TRACE, "Writing image to %s", filename);
      
      byte wCount = 0; // For counting # of writes
      while (jpglen > 0) {
        // read 32 bytes at a time;
        // TODO: where does this memory come from? who allocates it? is it ever freed?
        uint8_t *buffer;
        uint8_t bytesToRead = min(64, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
        buffer = cam.readPicture(bytesToRead);
        imgFile.write(buffer, bytesToRead);
    
        if(++wCount >= 32) { // Every 2K, give a little feedback so it doesn't appear locked up
          Serial.print('.');
          wCount = 0;
        }
        //debugprint(DEBUG_TRACE, "Read %d bytes", bytesToRead);
    
        jpglen -= bytesToRead;
      }
      imgFile.close();
      debugprint(DEBUG_TRACE, "Done");
      cam.resumeVideo();

      oled.clearMsgArea();
      oled.println("Image saved!");
      oled.println("Waiting...");
      oled.display();
    }
  }

  // Turn motion detection back on...
  cam.setMotionDetect(true);
}

/**************************************************************************/
/*!
    @brief  The setup function runs once when the board comes out of reset
*/
/**************************************************************************/
void setup()
{
  // ALWAYS START SERIAL FIRST!!!
  Serial.begin(115200);
  delay(2000);

  debugprint(DEBUG_TRACE, "IHATECATS 0.1.0");

  // Setup the LED pin
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);

  // Set up the SD card pin
  pinMode(SDCARD_PIN, OUTPUT);
  digitalWrite(SDCARD_PIN, HIGH);

  // Set up the relay pin
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // Set up the battery sense pin
  pinMode(VBAT_PIN, INPUT_ANALOG);

  // Setup the OLED display
  oled.init();
  oled.clearDisplay();
  oled.println("IHATECATS 0.1.0");
  oled.println("Reading config...");
  oled.display();

  // Start SPI and open the SD card
  SPI.begin();
  if ( ! SD.begin(SDCARD_PIN) ) {
    oled.println("No SD card?");
    oled.display();
    debugprint(DEBUG_ERROR, "HALT: Failed to initialize the SD card!");
    while ( 1 );
  }

  if ( ! readConfig() ) {
    oled.println("Bad config file?");
    oled.display();
    debugprint(DEBUG_ERROR, "HALT: Failed to read config file!");
    while(1);
  }

  updateVbat();
  updateRSSI();

  // Turn off the LED to show we're connected
  digitalWrite(BOARD_LED_PIN, LOW);

  // Follow the feeds
  //feedVBAT.follow(aio_vbat_callback);
  //feedRSSI.follow(aio_rssi_callback);

  // Connect the camera
  if ( ! cam.begin() ) {
    oled.println("Camera attached?");
    oled.display();
    debugprint(DEBUG_ERROR, "HALT: No camera found!");
    while(1);
  }
  else {
    debugprint(DEBUG_TRACE, "Camera connected");
  }

  // Set up the camera
  cam.setImageSize(VC0706_640x480);
  cam.setMotionDetect(true);

  oled.println("Startup complete");
  oled.display();
  debugprint(DEBUG_INFO, "Startup complete");

  delay(2500);
}

/**************************************************************************/
/*!
    @brief  This loop function runs over and over again
*/
/**************************************************************************/
void loop()
{
  if ( Feather.connected() )
  {
    if ( millis() > updateTime ) {
      // Update the battery level
      updateVbat();
  
      // Update the RSSI level
      updateRSSI();
  
      // Update the OLED
      oled.refreshIcons();
      oled.display();

      updateTime = millis() + updateWait;
    }
  
    // Check the camera
    checkCamera();
  }
  else {
    reconnect();
  }

  oled.refreshIcons();
  delay(200);
}

