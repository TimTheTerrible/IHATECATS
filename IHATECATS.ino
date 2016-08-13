#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
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

#define SD_SELECT                 PB4
#define VBAT_PIN                  PA1

/*

#define WLAN_SSID                 "dforge"
#define WLAN_PASS                 "qpwoeiruty"

#define AIO_USERNAME              "TimTheTerrible"
#define AIO_KEY                   "397b3924e250425eaac3dabfd8786cab"

*/

const size_t bufferlen = 80;
char wlanSSID[bufferlen];
char wlanKey[bufferlen];
char aioEndpoint[bufferlen];
char aioUsername[bufferlen];
char aioKey[bufferlen];

#define FEED_VBAT                 "vbat"
#define FEED_RSSI                 "rssi"

AdafruitAIO                        aio("none","none");
AdafruitAIOFeedGauge<float>        feedVBAT(&aio, FEED_VBAT);
AdafruitAIOFeedGauge<float>        feedRSSI (&aio, FEED_RSSI);

Adafruit_FeatherOLED_WiFi  oled = Adafruit_FeatherOLED_WiFi();

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
    debugprint(DEBUG_ERROR, "Failed to open config file");
    return false;
  }

  // Validate the config file
  if ( ! ini.validate(buffer, bufferLen) ) {
    debugprint(DEBUG_ERROR, "Invalid config file");
    return false;
  }

  if ( ! ini.getValue("wifi", "ssid", buffer, bufferlen) ) {
      debugprint(DEBUG_ERROR, "Failed to read WiFi SSID from config file");
      return false;
  }

  // Copy the buffer to the WiFi SSID
  strncpy(wlanSSID, buffer, bufferlen);
  
  if ( ! ini.getValue("wifi", "key", buffer, bufferlen) ) {
    debugprint(DEBUG_ERROR, "Failed to read WiFi key from config file");
    return false;
  }

  // Copy the buffer to the WiFi Key
  strncpy(wlanKey, buffer, bufferlen);

  if ( ! ini.getValue("aio", "endpoint", buffer, bufferlen) ) {
    debugprint(DEBUG_ERROR, "Failed to read AIO endpoint from config file");
    return false;
  }

  // Copy the buffer to the AIO endpoint
  strncpy(aioEndpoint, buffer, bufferlen);

  if ( ! ini.getValue("aio", "username", buffer, bufferlen) ) {
    debugprint(DEBUG_ERROR, "Failed to read AIO username from config file");
    return false;
  }

  // Copy the buffer to the AIO username
  strncpy(aioUsername, buffer, bufferlen);

  if ( ! ini.getValue("aio", "key", buffer, bufferlen) ) {
    debugprint(DEBUG_ERROR, "Failed to read AIO key from config file");
    return false;
  }

  // Copy the buffer to the AIO Key
  strncpy(aioKey, buffer, bufferlen);

  debugprint(DEBUG_TRACE, "Config file read!");
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
  oled.println("Connecting to...");
  oled.println(wlanSSID);
  oled.display();

  // Attempt to connect to the AP
  if ( Feather.connect(wlanSSID, wlanKey) )
  {
    debugprint(DEBUG_TRACE, "Connected!");

    int8_t rssi = Feather.RSSI();
    uint32_t ipAddress = Feather.localIP();
    oled.setConnected(true);
    oled.setRSSI(rssi);
    oled.setIPAddress(ipAddress);
    oled.refreshIcons();
    oled.clearMsgArea();
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

    debugprint(DEBUG_ERROR, "Failed to connect to AP");
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
  oled.println(aioEndpoint);
  oled.display();

  // Connect to AIO server
  if ( aio.connect() )
  {
    debugprint(DEBUG_TRACE, "Connected!");
    oled.println("Connected!");
    oled.display();
  }
  else
  {
    debugprint(DEBUG_ERROR, "Failed to connect!");
    oled.print("Failed! Error: ");
    oled.println(aio.errno(), HEX);
    oled.display();
    delay(3000);

    return false;
  }

  return true;
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
    @brief  The setup function runs once when the board comes out of reset
*/
/**************************************************************************/
void setup()
{
  // ALWAYS START SERIAL FIRST!!!
  Serial.begin(115200);
  delay(2000);

  debugprint(DEBUG_TRACE, "IHATECATS 0.99.0");

  // Setup the LED pin
  pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, HIGH);

  // Set up the SD card pin
  pinMode(SD_SELECT, OUTPUT);
  digitalWrite(SD_SELECT, HIGH);

  // Set up the battery sense pin
  pinMode(VBAT_PIN, INPUT_ANALOG);

  // Setup the OLED display
  oled.init();
  oled.clearDisplay();

  SPI.begin();

  if ( ! SD.begin(SD_SELECT) ) {
    oled.println("No SD card?");
    oled.display();
    debugprint(DEBUG_ERROR, "HALT: Failed to initialize the SD card!");
    while ( 1 );
  }

  if ( ! readConfig() ) {
    debugprint(DEBUG_ERROR, "HALT: Failed to read config file");
    while(1);
  }
    
  oled.setBatteryIcon(true);

  updateVbat();
  updateRSSI();

  // Turn off the LED to show we're connected
  digitalWrite(BOARD_LED_PIN, LOW);

  // Follow the feeds
  //feedVBAT.follow(aio_vbat_callback);
  //feedRSSI.follow(aio_rssi_callback);
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
    // Update the battery level
    updateVbat();

    // Update the RSSI level
    updateRSSI();

    // Update the OLED
    oled.refreshIcons();
  }
  else
  {
    // The connection was lost ... reset the status icons
    oled.setConnected(false);
    oled.setRSSI(0);
    oled.setIPAddress(0);
    oled.clearMsgArea();
    oled.println("Disconnected!");
    oled.println("Connecting...");
    oled.display();

    // Try to re-connect to WiFi
    if ( connectAP() ) {

      // Try to re-connect to broker
      if ( connectIOT() ) {
        // do something
      }
      else {
        debugprint(DEBUG_ERROR, "Failed to connect IOT broker");
      }
    }
    else {
      debugprint(DEBUG_ERROR, "Failed to connect WiFi AP");
    }
  }

  oled.refreshIcons();
  delay(10000);
}

