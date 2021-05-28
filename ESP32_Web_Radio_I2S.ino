// ESP32_Web_Radio_I2S.ino

#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_wifi.h>
#include <Arduino.h>
#include "RadioStation.h"
#include "config.h"

#define DEBUG false
#define LOGSERVER      "192.168.1.4"
#define UDP_LOGGER
#include <SimpleUDPLogger.h>

// Call up the SPIFFS FLASH filing system this is part of the ESP Core
#define FS_NO_GLOBALS
#include <FS.h>

//------------------------------------------------------
// Display
//------------------------------------------------------
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library
#include <SPI.h>
#define TFT_CS     5 
#define TFT_RST    17
#define TFT_DC     16
Adafruit_ST7735 display = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

// JPEG decoder library
#include <JPEGDecoder.h>

//------------------------------------------------------------------------------
// Audio module
//------------------------------------------------------------------------------
#include "Audio.h"
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26
#define MAX_VOLUME    21
#define MIN_VOLUME    0
int volume = 6; 
Audio audio;

//------------------------------------------------------------------------------
// radio stations
//------------------------------------------------------------------------------
  StationList stations;

//------------------------------------------------------------------------------
// Rotary Encoders
// copied from https://github.com/Edzelf/ESP32-Radio
//------------------------------------------------------------------------------
  #define sv DRAM_ATTR static volatile    // alias

  //-----------------------------------------------------------------------------
  // radio station
  //-----------------------------------------------------------------------------
  #define ENC_STATION_CLK_PIN 34
  #define ENC_STATION_DT_PIN 35
  sv int16_t    rotationcount_station = 0;
  int16_t       oldcount_station = 0;

  //-----------------------------------------------------------------------------
  static void IRAM_ATTR isr_enc_station()
  //-----------------------------------------------------------------------------
  {
    sv uint32_t     old_state = 0x0001 ;                          // Previous state
    sv int16_t      locrotcount = 0 ;                             // Local rotation count
    uint8_t         act_state = 0 ;                               // The current state of the 2 PINs
    uint8_t         inx ;                                         // Index in enc_state
    sv const int8_t enc_states [] =                               // Table must be in DRAM (iram safe)
    { 0,                    // 00 -> 00
      -1,                   // 00 -> 01                           // dt goes HIGH
      1,                    // 00 -> 10
      0,                    // 00 -> 11
      1,                    // 01 -> 00                           // dt goes LOW
      0,                    // 01 -> 01
      0,                    // 01 -> 10
      -1,                   // 01 -> 11                           // clk goes HIGH
      -1,                   // 10 -> 00                           // clk goes LOW
      0,                    // 10 -> 01
      0,                    // 10 -> 10
      1,                    // 10 -> 11                           // dt goes HIGH
      0,                    // 11 -> 00
      1,                    // 11 -> 01                           // clk goes LOW
      -1,                   // 11 -> 10                           // dt goes HIGH
      0                     // 11 -> 11
    } ;
    // Read current state of CLK, DT pin. Result is a 2 bit binary number: 00, 01, 10 or 11.
    act_state = ( digitalRead ( ENC_STATION_CLK_PIN ) << 1 ) +
                  digitalRead ( ENC_STATION_DT_PIN ) ;
    inx = ( old_state << 2 ) + act_state ;                        // Form index in enc_states
    locrotcount += enc_states[inx] ;                              // Get delta: 0, +1 or -1
    if ( locrotcount == 4 )
    {
      rotationcount_station++ ;                                   // Divide by 4
      locrotcount = 0 ;
    }
    else if ( locrotcount == -4 )
    {
      rotationcount_station-- ;                                   // Divide by 4
      locrotcount = 0 ;
    }
    old_state = act_state ;                                       // Remember current status
  }

  //-----------------------------------------------------------------------------
  // volume
  //-----------------------------------------------------------------------------
  #define ENC_VOLUME_CLK_PIN 32
  #define ENC_VOLUME_DT_PIN 33
  sv int16_t    rotationcount_volume = volume;
  int16_t       oldcount_volume = 0;

  //-----------------------------------------------------------------------------
  static void IRAM_ATTR isr_enc_volume()
  //-----------------------------------------------------------------------------
  {
    sv uint32_t     old_state = 0x0001 ;                          // Previous state
    sv int16_t      locrotcount = 0 ;                             // Local rotation count
    uint8_t         act_state = 0 ;                               // The current state of the 2 PINs
    uint8_t         inx ;                                         // Index in enc_state
    sv const int8_t enc_states [] =  { 0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0 } ;
    // Read current state of CLK, DT pin. Result is a 2 bit binary number: 00, 01, 10 or 11.
    act_state = ( digitalRead ( ENC_VOLUME_CLK_PIN ) << 1 ) +
                  digitalRead ( ENC_VOLUME_DT_PIN ) ;
    inx = ( old_state << 2 ) + act_state ;                        // Form index in enc_states
    locrotcount += enc_states[inx] ;                              // Get delta: 0, +1 or -1
    if ( locrotcount == 4 )
    {
      rotationcount_volume++ ;                                    // Divide by 4
      locrotcount = 0 ;
    }
    else if ( locrotcount == -4 )
    {
      rotationcount_volume-- ;                                    // Divide by 4
      locrotcount = 0 ;
    }
    old_state = act_state ;                                       // Remember current status
  }

  //------------------------------------------------------------------------------
  void encoderStation_loop() 
  //------------------------------------------------------------------------------
  {
    //lets see if anything changed
    if ( rotationcount_station != oldcount_station )
    {
      if ( rotationcount_station >= stations.numStations )
      {
        rotationcount_station = 0;
      }
      if ( rotationcount_station < 0 )
      {
        rotationcount_station = stations.numStations;
      }
      oldcount_station = rotationcount_station;
      stations.radioStation = rotationcount_station;  
    }
  }
  
  //------------------------------------------------------------------------------
  void encoderVolume_loop() 
  //------------------------------------------------------------------------------
  {
    //lets see if anything changed
    if ( rotationcount_volume != oldcount_volume )
    {
      if ( rotationcount_volume > MAX_VOLUME )
      {
        rotationcount_volume = MAX_VOLUME;
      }
      if ( rotationcount_volume < MIN_VOLUME )
      {
        rotationcount_volume = MIN_VOLUME;
      }
      oldcount_volume = rotationcount_volume;
      volume = (int) rotationcount_volume;
      audio.setVolume(volume);
      if(DEBUG) {
        Serial.print("set volume to ");
        Serial.println(volume); }
      UDP_LOG_INFO("set volume to %lu", volume);
    }
  }

//------------------------------------------------------------------------------
// Network
//------------------------------------------------------------------------------
  HTTPClient  http_stream;
  WiFiClient  * stream; 
  unsigned long timeout;
   
  //------------------------------------------------------------------------------
  void connectToWIFI()
  //------------------------------------------------------------------------------
  {
    if(DEBUG) {Serial.print("Connecting to Wifi");}
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);

    IPAddress staticIP(192, 168, 1, 135); //ESP static ip
    IPAddress gateway(192, 168, 1, 1);    //IP Address of your WiFi Router (Gateway)
    IPAddress subnet(255, 255, 255, 0);   //Subnet mask
    IPAddress dns(192, 168, 1, 4);       //DNS
    
    WiFi.config(staticIP, gateway, subnet, dns);
    delay(100);
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) 
    {
      if(DEBUG) {Serial.print(".");}
      delay(500);
    }
    if(DEBUG) {Serial.println("\nWifi connected");}  
  }

  //------------------------------------------------------------------------------
  void station_connect (int station_no ) 
  //------------------------------------------------------------------------------
  {
    char rc[4];
    int httpCode;  
    char url[100];
    const char* headerNames[] = { "Location" };
    String Location;
    int pos;
    int16_t   x1, y1;
    uint16_t  w, h;

    if(DEBUG) { Serial.println(stations.station[station_no].label); }
    if(DEBUG) { Serial.println(stations.station[station_no].url); }
    UDP_LOG_INFO(stations.station[station_no].label);      
    UDP_LOG_INFO(stations.station[station_no].url);
    
    //-------------------------------
    //--- check for http redirect ---
    //-------------------------------
    HTTPClient http;
    http.begin(stations.station[station_no].url);
    http.collectHeaders(headerNames, sizeof(headerNames)/sizeof(headerNames[0]));
    httpCode = http.GET();    

    if(DEBUG) { Serial.println(httpCode); }
    itoa(httpCode, rc, 10);
    UDP_LOG_INFO(rc);

    //------- follow redirect -----------
    if (httpCode == 302)
    {
      if(DEBUG) { Serial.println("following redirect"); }
      UDP_LOG_INFO("following redirect");
      Location = http.header("Location");
      pos = Location.indexOf('?');
      if (pos == -1)
      {
        Location.toCharArray(url, Location.length()+1);
      }
      else
      {
        Location.toCharArray(url, pos+1);
      }
      if(DEBUG) { Serial.println(url); }
      UDP_LOG_INFO(url);
    }
    else
    {
      strcpy(url, stations.station[station_no].url);
    }
    http.end();

    // write station to display
    display.fillRect(0, 72, 127, 127, ST7735_BLACK);
    int len = strlen(stations.station[station_no].label);
    char buf1[17];
    char buf2[9];
    if (len < 17 )
      { 
        display.getTextBounds(stations.station[station_no].label, 0, 80, &x1, &y1, &w, &h);
        display.setCursor(62-(w/2), 80);
        display.print(stations.station[station_no].label); 
      }
    else
      { 
        memcpy( buf1, &stations.station[station_no].label[0], 16);
        buf1[16] = 0;
        display.getTextBounds(buf1, 0, 80, &x1, &y1, &w, &h);
        display.setCursor(62-(w/2), 80);
        display.print(buf1);
        memcpy( buf2, &stations.station[station_no].label[16], len-16);
        buf2[len-16] = 0;
        display.getTextBounds(buf2, 0, 80, &x1, &y1, &w, &h);
        display.setCursor(62-(w/2), 90);
        display.print(buf2);
      }

    audio.connecttohost(url);
  }

//------------------------------------------------------------------------------
// setup
//------------------------------------------------------------------------------
void setup () 
{
  if(DEBUG) { Serial.begin(115200); }

  // SPIFFS
  if(DEBUG) { Serial.println("Init SPIFFS..."); }
  if (!SPIFFS.begin()) {
    if(DEBUG) { Serial.println("SPIFFS initialisation failed!"); }
    while (1) yield();
  }
  
  // Display
  if(DEBUG) { Serial.println("Init display..."); }
  display.initR(INITR_144GREENTAB); // Init ST7735R chip, green tab
  display.fillScreen(ST7735_BLACK);
  display.setRotation(2);
  // show jpeg image
  drawJpeg("/philco45c.jpg", 30, 0);
  // show text
  display.setCursor(39, 80);
  display.setTextWrap(true);
  display.setTextColor(ST7735_ORANGE);
  display.print("WebRadio");

  // connect to access point
  connectToWIFI();

  // setup rsyslog
  UDP_LOG_BEGIN(LOGSERVER, LOG_MODE_DEBUG);
  UDP_LOG_INFO("Starting Philco Internet Radio");

  // import station list
  if(DEBUG) { Serial.println("Reading station list..."); }
  UDP_LOG_INFO("Reading station list...");
  HTTPClient  http;
  http.begin(url_stationlist);
  int httpCode = http.GET();
  if(httpCode == HTTP_CODE_OK)
  {
    String payload = http.getString();
    stations.parseStations(payload);
  } 
  else
  {
    if(DEBUG) { Serial.println("Reading station list failed!"); }
    UDP_LOG_ERROR("Reading station list failed!");      
  }
  http.end();
  
  // initialize the Audio
  if(DEBUG) { Serial.println("Initializing Audio..."); }
  UDP_LOG_INFO("Initializing Audio...");
  audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
  audio.setVolume(volume); // 0...21

  // initialize rotary encoder
  pinMode(ENC_STATION_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_STATION_DT_PIN, INPUT_PULLUP);
  attachInterrupt ( digitalPinToInterrupt(ENC_STATION_CLK_PIN), isr_enc_station,   CHANGE ) ;
  attachInterrupt ( digitalPinToInterrupt(ENC_STATION_DT_PIN),  isr_enc_station,   CHANGE ) ;   
  pinMode(ENC_VOLUME_CLK_PIN, INPUT_PULLUP);
  pinMode(ENC_VOLUME_DT_PIN, INPUT_PULLUP);
  attachInterrupt ( digitalPinToInterrupt(ENC_VOLUME_CLK_PIN), isr_enc_volume,   CHANGE ) ;
  attachInterrupt ( digitalPinToInterrupt(ENC_VOLUME_DT_PIN),  isr_enc_volume,   CHANGE ) ;      

}

//------------------------------------------------------------------------------
// loop
//------------------------------------------------------------------------------
void loop() 
{
  encoderStation_loop();
  encoderVolume_loop();

  // switch radio station
  if(stations.radioStation!=stations.previousRadioStation)
  {
    station_connect(stations.radioStation);
    stations.previousRadioStation = stations.radioStation;
  }
  
  // play mp3 stream
  audio.loop();
}
