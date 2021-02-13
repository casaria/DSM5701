
#define LOAD_GFXFF


#define CF_OL24 &Orbitron_Light_24
#define CF_OL32 &Orbitron_Light_32
#define CF_RT24 &Roboto_Thin_24
#define CF_S24 &Satisfy_24
#define CF_Y32 &Yellowtail_32

//fix CPU  idle WD trigger
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include "Arduino.h"
//#include  <avr/wdt.h>
#include <EEPROM.h>
//#include "heltec.h"
#include <math.h>
//#include "oled/OLEDDisplayUi.h"
#include <string.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <TimeLib.h>
#include <time.h>
#include <ArduinoJson.h> //use version 5.13.5
#include <HTTPClient.h>
#include <DNSServer.h>

#include <MD_MAX72xx.h>
#include <SPI.h>

#include <MD_Parola.h>
#include "TFT_eSPI.h"

#include <Wire.h>
#include <Button2.h>
#include <esp_task_wdt.h>
#include "esp_adc_cal.h"

//#include <task.h>

//#include "images.h"
#include "bmp.h"

// Optional functionality. Comment out defines to disable feature

#define HTTP_OTA         // Enable OTA updates from http server
#define LED_STATUS_FLASH // Enable flashing LED status
#define STATUS_LED 2     // Built-in blue LED on pin 2

#ifdef ARDUINO_OTA
/* Over The Air updates directly from Arduino IDE */
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#define ARDUINO_OTA_PORT 8266
#define ARDUINO_OTA_HOSTNAME "esp32"
#define ARDUINO_OTA_PASSWD "123"
#endif

#ifdef HTTP_OTA
/* Over The Air automatic firmware update from a web server.  ESP8266 will contact the
    *  server on every boot and check for a firmware update.  If available, the update will
    *  be downloaded and installed.  Server can determine the appropriate firmware for this 
    *  device from any combination of HTTP_OTA_VERSION, MAC address, and firmware MD5 checksums.
    */
#include <HTTPClient.h>
#include <HTTPUpdate.h>

#define HTTP_OTA_ADDRESS F("CASARIA.NET")    // Address of OTA update server
#define HTTP_OTA_PATH F("/esp32-ota/update") // Path to update firmware
#define HTTP_OTA_PORT 1880                   // Port of update server \
                                             // Name of firmware
#define HTTP_OTA_VERSION String(__FILE__).substring(String(__FILE__).lastIndexOf('/') + 1) + ".esp32"
#endif

// Stock font and GFXFF reference handle
#define GFXFF 1
#define FF18 &FreeSans12pt7b

// Custom are fonts added to library "TFT_eSPI\Fonts\Custom" folder
// a #include must also be added to the "User_Custom_Fonts.h" file
// in the "TFT_eSPI\User_Setups" folder. See example entries.

#include <Ticker.h>
Ticker watchdog;
#define WATCHDOG_SETUP_SECONDS 30 // Setup should complete well within this time limit
#define WATCHDOG_LOOP_SECONDS 20  // Loop should complete well within this time limit

#define CONFIG_ESP_TASK_WDT_TIMEOUT 10000
#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0 NO
#define CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 NO

#ifndef TFT_DISPOFF
#define TFT_DISPOFF 0x28
#endif

#ifndef TFT_SLPIN
#define TFT_SLPIN 0x10
#endif

#define TFT_BACKLIGHT_ON HIGH // HIGH or LOW are options
#define ADC_EN 14
#define ADC_PIN 34
#define BUTTON_1 35
#define BUTTON_2 0

#define ac1 37
#define ac2 38

#define relay1 39
#define relay2 32
#define statusLED 17
#define lockLatch 25
#define lockUnLatch 26

#define timeSeconds 3
#define CHAR_SPACING 1

const char *UNIT = "DOOR2";
const char *GROUP = "WFS";

// Define the number of devices we have in the chain and the hardware interface
// NOTE: These pin numbers will probably not work with your hardware and may
// need to be adaptedW // RED   // blue FC16_HW
#define MAX_DEVICES 8
#define MATRIX_INTENSITY 2
#define MATRIC_AUTO_ADJUST_INTENSITY

#define DEMO_DURATION 3000
typedef void (*Demo)(void);

#define HARDWARE_TYPE MD_MAX72XX::FC16_HW //ICSTATION_HW  //FC16_HW
#define CLK_PIN 15                        //13
#define DATA_PIN 12                       //14  //2
#define CS_PIN 13                         //27  //17

//
//extern Heltec_ESP32 Heltec;
//OLEDDisplayUi ui( Heltec.display );

//TimeZone variables
String TZAPIKEY = "WO3E5U09CCNS"; //http://timezonedb.com used to get gmtOffset
String payload, TZone, lastUpdate;
String coordinate;   // = lat,lon
String coordinateTZ; //lat=xx.xxx&lng=xxx.xxx
long gmtOffset_sec;
struct astroSunTimes {
  float sunSet;
  float sunRise;
};
long secs2000 = SECS_YR_2000;

const char *ssid = "CASARIA-CL2";
const char *password = "peterthegreat";
const char *mqttServer = "ccc.casaria.net";
const int mqttPort = 1883;
const char *mqttUser = "IOTnode";
const char *mqttPassword = "cargo57130";

enum Alerts
{
  amIdle,
  amGreen,
  amOrange,
  amRed
};
enum Alerts AlertMode = amIdle;

char szText[20];
float tempc1, tempc2, hum1, hum2;
String TEMPC1, TEMPC2, szHUM1, szHUM2, szMESSAGE, CMD;
char C1[7];
char C2[7];
char HUM1[7];
char HUM2[7];
char timeString[14];
char shortTimeString[10];
int cheer_red = 0;
int cheer_green = 0;
int cheer_blue = 0;
unsigned int rgb565Decimal = 8090; //5338; //0x8410;
unsigned int newrgb565Decimal;
String rgb565Hex;
String colourString = "CASARIA";
String strData;
String topicStr;

bool bPushTrigger = false;

char buff[512];
int vref = 1100;
int btnCick = false;

// SPI hardware interface
//MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);
// Arbitrary pins/
//MD_MAX72XX mx = MD_MAX72XX(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

// Text parameters

MD_Parola P = MD_Parola(HARDWARE_TYPE, DATA_PIN, CLK_PIN, CS_PIN, MAX_DEVICES);

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
Button2 btn1(BUTTON_1);
Button2 btn2(BUTTON_2);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);
WiFiClient espClient;
PubSubClient client(espClient);

SemaphoreHandle_t serialMutex = NULL;

#ifdef LED_STATUS_FLASH
Ticker flasher;

void flash()
{
  digitalWrite(STATUS_LED, !digitalRead(STATUS_LED));
}
#endif
/*

    void msOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->setFont(ArialMT_Plain_10);
    display->drawString(128, 0, String(millis()));
    }

    void drawFrame1(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    display->drawXbm(x, y, BT_width, BT_height, BT_bits);
    display->drawXbm(x + 12 + 1, y, WIFI_width, WIFI_height, WIFI_bits);
    display->drawXbm(x + 108, y, BAT_width, BAT_height, BAT_bits);
    display->drawXbm(x + 34, y + 14, WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits);
    }

    void drawFrame2(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    display->drawXbm(x, y, BT_width, BT_height, BT_bits);
    display->drawXbm(x + 12 + 1, y, WIFI_width, WIFI_height, WIFI_bits);
    display->drawXbm(x + 108, y, BAT_width, BAT_height, BAT_bits);
    display->drawXbm(x + 34, y + 12, LoRa_Logo_width, LoRa_Logo_height, LoRa_Logo_bits);
    }

    void drawFrame3(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    display->drawXbm(x, y + 5, HelTec_LOGO_width, HelTec_LOGO_height, HelTec_LOGO_bits);
    }

    void drawFrame4(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->setFont(ArialMT_Plain_16);
    display->drawString(x, y, "CASARIA");
    display->setFont(ArialMT_Plain_10);
    display->drawString(x, y + 25, "CASARIA TECHNOLOGY INC.");
    display->drawString(x, y + 35, "www.casaria.net");
    }

    FrameCallback frames[] = { drawFrame1, drawFrame2, drawFrame3, drawFrame4 };

    int frameCount = 4;

  */

// Global message buffers shared by Serial and Scrolling functions
#define BUF_SIZE 75
char message[BUF_SIZE] = {"Hello!"};
bool newMessageAvailable = true;
float latitude, longitude;

unsigned long now2 = millis();
unsigned long lastTrigger = 0;
boolean startTimer = false;

static int count;
bool doorOpen;

// Global data
struct sCatalog
{
  textEffect_t effect; // text effect to display
  char *psz;           // text string nul terminated
  uint16_t speed;      // speed multiplier of library default
  uint16_t pause;      // pause multiplier for library default
};

sCatalog catalog[] =
    {
        {PA_PRINT, "WFS-VS", 1, 2},
        {PA_SCROLL_UP, "LEFT: ", 1, 1},
        {PA_SCROLL_DOWN_LEFT, C1, 1, 4},
        {PA_SCROLL_DOWN, "RIGHT: ", 1, 1},
        {PA_SCROLL_DOWN_RIGHT, C2, 1, 4},
        {PA_SCROLL_LEFT, "COOL ROOMS", 2, 1},
//{ PA_FADE, "time", 5, 4},
#if ENA_SPRITE
        {PA_SPRITE, "DATA", 2, 1},
        {PA_SCROLL_DOWN, shortTimeString, 1, 4},
#endif

        /*
      #if ENA_MISC
      { PA_SLICE, "SLICE", 1, 1 },
      { PA_MESH, "MESH", 20, 1 },
      { PA_FADE, "FADE", 20, 2 },
      { PA_DISSOLVE, "DSLVE", 7, 1 },
      { PA_BLINDS, "BLIND", 7, 1 },
      { PA_RANDOM, "RAND", 3, 1 },
      #endif
      #if ENA_WIPE
      { PA_WIPE, "WIPE", 5, 1 },
      { PA_WIPE_CURSOR, "WPE_C", 4, 1 },
      #endif
      #if ENA_SCAN
      { PA_SCAN_HORIZ, "SCNH", 4, 1 },
      { PA_SCAN_HORIZX, "SCNHX", 4, 1 },
      { PA_SCAN_VERT, "SCNV", 3, 1 },
      { PA_SCAN_VERTX, "SCNVX", 3, 1 },
      #endif
      #if ENA_OPNCLS
      { PA_OPENING, "OPEN", 3, 1 },
      { PA_OPENING_CURSOR, "OPN_C", 4, 1 },
      { PA_CLOSING, "CLOSE", 3, 1 },
      { PA_CLOSING_CURSOR, "CLS_C", 4, 1 },
      #endif
      #if ENA_SCR_DIA
      { PA_SCROLL_UP_LEFT, "SCR_UL", 7, 1 },
      { PA_SCROLL_UP_RIGHT, "SCR_UR", 7, 1 },
      { PA_SCROLL_DOWN_LEFT, "SCR_DL", 7, 1 },
      { PA_SCROLL_DOWN_RIGHT, "SCR_DR", 7, 1 },
      #endif
      #if ENA_GROW
      { PA_GROW_UP, "GRW_U", 7, 1 },
      { PA_GROW_DOWN, "GRW_D", 7, 1 },
      #endif
    */
};

// Sprite Definitions
const uint8_t F_PMAN1 = 6;
const uint8_t W_PMAN1 = 8;
static const uint8_t PROGMEM pacman1[F_PMAN1 * W_PMAN1] = // gobbling pacman animation
    {
        0x00,
        0x81,
        0xc3,
        0xe7,
        0xff,
        0x7e,
        0x7e,
        0x3c,
        0x00,
        0x42,
        0xe7,
        0xe7,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x24,
        0x66,
        0xe7,
        0xff,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x3c,
        0x7e,
        0xff,
        0xff,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x24,
        0x66,
        0xe7,
        0xff,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x00,
        0x42,
        0xe7,
        0xe7,
        0xff,
        0xff,
        0x7e,
        0x3c,
};

const uint8_t F_PMAN2 = 6;
const uint8_t W_PMAN2 = 18;
static const uint8_t PROGMEM pacman2[F_PMAN2 * W_PMAN2] = // ghost pursued by a pacman
    {
        0x00,
        0x81,
        0xc3,
        0xe7,
        0xff,
        0x7e,
        0x7e,
        0x3c,
        0x00,
        0x00,
        0x00,
        0xfe,
        0x7b,
        0xf3,
        0x7f,
        0xfb,
        0x73,
        0xfe,
        0x00,
        0x42,
        0xe7,
        0xe7,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x00,
        0x00,
        0x00,
        0xfe,
        0x7b,
        0xf3,
        0x7f,
        0xfb,
        0x73,
        0xfe,
        0x24,
        0x66,
        0xe7,
        0xff,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x00,
        0x00,
        0x00,
        0xfe,
        0x7b,
        0xf3,
        0x7f,
        0xfb,
        0x73,
        0xfe,
        0x3c,
        0x7e,
        0xff,
        0xff,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x00,
        0x00,
        0x00,
        0xfe,
        0x73,
        0xfb,
        0x7f,
        0xf3,
        0x7b,
        0xfe,
        0x24,
        0x66,
        0xe7,
        0xff,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x00,
        0x00,
        0x00,
        0xfe,
        0x73,
        0xfb,
        0x7f,
        0xf3,
        0x7b,
        0xfe,
        0x00,
        0x42,
        0xe7,
        0xe7,
        0xff,
        0xff,
        0x7e,
        0x3c,
        0x00,
        0x00,
        0x00,
        0xfe,
        0x73,
        0xfb,
        0x7f,
        0xf3,
        0x7b,
        0xfe,
};

void wifi_scan()
{
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  tft.drawString("Scan Network", tft.width() / 2, tft.height() / 2);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  int16_t n = WiFi.scanNetworks();
  tft.fillScreen(TFT_BLACK);
  if (n == 0)
  {
    tft.drawString("no networks found", tft.width() / 2, tft.height() / 2);
  }
  else
  {
    tft.setTextDatum(TL_DATUM);
    tft.setCursor(0, 0);
    Serial.printf("Found %d net\n", n);
    for (int i = 0; i < n; ++i)
    {
      sprintf(buff,
              "[%d]:%s(%d)",
              i + 1,
              WiFi.SSID(i).c_str(),
              WiFi.RSSI(i));
      tft.println(buff);
      ;
    }
  }
  WiFi.mode(WIFI_OFF);
}

//! Long time delay, it is recommended to use shallow sleep, which can effectively reduce the current consumption
void espDelay(int ms)
{
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  esp_light_sleep_start();
}

void showVoltage()
{
  static uint64_t timeStamp = 0;
  if (millis() - timeStamp > 1000)
  {
    timeStamp = millis();
    uint16_t v = analogRead(ADC_PIN);
    float battery_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0);
    String voltage = "Voltage :" + String(battery_voltage) + "V";
    Serial.println(voltage);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_RED);
    tft.setTextDatum(MC_DATUM);
    tft.drawString(voltage, tft.width() / 2, tft.height() / 2);
  }
}

void button_init()
{
  btn1.setLongClickHandler([](Button2 &b) {
    btnCick = false;
    int r = digitalRead(TFT_BL);
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_PURPLE, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.drawString("Press again to wake up", tft.width() / 2, tft.height() / 2);
    espDelay(6000);
    digitalWrite(TFT_BL, !r);

    tft.writecommand(TFT_DISPOFF);
    tft.writecommand(TFT_SLPIN);
    esp_sleep_enable_ext1_wakeup(GPIO_SEL_35, ESP_EXT1_WAKEUP_ALL_LOW);
    esp_deep_sleep_start();
  });
  btn1.setPressedHandler([](Button2 &b) {
    Serial.println("Detect Voltage..");
    btnCick = true;
  });

  btn2.setPressedHandler([](Button2 &b) {
    btnCick = false;
    Serial.println("btn press wifi scan");
    wifi_scan();
  });
}

void button_loop()
{
  btn1.loop();
  btn2.loop();
}

void reconnect()
{
  std::string clientId;

  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID

    /*    String clientId = "ESP32Client-";
          clientId += String(random(0xffff), HEX);
      */

    clientId.append(GROUP);
    clientId.append(UNIT);
    clientId.append(String(random(0xffff), HEX).c_str());
    // Attempt to connect
    if (client.connect(clientId.c_str(), mqttUser, mqttPassword))
    {
      Serial.println("connected");

      client.publish("WFS/STATUS", "MAX_ACTIVE");
      client.subscribe("WFS/SENSOR/TEMP1");
      client.subscribe("WFS/SENSOR/TEMP2");
      client.subscribe("WFS/SENSOR/HUM1");
      client.subscribe("WFS/SENSOR/HUM2");
      client.subscribe("WFS/CTRL");
      client.subscribe("DSM5701", 1);
      client.subscribe("DSM5701/rgb888Decimal", 1);
      client.subscribe("DSM5701/rgb565Decimal", 1);
      client.subscribe("DSM5701/rgb565Hex", 1);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      vTaskDelay(5000 / portTICK_RATE_MS);
    }
  }
}

// Checks if motion was detected, sets LED HIGH and starts a timer
void IRAM_ATTR StatusDecode()
{

  static bool StartCount = false;
  bool pinin;

  pinin = digitalRead(statusLED);
  if (!startTimer)
  {
    if (pinin)
    {
      startTimer = true;
      digitalWrite(relay1, HIGH);
    }
  }
  if (pinin)
    count++;
  lastTrigger = millis();
}

void IRAM_ATTR LockOn()
{
  doorOpen = false;
}

void IRAM_ATTR LockOff()
{
  doorOpen = true;
}

uint16_t rgb888torgb565(uint32_t rgb888Pixel)
{
  uint8_t red = rgb888Pixel >> 16 & 0xFF;
  uint8_t green = rgb888Pixel >> 8 & 0xFF;
  uint8_t blue = rgb888Pixel & 0xFF;

  uint16_t b = (blue >> 3) & 0x1f;
  uint16_t g = ((green >> 2) & 0x3f) << 5;
  uint16_t r = ((red >> 3) & 0x1f) << 11;

  return (uint16_t)(r | g | b);
}

void httpOTAquery()
{
  WiFiClient client;
  t_httpUpdate_return ret = httpUpdate.update(client, HTTP_OTA_ADDRESS, HTTP_OTA_PORT, HTTP_OTA_PATH, HTTP_OTA_VERSION);
  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILED Error (%d): %s\n", httpUpdate.getLastError(), httpUpdate.getLastErrorString().c_str());
    break;

  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;

  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  char p[length + 1];
  int i;
  memcpy(p, payload, length);
  p[length] = NULL;
  String message(p);
  topicStr = topic;

  Serial.print("Message arrived in topic: ");
  Serial.println(topic);

  Serial.print("Message:");
  for (i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }

  Serial.print((char)payload[i]);

  Serial.println();
  Serial.println("-----------------------");

  if (!strcmp(topic, "WFS/SENSOR/TEMP1"))
  {
    tempc1 = message.toFloat();
    TEMPC1 = message;
    memcpy(C1, payload, length);
    C1[length] = 'c';
    C1[length + 1] = NULL;
    Serial.printf("float Temp1 %3.2f  string: %s\n", tempc1, TEMPC1);
  }
  if (!strcmp(topic, "WFS/SENSOR/TEMP2"))
  {
    tempc2 = message.toFloat();
    TEMPC2 = message;
    memcpy(C2, payload, length);
    C2[length] = 'c';
    C2[length + 1] = NULL;
    Serial.printf("float Temp2 %3.2f  string: %s\n", tempc2, TEMPC2);
  }
  if (!strcmp(topic, "WFS/SENSOR/HUM1"))
  {
    hum1 = message.toFloat();
    szHUM1 = message;
    memcpy(HUM1, payload, length);
    HUM1[length] = '%';
    HUM1[length + 1] = 'r';
    HUM1[length + 2] = 'H';
    HUM1[length + 3] = NULL;
    Serial.printf("float humidity1 %3.2f  string: %s\n", hum1, szHUM1);
  }
  if (!strcmp(topic, "WFS/SENSOR/HUM2"))
  {
    hum2 = message.toFloat();
    szHUM2 = message;
    memcpy(HUM2, payload, length);
    HUM2[length] = '%';
    HUM2[length + 1] = 'r';
    HUM2[length + 2] = 'H';
    HUM2[length + 3] = NULL;
    Serial.printf("float humidity1 %3.2f  string: %s\n", hum2, szHUM2);
  }
  if (!strcmp(topic, "WFS/CTRL/DOOR1"))
  {
    tempc2 = message.toFloat();
    CMD = message;

    //memcpy(CMD, payload, length);

    CMD[length + 1] = NULL;
    Serial.printf("door CTRL CMD: %s", CMD);
  }

  Serial.println();
  Serial.println("-----------------------");

  if (topicStr.endsWith("DSM5701/rgb565Hex"))
  {

    //colourString = newColourString;
    rgb565Hex = message.toInt();
    Serial.println("Hex");

    Serial.println(rgb565Hex);
  }

  if (topicStr.endsWith("DSM5701/rgb888Decimal"))
  {

    rgb565Decimal = rgb888torgb565((uint32_t)message.toInt());
    Serial.println("*******");

    Serial.println(rgb565Decimal);
  }

  if (topicStr.endsWith("DSM5701"))
  {

    colourString = message;
    //sixteenBitHex = newSixteenBitHex;
    Serial.println(message);
  }

  if (topicStr.endsWith("DSM5701/rgb565Decimal"))
  {

    rgb565Decimal = message.toInt();
    Serial.println("*******");

    Serial.println(rgb565Decimal);
  }

  if (topicStr.endsWith("DSM5701/rgbHex"))
  {

    //colourString = newColourString;
    rgb565Hex = message; //.toInt();
    Serial.println("Hex");

    Serial.println(rgb565Hex);
  }

  if (topicStr.endsWith("DSM5701/PUSH-OTA"))
  {

    bPushTrigger = true;

    Serial.println(message);
  }

  if (message.startsWith("DSM5701/RED"))
  {
    AlertMode = amRed;
  }

  if (message.startsWith("DSM5701/GREEN"))
  {
    AlertMode = amGreen;
  }

  if (message.startsWith("DSM5701/GREEN"))
  {
    AlertMode = amIdle;
  }
}

void getJson(String url)
{

  if (WiFi.status() == WL_CONNECTED)
  {                            //Check WiFi connection status
    HTTPClient http;           //Declare an object of class HTTPClient
    http.begin(url);           //Specify request destination
    int httpCode = http.GET(); //Send the request
    if (httpCode > 0)
    {                             //Checkthe returning code
      payload = http.getString(); //Get the request response payload
    }

    http.end(); //Close connection
  }
}

void geolocation()
{

  String url,test5;
  url = "http://ip-api.com/json";
  getJson(url);

  const size_t capacity = JSON_OBJECT_SIZE(3) + JSON_ARRAY_SIZE(2) + 60;
  DynamicJsonBuffer jsonBuffer(capacity);
  JsonObject &root = jsonBuffer.parseObject(payload);
  root.printTo(Serial);
  if (!root.success())
  {
    Serial.println("parseObject() failed");
    return;
  }

  // Extract values
  Serial.println("Response:");
  Serial.println(root["lat"].as<char *>());
  Serial.println(root["lon"].as<char *>());
  test5 = (root["lon"].as<char *>());
  longitude = test5.toFloat();
  Serial.printf("Latitude  is : %s , %d   ", test5, longitude);


  coordinate = String(root["lat"].as<char *>()) + "," + String(root["lon"].as<char*>()); //do we use this anywhere (?)
  //coordinatePrev = "lat=" + String(root["lat"].as<char*>()) + "&lon=" + String(root["lon"].as<char*>()); //do we use this anywhere (?)
  coordinateTZ = "lat=" + String(root["lat"].as<char *>()) + "&lng=" + String(root["lon"].as<char *>());
  //mylat = root["lat"];
  //mylon = root["lon"];
  // M5.Lcd.println("My Lat/Lon: "+coordinate);
  Serial.println("My Lat/Lon: " + coordinate);

  Serial.printf("linux timestamp atY2K: %d %d %d", year(secs2000), month (secs2000), day(secs2000));
}

/*
      Used to get the GMT offset for your timezone
    */

void timeZone()
{
  String url;
  url = "http://api.timezonedb.com/v2/get-time-zone?key=" + TZAPIKEY + "&format=json&fields=gmtOffset,abbreviation&by=position&" + coordinateTZ;
  getJson(url);
  StaticJsonBuffer<512> jsonBuffer;
  JsonObject &root = jsonBuffer.parseObject(payload);
  if (!root.success())
  {
    Serial.println("parseObject() failed");
    return;
  }
  gmtOffset_sec = root["gmtOffset"];
  const char *temp1 = root["abbreviation"];
  TZone = (String)temp1;
  Serial.print("GMT Offset = ");
  Serial.print(gmtOffset_sec);
  Serial.println(" " + TZone);
  //M5.Lcd.println("GMT Offset = "+(String)gmtOffset_sec);
}

void auxTask(void *pvParameters)
{
  (void)pvParameters;
  while (1)
  {

    if (xSemaphoreTake(serialMutex, (TickType_t)10) == pdTRUE)
    {
      if (btnCick)
      {
        showVoltage();
      }
  
      button_loop();

      xSemaphoreGive(serialMutex);
      if (bPushTrigger) {
          esp_task_wdt_reset();
          Serial.print("Checking for firmware updates from server http://");
          Serial.print(HTTP_OTA_ADDRESS);
          Serial.print(":");
          Serial.print(HTTP_OTA_PORT);
          Serial.println(HTTP_OTA_PATH);
                
          httpOTAquery();
          bPushTrigger = false;
      }

      esp_task_wdt_reset();

    }

    esp_task_wdt_reset();
    yield();
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void updateMatrix(void *pvParameters)
{
  (void)pvParameters;
  static textPosition_t just = PA_LEFT;
  static uint8_t i = 0, j = 0;
  while (1)
  {
    // if (xSemaphoreTake (serialMutex, (TickType_t)10) == pdTRUE) {
    if (P.displayAnimate()) // animates and returns true when an animation is completed
    {
      // rotate the justification if needed

      if (i == ARRAY_SIZE(catalog))
      {
        /*
                          j++;
                          if (j == 3) j = 0;

                          switch (j)
                          {
                            case 0: just = PA_LEFT;    break;
                            case 1: just = PA_CENTER;  break;
                            case 2: just = PA_RIGHT;   break;
                          }
                          */
        i = 0; // reset loop index
      }

      just = PA_CENTER;
      // set up new animation
      P.displayText(catalog[i].psz, just, catalog[i].speed, catalog[i].pause, catalog[i].effect, catalog[i].effect);
      esp_task_wdt_reset();
      //vTaskDelay(e);  // wait a while to show the text ...
      vTaskDelay(catalog[i].pause / portTICK_PERIOD_MS);
      yield();
      i++;
      // ... then set up for next text effectset up for )next text effect
    }
    //       xSemaphoreGive (serialMutex);
    // }
    esp_task_wdt_reset();
  }
}

void updatePubSub(void *pvParameters)
{
  (void)pvParameters;
  while (1)
  {
    if (!client.connected())
    {

      reconnect();
    }
    client.loop();

    esp_task_wdt_reset();
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}

void updateNTP(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    if (xSemaphoreTake(serialMutex, (TickType_t)10) == pdTRUE)
    {
      while (!timeClient.update())
      {
        timeClient.forceUpdate();
      }
      setTime(timeClient.getEpochTime());
      xSemaphoreGive(serialMutex);
    }
    esp_task_wdt_reset();
    vTaskDelay((1000 / portTICK_PERIOD_MS) * 60 * 60); // update every hour
  }
}

void timeout_cb()
{
  // This sleep happened because of timeout. Do a restart after a sleep
  Serial.println(F("Watchdog timeout..."));

#ifdef DEEP_SLEEP_SECONDS
  // Enter DeepSleep so that we don't exhaust our batteries by countinuously trying to
  // connect to a network that isn't there.
  ESP.deepSleep(DEEP_SLEEP_SECONDS * 1000, WAKE_RF_DEFAULT);
  // Do nothing while we wait for sleep to overcome us
  while (true)
  {
  };

#else
  vTaskDelay(20000 / portTICK_PERIOD_MS);

  ESP.restart();
#endif
}

void updateScreen(void *pvParameters)
{
  (void)pvParameters;

  for (;;)
  {
    if (xSemaphoreTake(serialMutex, (TickType_t)10) == pdTRUE)
    {

      time_t t = now();
      sprintf(timeString, "%02i:%02i:%02i", hour(t), minute(t), second(t));
      sprintf(shortTimeString, "%02i:%02i", hour(t), minute(t));
      xSemaphoreGive(serialMutex);

      tft.setTextColor(0x39C4, TFT_BLACK);
      tft.drawString("88:88:88", 120, 25, 7);
      tft.setTextColor(rgb565Decimal, TFT_BLACK);
      tft.drawString(timeString, 120, 25, 7);

      // tft.setTextDatum(BC_DATUM);
      tft.setTextColor(TFT_ORANGE, TFT_BLACK);
      tft.setFreeFont(CF_RT24); //CF_OL24 CF_RT24  CF_Y32
      tft.drawString(colourString, 80, 110, GFXFF);
      tft.setFreeFont(CF_RT24);
    }
    esp_task_wdt_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup()
{

  Serial.begin(115200);
  Serial.println("Start");
  pinMode(ac1, INPUT_PULLUP);
  pinMode(ac2, INPUT_PULLUP);
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);

  pinMode(lockLatch, INPUT_PULLUP);
  pinMode(lockUnLatch, INPUT_PULLUP);
  pinMode(statusLED, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(statusLED), StatusDecode, CHANGE);
  attachInterrupt(digitalPinToInterrupt(lockLatch), LockOn, RISING);
  attachInterrupt(digitalPinToInterrupt(lockUnLatch), LockOff, RISING);

  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);

  serialMutex = xSemaphoreCreateMutex();

  if (TFT_BL > 0)
  {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
  }

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  tft.setTextColor(TFT_ORANGE, TFT_BLACK); // Note: the new fonts do not draw the background colour

  tft.fillScreen(TFT_BLACK);
  tft.setTextSize(2);
  tft.setTextColor(TFT_WHITE);
  tft.setCursor(0, 0);
  tft.setTextDatum(MC_DATUM);
  tft.setTextSize(1);

  Serial.print("TFT setup complete ");

  WiFi.begin(ssid, password);

  if (TFT_BL > 0)
  {                          // TFT_BL has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
    pinMode(TFT_BL, OUTPUT); // Set backlight pin to output mode
    //digitalWrite(TFT_BL, TFT_BACKLIGHT_ON); // Turn backlight on. TFT_BACKLIGHT_ON has been set in the TFT_eSPI library in the User Setup file TTGO_T_Display.h
  }

  tft.setSwapBytes(true);
  tft.pushImage(0, 0, 240, 135, ttgo);
  espDelay(3000);

  tft.setRotation(1);

  /*   while (i--) {
          tft.fillScreen(TFT_RED);
          espDelay(1000);
          tft.fillScreen(TFT_PURPLE);
          espDelay(1000);
          tft.fillScreen(TFT_ORANGE);
          espDelay(1000);
      }
    */
  //tft.fillScreen(TFT_BLACK);
  button_init();

  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize((adc_unit_t)ADC_UNIT_1, (adc_atten_t)ADC1_CHANNEL_6, (adc_bits_width_t)ADC_WIDTH_BIT_12, 1100, &adc_chars);
  //Check type of calibration value used to characterize ADC
  if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
  {
    Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
    vref = adc_chars.vref;
  }
  else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
  {
    Serial.printf("Two Point --> coeff_a:%umV coeff_b:%umV\n", adc_chars.coeff_a, adc_chars.coeff_b);
  }
  else
  {
    Serial.println("Default Vref: 1100mV");
  }

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_ORANGE); // Clear screen
  //tft.setTextDatum(BC_DATUM);

  tft.setFreeFont(CF_RT24); //CF_OL24 CF_RT2  CF_Y32

  tft.drawString("CASARIA", 80, 110, GFXFF); // Print the string name of the
  tft.setFreeFont(CF_RT24);
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  /*  while (!client.connected()) {
      Serial.println("Connecting to MQTT...");

      if (client.connect("DOOR2@WFS", mqttUser, mqttPassword )) {

      Serial.println("connected");

      } else {

      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);

      }
      }  */

  //Heltec.begin(true /*DisplayEnable Enable*/, false /*LoRa Disable*/, true /*Serial Enable*/);
  /*
      ui.setTargetFPS(30);

      // Customize the active and inactive symbol
      ui.setActiveSymbol(activeSymbol);
      ui.setInactiveSymbol(inactiveSymbol);

      // You can change this to
      // TOP, LEFT, BOTTOM, RIGHT
      ui.setIndicatorPosition(BOTTOM);

      // Defines where the first frame is located in the bar.
      ui.setIndicatorDirection(LEFT_RIGHT);

      // You can change the transition that is used
      // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
      ui.setFrameAnimation(SLIDE_LEFT);

      // Add frames
      ui.setFrames(frames, frameCount);

      // Initialising the UI will init the display too.
      ui.init();

      Heltec.display->flipScreenVertically();

    */

  geolocation(); //get latitude/longitude from external IP address (used to find Time zone)
  timeZone();    //used to get time zone

  timeClient.begin();
  timeClient.setTimeOffset(gmtOffset_sec); //timeClient.setTimeOffset(3600);

  tft.setTextSize(1);

  Serial.print("location amd timezone determined\n ");

  xTaskCreate(updateNTP, "NTP Client", 4096, NULL, 3, NULL);
  xTaskCreate(updateScreen, "Screen", 4096, NULL, 10, NULL);

  P.begin();
#if ENA_SPRITE
  P.setSpriteData(pacman1, W_PMAN1, F_PMAN1, pacman2, W_PMAN2, F_PMAN2);
#endif

  for (uint8_t i = 0; i < ARRAY_SIZE(catalog); i++)
  {
    catalog[i].speed *= P.getSpeed();
    catalog[i].pause *= 500;
  }

  P.setIntensity(MATRIX_INTENSITY);

#ifdef LED_STATUS_FLASH
  pinMode(STATUS_LED, OUTPUT);
  flasher.attach(0.6, flash);
#endif

// Set up WiFi connection
// Previous connection details stored in eeprom
#ifdef WIFI_PORTAL
#ifdef WIFI_PORTAL_TRIGGER_PIN
  pinMode(WIFI_PORTAL_TRIGGER_PIN, INPUT_PULLUP);
  delay(100);
  if (digitalRead(WIFI_PORTAL_TRIGGER_PIN) == LOW)
  {
    watchdog.detach();
    if (!wifiManager.startConfigPortal(SSID, NULL))
    {
      Serial.println(F("Config Portal Failed!"));
      timeout_cb();
    }
  }
  else
  {
#endif

    wifiManager.setConfigPortalTimeout(180);
    wifiManager.setAPCallback(configModeCallback);
    if (!wifiManager.autoConnect())
    {
      Serial.println(F("Connection Failed!"));
      timeout_cb();
    }

#ifdef WIFI_PORTAL_TRIGGER_PIN
  }
#endif

#else
  // Save boot up time by not configuring them if they haven't changed

#endif

  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println(F("Connection Failed!"));
    timeout_cb();
  }

  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());

#ifdef LED_STATUS_FLASH
  flasher.detach();
  digitalWrite(STATUS_LED, HIGH);
#endif

#ifdef HTTP_OTA
  // Check server for firmware updates
  Serial.print("Checking for firmware updates from server http://");
  Serial.print(HTTP_OTA_ADDRESS);
  Serial.print(":");
  Serial.print(HTTP_OTA_PORT);
  Serial.println(HTTP_OTA_PATH);

  httpOTAquery();

#endif

#ifdef ARDUINO_OTA
  // Arduino OTA Initalisation
  ArduinoOTA.setPort(ARDUINO_OTA_PORT);
  ArduinoOTA.setHostname(SSID);
  ArduinoOTA.setPassword(ARDUINO_OTA_PASSWD);
  ArduinoOTA.onStart([]() {
    watchdog.detach();
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
      Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)
      Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR)
      Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR)
      Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)
      Serial.println("End Failed");
  });
  ArduinoOTA.begin();
#endif

  Serial.print("Starting  3 more RTOS tasks\n ");

  xTaskCreate(updatePubSub, "PubSub", 4096, NULL, 3, NULL);
  xTaskCreate(updateMatrix, "MaxMatrix", 4096, NULL, 4, NULL);
  xTaskCreate(auxTask, "auxTask", 4086, NULL, 2, NULL);

  Serial.printf("RTOS tasks running %d \n ", now());
}

astroSunTimes getSunRiseSunSetTime(float latitude, float longitude, float elevation, long unixtime)
{
  int ayear, amon, aday;
  ayear = year(unixtime);
  amon = month(unixtime);
  aday = day(unixtime);   

  long JDN = 367 * ayear - (7 * (ayear + 5001 + (amon - 9) / 7)) / 4 + (275 * amon) / 9 + aday + 1729777;
  float n = JDN * 2451545.0 + 0.0008;
  float J = n - (longitude) / 360;                     // mean solar TIME
  float M = fmodf((357.5291 + 0.98560028 * J), 360.0); //solar Anamoly  aoiBut c that that that that
  float C = 1.9148 * sin(M) + 0.0200 * sin(2 * M) + 0.0003 * sin(3 * M);
  float LAMBDA = fmodf((M + C + 180 + 102.9372), 360.0);
  float sin_DELTA = sin(LAMBDA) * sin(23.44);
  float OMEGA_ZER0 = (sin(-.083 - 2.076 * sqrt(elevation) / 60) - sin(latitude) * sin_DELTA) /
                     (cos(latitude) * cos(asin(sin_DELTA)));
  float Jtransit = 2451545.0 + J + 0.0053 * sin(M) - 0.0069 * sin(2 * LAMBDA);


  Serial.printf("JDN: %d, n: %f12.6 /n", JDN, n);
  
  float sunrise = Jtransit - (OMEGA_ZER0 / 360) - JDN;
  float sunset = Jtransit + (OMEGA_ZER0 / 360) - JDN;
  astroSunTimes suntimes;

  suntimes.sunRise = sunrise;
  suntimes.sunSet = sunset;
  return suntimes;
}

void loop()
{
  esp_task_wdt_reset();
  vTaskDelay(1);

  //}
}
