#include <Arduino.h>

#include "esp_system.h"  //Used for WDT timer
#include "WiFi.h"
// #include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <ArduinoOTA.h>
extern "C" {
	#include "freertos/FreeRTOS.h"
	#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
// #include <HardwareSerial.h>
#include "credentials.h"
#include "FS.h"
#include "SPIFFS.h" //A way to store configuration with some memory protection

// HardwareSerial HWSerial1(1); //Pins 16, 17
// HardwareSerial HWSerial2(2); //Pins 4,15

//****************** FUNCTION PROTOTYPES ******************
void initOTA(void);




//****************** MQTT CALLBACK FUNCTION ******************
void msgRecieved(char* topic, byte* payload, unsigned int length)
{
    if(strcmp(topic, SOME_SUB_TOPIC) == 0) {
        Serial.println("Matched topic!");
    }
}


//****************** INSTANCES ******************
WiFiClient espClient;
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
// WiFiServer server(80);



WiFiUDP ntpUDP;
// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionaly you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
#define TIMEZONE 10  //Australia Brisbane GMT+10
NTPClient timeClient(ntpUDP, "au.pool.ntp.org", 3600*TIMEZONE, 60000);

// uint8_t connectionRetries;
// #define CONNECTION_RETRY_TIME 300000

bool watchdog_active = false;
#define WDT_TIMEOUT 3000  //time in ms to trigger the watchdog
hw_timer_t *watchDog_Timer = NULL;

#include "fileSystem.esp"
#include "connection.esp"

//*******************************SETUP**********************************
void setup()
{

    // WiFi.mode(WIFI_OFF);  //Allow cap to charge

    Serial.begin(115200);
    // HWSerial1.begin(9600, SERIAL_8N1, 16, 17); //Pins 16, 17
    // HWSerial2.begin(9600, SERIAL_8N1, 4, 15);  //Pins 4, 15

    // attachInterrupt(SOME_PIN, ISR_FUNCTION, RISING);

    initWifi();
    initSpiffs();
    enableWDT();  //This is the core 1 watchdog timer, it's feed in houseKeeping.
}

void loop()
{
    houseKeeping();
}
