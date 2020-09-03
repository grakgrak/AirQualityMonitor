#include <Arduino.h>
#include <ArduinoOTA.h> // https://github.com/esp8266/Arduino/blob/master/libraries/ArduinoOTA/ArduinoOTA.h
#include <AsyncMqttClient.h>
#include <Ticker.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>
#include "..\..\Credentials.h" // contains definitions of WIFI_SSID and WIFI_PASSWORD
#include "PMS.h"

#define HOSTNAME "AirQualityMonitor"
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASSWORD;

#define WIFI_CONNECT_TIMEOUT 15000
#define MQTT_HOST IPAddress(192, 168, 1, 210)
#define MQTT_PORT 1883
#define MQTT_PUBLISH_SECS 5
#define PMS_SLEEP_SECS 300

#define RX_PIN 5
#define TX_PIN 4

SoftwareSerial swSerial;
AsyncMqttClient mqttClient;
WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker mqttReconnectTimer;
Ticker wifiReconnectTimer;
Ticker mqttPublishTimer;
Ticker pmsSleepTimer;

PMS pms(swSerial);
PMS::DATA data;
bool pmsAwake = true;
unsigned long pmsAwakeAt = 0;

//--------------------------------------------------------------------
class ArithmeticMean
{
private:
    const double ALPHA = 0.35;
    double _average = 0.0;
public:
    void Update(double val)
    {
        _average = ALPHA * val + (1.0 - ALPHA) * _average;
    }
    double Average() { return _average; }
};

//--------------------------------------------------------------------
ArithmeticMean PM10, PM25, PM100;
ArithmeticMean PPD03, PPD05, PPD10;

//------------------------------------------------------------------------
unsigned long AwakeForMillis()
{
    return pmsAwake ? millis() - pmsAwakeAt : 0;
}
//------------------------------------------------------------------------
void SleepPMS()
{
    Serial.println("Sleeping.");

    pmsAwake = false;
    pms.sleep();

    // sleep between recording events
    pmsSleepTimer.once(PMS_SLEEP_SECS, []() {
        pms.wakeUp();
        pmsAwake = true;
        pmsAwakeAt = millis();
        Serial.println("Awake.");
    });
}

//--------------------------------------------------------------------
void init_OTA()
{
    Serial.println("init OTA");

    // ArduinoOTA callback functions
    ArduinoOTA.onStart([]() {
        Serial.println("OTA starting...");
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("OTA done.Reboot...");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned int prevPcnt = 100;
        unsigned int pcnt = (progress / (total / 100));
        unsigned int roundPcnt = 5 * (int)(pcnt / 5);
        if (roundPcnt != prevPcnt)
        {
            prevPcnt = roundPcnt;
            Serial.println("OTA upload " + String(roundPcnt) + "%");
        }
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.print("OTA Error " + String(error) + ":");
        const char *line2 = "";
        switch (error)
        {
        case OTA_AUTH_ERROR:
            line2 = "Auth Failed";
            break;
        case OTA_BEGIN_ERROR:
            line2 = "Begin Failed";
            break;
        case OTA_CONNECT_ERROR:
            line2 = "Connect Failed";
            break;
        case OTA_RECEIVE_ERROR:
            line2 = "Receive Failed";
            break;
        case OTA_END_ERROR:
            line2 = "End Failed";
            break;
        }
        Serial.println(line2);
    });

    ArduinoOTA.setPort(8266);
    ArduinoOTA.setHostname(HOSTNAME);
    ArduinoOTA.setPassword(HOSTNAME);

    ArduinoOTA.begin();
}

//------------------------------------------------------------------------
void connectToWifi()
{
    Serial.println("Connecting to Wi-Fi...");
    WiFi.enableSTA(true);
    WiFi.begin(ssid, password);
}

//------------------------------------------------------------------------
void connectToMqtt()
{
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

//------------------------------------------------------------------------
void publishSensorValue()
{
    // only publish after running for 30 seconds
    if (AwakeForMillis() > 30000ul)
    {
        mqttClient.publish("AirQuality/pm10", 0, true, String(PM10.Average()).c_str());
        mqttClient.publish("AirQuality/pm25", 0, true, String(PM25.Average()).c_str());
        mqttClient.publish("AirQuality/pm100", 0, true, String(PM100.Average()).c_str());

        mqttClient.publish("AirQuality/ppd03", 0, true, String(PPD03.Average()).c_str());
        mqttClient.publish("AirQuality/ppd05", 0, true, String(PPD05.Average()).c_str());
        mqttClient.publish("AirQuality/ppd10", 0, true, String(PPD10.Average()).c_str());

        Serial.println("Published Readings.");
        SleepPMS(); // send the PMS back to sleep now that we have sent the values
    }
}

//------------------------------------------------------------------------
void onMqttConnect(bool sessionPresent)
{
    mqttPublishTimer.attach(MQTT_PUBLISH_SECS, publishSensorValue);

    Serial.println("Connected to MQTT.");
}

//------------------------------------------------------------------------
void onMqttDisconnect(AsyncMqttClientDisconnectReason reason)
{
    Serial.println("Disconnected from MQTT.");

    mqttPublishTimer.detach();

    if (WiFi.isConnected())
        mqttReconnectTimer.once(2, connectToMqtt);
}

//------------------------------------------------------------------------
void init_mqtt()
{
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);

    connectToWifi();
}
//------------------------------------------------------------------------
void WiFiEvent(WiFiEvent_t event)
{
    switch (event)
    {
    case WIFI_EVENT_STAMODE_GOT_IP:
        connectToMqtt();
        break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
        mqttReconnectTimer.detach();
        wifiReconnectTimer.once(2, connectToWifi);
        break;
    default:
        break;
    }
}

//------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    delay(10);
    //Serial.setDebugOutput(true);
    Serial.println("\nSetup");

    WiFi.onEvent(WiFiEvent);

    init_OTA();
    init_mqtt();

    swSerial.begin(9600, SWSERIAL_8N1, RX_PIN, TX_PIN);

    pms.activeMode();
    pms.wakeUp();

    Serial.println("Setup Done.");
}

//------------------------------------------------------------------------
void loop()
{
    // read the sensor
    if (AwakeForMillis() > 0 && pms.read(data))
    {
        // Serial.print("Awake for : ");
        // Serial.println(AwakeForMillis());

        // Serial.println(data.PM_SP_UG_1_0);
        // Serial.println(data.PM_SP_UG_2_5);
        // Serial.println(data.PM_SP_UG_10_0);
        // Serial.println(data.PM_AE_UG_1_0);
        // Serial.println(data.PM_AE_UG_2_5);
        // Serial.println(data.PM_AE_UG_10_0);

        // Serial.println(data.PPDL_0_3);
        // Serial.println(data.PPDL_0_5);
        // Serial.println(data.PPDL_1_0);
        // Serial.println(data.PPDL_2_5);
        // Serial.println(data.PPDL_5_0);
        // Serial.println(data.PPDL_10_0);

        // Serial.println();

        if (AwakeForMillis() > 25000ul)
        {
            Serial.print(".");

            PM10.Update(data.PM_AE_UG_1_0);
            PM25.Update(data.PM_AE_UG_2_5);
            PM100.Update(data.PM_AE_UG_10_0);
            PPD03.Update(data.PPDL_0_3);
            PPD05.Update(data.PPDL_0_5);
            PPD10.Update(data.PPDL_10_0);
        }
    }

    // go to sleep if been awake for longer than 60 secs
    if (AwakeForMillis() > 60000ul)
        SleepPMS();

    ArduinoOTA.handle();

    delay(10);
}