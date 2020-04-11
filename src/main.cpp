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
#define PMS_SLEEP_SECS 120

#define ALPHA 0.35

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
    double _alpha;
    double _average = 0.0;
public:
    ArithmeticMean(double alpha) { _alpha = alpha; }

    void Update(double val)
    {
        _average = _alpha * val + (1.0 - _alpha) * _average;
    }
    double Average() { return _average; }
};

//--------------------------------------------------------------------
ArithmeticMean PM10(ALPHA);
ArithmeticMean PM25(ALPHA);
ArithmeticMean PM100(ALPHA);
ArithmeticMean PPD03(ALPHA);
ArithmeticMean PPD05(ALPHA);
ArithmeticMean PPD10(ALPHA);

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

    // sleep for 120 secs
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

    swSerial.begin(9600, SWSERIAL_8N1, 5, 4);

    pms.activeMode();
    pms.wakeUp();

    Serial.println("Setup Done.");
}


//------------------------------------------------------------------------
// bool pmsA003ReadData()
// {

//     // while (swSerial.read()!=-1) {}; //clear buffer

//     if (swSerial.available() < 32)
//     {
//         if (swSerial.available() == 0)
//         {
//             delay(150);
//             return 0;
//         };
//         if (swSerial.available() > 16)
//         {
//             delay(10);
//             return 0;
//         };
//         if (swSerial.available() > 0)
//         {
//             delay(30);
//             return 0;
//         };
//         delay(100);
//         return 0;
//     }
//     if (swSerial.read() != 0x42)
//         return 0;
//     if (swSerial.read() != 0x4D)
//         return 0;

//     inputChecksum = 0x42 + 0x4D;

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     if (inputHigh != 0x00)
//         return 0;
//     if (inputLow != 0x1c)
//         return 0;

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     concPM1_0_CF1 = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     concPM2_5_CF1 = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     concPM10_0_CF1 = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     concPM1_0_amb = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     concPM2_5_amb = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     concPM10_0_amb = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     rawGt0_3um = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     rawGt0_5um = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     rawGt1_0um = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     rawGt2_5um = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     rawGt5_0um = inputLow + (inputHigh << 8);

//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     inputChecksum += inputHigh + inputLow;
//     rawGt10_0um = inputLow + (inputHigh << 8);

//     inputLow = swSerial.read();
//     inputChecksum += inputLow;
//     version = inputLow;

//     inputLow = swSerial.read();
//     inputChecksum += inputLow;
//     errorCode = inputLow;
//     /*
//  Serial.print("PMSA003;\t");
//  Serial.print(concPM1_0_CF1);
//  Serial.print(';');
//  Serial.print(concPM2_5_CF1);
//  Serial.print(';');
//  Serial.print(concPM10_0_CF1);
//  Serial.print(";\t");
//  Serial.print(concPM1_0_amb);
//  Serial.print(';');
//  Serial.print(concPM2_5_amb);
//  Serial.print(';');
//  Serial.print(concPM10_0_amb);
//  Serial.print(";\t");
//  Serial.print(rawGt0_3um);
//  Serial.print(';');
//  Serial.print(rawGt0_5um);
//  Serial.print(';');
//  Serial.print(rawGt1_0um);
//  Serial.print(';');
//  Serial.print(rawGt2_5um);
//  Serial.print(';');
//  Serial.print(rawGt5_0um);
//  Serial.print(';');
//  Serial.print(rawGt10_0um);
//  Serial.print(';');
//  Serial.print(version);
//  Serial.print(';');
//  Serial.print(errorCode);
//  Serial.println();
//  */
//     inputHigh = swSerial.read();
//     inputLow = swSerial.read();
//     checksum = inputLow + (inputHigh << 8);
//     if (checksum != inputChecksum)
//     {
//         /*
//  Serial.print(';');
//  Serial.print(checksum);
//  Serial.print(';');
//  Serial.print(inputChecksum);
//  Serial.println();
//  */
//         return 0;
//     }

//     // only update the averages after 20 seconds
//     if (millis() > 20000ul)
//     {
//         PM10.Update(concPM1_0_amb);
//         PM25.Update(concPM2_5_amb);
//         PM100.Update(concPM10_0_amb);
//         PPD03.Update(rawGt0_3um);
//         PPD05.Update(rawGt0_5um);
//         PPD10.Update(rawGt1_0um);
//     }

//     delay(700); // higher will get you checksum errors

//     return 1;
// }

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