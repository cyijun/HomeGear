/**
 *  @file HomeGear.ino
 *  @author Chen Yijun
 *  @brief
 *  @version 1.0
 *  @date 2022-02-23
 *
 *  Copyright 2022 Chen Yijun
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include <Arduino.h>
#include <assert.h>
#include <Wire.h>
#include <FS.h>
#include <WiFiManager.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "HAMDHandler.h"
#include "EspMQTTClient.h"

//***** Config *****

#define PLATFORM "HomeGear"
#define HOMEGEAR_SERIAL_NUMBER chipId
#define MANUFACTURER "Chen Yijun"
#define DEVICE_MODEL "HomeGear Scalable Hardware Stack"
#define SOFTWARE_VERSION "HomeGear 1.0"

#define THI_MODULE true
#define IAQ_MODULE false
#define IR_MODULE true
#define RF_MODULE true
#define ZGB_MODULE true

#define SERIAL_DEBUG false

#define IR_GREE_AC false
#define IR_SINGFUN_FAN true
#define RF_PHILIPS_FAN_LIGHT true

//***** Basic *****

#define ESP_getChipId() ((uint32_t)ESP.getEfuseMac())
const String chipId = String(ESP_getChipId(), HEX);
char nodename[32];

unsigned int timeDrivenPubInterval = 300;
unsigned long lastPubMs;
unsigned long startLoopMs;

void dataShift(float data[2])
{
	data[0] = data[1];
}

String moduleList()
{
	String modList = " / ";
#if THI_MODULE
	modList += "Temperature-Humidity-Illuminance Module / ";
#endif
#if IAQ_MODULE
	modList += "IAQ Module / ";
#endif
#if IR_MODULE
	modList += "IR Module / ";
#endif
#if RF_MODULE
	modList += "RF Module / ";
#endif
#if ZGB_MODULE
	modList += "ZigBee Module / ";
#endif
	return modList;
}

//***** Modules *****

HAMqttDiscoveryHandler hamdhHG(PLATFORM, HOMEGEAR_SERIAL_NUMBER, MANUFACTURER, DEVICE_MODEL, SOFTWARE_VERSION);
const char *avt = hamdhHG.getAvailabilityTopic();

#if THI_MODULE
bool thimEvent = false;
bool thimHasNewData = false;
unsigned long lastReadTHIMs = 0;
// SHTC3
#include "SparkFun_SHTC3.h"
SHTC3 shtc3;

// BH1750*2
#include <BH1750.h>
BH1750 bh1750_1;
BH1750 bh1750_2;

float thim_temperature[2] = {-100, 1};
float thim_humidity[2] = {-100, 1};
float thim_illuminance[2] = {-100, 1};
#endif

#if IAQ_MODULE // BME680
bool bme680HasNewData = false;
bool iaqIniting = true;
bool iaqmEvent = false;
#include "bsec.h"
Bsec bme680;

// use 2 elements arrays to calculate the differences of new data
float iaqm_temperature[2] = {-100, 1};
float iaqm_humidity[2] = {-100, 1};
float iaqm_pressure[2];
float iaqm_iaq[2] = {-100, 1};
float iaqm_co2Equivalent[2];
float iaqm_breathVocEquivalent[2];
#endif

#if IR_MODULE
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include "IR_Code.h"
const uint16_t kIrLed = 4;
IRsend irsend(kIrLed);

#if IR_SINGFUN_FAN
HAMqttDiscoveryHandler hamdhSF("homegear_ir", "00001", "SingFun", "SingFun Floor Fan", SOFTWARE_VERSION, hamdhHG);
#endif

#if IR_GREE_AC
#include <ir_Gree.h>
HAMqttDiscoveryHandler hamdhGree("homegear_ir", "ac001", "Gree", "Gree AC", SOFTWARE_VERSION, hamdhHG);
IRGreeAC greeAC(kIrLed);
#endif
#endif

#if RF_MODULE

HardwareSerial rfSerial(1);

#if RF_PHILIPS_FAN_LIGHT
HAMqttDiscoveryHandler hamdhPFL("homegear_rf", "00001", "Philips", "Philips RF Remote Fan Light", SOFTWARE_VERSION, hamdhHG);
#endif

#include "RF_Code.h"
void sendRfSig(const unsigned char data[6])
{
	rfSerial.write(rf_module_uart_protocol[0]);
	for (int i = 0; i < 6; i++)
	{
		rfSerial.write(data[i]);
	}
	rfSerial.write(rf_module_uart_protocol[1]);
}
#endif

#if ZGB_MODULE

#include <WiFiClient.h>
#include <WiFiServer.h>
#define ZGB_BAUD_RATE 115200
#define TCP_LISTEN_PORT 1234
// serial and ethernet buffer size
#define BUFFER_SIZE 256
WiFiServer ser2NetServer(TCP_LISTEN_PORT);
WiFiClient wClient;

HardwareSerial zgbSerial(2);
#endif

//***** network communication *****

// MQTT
EspMQTTClient mqttClient;

// define your default values here, if there are different values in config.json, they are overwritten.
#define SERVER_ADDR_LEN 64
#define NW_PORT_LEN 6
#define USER_PWD_LEN 32
#define TOKEN_LEN 128
#define FLAG_LEN 6

// wm_config_params
char mqtt_server[SERVER_ADDR_LEN] = "";
char mqtt_port[NW_PORT_LEN] = "1883";
char mqtt_user[USER_PWD_LEN] = "";
char mqtt_pwd[USER_PWD_LEN] = "";

char homeassistant_server[SERVER_ADDR_LEN] = "";
char homeassistant_port[NW_PORT_LEN] = "8123";
char homeassistant_token[128] = "";

// flag for saving data
bool shouldSaveConfig = false;

// callback notifying us of the need to save config
void saveConfigCallback()
{
	Serial.println("Should save config");
	shouldSaveConfig = true;
}

void loadConfigFromFS()
{
	// clean FS, for testing
	// SPIFFS.format();

	// read configuration from FS json
	Serial.println("mounting FS...");

	if (SPIFFS.begin())
	{
		Serial.println("mounted file system");
		if (SPIFFS.exists("/config.json"))
		{
			// file exists, reading and loading
			Serial.println("reading config file");
			File configFile = SPIFFS.open("/config.json", "r");
			if (configFile)
			{
				Serial.println("opened config file");
				size_t size = configFile.size();
				// Allocate a buffer to store contents of the file.
				std::unique_ptr<char[]> buf(new char[size]);

				configFile.readBytes(buf.get(), size);

				DynamicJsonDocument json(1024);
				auto deserializeError = deserializeJson(json, buf.get());
				serializeJson(json, Serial);
				if (!deserializeError)
				{
					// glob_copy_to_json
					strcpy(mqtt_server, json["mqtt_server"]);
					strcpy(mqtt_port, json["mqtt_port"]);
					strcpy(mqtt_user, json["mqtt_user"]);
					strcpy(mqtt_pwd, json["mqtt_pwd"]);
					strcpy(homeassistant_server, json["homeassistant_server"]);
					strcpy(homeassistant_port, json["homeassistant_port"]);
					strcpy(homeassistant_token, json["homeassistant_token"]);
				}
				else
				{
					Serial.println("failed to load json config");
				}
				configFile.close();
			}
		}
	}
	else
	{
		Serial.println("failed to mount FS, formating");
		SPIFFS.format();
	}
	// end read
}

void saveConfigToFS()
{
	// save the custom parameters to FS
	if (shouldSaveConfig)
	{
		Serial.println("saving config");
		DynamicJsonDocument json(2048);
		json["mqtt_server"] = mqtt_server;
		json["mqtt_port"] = mqtt_port;
		json["mqtt_user"] = mqtt_user;
		json["mqtt_pwd"] = mqtt_pwd;
		json["homeassistant_server"] = homeassistant_server;
		json["homeassistant_port"] = homeassistant_port;
		json["homeassistant_token"] = homeassistant_token;

		File configFile = SPIFFS.open("/config.json", "w");
		if (!configFile)
		{
			Serial.println("failed to open config file for writing");
		}

		serializeJson(json, configFile);
		configFile.close();
		// end save
	}
}

void wmStart()
{
	// The extra parameters to be configured (can be either global or just in the setup)
	// After connecting, parameter.getValue() will get you the configured value
	// id/name placeholder/prompt default length
	WiFiManagerParameter custom_mqtt_server("mqttserver", "MQTT Server", mqtt_server, SERVER_ADDR_LEN);
	WiFiManagerParameter custom_mqtt_port("mqttport", "MQTT Port", mqtt_port, NW_PORT_LEN);
	WiFiManagerParameter custom_mqtt_user("mqttuser", "MQTT Username", mqtt_user, USER_PWD_LEN);
	WiFiManagerParameter custom_mqtt_pwd("mqttpwd", "MQTT Password", mqtt_pwd, USER_PWD_LEN);
	WiFiManagerParameter custom_homeassistant_server("homeassistantserver", "Home Assistant Server", homeassistant_server, SERVER_ADDR_LEN);
	WiFiManagerParameter custom_homeassistant_port("homeassistantport", "Home Assistant Port", homeassistant_port, NW_PORT_LEN);
	WiFiManagerParameter custom_homeassistant_token("homeassistanttoken", "Home Assistant Token", homeassistant_token, 128);

	// WiFiManager
	// Local intialization. Once its business is done, there is no need to keep it around
	WiFiManager wifiManager;

	// set config save notify callback
	wifiManager.setSaveConfigCallback(saveConfigCallback);

	// add all your parameters here
	wifiManager.addParameter(&custom_mqtt_server);
	wifiManager.addParameter(&custom_mqtt_port);
	wifiManager.addParameter(&custom_mqtt_user);
	wifiManager.addParameter(&custom_mqtt_pwd);
	wifiManager.addParameter(&custom_homeassistant_server);
	wifiManager.addParameter(&custom_homeassistant_port);
	wifiManager.addParameter(&custom_homeassistant_token);

	// reset settings - for testing
	// wifiManager.resetSettings();

	if (!wifiManager.autoConnect(nodename, "homegearconfig"))
	{
		Serial.println("failed to connect and hit timeout");
		delay(3000);
		// reset and try again, or maybe put it to deep sleep
		ESP.restart();
		delay(5000);
	}

	// if you get here you have connected to the WiFi
	Serial.println("connected...yeey :)");

	// read updated parameters
	strcpy(mqtt_server, custom_mqtt_server.getValue());
	strcpy(mqtt_port, custom_mqtt_port.getValue());
	strcpy(mqtt_user, custom_mqtt_user.getValue());
	strcpy(mqtt_pwd, custom_mqtt_pwd.getValue());
	strcpy(homeassistant_server, custom_homeassistant_server.getValue());
	strcpy(homeassistant_port, custom_homeassistant_port.getValue());
	strcpy(homeassistant_token, custom_homeassistant_token.getValue());
}

void mqttStart()
{
	mqttClient.setMaxPacketSize(2048);
#if SERIAL_DEBUG
	mqttClient.enableDebuggingMessages();
#endif
	mqttClient.enableHTTPWebUpdater("admin", "homegearota", "/ota");
	mqttClient.enableLastWillMessage(avt, "offline", true);
	// Set the WiFi and MQTT information that we loaded from global param
	mqttClient.setMqttClientName(nodename);
	mqttClient.setMqttServer(mqtt_server, mqtt_user, mqtt_pwd, atoi(mqtt_port));
}

void modulesStart()
{
#if (THI_MODULE || IAQ_MODULE)
	Wire.begin();
#endif

#if THI_MODULE
	bh1750_1.begin(BH1750::ONE_TIME_HIGH_RES_MODE, 0x23);
	bh1750_2.begin(BH1750::ONE_TIME_HIGH_RES_MODE, 0x5C);
	shtc3.begin();
#endif

#if IAQ_MODULE
	bme680.begin(BME680_I2C_ADDR_PRIMARY, Wire);
	bsec_virtual_sensor_t bme680SenseList[6] = {
		BSEC_OUTPUT_RAW_PRESSURE,
		BSEC_OUTPUT_STATIC_IAQ,
		BSEC_OUTPUT_CO2_EQUIVALENT,
		BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
	};

	bme680.updateSubscription(bme680SenseList, 6, BSEC_SAMPLE_RATE_LP);
#endif

#if IR_MODULE
	irsend.begin();
#if IR_GREE_AC
	greeAC.begin();
	greeAC.off();
	greeAC.setFan(1);
	// kGreeAuto, kGreeDry, kGreeCool, kGreeFan, kGreeHeat
	greeAC.setMode(kGreeCool);
	greeAC.setTemp(26); // 16-30C
	greeAC.setSwingVertical(false, kGreeSwingAuto);
	greeAC.setSwingHorizontal(false);
	greeAC.setXFan(false);
	greeAC.setLight(false);
	greeAC.setSleep(false);
	greeAC.setTurbo(false);
#endif
#endif

#if RF_MODULE
	rfSerial.begin(9600, SERIAL_8N1, 13, 12);
#endif

#if ZGB_MODULE
	zgbSerial.begin(115200);
	ser2NetServer.begin();
#endif
}

void setup()
{
	// device name
	Serial.begin(115200);
	Serial.println();

	String nodenameStr = "HomeGear_" + chipId;
	nodenameStr.toUpperCase();
	strcpy(nodename, nodenameStr.c_str());

	loadConfigFromFS();
	wmStart();
	saveConfigToFS();
	mqttStart();
	modulesStart();

	startLoopMs = millis();
	lastPubMs = millis();
}

void onConnectionEstablished()
{
	// Publish online message to availabilityTopic
	mqttClient.publish(avt, "online", true);

	// Subscribe to cmdTopic
	mqttClient.subscribe(hamdhHG.getCmdTopic(), [](const String &payload)
						 {
    if (payload.equals("restore"))
    {
      SPIFFS.format();
      WiFi.disconnect(true);
      ESP.restart();
      delay(1000);
    }
    if (payload.equals("reboot"))
    {
      ESP.restart();
      delay(1000);
    } });

	// publish HA MQTT Discovery messages
#if (THI_MODULE || IAQ_MODULE)
	HAMqttDiscoverySensor hamdhSensor(hamdhHG);
#endif

#if THI_MODULE
	hamdhSensor.setDeviceClass("temperature");
	hamdhSensor.setDeviceNameMin("thim_temperature");
	hamdhSensor.setStateClass("measurement");
	hamdhSensor.setUnitOfMeasurement("°C");
	hamdhSensor.setValueTemplate("{{ value_json.thim_temperature }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

	hamdhSensor.setDeviceClass("humidity");
	hamdhSensor.setDeviceNameMin("thim_humidity");
	hamdhSensor.setUnitOfMeasurement("%");
	hamdhSensor.setValueTemplate("{{ value_json.thim_humidity }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

	hamdhSensor.setDeviceClass("illuminance");
	hamdhSensor.setDeviceNameMin("thim_illuminance");
	hamdhSensor.setUnitOfMeasurement("lx");
	hamdhSensor.setValueTemplate("{{ value_json.thim_illuminance }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);
#endif

#if IAQ_MODULE
	hamdhSensor.setDeviceClass("temperature");
	hamdhSensor.setDeviceNameMin("iaqm_temperature");
	hamdhSensor.setStateClass("measurement");
	hamdhSensor.setUnitOfMeasurement("°C");
	hamdhSensor.setValueTemplate("{{ value_json.iaqm_temperature }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

	hamdhSensor.setDeviceClass("humidity");
	hamdhSensor.setDeviceNameMin("iaqm_humidity");
	hamdhSensor.setUnitOfMeasurement("%");
	hamdhSensor.setValueTemplate("{{ value_json.iaqm_humidity }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

	hamdhSensor.setDeviceClass("pressure");
	hamdhSensor.setDeviceNameMin("iaqm_pressure");
	hamdhSensor.setUnitOfMeasurement("Pa");
	hamdhSensor.setValueTemplate("{{ value_json.iaqm_pressure }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

	hamdhSensor.setDeviceClass("aqi");
	hamdhSensor.setDeviceNameMin("iaqm_iaq");
	hamdhSensor.setUnitOfMeasurement("");
	hamdhSensor.setValueTemplate("{{ value_json.iaqm_iaq }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

	hamdhSensor.setDeviceClass("carbon_dioxide");
	hamdhSensor.setDeviceNameMin("iaqm_co2Equivalent");
	hamdhSensor.setUnitOfMeasurement("ppm");
	hamdhSensor.setValueTemplate("{{ value_json.iaqm_co2Equivalent }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);

	hamdhSensor.setDeviceClass("volatile_organic_compounds");
	hamdhSensor.setDeviceNameMin("iaqm_breathVocEquivalent");
	hamdhSensor.setUnitOfMeasurement("µg/m³");
	hamdhSensor.setValueTemplate("{{ value_json.iaqm_breathVocEquivalent }}");
	hamdhSensor.construct();
	mqttClient.publish(hamdhSensor.getMqttDiscoveryConfigTopic(), hamdhSensor.getMqttDiscoveryMesg(), true);
#endif

#if IR_MODULE
#if IR_SINGFUN_FAN
	HAMqttDiscoveryFan hamdhSFF(hamdhSF);
	hamdhSFF.setDeviceNameMin("IR_SINGFUN_FAN");
	hamdhSFF.setCommandTemplate("{\"sff_state\": \"{{ value }}\"}");
	hamdhSFF.setOscillationCommandTemplate("{\"sff_oscillation\": \"{{ value }}\"}");
	hamdhSFF.setStateValueTemplate("{{value_json.sff_state}}");
	hamdhSFF.setOscillationValueTemplate("{{value_json.sff_oscillation}}");
	hamdhSFF.construct();
	mqttClient.publish(hamdhSFF.getMqttDiscoveryConfigTopic(), hamdhSFF.getMqttDiscoveryMesg(), true);

	mqttClient.subscribe(hamdhSFF.getCtrlTopic(), [](const String &payload)
						 {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
      if (doc.containsKey("sff_state")) {
        String sffOnOff = doc["sff_state"];
        irsend.sendRaw(singfun_fan_power, 75, 38);
        feedbackDoc["sff_state"] = sffOnOff;
        if (sffOnOff.equals("OFF")) {
          feedbackDoc["sff_oscillation"] = "oscillate_off";
        }
      }
      if (doc.containsKey("sff_oscillation")) {
        String sffOsciOnOff = doc["sff_oscillation"];
        irsend.sendRaw(singfun_fan_oscillating, 75, 38);
        feedbackDoc["sff_oscillation"] = sffOsciOnOff;
      }
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhSF.getStateTopic(), feedbackMesg, true);
    } });
#endif
#if IR_GREE_AC
	HAMqttDiscoveryClimate hamdhGreeAC(hamdhGree);
	hamdhGreeAC.setDeviceNameMin("gree_ac");
	hamdhGreeAC.setModeCommandTemplate("{\"acgree_mode\": \"{{ value }}\"}");
	hamdhGreeAC.setFanModeCommandTemplate("{\"acgree_fan_mode\": \"{{ value }}\"}");
	hamdhGreeAC.setSwingModeCommandTemplate("{\"acgree_swing_mode\": \"{{ value }}\"}");
	hamdhGreeAC.setTemperatureCommandTemplate("{\"acgree_temperature\": \"{{ value }}\"}");
	hamdhGreeAC.setfanModeStateTemplate("{{ value_json.acgree_fan_mode }}");
	hamdhGreeAC.setModeStateTemplate("{{ value_json.acgree_mode }}");
	hamdhGreeAC.setSwingModeStateTemplate("{{ value_json.acgree_swing_mode }}");
	hamdhGreeAC.setTemperatureStateTemplate("{{ value_json.acgree_temperature }}");
	String modes[10] = {"auto", "cool", "dry", "fan_only", "heat", "off", "", "", "", ""};
	String swingModes[10] = {"off", "vertical", "horizontal", "both", "", "", "", "", "", ""};
	String fanModes[10] = {"low", "medium", "high", "", "", "", "", "", "", ""};
	hamdhGreeAC.setModes(modes);
	hamdhGreeAC.setSwingModes(swingModes);
	hamdhGreeAC.setFanModes(fanModes);

	hamdhGreeAC.construct();
	mqttClient.publish(hamdhGreeAC.getMqttDiscoveryConfigTopic(), hamdhGreeAC.getMqttDiscoveryMesg(), true);

	StaticJsonDocument<100> greeACdefaultStateDoc;
	String greeACdefaultState;
	greeACdefaultStateDoc["acgree_mode"] = "off";
	greeACdefaultStateDoc["acgree_fan_mode"] = "medium";
	greeACdefaultStateDoc["acgree_swing_mode"] = "off";
	greeACdefaultStateDoc["acgree_temperature"] = "26";
	serializeJson(greeACdefaultStateDoc, greeACdefaultState);
	mqttClient.publish(hamdhGreeAC.getStateTopic(), greeACdefaultState, true);

	mqttClient.subscribe(hamdhGreeAC.getCtrlTopic(), [](const String &payload)
						 {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
	  bool isOffCmd = false;
      if (doc.containsKey("acgree_mode"))
      {
        String mode = doc["acgree_mode"];
		greeAC.on();
        if (mode.equals("auto")) {
          greeAC.setMode(kGreeAuto);
        }
        else if (mode.equals("cool")) {
          greeAC.setMode(kGreeCool);
        }
        else if (mode.equals("dry")) {
          greeAC.setMode(kGreeDry);
        }
        else if (mode.equals("fan_only")) {
          greeAC.setMode(kGreeFan);
        }
        else if (mode.equals("heat")) {
          greeAC.setMode(kGreeHeat);
        }
		else if (mode.equals("off")) {
		  greeAC.off();
		  isOffCmd = true;
		}
        feedbackDoc["acgree_mode"] = mode;
      }
      if (doc.containsKey("acgree_fan_mode"))
      {
        String fanMode = doc["acgree_fan_mode"];
        if (fanMode.equals("low")) {
          greeAC.setFan(1);
        }
        else if (fanMode.equals("medium")) {
          greeAC.setFan(2);
        }
        else if (fanMode.equals("high")) {
          greeAC.setFan(3);
        }
        feedbackDoc["acgree_fan_mode"] = fanMode;
      }
      if (doc.containsKey("acgree_swing_mode"))
      {
        String swingMode = doc["acgree_swing_mode"];
        if (swingMode.equals("off")) {
          greeAC.setSwingVertical(false, kGreeSwingAuto);
          greeAC.setSwingHorizontal(false);
        }
        else if (swingMode.equals("vertical")) {
        greeAC.setSwingVertical(true, kGreeSwingAuto);
        }
        else if (swingMode.equals("horizontal")) {
        greeAC.setSwingHorizontal(true);
        }
        else if (swingMode.equals("both")) {
        greeAC.setSwingVertical(true, kGreeSwingAuto);
          greeAC.setSwingHorizontal(true);
        }
        feedbackDoc["acgree_swing_mode"] = swingMode;
      }
      if (doc.containsKey("acgree_temperature"))
      {
        String tempStr = doc["acgree_temperature"];
		float temp = tempStr.toFloat();
        int target = round(temp);
        greeAC.setTemp(target);
        feedbackDoc["acgree_temperature"] = tempStr;
      }
	  if(isOffCmd || ((!isOffCmd) && greeAC.getPower())){
		  greeAC.send();
	  }
      
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhGree.getStateTopic(), feedbackMesg, true);
    } });
#endif
#endif

#if RF_MODULE
#if RF_PHILIPS_FAN_LIGHT
	HAMqttDiscoveryFan hamdhPFLF(hamdhPFL);
	HAMqttDiscoveryLight hamdhPFLL(hamdhPFL);

	hamdhPFLF.setDeviceNameMin("philips_fanlight_fan");
	hamdhPFLF.setCommandTemplate("{\"pflf_state\": \"{{ value }}\"}");
	hamdhPFLF.setPercentageCommandTemplate("{\"pflf_percentage\": {{ value }}}");
	hamdhPFLF.setSpeedRangeMin(1);
	hamdhPFLF.setSpeedRangeMax(3);
	hamdhPFLF.setStateValueTemplate("{{value_json.pflf_state}}");
	hamdhPFLF.setPercentageValueTemplate("{{value_json.pflf_percentage}}");
	hamdhPFLF.construct();
	mqttClient.publish(hamdhPFLF.getMqttDiscoveryConfigTopic(), hamdhPFLF.getMqttDiscoveryMesg(), true);

	hamdhPFLL.setDeviceNameMin("philips_fanlight_light");
	hamdhPFLL.setPayloadOn("{\"pfll_state\":\"ON\"}");
	hamdhPFLL.setPayloadOff("{\"pfll_state\":\"OFF\"}");
	hamdhPFLL.setStateTemplate("{{ value_json.pfll_state }}");
	hamdhPFLL.construct();
	mqttClient.publish(hamdhPFLL.getMqttDiscoveryConfigTopic(), hamdhPFLL.getMqttDiscoveryMesg(), true);

	mqttClient.subscribe(hamdhPFL.getCtrlTopic(), [](const String &payload)
						 {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
      if (doc.containsKey("pflf_state"))
      {
        String pflf_state = doc["pflf_state"];
        feedbackDoc["pflf_state"] = pflf_state;
        if (pflf_state.equals("ON"))
        {
          sendRfSig(philips_fanlight_fan_low);
          feedbackDoc["pflf_percentage"] = 1;
        }
        else if (pflf_state.equals("OFF"))
        {
          sendRfSig(philips_fanlight_fan_off);
          feedbackDoc["pflf_percentage"] = 0;
        }
      }
      if (doc.containsKey("pflf_percentage"))
      {
        int val = doc["pflf_percentage"];
        feedbackDoc["pflf_percentage"] = val;
        if (val == 0) {
          sendRfSig(philips_fanlight_fan_off);
          feedbackDoc["pflf_state"] = "OFF";
        } else if (val == 1) {
          sendRfSig(philips_fanlight_fan_low);
          feedbackDoc["pflf_state"] = "ON";
        } else if (val == 2) {
          sendRfSig(philips_fanlight_fan_mid);
          feedbackDoc["pflf_state"] = "ON";
        } else if (val == 3) {
          sendRfSig(philips_fanlight_fan_high);
          feedbackDoc["pflf_state"] = "ON";
        }
      }
      if (doc.containsKey("pfll_state"))
      {
        String pfll_state = doc["pfll_state"];
        feedbackDoc["pfll_state"] = pfll_state;
        if (pfll_state.equals("ON")) {
          sendRfSig(philips_fanlight_light_on);
        }
        else if (pfll_state.equals("OFF")) {
          sendRfSig(philips_fanlight_light_off);
        }
      }
      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhPFL.getStateTopic(), feedbackMesg, true);
    } });

#endif
#endif
}

void pubSensorsData()
{

	DynamicJsonDocument json(1024);
	String pubPayload;
#if THI_MODULE
	if (thimHasNewData)
	{
		dataShift(thim_humidity);
		dataShift(thim_temperature);
		dataShift(thim_illuminance);

		json["thim_temperature"] = String(thim_temperature[0]);
		json["thim_humidity"] = String(thim_humidity[0]);
		json["thim_illuminance"] = String(thim_illuminance[0]);

		thimHasNewData = false;
	}
#endif

#if IAQ_MODULE
	if (bme680HasNewData)
	{
		dataShift(iaqm_temperature);
		dataShift(iaqm_humidity);
		dataShift(iaqm_pressure);
		dataShift(iaqm_iaq);
		dataShift(iaqm_co2Equivalent);
		dataShift(iaqm_breathVocEquivalent);

		json["iaqm_temperature"] = String(iaqm_temperature[0]);
		json["iaqm_humidity"] = String(iaqm_humidity[0]);
		json["iaqm_pressure"] = String(iaqm_pressure[0]);

		if (iaqIniting)
		{
			if (millis() - startLoopMs > 360000)
			{
				iaqIniting = false;
			}
		}
		else
		{
			json["iaqm_iaq"] = String(iaqm_iaq[0]);
			json["iaqm_co2Equivalent"] = String(iaqm_co2Equivalent[0]);
			json["iaqm_breathVocEquivalent"] = String(iaqm_breathVocEquivalent[0] * 1000);
		}

		bme680HasNewData = false;
	}

#endif

	if (!json.isNull())
	{
		serializeJson(json, pubPayload);
		mqttClient.publish(hamdhHG.getStateTopic(), pubPayload);
	}
}

void loop()
{
	mqttClient.loop();
#if (THI_MODULE || IAQ_MODULE) // if there is sensor need to public

#if THI_MODULE
	if (millis() - lastReadTHIMs > 2000) // read thi mod data to the arrays
	{
		shtc3.update();
		if (shtc3.lastStatus == SHTC3_Status_Nominal)
		{
			thim_temperature[1] = shtc3.toDegC();
			thim_humidity[1] = shtc3.toPercent();
		}

		int tempInt = round(thim_temperature[1]);
		float tempFactor = 1.0f - (tempInt - 20.0f) * 0.0005f;

		while ((!bh1750_1.measurementReady(true)) || (!bh1750_2.measurementReady(true)))
		{
			yield();
		}
		float lux1 = bh1750_1.readLightLevel();
		float lux2 = bh1750_2.readLightLevel();

		bh1750_1.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
		bh1750_2.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
		thim_illuminance[1] = fmaxf(lux1, lux2) * tempFactor;

		if (abs(thim_temperature[0] - thim_temperature[1]) > 0.3 || abs(thim_humidity[0] - thim_humidity[1]) > 3 || abs(thim_illuminance[0] - thim_illuminance[1]) > 0.1 * thim_illuminance[0])
		{
			thimEvent = true;
		}

		thimHasNewData = true;
		lastReadTHIMs = millis();
	}
#endif

#if IAQ_MODULE

	if (bme680.run()) // read bme680 data to the arrays
	{
		bme680HasNewData = true;
		iaqm_temperature[1] = bme680.temperature;
		iaqm_humidity[1] = bme680.humidity;
		iaqm_pressure[1] = bme680.pressure;
		iaqm_iaq[1] = bme680.staticIaq;
		iaqm_co2Equivalent[1] = bme680.co2Equivalent;
		iaqm_breathVocEquivalent[1] = bme680.breathVocEquivalent;

		if (abs(iaqm_temperature[0] - iaqm_temperature[1]) > 0.3 || abs(iaqm_humidity[0] - iaqm_humidity[1]) > 3 || abs(iaqm_iaq[0] - iaqm_iaq[1]) > 15)
		{
			iaqmEvent = true; // event flag ture if measurements change too fast
		}
	}

#endif

	// time driven and event drivent public
	if ((millis() - lastPubMs > timeDrivenPubInterval * 1000)
#if IAQ_MODULE
		|| iaqmEvent
#endif
#if THI_MODULE
		|| thimEvent
#endif
	)
	{
		pubSensorsData();
		lastPubMs = millis();

#if IAQ_MODULE
		iaqmEvent = false; // reset event flags
#endif
#if THI_MODULE
		thimEvent = false;
#endif
	}
#endif

#if ZGB_MODULE
	size_t bytes_read;
	uint8_t net_buf[BUFFER_SIZE];
	uint8_t serial_buf[BUFFER_SIZE];

	if (WiFi.status() != WL_CONNECTED)
	{
		// we've lost the connection, so we need to reconnect
		if (wClient)
		{
			wClient.stop();
		}
	}
	// Check if a client has connected
	if (!wClient.connected())
	{
		// eat any bytes in the serial buffer as there is nothing to see them
		while (zgbSerial.available())
		{
			zgbSerial.read();
		}

		wClient = ser2NetServer.available();
	}

#define min(a, b) ((a) < (b) ? (a) : (b))

	if (wClient.connected())
	{
		// check the network for any bytes to send to the serial
		int count = wClient.available();
		if (count > 0)
		{
			bytes_read = wClient.read(net_buf, min(count, BUFFER_SIZE));
			zgbSerial.write(net_buf, bytes_read);
			zgbSerial.flush();
		}

		// now check the serial for any bytes to send to the network
		bytes_read = 0;
		while (zgbSerial.available() && bytes_read < BUFFER_SIZE)
		{
			serial_buf[bytes_read] = zgbSerial.read();
			bytes_read++;
		}

		if (bytes_read > 0)
		{
			wClient.write((const uint8_t *)serial_buf, bytes_read);
			wClient.flush();
		}
	}
	else
	{
		wClient.stop();
	}
#endif

#if SERIAL_DEBUG
	if (Serial.available())
	{
		String cmd = Serial.readString();
		if (cmd.indexOf("lsmod") != -1)
		{
			Serial.println(moduleList());
		}
		else if (cmd.indexOf("restore") != -1)
		{
			SPIFFS.format();
			WiFi.disconnect(true, true);
			ESP.restart();
			delay(1000);
		}
		else if (cmd.indexOf("reboot") != -1)
		{
			ESP.restart();
			delay(1000);
		}
	}

#endif
}
