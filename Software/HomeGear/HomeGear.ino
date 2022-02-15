/**
 * @file HomeGear.ino
 * @author Chen Yijun
 * @brief
 * @version 1.0
 * @date 2022-02-15
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
#include "HAMDHandler.h"
#include "EspMQTTClient.h"

//***** Basic *****

#define PLATFORM "HomeGear"
#define HOMEGEAR_SERIAL_NUMBER chipId
#define MANUFACTURER "Chen Yijun"
#define DEVICE_MODEL "HomeGear Scalable Hardware Stack"
#define SOFTWARE_VERSION "HomeGear 1.0"

#define ESP_getChipId() ((uint32_t)ESP.getEfuseMac())
const String chipId = String(ESP_getChipId(), HEX);
char nodename[32];

unsigned int measureInterval = 15;
unsigned long lastPubMs;
unsigned long startLoopMs;

void dataShift(float data[2])
{
	data[0] = data[1];
}

//***** Modules *****

#define THI_MODULE true
#define IAQ_MODULE true
#define IR_MODULE false
#define RF_MODULE true
#define ZGB_MODULE true

HAMqttDiscoveryHandler hamdhHG(PLATFORM, HOMEGEAR_SERIAL_NUMBER, MANUFACTURER, DEVICE_MODEL, SOFTWARE_VERSION);

#if RF_MODULE
HAMqttDiscoveryHandler hamdhPFL("homegear_rf", "00001", "Philips", "Philips RF Remote Fan Light", SOFTWARE_VERSION, hamdhHG.getDeviceId());
#endif

#if THI_MODULE
bool thimEvent = false;
bool thimHasNewData = false;
unsigned long lastReadTHIMs = 0;
// SHTC3
#include "SHTSensor.h"
SHTSensor shtc3(SHTSensor::SHTC3);

// BH1750*2
#include "BH1750FVI.h"
BH1750FVI bh1750_1(0x23);
BH1750FVI bh1750_2(0x5C);

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
#endif

#if RF_MODULE
#include "RF_Code.h"
void sendRfSig(const unsigned char data[6])
{
	for (int i = 0; i < 6; i++)
	{
		Serial1.write(rf_module_uart_protocol[0]);
		Serial1.write(data[i]);
		Serial1.write(rf_module_uart_protocol[1]);
	}
}
#define PHILIPS_FAN_LIGHT true
#if PHILIPS_FAN_LIGHT
void pflfFeedback(EspMQTTClient &client, HAMqttDiscoveryHandler &pfl, String state, int percent)
{
	String mesg = "{\"pflf_state\": \"" + state + "\",\"pflf_percentage\": " + String(percent) + "}";
	client.publish(pfl.getFeedbackTopic(), mesg, true);
}
#endif
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

#endif

//***** network communication *****

// MQTT
EspMQTTClient mqttClient;

//***** WM *****
#include <FS.h>			 //this needs to be first, or it all crashes and burns...
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <SPIFFS.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson

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
					Serial.println("\nparsed json");
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
		Serial.println("failed to mount FS");
	}
	// end read
}

void saveConfigToFS()
{
	// save the custom parameters to FS
	if (shouldSaveConfig)
	{
		Serial.println("saving config");
		DynamicJsonDocument json(1024);
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

		serializeJson(json, Serial);
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

	// set minimu quality of signal so it ignores AP's under that quality
	// defaults to 8%
	// wifiManager.setMinimumSignalQuality();

	// sets timeout until configuration portal gets turned off
	// useful to make it all retry or go to sleep
	// in seconds
	// wifiManager.setTimeout(120);

	// fetches ssid and pass and tries to connect
	// if it does not connect it starts an access point with the specified name
	// here  "AutoConnectAP"
	// and goes into a blocking loop awaiting configuration
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
	mqttClient.setMaxPacketSize(1024);
	mqttClient.enableDebuggingMessages();
	mqttClient.enableHTTPWebUpdater("admin", "homegearota", "/ota");
	mqttClient.enableLastWillMessage(hamdhHG.getAvailabilityTopic().c_str(), "offline", true);

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
	bh1750_1.powerOn();
	bh1750_2.setContHighRes();
	bh1750_1.powerOn();
	bh1750_2.setContHighRes();
	shtc3.init();
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
#endif

#if RF_MODULE
#define RXD1 13
#define TXD1 12
	Serial1.begin(9600, SERIAL_8N1, RXD1, TXD1);
#endif

#if ZGB_MODULE
	Serial2.begin(115200);
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
	mqttClient.publish(hamdhHG.getAvailabilityTopic(), "online", true);
#if RF_MODULE
	mqttClient.publish(hamdhPFL.getAvailabilityTopic(), "online", true);
#endif

	// Subscribe to cmdTopic
	mqttClient.subscribe(hamdhHG.getCmdTopic(), [](const String &payload)
						 {
                             if (payload.equals("restore"))
                             {
                                 SPIFFS.format();
                                 WiFi.disconnect();
                                 ESP.restart();
                             }
                             if (payload.equals("reboot"))
                             {
                                 ESP.restart();
                             }
                             delay(1000); });

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
#endif

#if RF_MODULE
#if PHILIPS_FAN_LIGHT
	HAMqttDiscoveryFan hamdhFan(hamdhPFL);
	HAMqttDiscoveryLight hamdhLight(hamdhPFL);

	hamdhFan.setDeviceNameMin("philips_fanlight_fan");
	hamdhFan.setCommandTemplate("{\"pflf_state\": \"{{ value }}\"}");
	hamdhFan.setPercentageCommandTemplate("{\"pflf_percentage\": {{ value }}}");
	hamdhFan.setSpeedRangeMin(1);
	hamdhFan.setSpeedRangeMax(3);
	hamdhFan.setStateValueTemplate("{{value_json.pflf_state}}");
	hamdhFan.setPercentageValueTemplate("{{value_json.pflf_percentage}}");
	hamdhFan.construct();
	mqttClient.publish(hamdhFan.getMqttDiscoveryConfigTopic(), hamdhFan.getMqttDiscoveryMesg(), true);

	hamdhLight.setDeviceNameMin("philips_fanlight_light");
	hamdhLight.setPayloadOn("{\"pfll_state\": \"ON\"}");
	hamdhLight.setPayloadOff("{\"pfll_state\": \"OFF\"}");
	hamdhLight.setStateTemplate("{{ value_json.pfll_state }}");
	hamdhLight.construct();
	mqttClient.publish(hamdhLight.getMqttDiscoveryConfigTopic(), hamdhLight.getMqttDiscoveryMesg(), true);

	mqttClient.subscribe(hamdhPFL.getCtrlTopic(), [](const String &payload)
						 {
							 DynamicJsonDocument doc(512);
							 if (deserializeJson(doc, payload)== ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
							 {
								 if (doc.containsKey("pflf_state"))
								 {
									 String pflf_state=doc["pflf_state"];
									 if (pflf_state.equals("ON"))
									 {
										 sendRfSig(philips_fanlight_fan_low);
										 pflfFeedback(mqttClient,hamdhPFL, "ON", 1);
									 }
									 else if(pflf_state.equals("OFF"))
									 {
										 sendRfSig(philips_fanlight_fan_off);
									 	 pflfFeedback(mqttClient,hamdhPFL, "OFF", 0);
									 }
								 }
								 if (doc.containsKey("pflf_percentage"))
								 {
									 int val = doc["pflf_percentage"];
									if (val==0){
										sendRfSig(philips_fanlight_fan_off);
										pflfFeedback(mqttClient,hamdhPFL, "OFF", val);
									}else if(val==1){
										sendRfSig(philips_fanlight_fan_low);
									 	pflfFeedback(mqttClient,hamdhPFL, "ON", val);
									}else if(val==2){
										sendRfSig(philips_fanlight_fan_mid);
									 	pflfFeedback(mqttClient,hamdhPFL, "ON", val);
									}else if(val==3){
										sendRfSig(philips_fanlight_fan_high);
									 	pflfFeedback(mqttClient,hamdhPFL, "ON", val);
									}
								 }
								 if (doc.containsKey("pfll_state"))
								 {
									 String pfll_state=doc["pfll_state"];
									 if(pfll_state.equals("ON")){
										 sendRfSig(philips_fanlight_light_on);
										mqttClient.publish(hamdhPFL.getFeedbackTopic(), "{\"pfll_state\": \"ON\"}",true);
									 }
									 else if(pfll_state.equals("OFF")){
										 sendRfSig(philips_fanlight_light_off);
										mqttClient.publish(hamdhPFL.getFeedbackTopic(), "{\"pfll_state\": \"OFF\"}",true);
									 }
								 }
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
		thim_temperature[1] = shtc3.getTemperature();
		thim_humidity[1] = shtc3.getHumidity();

		bh1750_1.setTemperature(round(thim_temperature[1]));
		bh1750_2.setTemperature(round(thim_temperature[1]));
		float lux1 = bh1750_1.getLux();
		float lux2 = bh1750_2.getLux();
		thim_illuminance[1] = fmaxf(lux1, lux2);

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
	if ((millis() - lastPubMs > measureInterval * 1000)
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

#if IR_MODULE
#endif

#if RF_MODULE
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
		while (Serial2.available())
		{
			Serial2.read();
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
			Serial2.write(net_buf, bytes_read);
			Serial2.flush();
		}

		// now check the serial for any bytes to send to the network
		bytes_read = 0;
		while (Serial2.available() && bytes_read < BUFFER_SIZE)
		{
			serial_buf[bytes_read] = Serial2.read();
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
}
