#include "LegacyRemotes.h"
#include "HomeGearConfig.h"
#include "MqttService.h"
#include "IrService.h"
#include "PowerPolicy.h"
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#if (IR_MODULE && IR_SINGFUN_FAN)
#include "IR_Code.h"
#endif
#if (IR_MODULE && IR_GREE_AC)
#include <ir_Gree.h>
#endif
#if (IR_MODULE && IR_MIDEA_AC)
#include <ir_Midea.h>
#endif
#if (IR_MODULE && IR_COOLIX_AC)
#include <ir_Coolix.h>
#endif
#if (IR_MODULE && IR_HAIER_AC)
#include <ir_Haier.h>
#endif
#if RF_MODULE
#include "RF_Code.h"
#endif
namespace homegear { namespace legacy {
namespace {
ESP32MQTTClient &mqttClient = mqtt::client();
HAMqttDiscoveryHandler &hamdhHG = mqtt::discovery();
#if IR_MODULE
const uint16_t kIrLed = ir::kIrLed;
IRsend &irsend = ir::sender();
#endif
#if IR_MODULE
#if IR_SINGFUN_FAN
HAMqttDiscoveryHandler hamdhSF("homegear_ir", "00001", "SingFun", "SingFun Floor Fan", SOFTWARE_VERSION, hamdhHG);
#endif

#if IR_GREE_AC
HAMqttDiscoveryHandler hamdhGree("homegear_ir", "ac001", "Gree", "Gree AC", SOFTWARE_VERSION, hamdhHG);
IRGreeAC greeAC(kIrLed);

void setGreeACDefaultState()
{
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
}
#endif

#if IR_MIDEA_AC
HAMqttDiscoveryHandler hamdhMidea("homegear_ir", "ac001", "Midea", "Midea AC", SOFTWARE_VERSION, hamdhHG);
IRMideaAC mideaAC(kIrLed);

void setMideaACDefaultState()
{
  mideaAC.off();
  mideaAC.setFan(0);
  mideaAC.setMode(kMideaACCool);
  mideaAC.setTemp(26); // 16-30C
  mideaAC.setSwingVToggle(false);
  mideaAC.setTurboToggle(false);
  mideaAC.setLightToggle(false);
  mideaAC.setQuiet(false);
  mideaAC.setSleep(false);
}
#endif

#if IR_COOLIX_AC
HAMqttDiscoveryHandler hamdhCoolix("homegear_ir", "ac001", "Coolix", "Coolix AC", SOFTWARE_VERSION, hamdhHG);
IRCoolixAC coolixAC(kIrLed);

void setCoolixACDefaultState()
{
  coolixAC.off();
  coolixAC.setTemp(26); // 16-30C
}
#endif

#if IR_HAIER_AC
HAMqttDiscoveryHandler hamdhHaier("homegear_ir", "ac001", "Haier", "Haier AC", SOFTWARE_VERSION, hamdhHG);
IRHaierAC176 haierAC(kIrLed);

void setHaierACDefaultState()
{
  haierAC.off();
  haierAC.setTemp(26); // 16-30C
}
#endif

#endif

#if RF_MODULE

HardwareSerial rfSerial(1);

#if RF_PHILIPS_FAN_LIGHT
HAMqttDiscoveryHandler hamdhPFL("homegear_rf", "00001", "Philips", "Philips RF Remote Fan Light", SOFTWARE_VERSION, hamdhHG);
#endif

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


#if (IR_MODULE && (IR_SINGFUN_FAN || IR_GREE_AC || IR_MIDEA_AC || IR_COOLIX_AC || IR_HAIER_AC)) || (RF_MODULE && RF_PHILIPS_FAN_LIGHT)
using Action = void (*)(const String &);
struct Command { Action action; char payload[513]; };
QueueHandle_t commandQueue = nullptr;
void queueLegacy(const String &payload, Action action) {
  if (!commandQueue || payload.length() >= sizeof(Command::payload)) { Serial.println("LEGACY ERROR COMMAND_REJECTED"); return; }
  Command command = {};
  command.action = action;
  payload.toCharArray(command.payload, sizeof(command.payload));
  if (xQueueSend(commandQueue, &command, 0) != pdTRUE) Serial.println("LEGACY ERROR QUEUE_FULL");
}
#endif
}
void begin() {
#if (IR_MODULE && (IR_SINGFUN_FAN || IR_GREE_AC || IR_MIDEA_AC || IR_COOLIX_AC || IR_HAIER_AC)) || (RF_MODULE && RF_PHILIPS_FAN_LIGHT)
  commandQueue = xQueueCreate(4, sizeof(Command));
  if (!commandQueue) Serial.println("LEGACY ERROR QUEUE_ALLOCATION_FAILED");
#endif
#if (IR_MODULE && IR_GREE_AC)
  greeAC.begin();
#endif
#if (IR_MODULE && IR_MIDEA_AC)
  mideaAC.begin();
#endif
#if (IR_MODULE && IR_COOLIX_AC)
  coolixAC.begin();
#endif
#if (IR_MODULE && IR_HAIER_AC)
  haierAC.begin();
#endif
#if RF_MODULE
  rfSerial.begin(115200, SERIAL_8N1, 13, 12);
#endif
}
void onConnected() {
#if IR_MODULE
#if IR_SINGFUN_FAN
  {
  HAMqttDiscoveryFan hamdhSFF(hamdhSF);
  hamdhSFF.setDeviceNameMin("singfun_fan");
  hamdhSFF.setCommandTemplate("{\"sff_state\": \"{{ value }}\"}");
  hamdhSFF.setOscillationCommandTemplate("{\"sff_oscillation\": \"{{ value }}\"}");
  hamdhSFF.setStateValueTemplate("{{value_json.sff_state}}");
  hamdhSFF.setOscillationValueTemplate("{{value_json.sff_oscillation}}");
  hamdhSFF.construct();
  mqttClient.publish(hamdhSFF.getMqttDiscoveryConfigTopic(), hamdhSFF.getMqttDiscoveryMesg(), true);

  mqttClient.subscribe(hamdhSFF.getCtrlTopic(), [](const String &payload) {
    queueLegacy(payload, [](const String &payload) {
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
  });
  }
#endif
#if IR_GREE_AC
  {
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
  // String modes[10] = {"auto", "cool", "dry", "fan_only", "heat", "off", "", "", "", ""};
  String modes[10] = {"auto", "cool", "dry", "fan_only", "off", "", "", "", "", ""};
  String swingModes[10] = {"off", "vertical", "horizontal", "both", "", "", "", "", "", ""};
  String fanModes[10] = {"low", "medium", "high", "", "", "", "", "", "", ""};
  hamdhGreeAC.setModes(modes);
  hamdhGreeAC.setSwingModes(swingModes);
  hamdhGreeAC.setFanModes(fanModes);

  hamdhGreeAC.construct();
  mqttClient.publish(hamdhGreeAC.getMqttDiscoveryConfigTopic(), hamdhGreeAC.getMqttDiscoveryMesg(), true);

  queueLegacy(String(), [](const String &) {
  setGreeACDefaultState();
  StaticJsonDocument<100> greeACdefaultStateDoc;
  String greeACdefaultState;
  greeACdefaultStateDoc["acgree_mode"] = "off";
  greeACdefaultStateDoc["acgree_fan_mode"] = "medium";
  greeACdefaultStateDoc["acgree_swing_mode"] = "off";
  greeACdefaultStateDoc["acgree_temperature"] = "26";
  serializeJson(greeACdefaultStateDoc, greeACdefaultState);
  mqttClient.publish(hamdhGree.getStateTopic(), greeACdefaultState, true);

  });

  mqttClient.subscribe(hamdhGreeAC.getCtrlTopic(), [](const String &payload) {
    queueLegacy(payload, [](const String &payload) {
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
  });
  }
#endif

#if IR_MIDEA_AC
  {
  HAMqttDiscoveryClimate hamdhMideaAC(hamdhMidea);
  hamdhMideaAC.setDeviceNameMin("midea_ac");
  hamdhMideaAC.setModeCommandTemplate("{\"acmidea_mode\": \"{{ value }}\"}");
  hamdhMideaAC.setFanModeCommandTemplate("{\"acmidea_fan_mode\": \"{{ value }}\"}");
  hamdhMideaAC.setSwingModeCommandTemplate("{\"acmidea_swing_mode\": \"{{ value }}\"}");
  hamdhMideaAC.setTemperatureCommandTemplate("{\"acmidea_temperature\": \"{{ value }}\"}");
  hamdhMideaAC.setfanModeStateTemplate("{{ value_json.acmidea_fan_mode }}");
  hamdhMideaAC.setModeStateTemplate("{{ value_json.acmidea_mode }}");
  hamdhMideaAC.setSwingModeStateTemplate("{{ value_json.acmidea_swing_mode }}");
  hamdhMideaAC.setTemperatureStateTemplate("{{ value_json.acmidea_temperature }}");
  String modes[10] = {"auto", "cool", "dry", "fan_only", "heat", "off", "", "", "", ""};
  // String modes[10] = {"auto", "cool", "dry", "fan_only", "off", "", "", "", "", ""};
  String swingModes[10] = {"off", "vertical", "", "", "", "", "", "", "", ""};
  String fanModes[10] = {"low", "medium", "high", "auto", "", "", "", "", "", ""};
  hamdhMideaAC.setModes(modes);
  hamdhMideaAC.setSwingModes(swingModes);
  hamdhMideaAC.setFanModes(fanModes);

  hamdhMideaAC.construct();
  mqttClient.publish(hamdhMideaAC.getMqttDiscoveryConfigTopic(), hamdhMideaAC.getMqttDiscoveryMesg(), true);

  queueLegacy(String(), [](const String &) {
  setMideaACDefaultState();
  StaticJsonDocument<100> mideaACdefaultStateDoc;
  String mideaACdefaultState;
  mideaACdefaultStateDoc["acmidea_mode"] = "off";
  mideaACdefaultStateDoc["acmidea_fan_mode"] = "medium";
  mideaACdefaultStateDoc["acmidea_swing_mode"] = "off";
  mideaACdefaultStateDoc["acmidea_temperature"] = "26";
  serializeJson(mideaACdefaultStateDoc, mideaACdefaultState);
  mqttClient.publish(hamdhMidea.getStateTopic(), mideaACdefaultState, true);

  });

  mqttClient.subscribe(hamdhMideaAC.getCtrlTopic(), [](const String &payload) {
    queueLegacy(payload, [](const String &payload) {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
	  bool isOffCmd = false;
      if (doc.containsKey("acmidea_mode"))
      {
        String mode = doc["acmidea_mode"];
		mideaAC.on();
        if (mode.equals("auto")) {
          mideaAC.setMode(kMideaACAuto);
        }
        else if (mode.equals("cool")) {
          mideaAC.setMode(kMideaACCool);
        }
        else if (mode.equals("dry")) {
          mideaAC.setMode(kMideaACDry);
        }
        else if (mode.equals("fan_only")) {
          mideaAC.setMode(kMideaACFan);
        }
        else if (mode.equals("heat")) {
          mideaAC.setMode(kMideaACHeat);
        }
		else if (mode.equals("off")) {
		  mideaAC.off();
		  isOffCmd = true;
		}
        feedbackDoc["acmidea_mode"] = mode;
      }
      if (doc.containsKey("acmidea_fan_mode"))
      {
        String fanMode = doc["acmidea_fan_mode"];
        if (fanMode.equals("low")) {
          mideaAC.setFan(1);
        }
        else if (fanMode.equals("medium")) {
          mideaAC.setFan(2);
        }
        else if (fanMode.equals("high")) {
          mideaAC.setFan(3);
        }
        else if (fanMode.equals("auto")) {
          mideaAC.setFan(0);
        }
        feedbackDoc["acmidea_fan_mode"] = fanMode;
      }
      if (doc.containsKey("acmidea_swing_mode"))
      {
        String swingMode = doc["acmidea_swing_mode"];
        if (swingMode.equals("off")) {
          mideaAC.setSwingVToggle(false);
        }
        else if (swingMode.equals("vertical")) {
        mideaAC.setSwingVToggle(true);
        }
        feedbackDoc["acmidea_swing_mode"] = swingMode;
      }
      if (doc.containsKey("acmidea_temperature"))
      {
        String tempStr = doc["acmidea_temperature"];
		float temp = tempStr.toFloat();
        int target = round(temp);
        mideaAC.setTemp(target);
        feedbackDoc["acmidea_temperature"] = tempStr;
      }
	  if(isOffCmd || ((!isOffCmd) && mideaAC.getPower())){
		  mideaAC.send();
	  }

      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhMidea.getStateTopic(), feedbackMesg, true);
    } });
  });
  }
#endif

#if IR_COOLIX_AC
  {
  HAMqttDiscoveryClimate hamdhCoolixAC(hamdhCoolix);
  hamdhCoolixAC.setDeviceNameMin("coolix_ac");
  hamdhCoolixAC.setModeCommandTemplate("{\"accoolix_mode\": \"{{ value }}\"}");
  hamdhCoolixAC.setSwingModeCommandTemplate("{\"accoolix_swing_mode\": \"{{ value }}\"}");
  hamdhCoolixAC.setTemperatureCommandTemplate("{\"accoolix_temperature\": \"{{ value }}\"}");
  hamdhCoolixAC.setModeStateTemplate("{{ value_json.accoolix_mode }}");
  hamdhCoolixAC.setSwingModeStateTemplate("{{ value_json.accoolix_swing_mode }}");
  hamdhCoolixAC.setTemperatureStateTemplate("{{ value_json.accoolix_temperature }}");
  String modes[10] = {"cool", "heat", "off", "", "", "", "", "", "", ""};
  String swingModes[10] = {"off", "vertical", "", "", "", "", "", "", "", ""};
  hamdhCoolixAC.setModes(modes);
  hamdhCoolixAC.setSwingModes(swingModes);

  hamdhCoolixAC.construct();
  mqttClient.publish(hamdhCoolixAC.getMqttDiscoveryConfigTopic(), hamdhCoolixAC.getMqttDiscoveryMesg(), true);

  queueLegacy(String(), [](const String &) {
  setCoolixACDefaultState();
  StaticJsonDocument<100> coolixACdefaultStateDoc;
  String coolixACdefaultState;
  coolixACdefaultStateDoc["accoolix_mode"] = "off";
  coolixACdefaultStateDoc["accoolix_swing_mode"] = "off";
  coolixACdefaultStateDoc["accoolix_temperature"] = "26";
  serializeJson(coolixACdefaultStateDoc, coolixACdefaultState);
  mqttClient.publish(hamdhCoolix.getStateTopic(), coolixACdefaultState, true);

  });

  mqttClient.subscribe(hamdhCoolixAC.getCtrlTopic(), [](const String &payload) {
    queueLegacy(payload, [](const String &payload) {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
	  bool isOffCmd = false;
      if (doc.containsKey("accoolix_mode"))
      {
        String mode = doc["accoolix_mode"];
		coolixAC.on();
        if (mode.equals("cool")) {
          coolixAC.setMode(kCoolixCool);
        }
        else if (mode.equals("heat")) {
          coolixAC.setMode(kCoolixHeat);
        }
		else if (mode.equals("off")) {
		  coolixAC.off();
		  isOffCmd = true;
		}
        feedbackDoc["accoolix_mode"] = mode;
      }
      if (doc.containsKey("accoolix_swing_mode"))
      {
        String swingMode = doc["accoolix_swing_mode"];
        coolixAC.setSwing();
        feedbackDoc["accoolix_swing_mode"] = swingMode;
      }
      if (doc.containsKey("accoolix_temperature"))
      {
        String tempStr = doc["accoolix_temperature"];
		float temp = tempStr.toFloat();
        int target = round(temp);
        coolixAC.setTemp(target);
        feedbackDoc["accoolix_temperature"] = tempStr;
      }
	  if(isOffCmd || ((!isOffCmd) && coolixAC.getPower())){
	    coolixAC.send();
	  }

      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhCoolix.getStateTopic(), feedbackMesg, true);
    } });
  });
  }
#endif

#if IR_HAIER_AC
  {
  HAMqttDiscoveryClimate hamdhHaierAC(hamdhHaier);
  hamdhHaierAC.setDeviceNameMin("haier_ac");
  hamdhHaierAC.setModeCommandTemplate("{\"achaier_mode\": \"{{ value }}\"}");
  hamdhHaierAC.setSwingModeCommandTemplate("{\"achaier_swing_mode\": \"{{ value }}\"}");
  hamdhHaierAC.setTemperatureCommandTemplate("{\"achaier_temperature\": \"{{ value }}\"}");
  hamdhHaierAC.setModeStateTemplate("{{ value_json.achaier_mode }}");
  hamdhHaierAC.setSwingModeStateTemplate("{{ value_json.achaier_swing_mode }}");
  hamdhHaierAC.setTemperatureStateTemplate("{{ value_json.achaier_temperature }}");
  String modes[10] = {"cool", "heat", "off", "", "", "", "", "", "", ""};
  String swingModes[10] = {"off", "vertical", "", "", "", "", "", "", "", ""};
  hamdhHaierAC.setModes(modes);
  hamdhHaierAC.setSwingModes(swingModes);

  hamdhHaierAC.construct();
  mqttClient.publish(hamdhHaierAC.getMqttDiscoveryConfigTopic(), hamdhHaierAC.getMqttDiscoveryMesg(), true);

  queueLegacy(String(), [](const String &) {
  setHaierACDefaultState();
  StaticJsonDocument<100> haierACdefaultStateDoc;
  String haierACdefaultState;
  haierACdefaultStateDoc["achaier_mode"] = "off";
  haierACdefaultStateDoc["achaier_swing_mode"] = "off";
  haierACdefaultStateDoc["achaier_temperature"] = "26";
  serializeJson(haierACdefaultStateDoc, haierACdefaultState);
  mqttClient.publish(hamdhHaier.getStateTopic(), haierACdefaultState, true);

  });

  mqttClient.subscribe(hamdhHaierAC.getCtrlTopic(), [](const String &payload) {
    queueLegacy(payload, [](const String &payload) {
    StaticJsonDocument<100> doc;
    if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
    {
      StaticJsonDocument<100> feedbackDoc;
      String feedbackMesg;
	  bool isOffCmd = false;
      if (doc.containsKey("achaier_mode"))
      {
        String mode = doc["achaier_mode"];
		haierAC.on();
        if (mode.equals("cool")) {
          haierAC.setMode(kHaierAcCool);
        }
        else if (mode.equals("heat")) {
          haierAC.setMode(kHaierAcHeat);
        }
		else if (mode.equals("off")) {
		  haierAC.off();
		  isOffCmd = true;
		}
        feedbackDoc["achaier_mode"] = mode;
      }
      if (doc.containsKey("achaier_swing_mode"))
      {
        String swingMode = doc["achaier_swing_mode"];
        if(swingMode.equals("off")){
          haierAC.setSwingV(kHaierAcYrw02SwingVOff);
        }else if(swingMode.equals("vertical")){
          haierAC.setSwingV(kHaierAcYrw02SwingVAuto);
        }

        feedbackDoc["achaier_swing_mode"] = swingMode;
      }
      if (doc.containsKey("achaier_temperature"))
      {
        String tempStr = doc["achaier_temperature"];
		float temp = tempStr.toFloat();
        int target = round(temp);
        haierAC.setTemp(target);
        feedbackDoc["achaier_temperature"] = tempStr;
      }
	  if(isOffCmd || ((!isOffCmd) && haierAC.getPower())){
	    haierAC.send();
	  }

      serializeJson(feedbackDoc, feedbackMesg);
      mqttClient.publish(hamdhHaier.getStateTopic(), feedbackMesg, true);
    } });
  });
  }
#endif
#endif

#if RF_MODULE
#if RF_PHILIPS_FAN_LIGHT
  {
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

  mqttClient.subscribe(hamdhPFL.getCtrlTopic(), [](const String &payload) {
    queueLegacy(payload, [](const String &payload) {
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
  });

  }
#endif
#endif
}
void tick() {
#if (IR_MODULE && (IR_SINGFUN_FAN || IR_GREE_AC || IR_MIDEA_AC || IR_COOLIX_AC || IR_HAIER_AC)) || (RF_MODULE && RF_PHILIPS_FAN_LIGHT)
  Command command;
  if (commandQueue && xQueueReceive(commandQueue, &command, 0) == pdTRUE) {
    power::ScopedIrClock clock;
    if (!clock.ready()) { Serial.println("LEGACY ERROR IR_CLOCK_UNAVAILABLE"); return; }
    command.action(String(command.payload));
  }
#endif
}
void setRfBaud() {
#if RF_MODULE
  sendRfSig(rf_module_set_baudrate_to_115200);
#endif
}
}}
