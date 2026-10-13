#include "HomeGearConfig.h"
#if (IR_MODULE && IR_GREE_AC)
#include "LegacyRemoteInternals.h"
#include "MqttService.h"
#include "IrService.h"
#include <ArduinoJson.h>
#include <ir_Gree.h>

namespace homegear {
namespace legacy {
namespace detail {
namespace gree {
namespace {
HAMqttDiscoveryHandler &handler() {
  static HAMqttDiscoveryHandler value("homegear_ir", "ac001", "Gree", "Gree AC", SOFTWARE_VERSION, mqtt::discovery());
  return value;
}
IRGreeAC &controller() {
  static IRGreeAC value(ir::kIrLed);
  return value;
}

void setGreeACDefaultState()
{
  controller().off();
  controller().setFan(1);
  // kGreeAuto, kGreeDry, kGreeCool, kGreeFan, kGreeHeat
  controller().setMode(kGreeCool);
  controller().setTemp(26); // 16-30C
  controller().setSwingVertical(false, kGreeSwingAuto);
  controller().setSwingHorizontal(false);
  controller().setXFan(false);
  controller().setLight(false);
  controller().setSleep(false);
  controller().setTurbo(false);
}
}  // namespace

void begin() {
  (void)handler();
  controller().begin();
}

void onConnected() {
  HAMqttDiscoveryClimate hamdhGreeAC(handler());
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
  mqtt::client().publish(hamdhGreeAC.getMqttDiscoveryConfigTopic(), hamdhGreeAC.getMqttDiscoveryMesg(), true);

  enqueue(String(), [](const String &) {
    setGreeACDefaultState();
    StaticJsonDocument<100> greeACdefaultStateDoc;
    String greeACdefaultState;
    greeACdefaultStateDoc["acgree_mode"] = "off";
    greeACdefaultStateDoc["acgree_fan_mode"] = "medium";
    greeACdefaultStateDoc["acgree_swing_mode"] = "off";
    greeACdefaultStateDoc["acgree_temperature"] = "26";
    serializeJson(greeACdefaultStateDoc, greeACdefaultState);
    mqtt::client().publish(handler().getStateTopic(), greeACdefaultState, true);

  });

  mqtt::client().subscribe(hamdhGreeAC.getCtrlTopic(), [](const String &payload) {
    enqueue(payload, [](const String &payload) {
      StaticJsonDocument<100> doc;
      if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
      {
        StaticJsonDocument<100> feedbackDoc;
        String feedbackMesg;
        bool isOffCmd = false;
        if (doc.containsKey("acgree_mode"))
        {
          String mode = doc["acgree_mode"];
          controller().on();
          if (mode.equals("auto")) {
            controller().setMode(kGreeAuto);
          }
          else if (mode.equals("cool")) {
            controller().setMode(kGreeCool);
          }
          else if (mode.equals("dry")) {
            controller().setMode(kGreeDry);
          }
          else if (mode.equals("fan_only")) {
            controller().setMode(kGreeFan);
          }
          else if (mode.equals("heat")) {
            controller().setMode(kGreeHeat);
          }
          else if (mode.equals("off")) {
            controller().off();
            isOffCmd = true;
          }
          feedbackDoc["acgree_mode"] = mode;
        }
        if (doc.containsKey("acgree_fan_mode"))
        {
          String fanMode = doc["acgree_fan_mode"];
          if (fanMode.equals("low")) {
            controller().setFan(1);
          }
          else if (fanMode.equals("medium")) {
            controller().setFan(2);
          }
          else if (fanMode.equals("high")) {
            controller().setFan(3);
          }
          feedbackDoc["acgree_fan_mode"] = fanMode;
        }
        if (doc.containsKey("acgree_swing_mode"))
        {
          String swingMode = doc["acgree_swing_mode"];
          if (swingMode.equals("off")) {
            controller().setSwingVertical(false, kGreeSwingAuto);
            controller().setSwingHorizontal(false);
          }
          else if (swingMode.equals("vertical")) {
            controller().setSwingVertical(true, kGreeSwingAuto);
          }
          else if (swingMode.equals("horizontal")) {
            controller().setSwingHorizontal(true);
          }
          else if (swingMode.equals("both")) {
            controller().setSwingVertical(true, kGreeSwingAuto);
            controller().setSwingHorizontal(true);
          }
          feedbackDoc["acgree_swing_mode"] = swingMode;
        }
        if (doc.containsKey("acgree_temperature"))
        {
          String tempStr = doc["acgree_temperature"];
          float temp = tempStr.toFloat();
          int target = round(temp);
          controller().setTemp(target);
          feedbackDoc["acgree_temperature"] = tempStr;
        }
        if(isOffCmd || ((!isOffCmd) && controller().getPower())){
          controller().send();
        }

        serializeJson(feedbackDoc, feedbackMesg);
        mqtt::client().publish(handler().getStateTopic(), feedbackMesg, true);
    } });
  });
}
}  // namespace gree
}  // namespace detail
}  // namespace legacy
}  // namespace homegear
#endif
