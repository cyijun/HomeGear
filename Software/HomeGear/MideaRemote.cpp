#include "HomeGearConfig.h"
#if (IR_MODULE && IR_MIDEA_AC)
#include "LegacyRemoteInternals.h"
#include "MqttService.h"
#include "IrService.h"
#include <ArduinoJson.h>
#include <ir_Midea.h>

namespace homegear {
namespace legacy {
namespace detail {
namespace midea {
namespace {
HAMqttDiscoveryHandler &handler() {
  static HAMqttDiscoveryHandler value("homegear_ir", "ac001", "Midea", "Midea AC", SOFTWARE_VERSION, mqtt::discovery());
  return value;
}
IRMideaAC &controller() {
  static IRMideaAC value(ir::kIrLed);
  return value;
}

void setMideaACDefaultState()
{
  controller().off();
  controller().setFan(0);
  controller().setMode(kMideaACCool);
  controller().setTemp(26); // 16-30C
  controller().setSwingVToggle(false);
  controller().setTurboToggle(false);
  controller().setLightToggle(false);
  controller().setQuiet(false);
  controller().setSleep(false);
}
}  // namespace

void begin() {
  (void)handler();
  controller().begin();
}

void onConnected() {
  HAMqttDiscoveryClimate hamdhMideaAC(handler());
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
  mqtt::client().publish(hamdhMideaAC.getMqttDiscoveryConfigTopic(), hamdhMideaAC.getMqttDiscoveryMesg(), true);

  enqueue(String(), [](const String &) {
    setMideaACDefaultState();
    StaticJsonDocument<100> mideaACdefaultStateDoc;
    String mideaACdefaultState;
    mideaACdefaultStateDoc["acmidea_mode"] = "off";
    mideaACdefaultStateDoc["acmidea_fan_mode"] = "medium";
    mideaACdefaultStateDoc["acmidea_swing_mode"] = "off";
    mideaACdefaultStateDoc["acmidea_temperature"] = "26";
    serializeJson(mideaACdefaultStateDoc, mideaACdefaultState);
    mqtt::client().publish(handler().getStateTopic(), mideaACdefaultState, true);

  });

  mqtt::client().subscribe(hamdhMideaAC.getCtrlTopic(), [](const String &payload) {
    enqueue(payload, [](const String &payload) {
      StaticJsonDocument<100> doc;
      if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
      {
        StaticJsonDocument<100> feedbackDoc;
        String feedbackMesg;
        bool isOffCmd = false;
        if (doc.containsKey("acmidea_mode"))
        {
          String mode = doc["acmidea_mode"];
          controller().on();
          if (mode.equals("auto")) {
            controller().setMode(kMideaACAuto);
          }
          else if (mode.equals("cool")) {
            controller().setMode(kMideaACCool);
          }
          else if (mode.equals("dry")) {
            controller().setMode(kMideaACDry);
          }
          else if (mode.equals("fan_only")) {
            controller().setMode(kMideaACFan);
          }
          else if (mode.equals("heat")) {
            controller().setMode(kMideaACHeat);
          }
          else if (mode.equals("off")) {
            controller().off();
            isOffCmd = true;
          }
          feedbackDoc["acmidea_mode"] = mode;
        }
        if (doc.containsKey("acmidea_fan_mode"))
        {
          String fanMode = doc["acmidea_fan_mode"];
          if (fanMode.equals("low")) {
            controller().setFan(1);
          }
          else if (fanMode.equals("medium")) {
            controller().setFan(2);
          }
          else if (fanMode.equals("high")) {
            controller().setFan(3);
          }
          else if (fanMode.equals("auto")) {
            controller().setFan(0);
          }
          feedbackDoc["acmidea_fan_mode"] = fanMode;
        }
        if (doc.containsKey("acmidea_swing_mode"))
        {
          String swingMode = doc["acmidea_swing_mode"];
          if (swingMode.equals("off")) {
            controller().setSwingVToggle(false);
          }
          else if (swingMode.equals("vertical")) {
            controller().setSwingVToggle(true);
          }
          feedbackDoc["acmidea_swing_mode"] = swingMode;
        }
        if (doc.containsKey("acmidea_temperature"))
        {
          String tempStr = doc["acmidea_temperature"];
          float temp = tempStr.toFloat();
          int target = round(temp);
          controller().setTemp(target);
          feedbackDoc["acmidea_temperature"] = tempStr;
        }
        if(isOffCmd || ((!isOffCmd) && controller().getPower())){
          controller().send();
        }

        serializeJson(feedbackDoc, feedbackMesg);
        mqtt::client().publish(handler().getStateTopic(), feedbackMesg, true);
    } });
  });
}
}  // namespace midea
}  // namespace detail
}  // namespace legacy
}  // namespace homegear
#endif
