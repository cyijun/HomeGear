#include "HomeGearConfig.h"
#if (IR_MODULE && IR_COOLIX_AC)
#include "LegacyRemoteInternals.h"
#include "MqttService.h"
#include "IrService.h"
#include <ArduinoJson.h>
#include <ir_Coolix.h>

namespace homegear {
namespace legacy {
namespace detail {
namespace coolix {
namespace {
HAMqttDiscoveryHandler &handler() {
  static HAMqttDiscoveryHandler value("homegear_ir", "ac001", "Coolix", "Coolix AC", SOFTWARE_VERSION, mqtt::discovery());
  return value;
}
IRCoolixAC &controller() {
  static IRCoolixAC value(ir::kIrLed);
  return value;
}

void setCoolixACDefaultState()
{
  controller().off();
  controller().setTemp(26); // 16-30C
}
}  // namespace

void begin() {
  (void)handler();
  controller().begin();
}

void onConnected() {
  HAMqttDiscoveryClimate hamdhCoolixAC(handler());
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
  mqtt::client().publish(hamdhCoolixAC.getMqttDiscoveryConfigTopic(), hamdhCoolixAC.getMqttDiscoveryMesg(), true);

  enqueue(String(), [](const String &) {
    setCoolixACDefaultState();
    StaticJsonDocument<100> coolixACdefaultStateDoc;
    String coolixACdefaultState;
    coolixACdefaultStateDoc["accoolix_mode"] = "off";
    coolixACdefaultStateDoc["accoolix_swing_mode"] = "off";
    coolixACdefaultStateDoc["accoolix_temperature"] = "26";
    serializeJson(coolixACdefaultStateDoc, coolixACdefaultState);
    mqtt::client().publish(handler().getStateTopic(), coolixACdefaultState, true);

  });

  mqtt::client().subscribe(hamdhCoolixAC.getCtrlTopic(), [](const String &payload) {
    enqueue(payload, [](const String &payload) {
      StaticJsonDocument<100> doc;
      if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
      {
        StaticJsonDocument<100> feedbackDoc;
        String feedbackMesg;
        bool isOffCmd = false;
        if (doc.containsKey("accoolix_mode"))
        {
          String mode = doc["accoolix_mode"];
          controller().on();
          if (mode.equals("cool")) {
            controller().setMode(kCoolixCool);
          }
          else if (mode.equals("heat")) {
            controller().setMode(kCoolixHeat);
          }
          else if (mode.equals("off")) {
            controller().off();
            isOffCmd = true;
          }
          feedbackDoc["accoolix_mode"] = mode;
        }
        if (doc.containsKey("accoolix_swing_mode"))
        {
          String swingMode = doc["accoolix_swing_mode"];
          controller().setSwing();
          feedbackDoc["accoolix_swing_mode"] = swingMode;
        }
        if (doc.containsKey("accoolix_temperature"))
        {
          String tempStr = doc["accoolix_temperature"];
          float temp = tempStr.toFloat();
          int target = round(temp);
          controller().setTemp(target);
          feedbackDoc["accoolix_temperature"] = tempStr;
        }
        if(isOffCmd || ((!isOffCmd) && controller().getPower())){
          controller().send();
        }

        serializeJson(feedbackDoc, feedbackMesg);
        mqtt::client().publish(handler().getStateTopic(), feedbackMesg, true);
    } });
  });
}
}  // namespace coolix
}  // namespace detail
}  // namespace legacy
}  // namespace homegear
#endif
