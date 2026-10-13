#include "HomeGearConfig.h"
#if (IR_MODULE && IR_HAIER_AC)
#include "LegacyRemoteInternals.h"
#include "MqttService.h"
#include "IrService.h"
#include <ArduinoJson.h>
#include <ir_Haier.h>

namespace homegear {
namespace legacy {
namespace detail {
namespace haier {
namespace {
HAMqttDiscoveryHandler &handler() {
  static HAMqttDiscoveryHandler value("homegear_ir", "ac001", "Haier", "Haier AC", SOFTWARE_VERSION, mqtt::discovery());
  return value;
}
IRHaierAC176 &controller() {
  static IRHaierAC176 value(ir::kIrLed);
  return value;
}

void setHaierACDefaultState()
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
  HAMqttDiscoveryClimate hamdhHaierAC(handler());
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
  mqtt::client().publish(hamdhHaierAC.getMqttDiscoveryConfigTopic(), hamdhHaierAC.getMqttDiscoveryMesg(), true);

  enqueue(String(), [](const String &) {
    setHaierACDefaultState();
    StaticJsonDocument<100> haierACdefaultStateDoc;
    String haierACdefaultState;
    haierACdefaultStateDoc["achaier_mode"] = "off";
    haierACdefaultStateDoc["achaier_swing_mode"] = "off";
    haierACdefaultStateDoc["achaier_temperature"] = "26";
    serializeJson(haierACdefaultStateDoc, haierACdefaultState);
    mqtt::client().publish(handler().getStateTopic(), haierACdefaultState, true);

  });

  mqtt::client().subscribe(hamdhHaierAC.getCtrlTopic(), [](const String &payload) {
    enqueue(payload, [](const String &payload) {
      StaticJsonDocument<100> doc;
      if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
      {
        StaticJsonDocument<100> feedbackDoc;
        String feedbackMesg;
        bool isOffCmd = false;
        if (doc.containsKey("achaier_mode"))
        {
          String mode = doc["achaier_mode"];
          controller().on();
          if (mode.equals("cool")) {
            controller().setMode(kHaierAcCool);
          }
          else if (mode.equals("heat")) {
            controller().setMode(kHaierAcHeat);
          }
          else if (mode.equals("off")) {
            controller().off();
            isOffCmd = true;
          }
          feedbackDoc["achaier_mode"] = mode;
        }
        if (doc.containsKey("achaier_swing_mode"))
        {
          String swingMode = doc["achaier_swing_mode"];
          if(swingMode.equals("off")){
            controller().setSwingV(kHaierAcYrw02SwingVOff);
          }else if(swingMode.equals("vertical")){
            controller().setSwingV(kHaierAcYrw02SwingVAuto);
          }

          feedbackDoc["achaier_swing_mode"] = swingMode;
        }
        if (doc.containsKey("achaier_temperature"))
        {
          String tempStr = doc["achaier_temperature"];
          float temp = tempStr.toFloat();
          int target = round(temp);
          controller().setTemp(target);
          feedbackDoc["achaier_temperature"] = tempStr;
        }
        if(isOffCmd || ((!isOffCmd) && controller().getPower())){
          controller().send();
        }

        serializeJson(feedbackDoc, feedbackMesg);
        mqtt::client().publish(handler().getStateTopic(), feedbackMesg, true);
    } });
  });
}
}  // namespace haier
}  // namespace detail
}  // namespace legacy
}  // namespace homegear
#endif
