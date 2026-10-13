#include "HomeGearConfig.h"
#if (IR_MODULE && IR_SINGFUN_FAN)
#include "LegacyRemoteInternals.h"
#include "MqttService.h"
#include "IrService.h"
#include <ArduinoJson.h>
#include "IR_Code.h"

namespace homegear {
namespace legacy {
namespace detail {
namespace singfun {
namespace {
HAMqttDiscoveryHandler &handler() {
  static HAMqttDiscoveryHandler value("homegear_ir", "00001", "SingFun", "SingFun Floor Fan", SOFTWARE_VERSION, mqtt::discovery());
  return value;
}
}  // namespace

void begin() {
  (void)handler();
}

void onConnected() {
  HAMqttDiscoveryFan hamdhSFF(handler());
  hamdhSFF.setDeviceNameMin("singfun_fan");
  hamdhSFF.setCommandTemplate("{\"sff_state\": \"{{ value }}\"}");
  hamdhSFF.setOscillationCommandTemplate("{\"sff_oscillation\": \"{{ value }}\"}");
  hamdhSFF.setStateValueTemplate("{{value_json.sff_state}}");
  hamdhSFF.setOscillationValueTemplate("{{value_json.sff_oscillation}}");
  hamdhSFF.construct();
  mqtt::client().publish(hamdhSFF.getMqttDiscoveryConfigTopic(), hamdhSFF.getMqttDiscoveryMesg(), true);

  mqtt::client().subscribe(hamdhSFF.getCtrlTopic(), [](const String &payload) {
    enqueue(payload, [](const String &payload) {
      StaticJsonDocument<100> doc;
      if (deserializeJson(doc, payload) == ARDUINOJSON_NAMESPACE::DeserializationError::Code::Ok)
      {
        StaticJsonDocument<100> feedbackDoc;
        String feedbackMesg;
        if (doc.containsKey("sff_state")) {
          String sffOnOff = doc["sff_state"];
          ir::sender().sendRaw(singfun_fan_power, 75, 38);
          feedbackDoc["sff_state"] = sffOnOff;
          if (sffOnOff.equals("OFF")) {
            feedbackDoc["sff_oscillation"] = "oscillate_off";
          }
        }
        if (doc.containsKey("sff_oscillation")) {
          String sffOsciOnOff = doc["sff_oscillation"];
          ir::sender().sendRaw(singfun_fan_oscillating, 75, 38);
          feedbackDoc["sff_oscillation"] = sffOsciOnOff;
        }
        serializeJson(feedbackDoc, feedbackMesg);
        mqtt::client().publish(handler().getStateTopic(), feedbackMesg, true);
    } });
  });
}
}  // namespace singfun
}  // namespace detail
}  // namespace legacy
}  // namespace homegear
#endif
