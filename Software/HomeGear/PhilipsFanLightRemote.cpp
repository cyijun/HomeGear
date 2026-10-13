#include "HomeGearConfig.h"
#if (RF_MODULE && RF_PHILIPS_FAN_LIGHT)
#include "LegacyRemoteInternals.h"
#include "MqttService.h"
#include "RF_Code.h"
#include <ArduinoJson.h>

namespace homegear {
namespace legacy {
namespace detail {
namespace philips {
namespace {
HAMqttDiscoveryHandler &handler() {
  static HAMqttDiscoveryHandler value("homegear_rf", "00001", "Philips", "Philips RF Remote Fan Light", SOFTWARE_VERSION, mqtt::discovery());
  return value;
}
}  // namespace
void begin() { (void)handler(); }
void onConnected() {
  HAMqttDiscoveryFan hamdhPFLF(handler());
  HAMqttDiscoveryLight hamdhPFLL(handler());

  hamdhPFLF.setDeviceNameMin("philips_fanlight_fan");
  hamdhPFLF.setCommandTemplate("{\"pflf_state\": \"{{ value }}\"}");
  hamdhPFLF.setPercentageCommandTemplate("{\"pflf_percentage\": {{ value }}}");
  hamdhPFLF.setSpeedRangeMin(1);
  hamdhPFLF.setSpeedRangeMax(3);
  hamdhPFLF.setStateValueTemplate("{{value_json.pflf_state}}");
  hamdhPFLF.setPercentageValueTemplate("{{value_json.pflf_percentage}}");
  hamdhPFLF.construct();
  mqtt::client().publish(hamdhPFLF.getMqttDiscoveryConfigTopic(), hamdhPFLF.getMqttDiscoveryMesg(), true);

  hamdhPFLL.setDeviceNameMin("philips_fanlight_light");
  hamdhPFLL.setPayloadOn("{\"pfll_state\":\"ON\"}");
  hamdhPFLL.setPayloadOff("{\"pfll_state\":\"OFF\"}");
  hamdhPFLL.setStateTemplate("{{ value_json.pfll_state }}");
  hamdhPFLL.construct();
  mqtt::client().publish(hamdhPFLL.getMqttDiscoveryConfigTopic(), hamdhPFLL.getMqttDiscoveryMesg(), true);

  mqtt::client().subscribe(handler().getCtrlTopic(), [](const String &payload) {
    enqueue(payload, [](const String &payload) {
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
            rf::send(philips_fanlight_fan_low);
            feedbackDoc["pflf_percentage"] = 1;
          }
          else if (pflf_state.equals("OFF"))
          {
            rf::send(philips_fanlight_fan_off);
            feedbackDoc["pflf_percentage"] = 0;
          }
        }
        if (doc.containsKey("pflf_percentage"))
        {
          int val = doc["pflf_percentage"];
          feedbackDoc["pflf_percentage"] = val;
          if (val == 0) {
            rf::send(philips_fanlight_fan_off);
            feedbackDoc["pflf_state"] = "OFF";
          } else if (val == 1) {
            rf::send(philips_fanlight_fan_low);
            feedbackDoc["pflf_state"] = "ON";
          } else if (val == 2) {
            rf::send(philips_fanlight_fan_mid);
            feedbackDoc["pflf_state"] = "ON";
          } else if (val == 3) {
            rf::send(philips_fanlight_fan_high);
            feedbackDoc["pflf_state"] = "ON";
          }
        }
        if (doc.containsKey("pfll_state"))
        {
          String pfll_state = doc["pfll_state"];
          feedbackDoc["pfll_state"] = pfll_state;
          if (pfll_state.equals("ON")) {
            rf::send(philips_fanlight_light_on);
          }
          else if (pfll_state.equals("OFF")) {
            rf::send(philips_fanlight_light_off);
          }
        }
        serializeJson(feedbackDoc, feedbackMesg);
        mqtt::client().publish(handler().getStateTopic(), feedbackMesg, true);
    } });
  });
}

}  // namespace philips
}  // namespace detail
}  // namespace legacy
}  // namespace homegear
#endif
