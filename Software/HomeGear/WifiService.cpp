#include "WifiService.h"
#include "HomeGearConfig.h"
#include "DeviceInfo.h"
#include "ConfigStore.h"
#include "WifiPortalPolicy.h"
#include <WiFiManager.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
namespace homegear { namespace wifi {
namespace {
bool shouldSaveConfig = false;
WifiPortalPolicy portalPolicy;
WiFiManager wifiManager;
const uint32_t kWiFiRetryIntervalMs = 30000;
const uint32_t kWiFiAttemptLimitMs = 45000;
struct WiFiRecoveryEvent
{
  uint32_t eventId;
  uint32_t atMs;
  uint8_t reason;
};
QueueHandle_t wifiRecoveryQueue = nullptr;
uint8_t wifiLastDisconnectReason = 0;
uint32_t wifiRetryCount = 0;
uint32_t wifiLastRetryMs = 0;
uint32_t wifiAttemptStartedMs = 0;
bool wifiAttemptPending = false;
bool wifiAssociated = false;
bool wifiInitialRetry = false;
WiFiManagerParameter custom_mqtt_server("mqttserver", "MQTT Server", "", HomeGearSerialConfig::kHostCapacity);
WiFiManagerParameter custom_mqtt_port("mqttport", "MQTT Port", "", HomeGearSerialConfig::kPortCapacity);
WiFiManagerParameter custom_mqtt_user("mqttuser", "MQTT Username", "", HomeGearSerialConfig::kCredentialCapacity);
WiFiManagerParameter custom_mqtt_pwd("mqttpwd", "MQTT Password", "", HomeGearSerialConfig::kCredentialCapacity, "type='password'");
WiFiManagerParameter custom_homeassistant_server("homeassistantserver", "Home Assistant Server", "", HomeGearSerialConfig::kHostCapacity);
WiFiManagerParameter custom_homeassistant_port("homeassistantport", "Home Assistant Port", "", HomeGearSerialConfig::kPortCapacity);
WiFiManagerParameter custom_homeassistant_token("homeassistanttoken", "Home Assistant Token", "", HomeGearSerialConfig::kTokenCapacity, "type='password'");


// The ESP32 event task only queues diagnostics; Wi-Fi recovery runs in loop().
void recordWiFiRecoveryEvent(WiFiEvent_t event, WiFiEventInfo_t info)
{
  switch (event)
  {
    case ARDUINO_EVENT_WIFI_STA_START:
    case ARDUINO_EVENT_WIFI_STA_STOP:
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      break;
    default:
      return;
  }
  if (!wifiRecoveryQueue) return;
  WiFiRecoveryEvent update = {static_cast<uint32_t>(event), millis(),
      event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED ? info.wifi_sta_disconnected.reason : static_cast<uint8_t>(0)};
  if (xQueueSend(wifiRecoveryQueue, &update, 0) != pdTRUE)
  {
    // Keep recent events if a burst fills the bounded diagnostic queue.
    WiFiRecoveryEvent discarded;
    xQueueReceive(wifiRecoveryQueue, &discarded, 0);
    xQueueSend(wifiRecoveryQueue, &update, 0);
  }
}

void initializeWiFiRecovery()
{
  wifiRecoveryQueue = xQueueCreate(16, sizeof(WiFiRecoveryEvent));
  if (!wifiRecoveryQueue) Serial.println("WIFI ERROR EVENT_QUEUE_ALLOCATION_FAILED");
  else WiFi.onEvent(recordWiFiRecoveryEvent);
  wifiLastRetryMs = millis();
}

void protectWiFiPortalSave()
{
  // HTTP save and its connection attempt span successive process() calls.
  portalPolicy.protectSave(millis());
}

bool hasSavedWiFiSettings()
{
  // Check only presence; never construct or log credential strings.
  wifi_config_t saved = {};
  return esp_wifi_get_config(WIFI_IF_STA, &saved) == ESP_OK && saved.sta.ssid[0] != 0;
}

void processWiFiRecovery()
{
  WiFiRecoveryEvent update;
  while (wifiRecoveryQueue && xQueueReceive(wifiRecoveryQueue, &update, 0) == pdTRUE)
  {
    Serial.printf("WIFI EVENT id=%lu reason=%u ms=%lu\n",
        static_cast<unsigned long>(update.eventId), update.reason,
        static_cast<unsigned long>(update.atMs));
    switch (update.eventId)
    {
      case ARDUINO_EVENT_WIFI_STA_START:
        portalPolicy.disconnected();
        wifiAssociated = false;
        wifiAttemptPending = true;
        wifiAttemptStartedMs = update.atMs;
        break;
      case ARDUINO_EVENT_WIFI_STA_STOP:
        portalPolicy.disconnected();
        wifiAssociated = false;
        wifiAttemptPending = false;
        break;
      case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      case ARDUINO_EVENT_WIFI_STA_LOST_IP:
        if (update.eventId == ARDUINO_EVENT_WIFI_STA_LOST_IP) portalPolicy.disconnected();
        wifiAssociated = true;
        wifiAttemptPending = true;
        wifiAttemptStartedMs = update.atMs;
        break;
      case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
        wifiLastDisconnectReason = update.reason;
        portalPolicy.disconnected();
        wifiAssociated = false;
        // Core auto-reconnect may already be associating again after this event.
        wifiAttemptPending = true;
        wifiAttemptStartedMs = update.atMs;
        break;
      case ARDUINO_EVENT_WIFI_STA_GOT_IP:
        wifiAssociated = true;
        wifiAttemptPending = false;
        break;
    }
  }

  const uint32_t now = millis();
  const bool saveGuard = portalPolicy.saveGuardActive(now);
  const wl_status_t status = WiFi.status();
  const bool staEnabled = (WiFi.getMode() & WIFI_STA) != 0;
  // Current core bits also protect DHCP if a diagnostic event was dropped.
  wifiAssociated = staEnabled && (WiFi.getStatusBits() & STA_CONNECTED_BIT) != 0;
  if (status == WL_CONNECTED)
  {
    wifiAttemptPending = false;
    wifiInitialRetry = false;
    if (portalPolicy.canReclaim(now, true, (WiFi.getMode() & WIFI_AP) != 0, WiFi.softAPgetStationNum())) {
      // shutdownConfigPortal() invokes handleClient() again. Close its public
      // listener first so a new /wifisave cannot be accepted during shutdown.
      if (wifiManager.server && wifiManager.server->client().connected()) return;
      if (wifiManager.getConfigPortalActive()) {
        if (wifiManager.server) wifiManager.server->stop();
        wifiManager.stopConfigPortal(); // autoConnect set _usermode to WIFI_STA.
      }
      if (WiFi.status() == WL_CONNECTED && (WiFi.getMode() & WIFI_AP)) WiFi.enableAP(false);
      Serial.printf("WIFI PORTAL_CLOSED mode=%u connected=%u\n",
          static_cast<unsigned int>(WiFi.getMode()), WiFi.status() == WL_CONNECTED ? 1U : 0U);
    }
    return;
  }
  portalPolicy.disconnected();
  if (!staEnabled) wifiAttemptPending = false;
  else if ((wifiAssociated || status == WL_IDLE_STATUS) && !wifiAttemptPending)
  {
    wifiAttemptPending = true;
    wifiAttemptStartedMs = now;
  }

  if (saveGuard) return;
  // Allow association/DHCP to progress, but bound a stalled attempt.
  if (wifiAttemptPending && static_cast<uint32_t>(now - wifiAttemptStartedMs) < kWiFiAttemptLimitMs) return;
  if (!wifiInitialRetry && static_cast<uint32_t>(now - wifiLastRetryMs) < kWiFiRetryIntervalMs) return;
  if (!hasSavedWiFiSettings())
  {
    wifiInitialRetry = false;
    wifiLastRetryMs = now;
    return;
  }

  wifiInitialRetry = false;
  wifiLastRetryMs = now;
  wifiAttemptStartedMs = now;
  wifiAttemptPending = true;
  ++wifiRetryCount;
  // begin() adds STA to the existing AP mode and uses saved credentials.
  // Avoid disconnect/reconnect or a second autoConnect/config portal.
  const wl_status_t result = WiFi.begin();
  Serial.printf("WIFI RETRY count=%lu status=%u mode=%u\n",
      static_cast<unsigned long>(wifiRetryCount), static_cast<unsigned int>(result),
      static_cast<unsigned int>(WiFi.getMode()));
}

void saveConfigCallback()
{
  shouldSaveConfig = true;
}

void syncPortalConfig()
{
  const auto &pendingConfig = config::pending();
  custom_mqtt_server.setValue(pendingConfig.mqtt_server, HomeGearSerialConfig::kHostCapacity);
  custom_mqtt_port.setValue(pendingConfig.mqtt_port, HomeGearSerialConfig::kPortCapacity);
  custom_mqtt_user.setValue(pendingConfig.mqtt_user, HomeGearSerialConfig::kCredentialCapacity);
  custom_mqtt_pwd.setValue(pendingConfig.mqtt_pwd, HomeGearSerialConfig::kCredentialCapacity);
  custom_homeassistant_server.setValue(pendingConfig.homeassistant_server, HomeGearSerialConfig::kHostCapacity);
  custom_homeassistant_port.setValue(pendingConfig.homeassistant_port, HomeGearSerialConfig::kPortCapacity);
  custom_homeassistant_token.setValue(pendingConfig.homeassistant_token, HomeGearSerialConfig::kTokenCapacity);
}

void wmStart()
{
  syncPortalConfig();
  wifiManager.setDebugOutput(false);
  wifiManager.setConfigPortalBlocking(false);
  // Allow association and DHCP to finish before falling back to the portal.
  wifiManager.setConnectTimeout(10);
  wifiManager.setPreSaveConfigCallback(protectWiFiPortalSave);
  wifiManager.setPreSaveParamsCallback(protectWiFiPortalSave);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.setSaveParamsCallback(saveConfigCallback);
#if ZGB_MODULE
  wifiManager.setHostname("HomeGear-ZigBee");
#endif
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pwd);
  wifiManager.addParameter(&custom_homeassistant_server);
  wifiManager.addParameter(&custom_homeassistant_port);
  wifiManager.addParameter(&custom_homeassistant_token);
  wifiManager.autoConnect(device::name(), "homegearconfig");
  // A portal fallback has disabled STA; safely restart saved STA once in loop().
  wifiInitialRetry = WiFi.status() != WL_CONNECTED && (WiFi.getMode() & WIFI_STA) == 0;
  Serial.println("CONFIG SERIAL_READY");
}

}
void begin() { initializeWiFiRecovery(); wmStart(); }
void syncConfig() { syncPortalConfig(); }
void tick() {
  wifiManager.process();
  processWiFiRecovery();
  if (shouldSaveConfig) {
    shouldSaveConfig = false;
    const char *values[] = {custom_mqtt_server.getValue(), custom_mqtt_port.getValue(),
        custom_mqtt_user.getValue(), custom_mqtt_pwd.getValue(), custom_homeassistant_server.getValue(),
        custom_homeassistant_port.getValue(), custom_homeassistant_token.getValue()};
    if (config::stagePortal(values)) config::save();
  }
}
void printStatus() {
  Serial.printf("CONFIG wifi_connected=%s\n", WiFi.status() == WL_CONNECTED ? "true" : "false");
  Serial.printf("CONFIG wifi_status=%u\n", static_cast<unsigned int>(WiFi.status()));
  Serial.printf("CONFIG wifi_mode=%u\n", static_cast<unsigned int>(WiFi.getMode()));
  Serial.printf("CONFIG wifi_last_disconnect_reason=%u\n", wifiLastDisconnectReason);
  Serial.printf("CONFIG wifi_portal_active=%u\n", wifiManager.getConfigPortalActive() ? 1U : 0U);
  Serial.printf("CONFIG wifi_retry_count=%lu\n", static_cast<unsigned long>(wifiRetryCount));
  Serial.printf("CONFIG local_ip=%s\n", WiFi.localIP().toString().c_str());
}
}}
