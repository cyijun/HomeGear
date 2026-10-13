/**
 *  @file HomeGear.ino
 *  @author Chen Yijun
 *  @brief
 *  @version 1.1
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
#include "HomeGearConfig.h"
#include "DeviceInfo.h"
#include "ConfigStore.h"
#include "WifiService.h"
#include "SerialConsole.h"
#include "MqttService.h"
#include "IrService.h"
#include "LegacyRemotes.h"
#include "SensorService.h"
#include "ZigbeeBridge.h"
#include "PowerPolicy.h"

void setup()
{
  Serial.begin(115200);
  Serial.println();
  homegear::power::begin();
  homegear::device::begin();
  homegear::config::begin();
  homegear::sensors::begin();
  homegear::ir::begin();
  homegear::legacy::begin();
  homegear::zigbee::beginSerial();
  homegear::wifi::begin();
  // lwIP must be initialized before the TCP bridge opens its listener.
  homegear::zigbee::beginNetwork();
  homegear::mqtt::begin();
}

void loop()
{
  homegear::power::beginLoop();
  homegear::wifi::tick();
  homegear::serial::tick();
  homegear::mqtt::tick();
  homegear::ir::tick();
  homegear::legacy::tick();
  homegear::sensors::tick(millis());
  homegear::power::endLoop();
}
