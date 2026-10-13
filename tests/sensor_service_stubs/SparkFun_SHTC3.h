#pragma once
#include "Arduino.h"
enum SHTC3_Status_TypeDef { SHTC3_Status_Nominal, SHTC3_Status_Error };
enum SHTC3_MeasurementModes_TypeDef { SHTC3_CMD_CSE_RHF_NPM };
class SHTC3 {
 public:
  bool passTcrc = false, passRHcrc = false;
  SHTC3_Status_TypeDef begin() { return sensor_mock::shtHealthy ? SHTC3_Status_Nominal : SHTC3_Status_Error; }
  void setMode(SHTC3_MeasurementModes_TypeDef) {}
  SHTC3_Status_TypeDef update() {
    ++sensor_mock::shtReads;
    passTcrc = passRHcrc = sensor_mock::crcHealthy;
    return sensor_mock::shtHealthy ? SHTC3_Status_Nominal : SHTC3_Status_Error;
  }
  float toDegC() const { return sensor_mock::temperature; }
  float toPercent() const { return 50.0f; }
  void sleep(bool) {}
};
