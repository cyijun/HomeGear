#pragma once

//***** Config *****

#ifndef PLATFORM
#define PLATFORM "HomeGear"
#endif
#ifndef HOMEGEAR_SERIAL_NUMBER
#define HOMEGEAR_SERIAL_NUMBER homegear::device::chipId()
#endif
#ifndef MANUFACTURER
#define MANUFACTURER "Chen Yijun"
#endif
#ifndef DEVICE_MODEL
#define DEVICE_MODEL "HomeGear Scalable Hardware Stack"
#endif
#ifndef SOFTWARE_VERSION
#define SOFTWARE_VERSION "HomeGear 1.1"
#endif

#ifndef THI_MODULE
#define THI_MODULE true
#endif
#ifndef IAQ_MODULE
#define IAQ_MODULE true
#endif
#ifndef IR_MODULE
#define IR_MODULE true
#endif
#ifndef RF_MODULE
#define RF_MODULE false
#endif
#ifndef ZGB_MODULE
#define ZGB_MODULE true
#endif

#ifndef SERIAL_DEBUG
#define SERIAL_DEBUG true
#endif

#ifndef IR_GREE_AC
#define IR_GREE_AC false
#endif
#ifndef IR_SINGFUN_FAN
#define IR_SINGFUN_FAN false
#endif
#ifndef IR_MIDEA_AC
#define IR_MIDEA_AC false
#endif
#ifndef IR_COOLIX_AC
#define IR_COOLIX_AC false
#endif
#ifndef IR_HAIER_AC
#define IR_HAIER_AC false
#endif
#ifndef IR_HITACHI_AC3
#define IR_HITACHI_AC3 true
#endif
#ifndef RF_PHILIPS_FAN_LIGHT
#define RF_PHILIPS_FAN_LIGHT false
#endif
