# HomeGear
An ESP32 based scalable smart home hardware stack

## 日立中央空调：25℃除湿开关

现已在用户的日立中央空调墙面控制器上实测 `HITACHI_AC3` 开机和关机帧。具体面板型号尚未确认；首版只开放已验证的 **25℃、除湿、低风开/关**。

默认启用 `IR_HITACHI_AC3`，关闭原海尔空调分支。发射引脚为 GPIO 4，非反相，载波 38 kHz。设备启动不发红外；软件在第一条明确命令前保持未知状态，MQTT 重连只重发最近命令的状态。

### Home Assistant / MQTT

Home Assistant MQTT Discovery 会生成 `hitachi_ac3_25c_dry` climate 实体，只提供 `off` 和 `dry` 两个模式。温度和风速固定为 25℃低风，不提供未经实测的调温或其他模式控件。

控制主题为 `homegear_ir/<chipId>_hitachi_ac3/ctrl`，使用以下两种消息，发送时不要启用 retain：

```json
{"achitachi_mode":"dry"}
```

上面表示以 25℃除湿低风开机；关机使用：

```json
{"achitachi_mode":"off"}
```

每条命令只发一次完整状态帧。消息只接受一个未转义的 `achitachi_mode` 键和字面量 `dry/off`，允许标准 JSON 空白；额外字段、重复键、尾随内容、转义、NUL、超过 128 字节或分片消息都会被拒绝。retained 控制消息不会被重放。

反馈主题为 `homegear_ir/<chipId>_hitachi_ac3/attr`。反馈是完整的 retained JSON，例如：

```json
{"achitachi_mode":"dry","achitachi_temperature":25,"achitachi_fan_mode":"low","achitachi_power":true,"state_source":"commanded"}
```

这是最近发送的命令记录，红外控制没有空调状态回读。MQTT 与串口命令进入同一队列，由主循环按顺序发送，避免帧交叠及状态发布乱序。

### 串口

115200 波特率，命令以换行结束：

- `hitachi_on`：25℃除湿低风开机。
- `hitachi_off`：保持这组模式和温度关机。
- `hitachi_status`：只查询软件的最近命令状态；启动时报告 `unknown`。

### 编译与验证

完整固件已使用 ESP32 Core **2.0.17** 编译通过。固定库版本与作者库的 Git 提交见 [build-dependencies.json](build-dependencies.json)；原项目使用旧版 Arduino `String` MQTT API，不能直接替换为新版 ESP32MQTTClient。

安装指定核心和依赖后，可编译 `Software/HomeGear`，FQBN 为 `esp32:esp32:esp32`。模块开关仍由 `.ino` 顶部配置；插入 ZigBee 模组后，需要按实际用途启用 `ZGB_MODULE`。

原生 C++11 测试核对实测 ON/OFF 帧、反码、初始未知状态、重复绝对开关，以及异常控制消息：

```sh
c++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/hitachi_ac3_control_test.cpp -o /tmp/homegear-hitachi-ac3-test
/tmp/homegear-hitachi-ac3-test
```

帧字段参考 [IRremoteESP8266 #1060](https://github.com/crankyoldgit/IRremoteESP8266/issues/1060) 及作者的 [原始捕获表](https://docs.google.com/spreadsheets/d/1x82Cb3-GtOMM5oTVOFdNY9mqwLnJ-qElhOj8MId7mrA/edit)。当前两条 25℃除湿帧均已在用户面板上验证。
