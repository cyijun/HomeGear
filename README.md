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

### 串口配置服务器

启动时先尝试连接已保存的 Wi-Fi，初次连接等待上限为 10 秒；随后即使 Wi-Fi 未连接，也可使用串口配置和非阻塞配网门户。后台会使用已保存凭据恢复连接，不会关闭配网 AP；正在关联、等待 DHCP 或刚提交配网时会暂缓重试。115200 波特率，每条命令以换行结束：

```text
config show
config set mqtt_server 192.168.0.109
config apply
```

`config show` 显示暂存配置、当前生效的服务器地址及网络连接状态，也包含 Wi-Fi 状态、模式、最近断连原因和后台重试次数；密码和令牌只显示 `set/empty`。`config set <参数名> <值>` 暂存修改，`config save` 保存到 SPIFFS，`config apply` 保存成功后重启并应用。暂存或保存不会修改正在使用的 MQTT 连接参数；只改服务器地址会保留端口、账号、密码和令牌。保存使用临时文件和备份文件，文件系统不可用时报告错误，不自动格式化。

支持 `mqtt_server`、`mqtt_port`、`mqtt_user`、`mqtt_pwd`、`homeassistant_server`、`homeassistant_port`、`homeassistant_token`。服务器填写 IP 或主机名，不含 `http://`、路径或端口；端口范围为 1–65535。服务器最长 63 字节，用户名/密码最长 31 字节，令牌最长 255 字节。参数名后第一个空格分隔值，后面的空格属于值，密码的首尾空格会保留；使用 `config set mqtt_pwd ""` 可清空密码，令牌同理。

上述 IP 是 MQTT 配置示例，源码没有固定服务器地址。`homeassistant_server` 用于可选的 HA 直连接口，与 MQTT 参数独立；如需配置，单独使用 `config set homeassistant_server <地址>`，使用 `config set homeassistant_server ""` 可清空这项参数，MQTT 配置会保留。

### 编译与验证

完整固件已使用 ESP32 Core **2.0.17** 编译通过。固定库版本与作者库的 Git 提交见 [build-dependencies.json](build-dependencies.json)；原项目使用旧版 Arduino `String` MQTT API，不能直接替换为新版 ESP32MQTTClient。

安装指定核心和依赖后，可编译 `Software/HomeGear`，FQBN 为 `esp32:esp32:esp32`。模块开关仍由 `.ino` 顶部配置；当前默认启用温湿度/光照、空气质量、红外和 ZigBee，与实机模组一致。

原生 C++11 测试核对实测 ON/OFF 帧、反码、初始未知状态、重复绝对开关，以及异常控制消息：

```sh
c++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/hitachi_ac3_control_test.cpp -o /tmp/homegear-hitachi-ac3-test
/tmp/homegear-hitachi-ac3-test
c++ -std=c++11 -Wall -Wextra -Werror -pedantic tests/serial_config_test.cpp -o /tmp/homegear-serial-config-test
/tmp/homegear-serial-config-test
```

串口配置测试覆盖端口和长度边界、异常命令、密码空格、清空凭据，以及修改地址时保留其他参数。刷入已有设备前备份 flash，确认分区一致后只更新应用分区，以保留 Wi-Fi 和 SPIFFS 配置；启动或配置应用不会自动发送空调开关信号。

帧字段参考 [IRremoteESP8266 #1060](https://github.com/crankyoldgit/IRremoteESP8266/issues/1060) 及作者的 [原始捕获表](https://docs.google.com/spreadsheets/d/1x82Cb3-GtOMM5oTVOFdNY9mqwLnJ-qElhOj8MId7mrA/edit)。当前两条 25℃除湿帧均已在用户面板上验证。
