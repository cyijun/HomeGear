# HomeGear
An ESP32 based scalable smart home hardware stack

## 1.1：模块拆分与运行功耗优化

`HomeGear.ino` 只保留启动顺序和循环调度；编译开关集中在 `Software/HomeGear/HomeGearConfig.h`。配置、配网、MQTT/HA、串口、传感器、红外、旧遥控协议和 ZigBee 转发分别由独立模块负责。

代码按职责组织，新增功能时优先扩展对应模块：

| 文件 | 职责 |
| --- | --- |
| `HomeGear.ino` / `HomeGearConfig.h` | 启动顺序、循环调度 / 编译开关 |
| `ConfigStore` / `SerialConfig` / `SerialConsole` | 配置持久化 / 命令解析 / 串口交互 |
| `WifiService` / `WifiPortalPolicy` | Wi-Fi 恢复与配网门户 / 门户回收条件 |
| `MqttService` / `DeviceInfo` | MQTT 连接与系统命令 / 设备标识 |
| `SensorService` / `SensorSnapshot` / `TrackedBsec` | 传感器采样和发布 / 快照与上报策略 / BSEC 输出跟踪 |
| `IrService` / `HitachiAc3Control` | 日立命令队列与发送 / 已验证的帧及解析 |
| `LegacyRemotes` 与各品牌遥控模块 | 旧协议的统一队列和调度 / 各品牌的发现、解析与控制 |
| `PowerPolicy` | 常态时钟、循环等待及红外发送期间的时钟恢复 |
| `ZigbeeBridge` / `ZigbeeBridgeBuffer` | 独立任务中的 TCP/UART 转发 / 有界缓冲 |

旧遥控协议按格力、美的、Coolix、海尔、先锋风扇和飞利浦风扇灯分别放在独立 `.cpp` 中；RF 串口传输也独立管理。MQTT 回调只把旧协议命令加入同一个有界队列，由主循环顺序执行。每个品牌通过自己的编译开关启用；默认仍只启用实测的日立协议。传感器、配网和配置保存保留各自完整的状态管理，避免跨文件共享可变状态。

- CPU 常态运行在 **160 MHz**，主循环每轮等待 **2 ms**。日立等红外发射时临时恢复 **240 MHz**，沿用已验证的库载波时序，发射后恢复；启动及 MQTT 重连不自动发射红外。
- Wi-Fi 稳定连接 30 秒，且没有配网客户端、HTTP 请求或最近 45 秒内的配置提交时，自动关闭遗留配网 AP。Wi-Fi 保持 `MIN_MODEM` 省电和 MQTT 在线；当前锁定核心未开启自动 Light-sleep。
- ZigBee TCP `1234` 转发使用独立任务，在双核 ESP32 上与主循环分属两核。UART2 接收/发送缓冲为 4096/1024 字节，双向各有 4096 字节队列，保留部分写入并处理背压。移除了会清空 TCP 接收数据的 `WiFiClient.flush()`；网络长期不读取时仍受有限缓冲限制。
- SHTC3 保持普通精度、2 秒采样、测量后休眠，与两颗 BH1750 独立调度；光照模组故障不会阻止温湿度采样。BME680 加载与硬件供电相符的 **3.3V / 3 秒 / LP** BSEC 配置，保留空气质量的 6 分钟预热，并报告算法准确度。
- 普通传感器变化合并为最短 **15 秒**上报，心跳为 **300 秒**；首次有效数据、MQTT 重连和 `pubdata` 请求可立即上报。每次发布完整的已知读数，消息与 HA Discovery 使用正确的 retain 参数；发布失败不会丢弃待发送变化。

115200 串口新增只读诊断命令：

```text
power status
sensors status
zigbee status
```

分别显示实际 CPU 频率和循环工作/等待时间、传感器读数与 BSEC 状态、ZigBee 连接与转发计数。循环工作百分比描述主循环的时间分配，不代表整机功耗；本轮不设置未经对照测量的固定温度偏移。`thim_*` 是 SHTC3 数据，`iaqm_*` 是 BME680/BSEC 数据。稳压器、气体传感器加热和外壳热耦合仍需在实际摆放环境中用参考温湿度计检查。

## 日立中央空调：除湿开关与目标温度

现已在用户的日立中央空调墙面控制器上实测 `HITACHI_AC3` 的 **25℃、除湿、低风开/关**。目标温度扩展为 **17–30℃、1℃步进**，编码来自 PC-LH3B 原始捕获表；当前墙面控制器的调温响应仍待现场实测，其他模式和风速未开放。启动默认目标为 25℃。

默认启用 `IR_HITACHI_AC3`，关闭原海尔空调分支。发射引脚为 GPIO 4，非反相，载波 38 kHz。设备启动不发红外；软件在第一条明确命令前保持未知状态，MQTT 重连只重发最近命令的状态。

### Home Assistant / MQTT

Home Assistant MQTT Discovery 会生成 climate 实体，提供 `off`、`dry` 两个模式和目标温度控件，风速固定低风。保留原 `hitachi_ac3_25c_dry` Discovery topic 和 unique ID，让已有实体及 HomeKit 桥接继续使用；显示名称更新为 `Hitachi AC3 Dry`。

控制主题为 `homegear_ir/<chipId>_hitachi_ac3/ctrl`，使用以下两种消息，发送时不要启用 retain：

```json
{"achitachi_mode":"dry"}
```

上面表示以当前目标温度除湿低风开机；关机使用：

```json
{"achitachi_mode":"off"}
```

调温使用独立消息，例如：

```json
{"achitachi_temperature":24}
```

温度可为 JSON 数字或数字字符串。范围内的小数按四舍五入变成整数℃，并反馈实际采用的整数目标，以兼容 HomeKit 的 0.1℃滑动精度；原值低于 17℃或高于 30℃会被拒绝。未知电源状态时只暂存目标，不发送红外；电源状态已知时发送完整状态帧，保留最近命令中的电源、除湿和低风设置。开关命令也保留当前目标温度。

消息只接受一个未转义的 `achitachi_mode` 键及字面量 `dry/off`，或一个 `achitachi_temperature` 键及十进制温度；允许标准 JSON 空白。额外字段、重复键、尾随内容、转义、指数、NUL、超过 128 字节或分片消息都会被拒绝。retained 控制消息不会被重放。

反馈主题为 `homegear_ir/<chipId>_hitachi_ac3/attr`。反馈是完整的 retained JSON，例如：

```json
{"achitachi_mode":"dry","achitachi_temperature":25,"achitachi_fan_mode":"low","achitachi_power":true,"state_source":"commanded","achitachi_current_temperature":null,"current_temperature_source":"unavailable"}
```

这是软件的目标及最近命令记录，红外控制没有空调状态回读。启动会发布 `unknown` 模式、`null` 电源和 25℃默认目标，重连重发当前软件快照，均不发红外。调温时完整状态帧中的电源来自最近的软件命令；如果之后用墙面面板改变了电源，软件无法知道，下一次调温可能恢复软件记录的开关状态。MQTT 与串口命令进入同一队列，由主循环按顺序执行，避免帧交叠及状态发布乱序。

`achitachi_temperature` 是目标设定，`achitachi_current_temperature` 是当前室温，后者明确为 `null`，不借用目标或 HomeGear 的传感器读数。HA 通过 `current_temperature_template: "{{ 'None' }}"` 清空当前室温；`initial: 25` 和目标温度 state topic 避免目标退回 HA 的 21℃默认值。HA MQTT climate 的空值必须渲染为字符串 `None`，空字符串或裸 `null` 不会清空。[HA 官方说明](https://www.home-assistant.io/integrations/climate.mqtt/)

HA 内置 HomeKit Bridge 的 `CurrentTemperature` 初值为 21℃，只在收到有效数字时更新；室温未知时 HomeKit 可能继续显示 21℃或最后一个有效值。普通 MQTT Discovery 不能把这个 HomeKit 特征改成空值，本项目保留 HA 中的真实未知状态。[HomeKit Bridge 稳定版实现](https://github.com/home-assistant/core/blob/2026.10.1/homeassistant/components/homekit/climate_base.py#L153)

### 串口

115200 波特率，命令以换行结束：

- `hitachi_on`：以当前目标温度除湿低风开机。
- `hitachi_off`：保持这组模式和温度关机。
- `hitachi_temp 24`：设置整数目标温度（17–30℃）；未知电源状态时只暂存。
- `hitachi_status`：只查询软件的最近命令状态；启动时报告 `unknown`。

### 串口配置服务器

启动时先尝试连接已保存的 Wi-Fi，初次连接等待上限为 10 秒；随后即使 Wi-Fi 未连接，也可使用串口配置和非阻塞配网门户。后台会使用已保存凭据恢复连接；正在关联、等待 DHCP 或刚提交配网时会暂缓重试。连接稳定后按上述条件回收配网 AP；更换到无法连接的网络后，重启设备会重新开放配网门户。115200 波特率，每条命令以换行结束：

```text
config show
config set mqtt_server 192.168.0.109
config apply
```

`config show` 显示暂存配置、当前生效的服务器地址及网络连接状态，也包含 Wi-Fi 状态、模式、最近断连原因和后台重试次数；密码和令牌只显示 `set/empty`。`config set <参数名> <值>` 暂存修改，`config save` 保存到 SPIFFS，`config apply` 保存成功后重启并应用。暂存或保存不会修改正在使用的 MQTT 连接参数；只改服务器地址会保留端口、账号、密码和令牌。保存使用临时文件和备份文件，文件系统不可用时报告错误，不自动格式化。

支持 `mqtt_server`、`mqtt_port`、`mqtt_user`、`mqtt_pwd`、`homeassistant_server`、`homeassistant_port`、`homeassistant_token`。服务器填写 IP 或主机名，不含 `http://`、路径或端口；端口范围为 1–65535。服务器最长 63 字节，用户名/密码最长 31 字节，令牌最长 255 字节。参数名后第一个空格分隔值，后面的空格属于值，密码的首尾空格会保留；使用 `config set mqtt_pwd ""` 可清空密码，令牌同理。

上述 IP 是 MQTT 配置示例，源码没有固定服务器地址。`homeassistant_server` 是独立的可选参数，MQTT Discovery 无需填写它；如需配置，单独使用 `config set homeassistant_server <地址>`，使用 `config set homeassistant_server ""` 可清空这项参数，MQTT 配置会保留。

### 编译与验证

完整固件已使用 ESP32 Core **2.0.17** 编译通过。固定库版本与作者库的 Git 提交见 [build-dependencies.json](build-dependencies.json)；原项目使用旧版 Arduino `String` MQTT API，不能直接替换为新版 ESP32MQTTClient。

安装指定核心和依赖后，可编译 `Software/HomeGear`，FQBN 为 `esp32:esp32:esp32`。模块开关由 `HomeGearConfig.h` 配置；当前默认启用温湿度/光照、空气质量、红外和 ZigBee，与实机模组一致。

原生 C++11 测试核对实测 ON/OFF 帧、捕获温度字段、严格温度解析与取整、串口配置、传感器快照/限流、部分读写和背压、配网门户回收条件，以及临时 CPU 时钟恢复。SensorService 和 IrService 集成测试执行真实服务与锁定 ArduinoJson 序列化器，使用模拟硬件和 MQTT 驱动；后者还核对未知状态不发红外、开关保留目标、发送期间入队、拒绝 retained/分片消息以及 Discovery 空室温定义：

```sh
tests/run-host-tests.sh
```

测试脚本默认从相邻 `HomeGear-build-env/libraries` 读取固定版本的 ArduinoJson 与 BSEC；在其他环境可通过 `HOMEGEAR_TEST_LIBRARIES` 指定同版本库所在目录。

串口配置测试覆盖端口和长度边界、异常命令、密码空格、清空凭据，以及修改地址时保留其他参数。刷入已有设备前备份 flash，确认分区一致后只更新应用分区，以保留 Wi-Fi 和 SPIFFS 配置；启动或配置应用不会自动发送空调开关信号。

帧字段参考 [IRremoteESP8266 #1060](https://github.com/crankyoldgit/IRremoteESP8266/issues/1060) 及作者的 [原始捕获表](https://docs.google.com/spreadsheets/d/1x82Cb3-GtOMM5oTVOFdNY9mqwLnJ-qElhOj8MId7mrA/edit)。完整 23 字节状态帧的第 15 字节（从 0 开始）为 `2 × 温度 - 13`，第 16 字节为反码。两条 25℃除湿帧已在用户面板上验证；新增温度字段已通过捕获数据和软件测试，现场响应待验证。
