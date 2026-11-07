# GoM_ComDemo_STM32_HAL

基于 **STM32F103C8T6 + STM32 HAL 库** 的 **宇树 GO-M8010-6 关节电机** RS485 通讯 Demo。

工程通过 CubeMX 生成，使用 Keil MDK-ARM 编译，演示了如何以 4 Mbps 波特率、约 1000 Hz 的频率向电机发送控制指令并解析反馈数据。

---

## 1. 功能特性

- USART1 以 **4 Mbps** 全双工与 RS485 收发器通讯
- RS485 方向控制（DE/RE）在收发之间手动切换
- 完整实现 GO-M8010-6 控制帧 / 反馈帧的组包与解析
- 使用 **CRC-CCITT (XMODEM, 初值 0)** 校验收发数据
- 板载 LED 闪烁作为心跳指示
- 统计通讯超时（`timeout`）与 CRC 错误（`bad_msg`）次数
- 反馈数据自动解算为力矩 / 速度 / 位置 / 温度 / 错误码 / 足端压力

---

## 2. 硬件要求

| 项目 | 说明 |
| --- | --- |
| MCU | STM32F103C8T6（Cortex-M3，72 MHz，64 KB Flash / 20 KB RAM） |
| 外部晶振 | 8 MHz HSE，经 PLL ×9 得到 72 MHz 主频 |
| 串口 | USART1，波特率 **4 000 000**，8N1，无硬件流控 |
| 电平转换 | TTL ↔ RS485 收发器模块，**必须支持 4 Mbps**（如宇树官方模块） |
| 电机 | 宇树 GO-M8010-6 关节电机 |

> STM32F103 在 APB2 = 72 MHz、16 倍过采样时最高可到 4.5 Mbps，4 Mbps 处于可用范围。
> 若移植到其他 MCU，建议选用带 FPU 的型号，并确认 USART 支持 4 Mbps。

### 2.1 接线

TTL 转 RS485 模块与 MCU：

| TTL 转 485 模块 | STM32F103C8T6 | 说明 |
| --- | --- | --- |
| RX | **PA10**（USART1_RX） | 模块 RX 接 MCU TX |
| TX | **PA9**（USART1_TX） | 模块 TX 接 MCU RX |
| RE/DE | **PA8**（`RS485_REDE`） | 高电平发送、低电平接收 |
| 5V | 5V | 供电 |
| GND | GND | 必须与电机/总线共地 |

> **注意**：仓库早期版本使用 PA11 作为 DE 引脚，最新代码已改为 **PA8**（`Core/Inc/main.h` 中的 `RS485_REDE_Pin`）。
> `gpio.c` 中 PA11 仍被配置为普通输出，属于历史遗留配置，可忽略或在 CubeMX 中删除。

RS485 总线侧：

- 推荐使用 GH1.25-3P 连接线或宇树官方电源线
- 为保证通讯可靠，请将 RS485 总线的 GND 与 MCU 的 GND 连接在一起

### 2.2 引脚分配

| 引脚 | 功能 | 配置 |
| --- | --- | --- |
| PA8 | `RS485_REDE` | 推挽输出（方向控制） |
| PA9 | USART1_TX | 复用推挽输出 |
| PA10 | USART1_RX | 浮空输入 |
| PA13 / PA14 | SWDIO / SWCLK | 下载调试 |
| PC13 | LED | 推挽输出，低电平点亮 |
| PD0 / PD1 | HSE 晶振 | 外部晶振 |

---

## 3. 目录结构

```
GoM_ComDemo_STM32_HAL/
├── Core/
│   ├── Inc/
│   │   ├── main.h            # LED / RS485_REDE 引脚定义
│   │   ├── usart.h
│   │   ├── gpio.h
│   │   └── stm32f1xx_hal_conf.h
│   └── Src/
│       ├── main.c            # 主循环：组包 → 发送 → 接收 → 解析
│       ├── usart.c           # USART1 初始化（4 Mbps）
│       ├── gpio.c            # GPIO 初始化
│       ├── stm32f1xx_it.c
│       └── system_stm32f1xx.c
├── Unitree/
│   ├── gom_protocol.h        # 协议结构体与电机指令/反馈结构体
│   ├── gom_protocol.c        # 组包 modify_data() / 解包 extract_data()
│   ├── crc_ccitt.h
│   └── crc_ccitt.c           # CRC-CCITT 查表实现
├── Drivers/                  # STM32 HAL + CMSIS
├── MDK-ARM/                  # Keil 工程与编译产物（.hex 在此）
├── GoM_ComDemo_STM32_HAL.ioc # CubeMX 配置
└── .gitignore
```

---

## 4. 通讯协议概要

GO-M8010-6 采用固定长度的二进制帧，所有多字节字段均为**小端序**。

### 4.1 控制帧（主机 → 电机，17 字节）

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `head[2]` | 2 B | 固定 `0xFE 0xEE` |
| `mode` | 1 B | bit0-3：电机 ID（0~14，15 为广播）；bit4-6：工作模式 |
| `tor_des` | 2 B | 期望力矩，q8，单位 N·m |
| `spd_des` | 2 B | 期望速度，q8，单位 rad/s |
| `pos_des` | 4 B | 期望位置，q15，单位 rad |
| `k_pos` | 2 B | 刚度系数，q15 |
| `k_spd` | 2 B | 阻尼系数，q15 |
| `CRC16` | 2 B | 对前 15 字节计算 CRC-CCITT |

工作模式 `status`：`0` 锁定 / `1` FOC 闭环 / `2` 编码器校准 / `3~7` 保留。

### 4.2 反馈帧（电机 → 主机，16 字节）

| 字段 | 长度 | 说明 |
| --- | --- | --- |
| `head[2]` | 2 B | 固定 `0xFD 0xEE` |
| `mode` | 1 B | 电机 ID 与当前模式 |
| `torque` | 2 B | 实际力矩，q8，N·m |
| `speed` | 2 B | 实际速度，q8，rad/s |
| `pos` | 4 B | 实际位置，q15，rad |
| `temp` | 1 B | 温度，-128 ~ 127 °C |
| `MError` | 3 bit | 错误码：0 正常 / 1 过热 / 2 过流 / 3 过压 / 4 编码器故障 |
| `force` | 12 bit | 足端气压传感器原始值（0~4095） |
| `none` | 1 bit | 保留 |
| `CRC16` | 2 B | 对前 14 字节计算 CRC-CCITT |

### 4.3 物理量换算（`Unitree/gom_protocol.c`）

发送方向：

```c
k_pos   = K_P / 25.6f   * 32768.0f;
k_spd   = K_W / 25.6f   * 32768.0f;
pos_des = Pos / 6.28318f * 32768.0f;
spd_des = W   / 6.28318f * 256.0f;
tor_des = T   * 256.0f;
```

接收方向：

```c
T   = torque / 256.0f;
W   = speed / 256.0f * 6.28318f;
Pos = pos / 32768.0f * 6.28318f;
```

发送前会对各物理量做饱和限幅：

| 量 | 范围 |
| --- | --- |
| `id` | 0 ~ 15 |
| `mode` | 0 ~ 7 |
| `K_P`, `K_W` | 0.0 ~ 25.599 |
| `T` | -127.99 ~ 127.99 N·m |
| `W` | -804.00 ~ 804.00 rad/s |
| `Pos` | -411774.0 ~ 411774.0 rad |

CRC 采用 **CRC-CCITT**（多项式 `0x1021`，反射查表 `0x8408`，初值 `0`），实现参考 Linux `lib/crc-ccitt.c`。

---

## 5. 软件流程

`Core/Src/main.c` 的主循环：

```c
while (1)
{
  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);   // 心跳

  modify_data(&cmd);                            // 填充并校验控制帧

  RS485_TxMode();                               // PA8 = 1，发送
  HAL_UART_Transmit(&huart1, (uint8_t *)&cmd.motor_send_data,
                    sizeof(cmd.motor_send_data), 1);

  RS485_RxMode();                               // PA8 = 0，接收
  HAL_UART_Receive(&huart1, (uint8_t *)&data.motor_recv_data,
                   sizeof(data.motor_recv_data), 1);

  extract_data(&data);                          // 校验 + 解算反馈

  HAL_Delay(1);                                 // 约 1 kHz 收发频率
}
```

通讯频率建议：

- 单电机推荐 **1000 Hz**
- RS485 总线最高轮询频率约 **3000 Hz**
- 频率过高会影响电机正常工作

---

## 6. 编译与烧录

1. 使用 **Keil MDK-ARM 5.32**（或更新版本）打开：

   ```
   MDK-ARM/GoM_ComDemo_STM32_HAL.uvprojx
   ```

2. 工程预定义宏与头文件路径已配置：

   - 宏：`USE_HAL_DRIVER, STM32F103xB`
   - 头文件路径：`Core/Inc`、`Unitree`、HAL 驱动与 CMSIS 路径

3. 编译（F7）后固件输出到：

   ```
   MDK-ARM/GoM_ComDemo_STM32_HAL/GoM_ComDemo_STM32_HAL.hex
   ```

4. 使用 ST-Link / J-Link / DAP-Link 烧录（工程默认调试配置在 `MDK-ARM/DebugConfig/`）。

### 使用 STM32CubeMX 重新生成

修改外设配置后打开 `GoM_ComDemo_STM32_HAL.ioc` 即可重新生成代码。用户代码均位于 `USER CODE BEGIN/END` 段内，重新生成不会丢失。

> 关键外设配置：`USART1` 异步模式 4 Mbps；`PA8`、`PC13` 为 GPIO 输出；HSE + PLL×9 = 72 MHz。

---

## 7. 如何修改控制量

在 `main.c` 的 `USER CODE BEGIN 2` 中设置初始指令：

```c
cmd.id   = 0;     // 电机 ID
cmd.mode = 0;     // 0：锁定（不输出力矩）
cmd.K_P  = 0.0f;  // 刚度
cmd.K_W  = 0.0f;  // 阻尼
cmd.Pos  = 0.0f;  // 目标位置 rad
cmd.W    = 0.0f;  // 目标速度 rad/s
cmd.T    = 0.0f;  // 前馈力矩 N·m
```

### 让电机进入 FOC 闭环并保持某个位置

```c
cmd.mode = 1;     // FOC 闭环
cmd.K_P  = 5.0f;  // 刚度，需大于 0 才会产生回复力矩
cmd.K_W  = 1.0f;  // 阻尼
cmd.Pos  = 0.0f;  // 目标位置
```

> **上电安全提示**：`mode = 0`（锁定）或 `K_P = K_W = 0` 时电机不输出控制力矩。
> 首次调试建议先以零增益运行，确认能收到正确反馈后，再逐步增大 `K_P` / `K_W`。

### 读取反馈

```c
if (data.correct) {
    float pos = data.Pos;     // rad
    float spd = data.W;       // rad/s
    float tor = data.T;       // N·m
    int   temp = data.Temp;   // °C
    int   err  = data.MError; // 0 正常
    int   ff   = data.footForce;
}
```

`data.timeout` 为接收超时次数，`data.bad_msg` 为 CRC 校验失败次数，可用于评估总线质量。

### 错误处理

`main.c` 中已预留错误分支：

```c
if (data.MError != 0)
{
    /* 电机进入错误状态：会切换为阻尼模式，且不再响应运动指令。
       此时应让其他电机进入阻尼状态，避免整机继续运动。*/
}
```

---

## 8. 注意事项

- RS485 总线**必须与 MCU 共地**，否则容易出现 CRC 错误或通讯断续。
- 请勿将 RS485 的 A/B 线误接到电源（例如 24 V），接错可能烧毁收发器或电机，且不在保修范围内。
- 该 Demo 使用阻塞式 `HAL_UART_Transmit` / `HAL_UART_Receive`，超时时间为 1 ms。若要控制多个电机，建议改为 **DMA + 中断** 方式，并注意总线时序。
- `Unitree/gom_protocol.c`、`Unitree/gom_protocol.h`、`MDK-ARM/README.txt` 等文件为 **GBK 编码**，使用 UTF-8 编辑器打开可能出现中文乱码。
- 编译中间文件已在 `.gitignore` 中忽略，最终固件 `*.hex` 与 Keil 工程文件保留入库。

---

## 9. 参考资料

- 宇树电机 SDK 开发指南（STM32 示例）：
  <https://support.unitree.com/home/zh/Motor_SDK_Dev_Guide/stm32_example>
- 4 通道 485 转 USB 模块说明：
  <https://support.unitree.com/home/zh/Motor_SDK_Dev_Guide/4-channel_485_to_USB_module>
- Linux CRC-CCITT 实现：
  <https://github.com/torvalds/linux/blob/master/lib/crc-ccitt.c>

---

## 10. 许可证与声明

- `Unitree/` 目录下协议代码版权归 **Unitree Robotics** 所有（Copyright © Unitree Robotics Co., Ltd. 2022）。
- STM32 HAL 驱动代码版权归 **STMicroelectronics** 所有，遵循其随附的 LICENSE。
- 本项目其余部分仅用于学习与开发调试，请自行评估在真实机器人系统上的安全性。
