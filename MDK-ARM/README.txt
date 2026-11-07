GO-M8010-6关节电机 STM32 HAL通讯协议Demo，基于STM32F103C8T6
如果要移植到其他型号的MCU，推荐使用带有FPU的MCU

硬件要求:
1.MCU UART/USART外设需要支持4Mbps的波特率
2.TTL转RS485模块(485收发器) 需要支持4Mbps的波特率(Unitree旗舰店有售)

TTL转RS485模块的连线:
[TTL转485模块]	[MCU]
 RX				 PA10
 TX				 PA9
 RE/DE			 PA11
 5V				 5V
 GND			 GND

RS485线序定义：
https://support.unitree.com/home/zh/Motor_SDK_Dev_Guide/4-channel_485_to_USB_module
推荐使用GH1.25-3P连接器，可以搭配Unitree电机附件包使用
为了保证可靠的通讯质量，RS485总线的GND与MCU的GND必须连接到一起

注意事项：
请勿将RS485的A或B连通高压，例如A与DC24v连通。
这会造成总线上的所有非隔离的RS485设备烧毁，并且这不在保修范围内。

https://support.unitree.com/home/zh/Motor_SDK_Dev_Guide/stm32_example