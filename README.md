# breeze_firmware_demo

This is the demo flight control source code of breeze mini quadcopter.

硬件设计bug
1. 使用8MHz的晶振而不是16MHz的，因为MCU最大支持72MHz，而使用PLL倍频只能倍频整数倍，所以使用16MHz的晶振是达不到72MHz的
2. 设计一个电源开关
软件设计bug
1. 使用系统定时器Systick产生中断的代码有问题
   1. 问题所在文件：driver_delay.c文件和driver_clock.c文件
