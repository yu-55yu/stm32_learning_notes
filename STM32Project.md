# stm32 project

[STM32F103C8T6](#基础介绍)

[一、标准库开发环境的搭建与配置](#标准库开发环境的搭建与配置)

[二、GPIO](#GPIO)

[三、中断](#中断)

[四、定时器](#定时器)

## 基础介绍

### F103C8T6

- 系列：主流系列STM32F1
- 内核：ARM Cortex-M3
- 主频：72MHz
- RAM：20K（SRAM）
- ROM：64K（Flash）
- 供电：2.0~3.6V（标准3.3V）
- 封装：LQFP48



### 引脚定义

<font color=red>电源相关引脚</font>

<font color=blue>最小系统相关引脚</font>

<font color=green>io口、功能口</font>



![](C:\Users\Lenovo\Desktop\STM32Project\引脚定义.png)

STM32 内部有多个时钟源



| **时钟名称**                   | **作用**                                                     |
| ------------------------------ | ------------------------------------------------------------ |
| **HSI（High-Speed Internal）** | 内部 8MHz 振荡器，精度较低（±1%），适合不用外部晶振的场景    |
| **HSE（High-Speed External）** | 外部高速晶振（常见 8MHz），精度高，推荐用于正式项目          |
| **LSE（Low-Speed External）**  | 32.768kHz 低速晶振，主要用于 RTC 实时时钟                    |
| **LSI（Low-Speed Internal）**  | 内部低速 RC 振荡器，频率大约 40kHz，适用于低功耗 RTC         |
| **PLL（Phase-Locked Loop）**   | 锁相环，可放大 HSE/HSI 频率，用于提高系统主频（如 8MHz → 72MHz） |







## 标准库开发环境的搭建与配置

###   <font color=red>1. 打开keil</font>

+ <b>project --><font color=blue>new uvision project </font></b>

  + 打开 **stm32 project** 文件夹

  + `contrl+shift+n` 新建文件夹，起此次工程名（eg.**2-1 LED闪烁**）

  + 点进去，`contrl+shift+n` 新建文件夹，起名 <mark>**Project**</mark>;

    > stm32 project 
    >
    > > 2-1 LED闪烁
    > >
    > > >  Project

* **<font color=blue>选择芯片</font>（`STMicroelectronics`-->`STM32F1 Series`-->`STM32F103C8`）**

  

### <font color=red>2. 打开工程文件夹（2-1）</font>

* <b><font color=blue>新建</font>四个文件夹 <mark>Start</mark>（头文件）、<mark>Library</mark>（库函数）、<mark>User</mark></b>
  + **`Start`**
    + arm文件夹 ：所有
    + STM32F10x文件：两个system文件+stm32f10x.h
    + CoreSupport：两个文件
  + **`User`**
    + Template文件夹：stm32f10x_conf.h + stm32f10x_it.c + stm32f10x_it.h
    + main.c
  + **`Library`**
    + src文件夹：所有
    + inc文件夹：所有

### <font color=red>3. 打开keil</font>

* <b>点<font color=blue>三个箱子</font>(Manage Project Items)</b>

  + <b>`Delete`</b> 

    >  把默认的group删掉

  + <b>`New`</b>

    > 新建三个组

    + <b>Start</b> -- `Add Files` --> 
      + 后缀为md的启动文件（.s）![](C:\Users\Lenovo\Desktop\STM32Project\STM32资料\图片1.png)
      + 所有（.c）(.h）文件
    + <b>Library</b> -- `Add Files` --> 
      + 所有文件
    + <b>User</b> -- `Add Files` --> 
      + 所有文件

* <b>点<font color=blue>魔术棒</font>(Options for Target)</b>

  + <b>`C/C++`</b>

    * <b>`Define` </b>

      >  写上<mark>USE_STDPERIPH_DRIVER</mark>
      >
      > > 这样`#include"stm32f10x_conf.h"`才有效

    * <b>`Include Paths`</b>

      > 添加 <mark>start、library、user</mark> 的路径

  + <b>`Debug`</b>

    + **`Use`**

      > 选择 `ST-Link Debugger`

      + **`Settings`**

        + <b>`Flash and Download`</b>

          >  勾选 `Reset and Run`
    
    

<font color=pink>(●ˇ∀ˇ●)接下来是开始写代码（熟练使用contrlC contrlV contrlF）</font>

## GPIO

> GPIO(General Purpose Input Output) 通用输入输出口
>
> 可配置为8种输入输出模式
>
> 引脚电平:0V~3.3V，部分引脚可容忍5V
>
> 输出模式下可控制端口输出高低电平，用以驱动LED、控制蜂鸣器模拟通信协议输出时序等
>
> 输入模式下可读取端口的高低电平或电压，用于读取按键输入、外接模块电平信号输入、ADC电压采集、模拟通信协议接收数据等
>
> ---

1. GPIO 8 种工作模式

   + GPIO_Mode_AIN 模拟输入 

   + GPIO_Mode_IN_FLOATING 浮空输入 

   + GPIO_Mode_IPD 下拉输入 

   + GPIO_Mode_IPU 上拉输入 

     ---

   + GPIO_Mode_Out_OD 开漏输出 

   + GPIO_Mode_Out_PP 推挽输出 

   + GPIO_Mode_AF_OD 复用开漏输出 

   + GPIO_Mode_AF_PP 复用推挽输出

2. 应用总结

   1. 上拉输入、下拉输入可以用来检测外部信号。例如，按键等
   2. 浮空输入模式，由于输入阻抗较大，一般把这种模式用于标准通信协议的I2C、 USART的接收端； 
   3. 普通推挽输出模式一般应用在输出电平为0和3.3V的场合。
   4. 普通开漏输出模式一般应用在电平不匹配的场合，如需要输出5V的高电平，就需要在外部一个上拉电 阻，电源为5V，把GPIO设置为开漏模式，当输出高阻态时，由上拉电阻和电源向外输 出5V电平。
   5. 对于相应的复用模式（复用输出来源片上外设），则是根据GPIO的复用功能来选择，如GPIO的引脚用作串口的输出（USART/SPI/CAN），则使用复用推挽输出模式。 如果用在I2C、SMBUS这些需要线与功能的复用场合，就使用复用开漏模式。
   6. 在使用任何一种开漏模式时，都需要接上拉电阻

![](C:\Users\Lenovo\Desktop\STM32Project\资料\GPIO.png)

1. 使用RCC开启GPIO的时钟

   ```c
   void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState)
   ```

2. 使用 `GPIO_Init`  初始化

   ```c
   void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
   ```

   + 定义结构体

     ```c
     GPIO_InitTypeDef GPIO_InitStructure;
     ```

   + 赋值结构体

     ```c
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;//这里可以用或同时复制多个
     GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
     ```

   + 初始化引脚

     ```c
     GPIO_Init(GPIOB,&GPIO_InitStructure);
     ```

3. 使用输出或输入函数控制GPIO口

   <mark>常用读写函数</mark>

   ```c
   uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
   uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);
   uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
   uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx);
   void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
   void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
   void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal);
   void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal);
   ```

### 一、GPIO输出

> 推挽模式  `GPIO_Mode_Out_PP`   高低电平都有驱动能力
>
> 开漏模式  `GPIO_Mode_Out_OD`   一般需要配上拉电阻，否则高电平没有驱动能力

- ### LED闪烁

- ### 流水灯

+ ### 蜂鸣器

### 二、GPIO输入

+ ### 按键控制LED

+ ### 光敏传感器控制蜂鸣器





## 中断

> 例子：按键控制LED

```c
#include "stm32f10x.h"

// EXTI0 中断处理函数
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & EXTI_PR_PR0) { // 检查 EXTI0 中断是否触发
        GPIOC->ODR ^= GPIO_ODR_ODR13;  // 切换 LED 状态（PC13）
        EXTI->PR |= EXTI_PR_PR0;  // 清除 EXTI0 中断标志
    }
}

// 初始化 EXTI0（PA0）为外部中断
void EXTI0_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);  // 使能 GPIOA、AFIO 和 GPIOC 时钟

    // 配置 PA0 为输入模式（浮空输入）
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 配置 PC13 为输出模式（LED）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP; // 推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    // 配置 PA0 为 EXTI0 信号源
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource0);  // 将 PA0 映射到 EXTI0

    // 配置 EXTI0
    EXTI_InitTypeDef EXTI_InitStruct;
    EXTI_InitStruct.EXTI_Line = EXTI_Line0;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;  // 设置为中断模式
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Falling; // 设置为下降沿触发
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;  // 使能 EXTI0
    EXTI_Init(&EXTI_InitStruct);

    // 使能 EXTI0 中断
    NVIC_EnableIRQ(EXTI0_IRQn);  // 启用 EXTI0 的中断
}

int main(void) {
    EXTI0_Init();  // 初始化 EXTI0 中断

    while (1) {
        // 主循环，等待外部中断触发
    }
}

```

![](C:\Users\Lenovo\Desktop\STM32Project\EXTI基本结构.png)

### 应用场景

+ 突发事件
+ 注意中断函数内不要耗时太久
+ 不要在中断函数和主函数中操作一个东西。可以在中断函数内操作标志位



## 定时器

在 STM32 中，**预分频值**（Prescaler）和 **自动重装值**（Auto-Reload Value，ARR）是定时器（Timer）工作模式下的重要参数，控制定时器的计数速率和周期。它们一起决定了定时器中断或事件触发的频率。

### **1. 预分频值（Prescaler）**

预分频值是一个 **分频器**，用来将 **定时器的输入时钟**频率（通常是系统时钟或者外部时钟）降低到一个较小的值。这个值控制了定时器的计数频率。

- **作用**：通过设置预分频器，可以使定时器的计数速率变慢，从而增加定时器溢出的时间周期。

- **计算**：定时器的计数频率由以下公式确定：

  定时器计数频率=定时器时钟频率预分频值+1\text{定时器计数频率} = \frac{\text{定时器时钟频率}}{\text{预分频值} + 1}定时器计数频率=预分频值+1定时器时钟频率

  例如，如果系统时钟频率为 72 MHz，预分频值为 7199，那么定时器的计数频率为：

  72,000,0007200=10,000 Hz\frac{72,000,000}{7200} = 10,000 \text{ Hz}720072,000,000=10,000 Hz

  即定时器每秒计数 10,000 次。

- **作用示例**：如果你希望定时器每秒触发一次中断，但系统时钟频率很高（比如 72 MHz），你可以通过设置预分频器来将定时器的计数频率降低到每秒 1 次。

------

### **2. 自动重装值（Auto-Reload Value，ARR）**

自动重装值是定时器的 **计数上限**，当定时器的计数器达到这个值时，它会自动重置为 0，并开始重新计数。

- **作用**：设置自动重装值决定了定时器的溢出周期（也就是定时器中断或事件的触发周期）。一旦计数器的值达到 **ARR**，定时器会触发溢出中断或者其他相关事件。
- **计算**：定时器的计数周期为 **ARR + 1**。例如，如果 **ARR = 9999**，则定时器的计数周期为 **10,000 次**（从 0 到 9999）。
- **作用示例**：假设你希望定时器每 1 秒钟触发一次中断，并且定时器的计数频率为 10,000 Hz（即每秒 10,000 次计数）。那么设置 **ARR = 9999**，定时器将计数 10,000 次后溢出，触发一次中断。



