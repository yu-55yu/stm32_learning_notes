# stm32 project

[STM32F103C8T6](#基础介绍)

[标准库开发环境的搭建与配置](#标准库开发环境的搭建与配置)

[一、GPIO](#GPIO)

[二·、中断](#中断)

[三、定时器](#定时器)



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

<font color=red>红色：电源相关引脚</font>

<font color=blue>蓝色：最小系统相关引脚</font>

<font color=green>绿色：io口、功能口</font>

![](C:\Users\Lenovo\Desktop\STM32Project\图片\引脚定义.png)

STM32 内部有多个时钟源

| **时钟名称**                   | **作用**                                                     |
| ------------------------------ | ------------------------------------------------------------ |
| **HSI（High-Speed Internal）** | 内部 8MHz 振荡器，精度较低（±1%），适合不用外部晶振的场景    |
| **HSE（High-Speed External）** | 外部高速晶振（常见 8MHz），精度高，推荐用于正式项目          |
| **LSE（Low-Speed External）**  | 32.768kHz 低速晶振，主要用于 RTC 实时时钟                    |
| **LSI（Low-Speed Internal）**  | 内部低速 RC 振荡器，频率大约 40kHz，适用于低功耗 RTC         |
| **PLL（Phase-Locked Loop）**   | 锁相环，可放大 HSE/HSI 频率，用于提高系统主频（如 8MHz → 72MHz） |

---



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

### 一、基本介绍

> GPIO(General Purpose Input Output) 通用输入输出口
>
> 可配置为8种输入输出模式
>
> 引脚电平:0V~3.3V，部分引脚可容忍5V
>
> 输出模式下可控制端口输出高低电平，用以驱动LED、控制蜂鸣器模拟通信协议输出时序等
>
> 输入模式下可读取端口的高低电平或电压，用于读取按键输入、外接模块电平信号输入、ADC电压采集、模拟通信协议接收数据等

---

1. **GPIO 8 种工作模式**

   - GPIO_Mode_AIN 模拟输入 &rarr;ADC专属模式

   - GPIO_Mode_IN_FLOATING 浮空输入

   - GPIO_Mode_IPD 下拉输入

   - GPIO_Mode_IPU 上拉输入

     ------

   - GPIO_Mode_Out_OD 开漏输出

   - GPIO_Mode_Out_PP 推挽输出

   - GPIO_Mode_AF_OD 复用开漏输出

   - GPIO_Mode_AF_PP 复用推挽输出

2. **应用总结**

   1. 上拉输入、下拉输入可以用来检测外部信号。例 如，按键等
   
      > 若外部模块空闲默认输出高电平，选择上拉输入（一般）
      >
      > 若外部模块空闲默认输出低电平，选择下拉输入
   
   2. 浮空输入模式，由于输入阻抗较大，一般把这种模式用于标准通信协议的I2C、 USART的接收端；
   
   3. 普通推挽输出模式一般应用在输出电平为0和3.3V的场合。
   
   4. 普通开漏输出模式一般应用在电平不匹配的场合，如需要输出5V的高电平，就需要在外部一个上拉电 阻，电源为5V，把GPIO设置为开漏模式，当输出高阻态时，由上拉电阻和电源向外输 出5V电平。
   
   5. 对于相应的复用模式（复用输出来源片上外设），则是根据GPIO的复用功能来选择，如GPIO的引脚用作串口的输出（USART/SPI/CAN），则使用复用推挽输出模式。 如果用在I2C、SMBUS这些需要线与功能的复用场合，就使用复用开漏模式。
   
   6. 在使用任何一种开漏模式时，都需要接上拉电阻

![](C:\Users\Lenovo\Desktop\STM32Project\图片\GPIO.png)

**具体应用步骤**

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

### 二、GPIO输出

> 推挽模式  `GPIO_Mode_Out_PP`   高低电平都有驱动能力
>
> 开漏模式  `GPIO_Mode_Out_OD`   一般需要配上拉电阻，否则高电平没有驱动能力

- ### LED闪烁

- ### 流水灯

+ ### 蜂鸣器

### 三、GPIO输入

> 浮空输入
>
> 上拉输入 `GPIO_MODE_IPU`
>
> 下拉输入

+ ### 按键控制LED

+ ### 光敏传感器控制蜂鸣器





## 中断

### 一、外部中断

![](C:\Users\Lenovo\Desktop\STM32Project\图片\外部中断基本结构.png)

**例**

+ **对射式红外传感器计次**
+ **旋转编码器计次**

**示例代码**

> 按键控制LED

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

<mark>**注意应用场景**</mark>

+ 突发事件

+ 在中断函数里不要执行耗时过长的代码

+ 最好不要在中断函数和主函数里调用相同的的函数或者操作同一个硬件

  > 可以在中断里操作变量或标志位
  >
  > 在主函数循环里显示变量、标志位。 
  >
  > <mark>低耦合 高内聚（？）</mark>





## 定时器

**计数器计数频率**

​	$$CK\_CNT =  \frac{CK\_PSC } {PSC + 1}$$

>  **CK_PSC（定时器时钟频率）:**  系统时钟（SYSCLK）或者外部时钟源（如外部晶振）的频率。默认72MHz。
>
>  **PSC（预分频器Prescaler）:**  降低定时器的计数频率.
>
>  **CK_CNT：** 计数器的实际计数频率。
>
>  `TIM_GetCounter(TIM2)` 可以看实际计数

**计数器溢出频率**

​	$$ CK\_CNT\_OV=\frac{CK\_CNT}{ARR+1} = \frac{CK\_PSC}{(PSC+1)(ARR+1)}$$

>  **ARR（自动重装值）：**定时器的计数周期，常用于实现定时器溢出中断或者事件触发。
>
> **CK_CNT_OV：**  定时器溢出的频率，决定了定时器中断的发生频率。



### 一、基本功能

1. **定时器定时中断**（系统时钟）
2. **定时器外部时钟**（ext）



### 二、输出比较（OutputCompare）

用来产生PWM波形  &rarr;  LED控制亮度，电机控速

**参数计算**

- 频率  

   $$Freq=\frac{CK\_PSC}{(PSC+1)(ARR+1)}$$

- 占空比

  $$Duty=\frac{CCR}{ARR+1}=\frac{T_o}{T_s}$$

- 分辨率（占空比变化步距）

  $$Reso=\frac{1}{ARR+1}$$

> `TIM_SetComparex(x=1~4)` 设置CCR的值
>
> `TIM_PrescalerConfig` PSC
>
> ARR会影响所有参数。最好先固定ARR。PSC决定频率，CCR决定占空比。

**应用**

- PWM驱动呼吸灯

- PWM驱动舵机

  > 输入PWM信号要求：周期为20ms，高电平宽度为0.5ms~2.5ms
  >
  > 0.5ms&rarr;0&deg; ; 2.5ms&rarr; 180&deg;

  0.5/20=ccr(0)/20k &rarr; 500

  2.5/20=ccr(180)/20k &rarr; 2500



### 三、输入捕获（InputCapture）

> 输入捕获模式下，当通道输入引脚出现指定电平跳变时，当前CNT的值将被锁存到CCR中，可用于测量PWM波形的频率、占空比、脉冲间隔、电平持续时间等参数

每个高级定时器和通用定时器都拥有4个输入捕获通道

**模式：**

- PWMI模式，同时测量频率和占空比

- 主从触发模式，实现硬件全自动测量

**应用：**

- 输入捕获模式测频率

- PWMI 模式测频率占空比

  

  > 可测量的最低频率：
  >
  > ​	$$ CK\_CNT\_OV=\frac{CK\_CNT}{ARR+1} = \frac{CK\_PSC（72MHz）}{(PSC+1)(ARR+1)}$$
  >
  > 可测量的最高频率：
  >
  > ​				$$\frac{72MHz*误差}{(PSC+1)}(eg.误差1)$$

内部触发0(ITR0) TI1的边沿检测器(TI1F_ED)  内部触发1(ITR1) 滤波后的定时器输入1(TI1FP1)



### 四、编码器接口







## ADC数模转换器

**简介**

- ADC可以将引脚上连续变化的模拟电压转换为内存中存储的数字变量，建立模拟电路到数字电路的桥梁

- 12位逐次逼近型ADC，1us转换时间

  > 分辨率：12位 &rarr; 0~2^12-1(4095)
  >
  > 转换频率：最大1MHz

- 输入电压范围：0\~3.3V，转换结果范围：0~4095

- 18个输入通道，可测量16个外部（16个GPIO口，可以直接接模拟信号测）和2个内部信号源（内部温度传感器+内部参考电压（1.2V左右的基准电压））

- 规则组（常规）和注入组两个转换单元

  > 规则通道：16通道 但只有一个数据寄存器，前面的会被后面覆盖。一般需要结合DMA转运数据
  >
  > 注入通道：4通道+4寄存器。数据不会被覆盖

- 模拟看门狗自动监测输入电压范围（可测光线强度、温度。超出阈值可以执行某些操作）

- STM32F103C8T6 ADC资源：ADC1、ADC2，10个外部输入通道

转换模式：

- 单次转换/连续转换
- 扫描转换/非扫描模式



























# 不太懂

```mermaid
graph LR;
    A[正弦波] -->|比较器| C[方波数字信号];
    C --> D[隔离放大器];
    D --> E[最终数字信号];


```

