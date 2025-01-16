# stm32 project

[一、标准库开发环境的搭建与配置](#标准库开发环境的搭建与配置)

[二、GPIO](#GPIO)



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



