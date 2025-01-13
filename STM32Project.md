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
      + 后缀为md的启动文件（.s）![](C:\Users\Lenovo\Desktop\jsy\STM32资料\图片1.png)
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



## GPIO

1. 使用RCC开启GPIO的时钟
2. 使用 `GPIO_Init`  初始化
3. 使用输出或输入函数控制GPIO口

>  涉及RCC和GPIO两个外设的函数

