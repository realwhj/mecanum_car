# 产品介绍
    麦轮循迹小车，具备蓝牙模式和循迹模式。在循迹模式下，通过移动到指定位置，利用机器视觉识别物品，实现物品的
    夹取和运输。在蓝牙模式下操作者可利用手机连接，自由控制小车行为
# 注意事项
    （1）该项目用于HITSZ大一立项，IDE使用keil5，编码格式为UTF-8，可用vscode打开
    （2）引脚配置使用STM32cubemx，采用HAL库
    （3）目前仅实现蓝牙控制底盘移动
    （4）项目完全开源，欢迎交流学习
# 材料使用规格和简介
    STM32F407ZG开发板（82*50*10）（高包含引脚）
        核心器件，与其余模块进行连接
    电机（圆柱高45，半径35，转长20）
        提供动力，12v
    电机驱动（20*18*1）
        单片机3.3v无法带动，需用驱动，另需连接电机
    循迹模块（）
        巡线作用，与单片机连接，安装于车底部的前端
    OLED（）
        显示屏，用于显示某些关键数据
    蓝牙模块（）
        与手机建立通信
    稳压模块（）
        电池输出需经过该模块才能降压为稳定5v，另需接电池
    蜂鸣器（）
        播放音乐
    电池（）
        提供电
    舵机（未买）
        精确控制转动角度
    OPENMV（未买）
        目前openmv为视觉识别待定方案
# 关于代码引用
    （1）除下述内容外其余部分由作者独立书写，借鉴代码
    （2）oled相关驱动代码借鉴大疆创新开源代码并对其进行了部分的修改以利于本项目开发
# 最近一次修改
    2022.7.16
