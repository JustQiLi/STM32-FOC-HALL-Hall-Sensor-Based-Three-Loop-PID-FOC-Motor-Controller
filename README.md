# STM32-FOC-HALL-Hall-Sensor-Based-Three-Loop-PID-FOC-Motor-Controller STM32-FOC-HALL  基于霍尔传感器的三环PID控制系统
基于 STM32F407 的 20kHz 电流环 / 1kHz 速度环 / 200Hz 位置环 三轴串级 FOC 实现，仅用 三路霍尔传感器 完成 50–3500 rpm 全区间速度闭环（±1.5%），内置 18 项补偿机制（摩擦前馈 / 死区补偿 / 角度助推 / PLL 观测器 / 抗积分饱和等），支持 VOFA+ 在线调参与实时遥测
