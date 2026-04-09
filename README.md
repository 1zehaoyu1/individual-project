# BMS — 锂电池管理系统（毕业设计）

基于 STM32G491RE 的 3S 锂电池组管理系统固件。

## 硬件平台

| 组件 | 型号 / 接口 |
|------|------------|
| 主控 MCU | STM32G491RE（ARM Cortex-M4，170 MHz） |
| 电流/电压传感器 | INA228（I2C2，20位ADC） |
| 温度传感器 | NTC 热敏电阻（ADC1） |
| 显示屏 | SSD1306 OLED 128×64（I2C3） |
| 负载控制 | MOSFET（GPIO PB5） |
| 开发板 | NUCLEO-G491RE |

## 主要功能

- SOC 电量估算（OCV 初始化 + 库仑计数 + 低电流校正）
- 实时显示电压、电流、温度、SOC、剩余时间
- 过温 / 过流 / 欠流故障保护
- MOSFET 热保护（>30°C 自动断开负载）
- 5页 OLED 界面，按钮翻页

---

## 版本目录

| 版本 | 文件夹 | 主要内容 |
|------|--------|----------|
| v1 | [version1/](./version1/) | 基础采集与显示（INA228、NTC、OLED） |
| v2 | [version2/](./version2/) | 故障检测优化 + Nucleo BSP |
| v3 | [version3/](./version3/) | 库仑计数SOC + 剩余时间估算 |
| v4 | [version4/](./version4/) | 完整工程基线（F1 预热 + F5 SOC 可视化，可编译） |
| v5（当前）| [version5/](./version5/) | + F2 设置模式、状态机架构、按键修复、防呆阈值 |

- version1–3：`main.c` 快照 + README
- version4–5：完整可编译工程（Core/、Drivers/、Debug/）
