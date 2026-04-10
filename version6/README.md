# Version 4 — 完整项目（当前最新版本）

**更新时间**：2026-04-08

## 相比 Version 3 新增/改动

- 补充完整项目结构（头文件、驱动库、链接脚本、IDE配置）
- 添加 CLAUDE.md 项目说明文档
- 添加 .gitignore（排除编译产物）

## 目录结构

```
version4/
├── Core/
│   ├── Inc/        头文件（main.h、HAL配置、中断定义）
│   ├── Src/        源文件（main.c、SSD1306驱动、HAL回调等）
│   └── Startup/    启动文件（startup_stm32g491retx.s）
├── Drivers/
│   ├── CMSIS/      ARM Cortex-M4 标准接口库
│   ├── STM32G4xx_HAL_Driver/   ST HAL 驱动库
│   └── BSP/STM32G4xx_Nucleo/   Nucleo板级支持包
├── Debug/
│   └── makefile    构建脚本（cd Debug && make 即可编译）
├── NUCLEO_G491RE_Test.ioc   STM32CubeMX 硬件配置文件
├── STM32G491RETX_FLASH.ld   Flash链接脚本
└── STM32G491RETX_RAM.ld     RAM链接脚本
```

## 编译方法

需要安装 `arm-none-eabi-gcc` 工具链：

```bash
cd Debug && make
```

输出文件：`Debug/NUCLEO_G491RE_Test.elf`（烧录用 STM32CubeIDE 或 ST-LINK）
