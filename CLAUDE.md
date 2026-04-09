# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.
It serves as the project-level entry point. For version-specific implementation details, see `version5/CLAUDE.md`.

---

## 项目概览

**毕业设计：基于 STM32G491RETx 的 3S 锂电池管理系统（BMS）**

- MCU：STM32G491RETx（ARM Cortex-M4F，170 MHz，FPU）
- 开发板：NUCLEO-G491RE
- 架构：裸机超循环（bare-metal superloop，无 RTOS）
- 语言：C，工具链：arm-none-eabi-gcc + GNU Make

## 仓库结构

```
individual-project/
├── README.md           # 项目总览与版本导航
├── version1/           # v1 代码快照（main.c + 说明）
├── version2/           # v2 代码快照
├── version3/           # v3 代码快照
├── version4/           # v4 完整工程基线（F1 预热 + F5 SOC 可视化，不含 F2）
└── version5/           # v5 当前完整工程（+ F2 设置模式、状态机架构）
    ├── Core/Src/       # 应用逻辑（main.c、ssd1306.c、font5x7.c 等）
    ├── Core/Inc/       # 头文件
    ├── Drivers/        # ST HAL / CMSIS / BSP 库
    ├── Debug/makefile  # 构建脚本
    └── CLAUDE.md       # 版本实现细节补充说明
```

## 构建命令

```bash
# 编译（需安装 arm-none-eabi-gcc）
cd version5/Debug && make

# 清理重编
cd version5/Debug && make clean
```

输出文件：`version5/Debug/NUCLEO_G491RE_Test.elf`
烧录方式：STM32CubeIDE 或 ST-LINK（不通过 make 烧录）

## 硬件外设一览

| 外设 | 接口 | 用途 |
|------|------|------|
| INA228 | I2C2，addr 0x40 | 电流/电压采集（20-bit ADC） |
| SSD1306 OLED | I2C3，addr 0x78 | 128×64 显示，5 页界面 |
| NTC 热敏电阻 | ADC1 CH1（PA0） | 温度采集（Steinhart-Hart 换算） |
| MOSFET | GPIO PB5 | 负载控制，HIGH = 接通 |
| 用户按钮 B1 | GPIO | 翻页，50 ms 消抖 |

主循环节拍约 10 ms；200 ms 采样一次数据，250 ms 刷新显示。
详细算法（SOC、故障检测逻辑、页面逻辑）见 `version5/CLAUDE.md`。

---

## 嵌入式编码规范

> 适用范围：本项目为 STM32 裸机毕业设计，硬件改动有限。
> 以下规范旨在提升代码可靠性，不作为大规模重构的依据。

### 内存管理

不在应用层调用 `malloc` / `calloc` / `free`。所有缓冲区和状态变量使用静态分配（`static` 局部变量或全局变量）。`main()` 启动后堆不应被触碰。

### HAL 调用失败处理

关键 HAL 调用需要有明确的失败处理，而不是忽略返回值：
- 初始化阶段失败：调用 `Error_Handler()` 进入安全状态
- 运行期失败（如 I2C 读取）：返回错误哨兵值（如 `NAN` 或 `0`），设置故障标志，或在下次采样时重试

不是每一行 HAL 调用都必须加检查，而是凡有实际失败风险的路径（外设读写、初始化）都应处理。

### ISR 设计

ISR 只做最少的事：设置 `volatile uint8_t` 标志或递增计数器，具体处理留给主循环。
不在 ISR 中调用：
- `HAL_*` 阻塞函数
- `printf` / `sprintf`
- 浮点运算密集操作

参考：当前 `SysTick_Handler` → `HAL_IncTick()` 即是正确模型。

### ISR 与主循环共享变量

ISR 和主循环共享的变量必须声明为 `volatile`。
多字节类型（如 `uint32_t`、`float`）的读写需用 `__disable_irq()` / `__enable_irq()` 保护，防止 Cortex-M4 上出现撕裂读。

### 时序与潜在问题

- `HAL_GetTick()` 在约 49 天后回绕（`uint32_t` 上限）。始终使用差值计算 `(now - last)`，不要用 `now >= last + N` 比较（当前代码已正确处理）
- INA228 I2C 读失败时返回 `0`，会悄悄污染 SOC 计算。如果修改采集逻辑，需在调用方检测异常值
- 使用 `float` 做温度计算时，在参与算术前用 `isnan()` 检查 NaN

### LL 驱动使用建议

当前 HAL 对 I2C2（INA228）、I2C3（OLED）、ADC1（NTC）的使用在 200 ms 采样频率下完全够用，无需替换。
如果未来新增时序要求严格（< 1 ms）的外设驱动，可考虑使用 STM32G4 的 LL 驱动以减少 HAL 开销，但不作强制要求。

### 提交前自查

修改 `version4/Core/Src/main.c` 或外设驱动后：
- `snprintf` 输出缓冲区使用了 `sizeof(buf)` 限制
- `SysTick_Handler` 没有新增任何工作（目标 < 10 µs 完成）
- 新增的 HAL 外设读写有明确的失败处理路径

---

## GitHub 同步规则（必须遵守）

**Remote**：`https://github.com/1zehaoyu1/individual-project`（分支：`main`）

### 每次会话开始时

```bash
git pull origin main
```

### 重要更新后的提交格式

```
[Update #N] <简要描述>

Date: YYYY-MM-DD
Changes:
- <改了什么，为什么>
```

N 通过 `git tag --sort=version:refname | tail -1` 确定后递增。

```bash
git add <具体文件>          # 不要用 git add -A
git commit -m "[Update #N] ..."
git tag vN.0
git push origin main
git push origin vN.0
```

### 硬性约束

- 禁止 force-push（`--force`）
- 禁止删除仓库中的文件（`git rm`）
- 禁止改写已推送的历史（`reset --hard`、`rebase`、`--amend` 已推送提交）
- 只推送到 `main`，不新建分支（除非明确要求）
- 编译产物（`*.o`、`*.elf`、`*.d`）已由 `.gitignore` 排除，不要强制添加
