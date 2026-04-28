/**
 * @file    main.c
 * @brief   基于 STM32G491RETx 的 3S 锂电池管理系统（BMS）主程序
 * @version v6.1
 *
 * 【功能概述】
 * 本固件运行在 NUCLEO-G491RE 开发板上，实现 3S 锂电池组（标称 11.1V，2.2Ah）的：
 *   - 电压/电流采集（INA228，I2C2）
 *   - 温度采集（NTC 热敏电阻，ADC1 CH1）
 *   - SOC 估算（OCV 初始化 + 库仑计数 + 轻量 OCV 校正）
 *   - OLED 显示（SSD1306，I2C3，8+1 页界面）
 *   - 负载控制（MOSFET，GPIO PB5）
 *   - 故障检测（过温 / 过流 / 欠流）
 *   - 运行时可调阈值（F2 设置模式，双击进入）
 *   - Flash 统计数据持久化（F6 页面：启动次数/运行时间/极值/能量/故障）
 *
 * 【架构】裸机超循环（bare-metal superloop），无 RTOS
 *   TIM6 ISR (5ms) → 按键采样去抖 → volatile 边沿标志
 *   主循环：Button_Update → Sensor_Update → Stats_Update → Control_Update → UI_ProcessEvent → Display_Update
 *
 * 【时序】
 *   - 按键扫描：5ms（TIM6 中断）
 *   - 传感器采样：200ms
 *   - 显示刷新：250ms
 *   - SOC 历史记录：5s
 *   - 预估剩余时间（TTE）：10s 滑动窗口
 *   - 统计数据 Flash 保存：1 分钟（仅 dirty 时）
 */

#include "main.h"
#include "stm32g4xx_nucleo.h"   /* Nucleo BSP，提供板载 B1 按钮操作接口 */

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "ssd1306.h"

/* ===== HAL 外设句柄 ===== */
COM_InitTypeDef BspCOMInit;       /* UART 串口配置（调试用） */
ADC_HandleTypeDef hadc1;          /* ADC1：NTC 温度采集 */

I2C_HandleTypeDef hi2c2;          /* I2C2：INA228 电流/电压传感器 */
I2C_HandleTypeDef hi2c3;          /* I2C3：SSD1306 OLED 显示屏 */

/* USER CODE BEGIN 2 */
TIM_HandleTypeDef htim6;          /* TIM6：5ms 定时器中断，用于按键采样去抖 */
/* USER CODE END 2 */

SSD1306 oled;                     /* OLED 驱动结构体（含 128×64/8 = 1024 字节帧缓冲） */

/* ===== I2C 外设映射 ===== */
#define OLED_I2C     hi2c3        /* OLED 挂载在 I2C3 上 */
#define INA228_I2C   hi2c2        /* INA228 挂载在 I2C2 上 */

/* ===== OLED 地址 ===== */
/* SSD1306 7位地址 0x3C，左移1位后写地址为 0x78 */
#define SSD1306_ADDR         0x78

/* ===== INA228 电流/电压传感器配置 ===== */
/* INA228 是 TI 的高精度（20-bit）双向电流/电压传感器 */
#define INA228_ADDR          (0x40 << 1)  /* 7位地址 0x40，HAL 需要左移1位 */
#define INA228_REG_CONFIG    0x00         /* 配置寄存器 */
#define INA228_REG_ADC_CONF  0x01         /* ADC 配置寄存器 */
#define INA228_REG_VSHUNT    0x04         /* 分流电压寄存器（有符号 20-bit） */
#define INA228_REG_VBUS      0x05         /* 母线电压寄存器（无符号 20-bit） */

#define INA228_CONFIG_RST      (1U << 15) /* 位15：软件复位 */
#define INA228_CONFIG_ADCRANGE (1U << 4)  /* 位4：ADC 量程选择 */
#define INA228_ADCRANGE_1      1          /* ADCRANGE=1 → ±40.96 mV（高精度），LSB=78.125 nV */

/* 分流电阻值（欧姆）和母线电压最小有效位 */
#define RSHUNT_OHM             0.015f     /* 15 mΩ 分流电阻 */
#define VBUS_LSB_V             0.0001953125f  /* 母线电压 LSB = 195.3125 µV */

/* ===== NTC 热敏电阻参数（ADC1_CH1，PA0） ===== */
/* 分压电路：VCC → Rfix(27kΩ) → NTC(10kΩ@25°C) → GND
 * ADC 采样 NTC 两端电压，用 Steinhart-Hart 简化公式（β参数法）计算温度 */
#define ADC_VDDA_V      3.3f              /* ADC 参考电压（VDDA） */
#define ADC_FULL_SCALE  4095.0f           /* 12-bit ADC 满量程 */

#define NTC_R0_OHM       10000.0f         /* NTC 25°C 时的标称电阻（10kΩ） */
#define NTC_BETA_K       3988.0f          /* NTC β 常数（3988K） */
#define NTC_T0_K         298.15f          /* 参考温度（25°C = 298.15K） */
#define NTC_RFIX_OHM     27000.0f         /* 分压电阻上拉（27kΩ） */
#define NTC_ADC_CH       ADC_CHANNEL_1    /* ADC 通道 1（PA0） */
/* 分压电路供电电压不再硬编码为 3.3V；
 * 改用 g_vdda_v（通过 VREFINT 动态测量）传入 NTC_TempC_FromDivider */

/* ===== MOSFET 负载控制（PB5） ===== */
/* HIGH = 接通负载，LOW = 断开负载 */
#define MOSFET_GPIO_Port GPIOB
#define MOSFET_Pin       GPIO_PIN_5

/* ===== 按键事件类型 ===== */
/* 主循环事件状态机的输出：通过 Button_Update() 返回 */
typedef enum {
    BTN_EVENT_NONE = 0,    /* 无事件 */
    BTN_EVENT_SINGLE,      /* 单击：正常模式下翻页，设置模式下递增参数 */
    BTN_EVENT_DOUBLE,      /* 双击：正常模式下进入设置，设置模式下保存退出 */
    BTN_EVENT_LONG         /* 长按：设置模式下取消退出 */
} BtnEvent;

/* ===== 按键检测时间参数 ===== */
#define BTN_LONG_MS       1500U   /* 长按判定阈值：持续按住 1.5s */
#define BTN_DOUBLE_MS     300U    /* 双击间隔窗口：第一次松手后 300ms 内再次按下视为双击 */

/* ===== 系统状态机 ===== */
/* 五态状态机控制整个系统行为 */
typedef enum {
    SYS_NORMAL = 0,       /* 正常显示模式：单击翻页，双击进入设置 */
    SYS_SETTING,          /* 设置模式：可编辑运行时阈值（过温/过流/SOC低电量） */
    SYS_FAULT,            /* 故障状态：显示故障信息，需硬件复位清除 */
    SYS_CLEAR_CONFIRM,    /* F6 清零确认页面：单击确认，4秒超时取消 */
    SYS_CLEAR_DONE        /* F6 清零完成提示：显示 1 秒后返回 */
} SysState;

/* ===== 编辑超时 ===== */
/* 设置模式下 10 秒无操作自动退出，不保存修改 */
#define EDIT_TIMEOUT_MS  10000U

/* ===== UI 页面枚举 ===== */
/* 8 页循环显示，单击翻页顺序：SOC → SOC_CURVE → TEMP → VOLT → CURR → TIME → STATS1 → STATS2 → SOC */
typedef enum { UI_SOC = 0, UI_SOC_CURVE, UI_TEMP, UI_VOLT, UI_CURR, UI_TIME, UI_STATS1, UI_STATS2 } UiPage;

/* ===== 故障类型枚举 ===== */
typedef enum { FAULT_NONE = 0, FAULT_WRONG_TEMP, FAULT_WRONG_LOAD } FaultType;

/* ===== 安全阈值常量 ===== */
#define TEMP_HI_C            60.0f               /* 过温故障阈值：60°C 触发 FAULT_WRONG_TEMP */
#define TEMP_MOSFET_OFF_C_DEFAULT  30.0f         /* MOSFET 断开温度默认值（运行时可调） */

/* 过流 / 欠流阈值：400mA / 6mA */
#define I_OVERCURRENT_A_DEFAULT    0.4f          /* 过流故障阈值默认值（运行时可调） */
#define I_UNDERCURRENT_A     0.006f              /* 欠流故障阈值（MOSFET 接通时电流 < 6mA） */
#define LOAD_FAULT_MS        5000U               /* SOC低/温度高/过流/欠流任一持续 > 5s 触发故障 */
#define CAPACITY_AH      2.2f                    /* 电池标称容量（2.2Ah） */
#define V_NOM_PACK_V     11.1f                   /* 3S 标称电压（3.7V × 3 = 11.1V） */

/* ===== 采样和滑动窗口参数 ===== */
#define SAMPLE_MS        200U                    /* 传感器采样周期：200ms */
#define RT_WIN_SEC       10U                     /* TTE（预估剩余时间）滑动窗口：10s */
#define RT_WIN_N         (RT_WIN_SEC*1000U/SAMPLE_MS)  /* 窗口内采样点数：50 */

/* ===== SOC 全局状态 ===== */
static float soc = 0.90f;          /* 当前 SOC（0.0~1.0），默认 90% */
static uint8_t soc_inited = 0;     /* SOC 是否已初始化（0=未初始化，1=已通过 OCV 或预热初始化） */
#define SOC_LOW_THRESH_DEFAULT  0.30f  /* SOC 低电量警告阈值默认值（30%，运行时可调） */

/* ===== F2: 运行时可调阈值 ===== */
/* 设置持久化开关：设为 1 开机从 Flash 加载上次保存的阈值；设为 0 每次恢复默认值 */
/* 当前默认关闭（=0），避免 Flash 擦写次数耗尽，调试稳定后再启用 */
#define ENABLE_SETTINGS_PERSISTENCE  0

/* 三个运行时可调阈值变量（在 F2 设置模式下可修改） */
static float g_mosfet_off_temp = TEMP_MOSFET_OFF_C_DEFAULT;  /* MOSFET 断开温度阈值（°C） */
static float g_overcurrent_a   = I_OVERCURRENT_A_DEFAULT;    /* 过流故障阈值（A） */
static float g_soc_low_thresh  = SOC_LOW_THRESH_DEFAULT;     /* SOC 低电量警告阈值（0~1） */

/* ===== F2: Flash 设置存储参数 ===== */
/* 使用 STM32G491RETx Flash 最后一页（Page 255）存储用户设置
 * Flash 页大小 2KB，地址 0x0807F800，不影响程序代码区 */
#define SETTINGS_FLASH_ADDR  0x0807F800U  /* Flash 末页起始地址（Page 255） */
#define SETTINGS_FLASH_PAGE  255U         /* Flash 页编号 */
#define SETTINGS_MAGIC       0xBEEFCAFEU  /* 魔数，用于判断 Flash 是否已写入有效数据 */

/* ===== F2: EditParamDef - 可编辑参数描述结构体 ===== */
/* 描述一个可在设置模式下编辑的参数，包括其关联页面、值指针、范围和步长 */
typedef struct {
  UiPage page;       /* 关联的显示页（在哪个页面双击可进入编辑） */
  float *pval;       /* 指向运行时变量的指针（修改后直接生效） */
  float  def;        /* 默认值 */
  float  lo;         /* 最小值 */
  float  hi;         /* 最大值 */
  float  step;       /* 步长（每次单击递增量，到达上限后回绕到最小值） */
} EditParamDef;

/* 三个可编辑参数描述表（在 main() 中初始化，C99 不允许非常量初始化器） */
static EditParamDef g_edit_params[3];

/**
 * @brief  根据页面查找对应的可编辑参数描述
 * @param  pg  UI 页面编号
 * @return 指向 EditParamDef 的指针，未找到返回 NULL
 * @note   用于判断某页面是否支持双击进入设置模式
 */
static const EditParamDef *EditParam_FindByPage(UiPage pg)
{
  for (int i = 0; i < 3; i++) {
    if (g_edit_params[i].page == pg) return &g_edit_params[i];
  }
  return (const EditParamDef *)0;
}

/* ===== F6: Flash 统计数据存储 ===== */
/* 使用 STM32G491RETx Flash Page 254 (0x0807F000) 存储统计数据
 * Page 255 已被 F2 设置模式使用 (0x0807F800)
 * 程序代码 + 只读数据 + 初始化数据段结束地址约 0x08008D64，远低于 Page 254 */
#define STATS_FLASH_ADDR   0x0807F000U   /* Flash Page 254 起始地址 */
#define STATS_FLASH_PAGE   254U          /* Flash 页编号 */
#define STATS_MAGIC        0xDEAD1234U   /* 统计数据魔数（区别于设置魔数 0xBEEFCAFE） */

/* 统计数据最后故障类型编码（存储在 Flash 中的 uint32_t） */
#define STATS_LASTFAULT_NONE  0U
#define STATS_LASTFAULT_OT    1U   /* Over-Temperature */
#define STATS_LASTFAULT_OC    2U   /* Over-Current / Wrong Load */

/**
 * @brief  Flash 统计数据结构体（精确 48 字节 = 6 doublewords）
 * @note   STM32G4 Flash 编程粒度为 doubleword (8 字节)
 *         布局：
 *           偏移 0x00: magic           (uint32_t)  魔数
 *           偏移 0x04: boot_count      (uint32_t)  启动次数
 *           偏移 0x08: total_runtime_s (uint32_t)  总运行时间（秒）
 *           偏移 0x0C: _pad0           (uint32_t)  填充
 *           偏移 0x10: hist_max_temp   (float)     历史最高温度
 *           偏移 0x14: total_energy_wh (float)     累计输出能量（Wh）
 *           偏移 0x18: max_current_a   (float)     历史最大电流（A）
 *           偏移 0x1C: max_power_w     (float)     历史最大功率（W）
 *           偏移 0x20: fault_total     (uint32_t)  故障总次数
 *           偏移 0x24: last_fault      (uint32_t)  最后一次故障类型
 *           偏移 0x28: checksum        (uint32_t)  校验和
 *           偏移 0x2C: _pad1           (uint32_t)  填充至 48 字节
 */
typedef struct {
  uint32_t magic;           /* 0x00 */
  uint32_t boot_count;      /* 0x04 */
  uint32_t total_runtime_s; /* 0x08 */
  uint32_t _pad0;           /* 0x0C */
  float    hist_max_temp;   /* 0x10 */
  float    total_energy_wh; /* 0x14 */
  float    max_current_a;   /* 0x18 */
  float    max_power_w;     /* 0x1C */
  uint32_t fault_total;     /* 0x20 */
  uint32_t last_fault;      /* 0x24 */
  uint32_t checksum;        /* 0x28 */
  uint32_t _pad1;           /* 0x2C */
} StatsFlash;

/* 编译期校验结构体大小（必须 = 48 字节 = 6 doublewords） */
_Static_assert(sizeof(StatsFlash) == 48, "StatsFlash must be exactly 48 bytes");

/* ===== F6: RAM 统计变量（运行时更新，低频写入 Flash） ===== */
static uint32_t stats_boot_count      = 0;
static uint32_t stats_total_runtime_s = 0;
static float    stats_hist_max_temp   = -100.0f;   /* 哨兵值：清零后 / 首次启动 */
static float    stats_total_energy_wh = 0.0f;
static float    stats_max_current_a   = 0.0f;
static float    stats_max_power_w     = 0.0f;
static uint32_t stats_fault_total     = 0;
static uint32_t stats_last_fault      = STATS_LASTFAULT_NONE;

static uint8_t  stats_dirty           = 0;         /* 1 = RAM 有未写入 Flash 的变更 */
static uint32_t stats_last_save_tick  = 0;          /* 上次 Flash 保存的 tick */
static uint32_t stats_runtime_accum_ms = 0;         /* 运行时间毫秒累计器（达 1000ms 时进位到 _s） */
static uint8_t  stats_fault_was_active = 0;         /* 故障边沿检测：上一轮是否处于故障状态 */

/* F6 清零确认/完成状态 */
static uint32_t clear_confirm_tick    = 0;          /* 进入确认页的 tick */
static uint32_t clear_done_tick       = 0;          /* 进入完成页的 tick */
static uint32_t btn_lock_until        = 0;          /* 按键锁定截止 tick（清零后 800ms） */
#define CLEAR_CONFIRM_TIMEOUT_MS 4000U              /* 确认页超时：4 秒 */
#define CLEAR_DONE_DISPLAY_MS    1000U              /* 完成提示显示：1 秒 */
#define CLEAR_BTN_LOCK_MS        800U               /* 清零后按键锁定：800ms */

/* Flash 统计保存周期：1 分钟
 * 权衡：短运行场景（< 15 分钟）能保留极值/能量/运行时间，
 *      代价是 Flash 擦写频率提高约 15 倍。
 *      STM32G4 Flash 耐久约 10k 次，按 1 分钟/次估算 ≈ 7 天连续满脏写入 */
#define STATS_SAVE_INTERVAL_MS   (1U * 60U * 1000U)

/**
 * @brief  计算 StatsFlash 校验和（覆盖 magic 和 _pad1 之间的所有字段）
 * @param  s  指向 StatsFlash 的指针
 * @return 校验和（所有 uint32_t 字段 + float 位模式的异或）
 */
static uint32_t Stats_Checksum(const StatsFlash *s)
{
  uint32_t cs = 0;
  cs ^= s->boot_count;
  cs ^= s->total_runtime_s;
  /* float 字段用 memcpy 提取位模式，避免 strict-aliasing 违规 */
  uint32_t tmp;
  memcpy(&tmp, &s->hist_max_temp,   sizeof(uint32_t)); cs ^= tmp;
  memcpy(&tmp, &s->total_energy_wh, sizeof(uint32_t)); cs ^= tmp;
  memcpy(&tmp, &s->max_current_a,   sizeof(uint32_t)); cs ^= tmp;
  memcpy(&tmp, &s->max_power_w,     sizeof(uint32_t)); cs ^= tmp;
  cs ^= s->fault_total;
  cs ^= s->last_fault;
  return cs;
}

/**
 * @brief  从 Flash 加载统计数据到 RAM
 * @note   在 main() 初始化阶段调用。如果 Flash 未写入或校验失败，保持默认值。
 */
static void Stats_LoadFromFlash(void)
{
  const volatile StatsFlash *p =
      (const volatile StatsFlash *)STATS_FLASH_ADDR;

  if (p->magic != STATS_MAGIC) return;   /* 未初始化 */

  /* 构造本地副本用于校验（从 volatile 逐字段拷贝） */
  StatsFlash local;
  local.magic           = p->magic;
  local.boot_count      = p->boot_count;
  local.total_runtime_s = p->total_runtime_s;
  local._pad0           = p->_pad0;
  local.hist_max_temp   = p->hist_max_temp;
  local.total_energy_wh = p->total_energy_wh;
  local.max_current_a   = p->max_current_a;
  local.max_power_w     = p->max_power_w;
  local.fault_total     = p->fault_total;
  local.last_fault      = p->last_fault;
  local.checksum        = p->checksum;
  local._pad1           = p->_pad1;

  if (local.checksum != Stats_Checksum(&local)) return;  /* 校验失败 */

  /* 加载到 RAM */
  stats_boot_count      = local.boot_count;
  stats_total_runtime_s = local.total_runtime_s;
  stats_hist_max_temp   = local.hist_max_temp;
  stats_total_energy_wh = local.total_energy_wh;
  stats_max_current_a   = local.max_current_a;
  stats_max_power_w     = local.max_power_w;
  stats_fault_total     = local.fault_total;
  stats_last_fault      = local.last_fault;

  /* 防御性检查：NaN 替换为安全值 */
  if (isnan(stats_hist_max_temp))   stats_hist_max_temp   = -100.0f;
  if (isnan(stats_total_energy_wh)) stats_total_energy_wh = 0.0f;
  if (isnan(stats_max_current_a))   stats_max_current_a   = 0.0f;
  if (isnan(stats_max_power_w))     stats_max_power_w     = 0.0f;
}

/**
 * @brief  将 RAM 统计数据保存到 Flash
 * @return 1=成功，0=失败
 * @note   写入后 readback 验证
 */
static uint8_t Stats_SaveToFlash(void)
{
  /* 1. 构造写入数据 */
  StatsFlash s;
  s.magic           = STATS_MAGIC;
  s.boot_count      = stats_boot_count;
  s.total_runtime_s = stats_total_runtime_s;
  s._pad0           = 0xFFFFFFFFU;
  s.hist_max_temp   = stats_hist_max_temp;
  s.total_energy_wh = stats_total_energy_wh;
  s.max_current_a   = stats_max_current_a;
  s.max_power_w     = stats_max_power_w;
  s.fault_total     = stats_fault_total;
  s.last_fault      = stats_last_fault;
  s.checksum        = Stats_Checksum(&s);
  s._pad1           = 0xFFFFFFFFU;

  /* 2. 解锁 Flash */
  if (HAL_FLASH_Unlock() != HAL_OK) return 0;

  /* 3. 擦除 Page 254 */
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error = 0;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks     = FLASH_BANK_1;
  erase.Page      = STATS_FLASH_PAGE;
  erase.NbPages   = 1;

  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
  }

  /* 4. 逐 doubleword 写入（48 字节 = 6 doublewords） */
  uint32_t addr = STATS_FLASH_ADDR;
  uint8_t ok = 1;
  const uint8_t *raw = (const uint8_t *)&s;

  for (int i = 0; i < 6; i++) {
    uint64_t dw;
    memcpy(&dw, raw + (uint32_t)i * 8U, 8U);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                          addr, dw) != HAL_OK) {
      ok = 0;
      break;
    }
    addr += 8U;
  }

  HAL_FLASH_Lock();

  /* 5. Readback 验证 */
  if (ok) {
    const volatile StatsFlash *rb =
        (const volatile StatsFlash *)STATS_FLASH_ADDR;
    if (rb->magic != STATS_MAGIC || rb->checksum != s.checksum) {
      ok = 0;
    }
  }

  if (ok) stats_dirty = 0;
  return ok;
}

/**
 * @brief  清零统计数据（保留 boot_count）并立即写入 Flash
 * @return 1=成功，0=失败
 */
static uint8_t Stats_ClearAndSave(void)
{
  /* 保留 boot_count，清零其余 */
  stats_total_runtime_s = 0;
  stats_hist_max_temp   = -100.0f;   /* 哨兵值 */
  stats_total_energy_wh = 0.0f;
  stats_max_current_a   = 0.0f;
  stats_max_power_w     = 0.0f;
  stats_fault_total     = 0;
  stats_last_fault      = STATS_LASTFAULT_NONE;
  stats_runtime_accum_ms = 0;

  return Stats_SaveToFlash();
}

/* ===== F5: SOC 历史采样（用于曲线图显示） ===== */
#define SOC_HIST_N            64U       /* 环形缓冲区大小：64 个采样点 */
#define SOC_HIST_INTERVAL_MS  5000U     /* 采样间隔：5 秒（64×5s = 320 秒 ≈ 5.3 分钟） */

/* ===== 电池图标像素参数 ===== */
/* 电池图标位于 page 2（y16-23），严格在一个 page 高度内（8px）
 * 布局：
 *   外框：宽 86px × 高 7px（含边框），位于 x=2, y=16
 *   正极突起：宽 4px × 高 3px，垂直居中偏移 (7-3)/2=2 → y18-20
 *   内部填充区域：去掉上下边框和内边距各 1px，高 3px，宽最大 82px */
#define BAT_X       2         /* 电池图标左边 x 坐标 */
#define BAT_Y      16         /* 电池图标顶部 y 坐标（对齐 page 2 上沿） */
#define BAT_W      86         /* 电池图标外框宽度（含边框） */
#define BAT_H       7         /* 电池图标外框高度（含边框，7px < 8px page 高度） */
#define BAT_NUB_W   4         /* 正极突起宽度 */
#define BAT_NUB_H   3         /* 正极突起高度 */

/* ===== SOC 历史曲线绘制区域 ===== */
/* 曲线显示在 page 4-5 区域（y32-47），高 16px，宽 128px */
#define CURVE_Y_TOP  32        /* 曲线区域顶部 y 坐标 */
#define CURVE_Y_BOT  47        /* 曲线区域底部 y 坐标 */
#define CURVE_H      16        /* 曲线区域高度（像素） */

/* SOC 历史环形缓冲区 */
static float    soc_hist[SOC_HIST_N];   /* 64 个 SOC 采样值（0.0~1.0） */
static uint32_t soc_hist_idx    = 0;    /* 下一个写入位置（环形递增） */
static uint8_t  soc_hist_filled = 0;     /* 环形缓冲区是否已满（1=已写满一轮，0=首次写入中） */
static uint32_t last_soc_hist   = 0;     /* 上次 SOC 历史采样的 tick */

/* ===== 应用全局状态（供各模块函数访问） ===== */
/* 这些变量从 while(1) 局部提升到文件作用域，使模块化函数可以访问 */
static SysState  sys_state = SYS_NORMAL;  /* 系统状态机当前状态 */
static UiPage    ui_page   = UI_SOC;      /* 当前显示页面 */
static FaultType ui_fault  = FAULT_NONE;  /* 当前故障类型（仅 SYS_FAULT 时有效） */

static float     app_vbus    = 0.0f;      /* 最新母线电压（V），INA228 读取 */
static float     app_ia      = 0.0f;      /* 最新电流（A），正=放电，负=充电，经低通滤波 */
static float     app_tC      = 0.0f;      /* 最新温度（°C），NTC 采集（main() 中初始化为 NAN） */
static float     app_tte_sec = 0.0f;      /* 预估剩余放电时间（秒）（main() 中初始化为 NAN） */

/* 实测 VDDA（通过 VREFINT 动态校准），替代硬编码 3.3V。
 * 每次采样周期刷新一次；首次测量之前使用标称值 3.3V。
 * NTC 分压和 ADC 参考都来自 VDDA 同一根轨，用实测值可消除电源偏差 */
static float     g_vdda_v    = 3.3f;      /* 实测 VDDA 电压（V） */

/* 设置模式临时状态 */
static float     edit_val         = 0.0f; /* 当前正在编辑的参数值（未保存的副本） */
static uint32_t  edit_last_action = 0;    /* 上次编辑操作的时间戳（用于超时检测） */

/* 周期调度时间戳 */
static uint32_t  last_sample_tick = 0;    /* 上次传感器采样的 tick */
static uint32_t  last_draw_tick   = 0;    /* 上次显示刷新的 tick */
static uint32_t  bad_load_ms      = 0;    /* SOC低/温度高/过流/欠流累计持续时间（ms） */

/* ===== 函数前向声明 ===== */
void SystemClock_Config(void);            /* 系统时钟配置（170 MHz） */
static void MX_GPIO_Init(void);           /* GPIO 初始化（MOSFET PB5 等） */
static void MX_ADC1_Init(void);           /* ADC1 初始化（NTC 温度采集） */
static void MX_I2C2_Init(void);           /* I2C2 初始化（INA228） */
static void MX_I2C3_Init(void);           /* I2C3 初始化（OLED） */
/* USER CODE BEGIN 3 */
static void MX_TIM6_Init(void);           /* TIM6 初始化（5ms 按键扫描定时器） */
/* USER CODE END 3 */

/* clampf 前向声明（Flash_LoadSettings 中使用） */
static float clampf(float v, float lo, float hi);

/* ===== F2: Flash 设置存储（条件编译） ===== */
/* 仅当 ENABLE_SETTINGS_PERSISTENCE=1 时编译以下代码
 * Flash 布局（24 字节 = 3 个 doubleword，对齐到 8 字节边界）：
 *   偏移 0x00: magic          (uint32_t) 0xBEEFCAFE — 魔数标识
 *   偏移 0x04: mosfet_off_temp (float)   — MOSFET 断开温度阈值
 *   偏移 0x08: overcurrent_a   (float)   — 过流故障阈值
 *   偏移 0x0C: soc_low_thresh  (float)   — SOC 低电量警告阈值
 *   偏移 0x10: checksum        (uint32_t) — 三个 float 的位模式异或校验
 *   偏移 0x14: padding         (uint32_t) 0xFFFFFFFF（填充到 8 字节对齐）
 */
#if ENABLE_SETTINGS_PERSISTENCE

/* Flash 存储数据结构体（24 字节） */
typedef struct {
  uint32_t magic;           /* 魔数：0xBEEFCAFE 表示数据有效 */
  float    mosfet_off_temp; /* MOSFET 断开温度阈值 */
  float    overcurrent_a;   /* 过流故障阈值 */
  float    soc_low_thresh;  /* SOC 低电量警告阈值 */
  uint32_t checksum;        /* 三个 float 的异或校验和 */
  uint32_t _pad;            /* 填充至 24 字节（3×8 doubleword） */
} SettingsFlash;

/**
 * @brief  计算三个 float 值的异或校验和
 * @param  a, b, c  三个待校验的 float 值
 * @return 三个 float 的位模式（uint32_t）异或结果
 * @note   用 memcpy 避免 strict-aliasing 违规和对齐问题
 */
static uint32_t Settings_Checksum(float a, float b, float c)
{
  uint32_t ua, ub, uc;
  memcpy(&ua, &a, sizeof(uint32_t));  /* float 的位模式复制到 uint32_t */
  memcpy(&ub, &b, sizeof(uint32_t));
  memcpy(&uc, &c, sizeof(uint32_t));
  return ua ^ ub ^ uc;
}

/**
 * @brief  从 Flash 加载用户保存的阈值设置
 * @note   在 main() 初始化阶段调用。如果 Flash 未写入或校验失败，保持默认值不变。
 *         使用 volatile 指针读取 Flash 内存映射区域。
 *         加载后对每个值做防御性边界检查（clampf），防止损坏数据导致异常行为。
 */
static void Flash_LoadSettings(void)
{
  /* 通过内存映射直接读取 Flash 内容（不需要 HAL_FLASH 读取 API） */
  const volatile SettingsFlash *p =
      (const volatile SettingsFlash *)SETTINGS_FLASH_ADDR;

  /* 检查魔数：如果 Flash 未写入过（全 0xFF），magic != 0xBEEFCAFE */
  if (p->magic != SETTINGS_MAGIC) return;  /* 未初始化：保持默认值 */

  /* 校验和验证：检测 Flash 数据是否损坏 */
  uint32_t expected = Settings_Checksum(
      p->mosfet_off_temp, p->overcurrent_a, p->soc_low_thresh);
  if (p->checksum != expected) return;      /* 校验失败：保持默认值 */

  /* 防御性边界检查：限制加载值在合理范围内 */
  g_mosfet_off_temp = clampf(p->mosfet_off_temp, 15.0f, 50.0f);
  g_overcurrent_a   = clampf(p->overcurrent_a,   0.1f,  0.8f);
  g_soc_low_thresh  = clampf(p->soc_low_thresh,  0.05f, 0.50f);
}

/**
 * @brief  将当前阈值设置保存到 Flash
 * @return 1=保存成功，0=保存失败（解锁/擦除/写入任一步骤出错）
 * @note   STM32G4 Flash 编程粒度为 doubleword（8 字节）。
 *         步骤：解锁 → 擦除整页 → 逐 doubleword 写入 → 锁定。
 */
static uint8_t Flash_SaveSettings(void)
{
  /* 1. 解锁 Flash 控制寄存器（写保护） */
  if (HAL_FLASH_Unlock() != HAL_OK) return 0;

  /* 2. 擦除末页（Page 255，2KB） */
  FLASH_EraseInitTypeDef erase;
  uint32_t page_error = 0;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.Banks     = FLASH_BANK_1;
  erase.Page      = SETTINGS_FLASH_PAGE;
  erase.NbPages   = 1;

  if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
    HAL_FLASH_Lock();
    return 0;
  }

  /* 3. 构造要写入的数据（24 字节 = 3 doubleword） */
  SettingsFlash s;
  s.magic          = SETTINGS_MAGIC;
  s.mosfet_off_temp = g_mosfet_off_temp;
  s.overcurrent_a   = g_overcurrent_a;
  s.soc_low_thresh  = g_soc_low_thresh;
  s.checksum        = Settings_Checksum(
      g_mosfet_off_temp, g_overcurrent_a, g_soc_low_thresh);
  s._pad            = 0xFFFFFFFFU;  /* Flash 擦除后的默认值，保持未使用字节为 0xFF */

  /* 4. 逐 doubleword 写入（用 memcpy 避免 strict-aliasing 和对齐问题）
   *   STM32G4 Flash 编程必须以 8 字节（doubleword）为单位 */
  uint32_t addr = SETTINGS_FLASH_ADDR;
  uint8_t ok = 1;
  const uint8_t *raw = (const uint8_t *)&s;

  for (int i = 0; i < 3; i++) {    /* 24 字节 ÷ 8 字节/dw = 3 次 */
    uint64_t dw;
    memcpy(&dw, raw + (uint32_t)i * 8U, 8U);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                          addr, dw) != HAL_OK) {
      ok = 0;
      break;
    }
    addr += 8U;
  }

  /* 5. 重新锁定 Flash（安全措施） */
  HAL_FLASH_Lock();
  return ok;
}
#endif /* ENABLE_SETTINGS_PERSISTENCE */

/* ===== INA228 高精度电流/电压传感器驱动 ===== */
/* INA228 是 TI 的 20-bit 双向电流/电压传感器
 * 数据格式：24-bit 寄存器，有效数据在 [23:4] 位，[3:0] 为保留位
 * 读取时需要右移 4 位提取有效 20-bit 数据 */

/**
 * @brief  读取 INA228 的 20-bit 有符号寄存器值
 * @param  reg  寄存器地址（如 INA228_REG_VSHUNT）
 * @return 20-bit 有符号值（-524288 ~ +524287）；I2C 失败返回 INT32_MIN
 * @note   INA228 的 24-bit 寄存器格式：[23:4] = 有效数据，[3:0] = 保留
 *         读取流程：读 3 字节 → 拼接 24-bit → 右移 4 位 → 20-bit 符号扩展
 *         哨兵值 INT32_MIN 远超合法范围（±2^19），调用方可安全检测 I2C 故障
 */
static int32_t INA228_ReadS20(uint8_t reg)
{
  uint8_t buf[3] = {0};
  if (HAL_I2C_Mem_Read(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK)
    return INT32_MIN;  /* I2C 通信失败，返回哨兵值 */

  /* 拼接 3 字节为 24-bit 原始值 */
  uint32_t raw24 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  /* 右移 4 位，提取 [23:4] 的 20-bit 有效数据 */
  int32_t v = (int32_t)(raw24 >> 4);

  /* 20-bit 符号扩展：如果 bit19=1，高位补 1 */
  if (v & 0x80000) v |= 0xFFF00000;
  return v;
}

/**
 * @brief  读取 INA228 的 20-bit 无符号寄存器值
 * @param  reg  寄存器地址（如 INA228_REG_VBUS）
 * @return 20-bit 无符号值（0 ~ 1048575）；I2C 失败返回 UINT32_MAX
 * @note   用于读取母线电压等无符号物理量
 *         哨兵值 UINT32_MAX 远超合法范围（0~0xFFFFF），调用方可安全检测故障
 */
static uint32_t INA228_ReadU20(uint8_t reg)
{
  uint8_t buf[3] = {0};
  if (HAL_I2C_Mem_Read(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK)
    return UINT32_MAX;  /* I2C 通信失败，返回哨兵值 */

  uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  return (raw >> 4) & 0xFFFFFU;  /* 右移 4 位，掩码取 20-bit */
}

/**
 * @brief  向 INA228 写入 16-bit 寄存器值
 * @param  reg    寄存器地址
 * @param  value  要写入的 16-bit 值（大端序发送）
 * @return HAL 状态
 */
static HAL_StatusTypeDef INA228_Write16(uint8_t reg, uint16_t value)
{
  uint8_t data[2];
  data[0] = (uint8_t)((value >> 8) & 0xFF);  /* 高字节 */
  data[1] = (uint8_t)(value & 0xFF);           /* 低字节 */
  return HAL_I2C_Mem_Write(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
}

/**
 * @brief  初始化 INA228 传感器
 * @note   配置流程：
 *   1. 软件复位（写 CONFIG 寄存器 RST 位）
 *   2. 等待 20ms 复位完成
 *   3. 设置 ADCRANGE=1（±40.96 mV 量程，高精度，LSB=78.125 nV）
 *   4. 配置 ADC：64 次平均，1.052ms 转换时间，连续分流+母线测量模式
 *   初始化失败直接调用 Error_Handler()（不可恢复）
 */
static void INA228_Init_Simple(void)
{
  /* 步骤 1：软件复位 */
  if (INA228_Write16(INA228_REG_CONFIG, INA228_CONFIG_RST) != HAL_OK) {
    Error_Handler();
  }
  HAL_Delay(20);  /* 等待复位完成 */

  /* 步骤 2：设置 ADCRANGE=1，缩小分流电压量程到 ±40.96 mV（提高精度 4 倍） */
  uint16_t cfg = 0;
  if (INA228_ADCRANGE_1) cfg |= INA228_CONFIG_ADCRANGE;
  if (INA228_Write16(INA228_REG_CONFIG, cfg) != HAL_OK) {
    Error_Handler();
  }

  /* 步骤 3：配置 ADC_CONFIG 寄存器 (Address=0x01)
   *   位域布局：MODE[15:12] | VBUSCT[11:9] | VSHCT[8:6] | VTCT[5:3] | AVG[2:0]
   *   AVG[2:0]   = 3   → 64 次平均（降低噪声）
   *   VBUSCT[2:0] = 5   → 1.052ms 母线电压转换时间
   *   VSHCT[2:0]  = 5   → 1.052ms 分流电压转换时间
   *   VTCT[2:0]   = 5   → 1.052ms 温度转换时间（本项目未使用 DIE 温度）
   *   MODE[3:0]   = 0xB → 连续分流+母线测量模式 */
  uint16_t avg    = 0x3;    /* 64 次平均 */
  uint16_t vbusct = 5;      /* 1.052ms */
  uint16_t vshct  = 5;      /* 1.052ms */
  uint16_t vtct   = 5;      /* 1.052ms */
  uint16_t mode   = 0xB;    /* Continuous Shunt + Bus */
  uint16_t adc_conf = (uint16_t)((mode << 12) | (vbusct << 9) | (vshct << 6) | (vtct << 3) | (avg << 0));
  if (INA228_Write16(INA228_REG_ADC_CONF, adc_conf) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  读取母线电压（单位：V）
 * @return 母线电压（V），I2C 失败返回 NAN
 * @note   VBUS = 读取值 × 195.3125 µV（LSB）
 *         对于 3S 锂电池组，范围约 9.0V（空电）~ 12.6V（满充）
 */
static float INA228_Vbus_V(void)
{
  uint32_t v = INA228_ReadU20(INA228_REG_VBUS);
  if (v == UINT32_MAX) return NAN;   /* I2C 失败：返回 NAN，调用方用 isnan() 检测 */
  return (float)v * VBUS_LSB_V;
}

/**
 * @brief  读取电流（单位：A），直接由分流电压/分流电阻计算
 * @return 电流（A），正=放电，负=充电；I2C 失败返回 NAN
 * @note   不依赖 SHUNT_CAL 寄存器校准，直接用物理公式：
 *         I = V_shunt / R_shunt
 *         V_shunt = 读取值 × LSB（ADCRANGE=1 时 LSB = 78.125 nV）
 *         ADCRANGE=0 → LSB = 312.5 nV，量程 ±163.84 mV → 最大电流 ±10.9A
 *         ADCRANGE=1 → LSB = 78.125 nV，量程 ±40.96 mV  → 最大电流 ±2.73A
 */
static float INA228_Current_A(void)
{
  int32_t vsh = INA228_ReadS20(INA228_REG_VSHUNT);
  if (vsh == INT32_MIN) return NAN;  /* I2C 失败：返回 NAN，调用方用 isnan() 检测 */
  float lsb_V = INA228_ADCRANGE_1 ? 78.125e-9f : 312.5e-9f;  /* 根据 ADCRANGE 选择 LSB */
  float v_shunt_V = (float)vsh * lsb_V;     /* 分流电压（V） */
  return v_shunt_V / RSHUNT_OHM;             /* 电流（A）= V/R */
}

/* ===== ADC 采样 + NTC 温度计算 ===== */
/* NTC 分压电路原理：
 *   VCC (3.3V) ─── Rfix (27kΩ) ──┬── NTC (10kΩ@25°C) ── GND
 *                                  │
 *                               ADC1_CH1 (PA0)
 *   ADC 采集中间节点电压 V_node，反推 NTC 电阻值，再用 β 参数法计算温度
 */

/**
 * @brief  读取 ADC 指定通道的多次采样平均值
 * @param  channel  ADC 通道号（如 ADC_CHANNEL_1）
 * @return 8 次采样的平均值（0~4095）；全部超时返回 0
 * @note   采用 8 次软件触发 + 轮询方式，减少 ADC 噪声
 *         采样时间 92.5 个 ADC 时钟周期，确保高阻抗 NTC 信号准确采样
 */
static uint16_t ADC_Read_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;  /* 92.5 周期：适合高阻抗源 */
  sConfig.SingleDiff = ADC_SINGLE_ENDED;             /* 单端输入 */
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  /* 8 次采样取平均，降低随机噪声 */
  uint32_t sum = 0;
  const int N = 8;
  int good = 0;
  for (int i = 0; i < N; i++) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK) {
      sum += (uint16_t)HAL_ADC_GetValue(&hadc1);
      good++;
    }
    HAL_ADC_Stop(&hadc1);
  }
  if (good == 0) return 0;  /* 全部超时，返回 0（NTC 计算会得到异常温度，调用方用 isnan 检测） */
  return (uint16_t)(sum / good);
}

/**
 * @brief  读取 ADC 通道电压（单位：V）
 * @param  channel  ADC 通道号
 * @return 电压值（V），范围 0.0 ~ VDDA
 * @note   用 g_vdda_v（由 VREFINT 动态测量）替代硬编码 3.3V，
 *         消除 VDDA 实际偏离 3.3V 带来的系统误差
 */
static float ADC_Channel_Voltage(uint32_t channel)
{
  uint16_t adc = ADC_Read_Channel(channel);
  return ((float)adc / ADC_FULL_SCALE) * g_vdda_v;
}

/**
 * @brief  通过 VREFINT 测量实际 VDDA 电压
 * @return 实际 VDDA 电压（V）；失败时保持上次值不变
 * @note   STM32G4 片内 VREFINT ≈ 1.212V，出厂校准值 VREFINT_CAL 存于 0x1FFF75AA
 *         公式：VDDA = 3.0V × VREFINT_CAL / VREFINT_ADC
 *         VREFINT 需要较长采样时间（>= 4μs），用 247.5 cycles @ 42.5MHz ADC 时钟。
 *         HAL_ADC_ConfigChannel 会自动使能 ADC 公共寄存器中的 VREFEN 位。
 */
static void ADC_UpdateVDDA(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel      = ADC_CHANNEL_VREFINT;
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;  /* VREFINT 要求 tSTART >= 4μs */
  sConfig.SingleDiff   = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset       = 0;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) return;

  /* 4 次采样取平均（VREFINT 本身稳定，不需要 8 次） */
  uint32_t sum = 0;
  int good = 0;
  for (int i = 0; i < 4; i++) {
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 50) == HAL_OK) {
      sum += HAL_ADC_GetValue(&hadc1);
      good++;
    }
    HAL_ADC_Stop(&hadc1);
  }
  if (good == 0) return;  /* 失败保留旧值 */

  uint16_t vrefint_adc = (uint16_t)(sum / good);
  uint32_t vdda_mv = __HAL_ADC_CALC_VREFANALOG_VOLTAGE(vrefint_adc, ADC_RESOLUTION_12B);

  /* 合理性钳位：VDDA 正常在 2.4~3.6V，超出视为测量异常 */
  if (vdda_mv >= 2400U && vdda_mv <= 3600U) {
    g_vdda_v = (float)vdda_mv / 1000.0f;
  }
}

/**
 * @brief  从 NTC 分压电路计算温度（单位：°C）
 * @param  v_supply  分压电路供电电压（V）
 * @param  v_node    ADC 测量的中间节点电压（V）
 * @return 温度（°C）；输入异常返回 NAN
 * @note   分压电路：VCC → Rfix(27kΩ) → V_node → NTC → GND
 *         计算步骤：
 *         1. 由分压关系反推 NTC 电阻：Rntc = Rfix × V_node / (V_supply - V_node)
 *         2. 用 β 参数法（Steinhart-Hart 简化）计算绝对温度：
 *            1/T = 1/T0 + (1/β) × ln(Rntc/R0)
 *         3. 转换为摄氏度：T(°C) = T(K) - 273.15
 *         边界保护：
 *         - V_node ≈ 0：NTC 短路或断路，返回 NAN
 *         - V_node ≈ V_supply：NTC 开路或极低温，返回 NAN
 *         - Rntc < 1Ω：钳位到 1Ω 防止 logf 参数为 0 或负数
 */
static float NTC_TempC_FromDivider(float v_supply, float v_node)
{
  /* 输入合法性检查 */
  if (v_supply < 0.1f) return NAN;                           /* 供电异常 */
  if (v_node <= 0.0005f) return NAN;                         /* V_node ≈ 0（短路/断路） */
  if (v_node >= (v_supply - 0.0005f)) return NAN;            /* V_node ≈ V_supply（开路） */

  /* 反推 NTC 电阻值 */
  float rntc = NTC_RFIX_OHM * (v_node / (v_supply - v_node));
  if (rntc < 1.0f) rntc = 1.0f;  /* 防止 logf 参数为 0 或负数 */

  /* β 参数法计算温度 */
  float invT = (1.0f / NTC_T0_K) + (1.0f / NTC_BETA_K) * logf(rntc / NTC_R0_OHM);
  float tK = 1.0f / invT;        /* 绝对温度（K） */
  return tK - 273.15f;            /* 转换为摄氏度（°C） */
}

/**
 * @brief  浮点数限幅函数
 * @param  v   输入值
 * @param  lo  下限
 * @param  hi  上限
 * @return 限幅后的值；输入为 NAN 返回 lo
 */
static float clampf(float v, float lo, float hi)
{
  if (isnan(v)) return lo;  /* NAN 视为无效，返回安全下限 */
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* ===== SOC 估算：OCV（开路电压）→ SOC 查找表 ===== */
/* 11 点线性插值查找表，基于单节锂电池的 OCV-SOC 特性曲线
 * 电压范围：3.00V（0%）~ 4.20V（100%），对应 3S 组 9.0V ~ 12.6V
 *
 * 使用方式：V_cell = V_bus / 3（假设 3 节串联均衡），查表得到 SOC */

/**
 * @brief  由单节电池开路电压（OCV）估算 SOC
 * @param  v_cell  单节电池电压（V），范围 3.00 ~ 4.20
 * @return SOC 估计值（0.0 ~ 1.0），超出范围钳位到边界值
 * @note   使用 11 点线性插值：在相邻两个标定点之间做线性插值
 *         表格基于典型 18650 锂电池的 OCV-SOC 曲线
 */
static float SOC_From_OCV_Cell(float v_cell)
{
  /* SOC 标定点（Y 轴） */
  static const float s[] = {0.00f,0.05f,0.10f,0.20f,0.30f,0.40f,0.50f,0.60f,0.70f,0.80f,0.90f,1.00f};
  /* 对应的单节电压标定点（X 轴） */
  static const float v[] = {3.00f,3.25f,3.40f,3.55f,3.65f,3.72f,3.78f,3.84f,3.90f,3.98f,4.08f,4.20f};

  /* 边界钳位 */
  if (v_cell <= v[0]) return s[0];   /* ≤ 3.00V → 0% */
  if (v_cell >= v[11]) return s[11];  /* ≥ 4.20V → 100% */

  /* 线性插值查找 */
  for (int i = 0; i < 11; i++) {
    if (v_cell >= v[i] && v_cell <= v[i+1]) {
      float t = (v_cell - v[i]) / (v[i+1] - v[i]);  /* 归一化插值系数 */
      return s[i] + t * (s[i+1] - s[i]);             /* 线性插值结果 */
    }
  }
  return 0.5f;  /* 兜底（理论上不会执行到这里） */
}

/* ===== 按键模块：TIM6 ISR 采样去抖 + 主循环事件状态机 =====
 *
 * 【v6 架构改造】
 * 将按键采样+去抖从主循环轮询移到 TIM6 定时器 ISR（5ms 硬件周期），
 * 主循环的 Button_Update() 只做事件状态机（单击/双击/长按判定）。
 *
 * 两层架构：
 *   底层（ISR，每 5ms）：
 *     TIM6 ISR → 读 B1 GPIO → 计数式去抖（4×5ms=20ms 确认）
 *           → 边沿检测 → 置位 volatile 标志
 *   上层（主循环，每次迭代）：
 *     Button_Update() → 原子读取 volatile 标志 → 事件状态机 → BtnEvent
 *
 * 解决的问题：主循环中 OLED 刷新（~100ms）和传感器采样会拉伸轮询间隔，
 * 导致按键扫描不及时、快速按下被漏检。ISR 不受主循环阻塞影响。
 *
 * 状态机流程图：
 *   IDLE ──[press]──→ HELD ──[release]──→ WAIT_DOUBLE ──[press]──→ HELD_2ND
 *     ↑                 │                     │                       │
 *     │            [≥1.5s: LONG]         [≥300ms:           [release: DOUBLE]
 *     │                 │                  SINGLE]                  │
 *     │                 ↓                     ↓                      ↓
 *     ←──────── WAIT_RELEASE ←─────────── IDLE ←───────────────────┘
 */

/* ISR → 主循环的边沿标志（在 stm32g4xx_it.c 的 TIM6 ISR 中置位） */
extern volatile uint8_t g_btn_press_edge;     /* 按下边沿：去抖后从松开变为按下 */
extern volatile uint8_t g_btn_release_edge;   /* 松开边沿：去抖后从按下变为松开 */

/* 按键事件状态机的内部状态 */
typedef enum {
    BS_IDLE = 0,         /* 空闲：等待按下 */
    BS_HELD,             /* 已按下：等待松手或长按超时 */
    BS_WAIT_DOUBLE,      /* 第一次松手：等待双击窗口内的第二次按下 */
    BS_HELD_2ND,         /* 第二次按下：等待松手确认双击 */
    BS_WAIT_RELEASE      /* 长按已触发：等待松手回到空闲 */
} BtnInternalState;

/**
 * @brief  按键事件状态机（主循环侧调用）
 * @param  now  当前 tick（HAL_GetTick()）
 * @return 本次调用产生的事件（BTN_EVENT_NONE / SINGLE / DOUBLE / LONG）
 * @note   每次主循环迭代调用一次。内部维护状态机，跨调用追踪按键序列。
 *         读取 ISR 边沿标志时关中断保护，防止读-清之间被 ISR 打断丢失事件。
 *
 *         判定规则：
 *         - 单击：按下 → 松手 → 300ms 内无第二次按下
 *         - 双击：按下 → 松手 → 300ms 内第二次按下 → 松手
 *         - 长按：按下持续 ≥ 1.5s（通过主循环多次调用的时间差判定）
 */
static BtnEvent Button_Update(uint32_t now)
{
  static BtnInternalState state = BS_IDLE;
  static uint32_t state_tick    = 0U;   /* 当前状态的进入时间 */

  /* 原子读取并清除 ISR 产生的边沿标志
   * 关中断保护：防止读和清之间被 ISR 打断，导致事件丢失 */
  __disable_irq();
  uint8_t press_edge   = g_btn_press_edge;
  uint8_t release_edge = g_btn_release_edge;
  g_btn_press_edge   = 0;
  g_btn_release_edge = 0;
  __enable_irq();

  /* --- 事件状态机 --- */
  switch (state) {
  case BS_IDLE:
    if (press_edge) {
      state      = BS_HELD;       /* 检测到按下：进入 HELD 状态 */
      state_tick = now;           /* 记录按下时间 */
    }
    break;

  case BS_HELD:
    if (release_edge) {
      /* 第一次松手：进入双击等待窗口（300ms） */
      state      = BS_WAIT_DOUBLE;
      state_tick = now;           /* 重置计时器，开始双击窗口倒计时 */
    } else if ((now - state_tick) >= BTN_LONG_MS) {
      /* 持续按住超过 1.5s：判定为长按
       * ISR 保证持续按住时 press_edge 不会重复置位，
       * 但主循环会多次进入此分支，用时间差判定长按 */
      state = BS_WAIT_RELEASE;
      return BTN_EVENT_LONG;
    }
    break;

  case BS_WAIT_DOUBLE:
    if (press_edge) {
      /* 双击窗口内检测到第二次按下 */
      state      = BS_HELD_2ND;
      state_tick = now;
    } else if ((now - state_tick) >= BTN_DOUBLE_MS) {
      /* 300ms 超时无第二次按下：确认为单击 */
      state = BS_IDLE;
      return BTN_EVENT_SINGLE;
    }
    break;

  case BS_HELD_2ND:
    if (release_edge) {
      /* 第二次松手：确认双击 */
      state = BS_IDLE;
      return BTN_EVENT_DOUBLE;
    } else if ((now - state_tick) >= BTN_LONG_MS) {
      /* 第二次按住超时：当做双击处理 */
      state = BS_WAIT_RELEASE;
      return BTN_EVENT_DOUBLE;
    }
    break;

  case BS_WAIT_RELEASE:
    if (release_edge) {
      state = BS_IDLE;     /* 松手后回到空闲 */
    }
    break;
  }

  return BTN_EVENT_NONE;
}

/* ===== UI 渲染模块 ===== */
/* 所有 UI 函数操作帧缓冲区（oled.buf），由 Display_Update() 统一调用 SSD1306_Update 发送到屏幕 */

/**
 * @brief  在帧缓冲区中绘制单个像素点
 * @param  x  列坐标（0~127）
 * @param  y  行坐标（0~63）
 * @note   SSD1306 是 page-based 显存：每 8 行为一个 page（0~7）
 *         像素映射：buf[(y/8)*128 + x] 的 bit (y%8)
 */
static void OLED_DrawPixel(uint8_t x, uint8_t y)
{
  if (x >= SSD1306_W || y >= SSD1306_H) return;
  oled.buf[(y / 8u) * SSD1306_W + x] |= (uint8_t)(1u << (y % 8u));
}

/**
 * @brief  绘制页面标题栏（清屏 + 显示标题文字）
 * @param  title  标题字符串（显示在 page 0，即 y0-7）
 */
static void UI_DrawHeader(const char *title)
{
  SSD1306_Clear(&oled);              /* 清空帧缓冲区 */
  SSD1306_DrawString(&oled, 0, 0, title);  /* page 0 显示标题 */
}

/**
 * @brief  绘制页面底部操作提示（页脚）
 * @note   根据 sys_state 和 ui_page 动态显示不同的操作提示
 *         S = 单击（Single），D = 双击（Double）
 *         - 设置模式："S:+step D:save"（单击递增，双击保存）
 *         - 正常模式（可编辑页）："S:page D:edit"（单击翻页，双击进入设置）
 *         - 正常模式（不可编辑页）："S:page"（仅翻页）
 */
static void UI_DrawFooter(void)
{
  if (sys_state == SYS_SETTING) {
    SSD1306_DrawString(&oled, 0, 6, "S:+step D:save");
  } else {
    if (EditParam_FindByPage(ui_page) != (const EditParamDef *)0) {
      SSD1306_DrawString(&oled, 0, 6, "S:page D:edit");
    } else {
      SSD1306_DrawString(&oled, 0, 6, "S:page");
    }
  }
}

/**
 * @brief  绘制 SOC 历史曲线（在 SOC HIST 页面调用）
 * @note   从环形缓冲区 soc_hist[] 中读取历史 SOC 数据，
 *         在 y32-47 区域绘制点状曲线图（每个采样点占 2 像素宽）。
 *         使用环形缓冲区的旧→新顺序绘制，支持缓冲区回绕。
 *         曲线从左到右代表时间流逝（最多 64 点 × 5s = 320 秒）。
 */
static void UI_DrawSOCCurve(void)
{
  uint32_t n = soc_hist_filled ? SOC_HIST_N : soc_hist_idx;  /* 有效采样点数 */
  if (n == 0) return;

  uint32_t oldest  = soc_hist_filled ? soc_hist_idx : 0u;    /* 环形缓冲区中最旧数据的位置 */
  uint32_t x_start = (n < SOC_HIST_N) ? (128u - n * 2u) : 0u; /* 未满时右对齐 */

  for (uint32_t i = 0; i < n; i++) {
    uint32_t ridx   = (oldest + i) % SOC_HIST_N;  /* 环形索引：从旧到新 */
    float    sample = soc_hist[ridx];
    if (sample < 0.0f) sample = 0.0f;   /* 防御性限幅 */
    if (sample > 1.0f) sample = 1.0f;

    uint8_t px = (uint8_t)(x_start + i * 2u);  /* x 坐标（每个点占 2px 宽） */
    uint8_t py = (uint8_t)(CURVE_Y_BOT -
                 (uint8_t)(sample * (float)(CURVE_H - 1) + 0.5f));  /* SOC=1.0 在顶部，SOC=0.0 在底部 */
    OLED_DrawPixel(px,     py);   /* 左半像素 */
    OLED_DrawPixel(px + 1, py);   /* 右半像素（2px 宽点） */
  }
}

/**
 * @brief  绘制 SOC 主页面（电池图标 + 百分比 + 低电量警告）
 * @param  soc_percent  SOC 百分比（0~100）
 * @param  soc_low      低电量标志（1=低于阈值，触发闪烁）
 * @note   布局：
 *         page 0 (y0-7):   标题 "SOC"
 *         page 2 (y16-23): 电池图标（左）+ 百分比文字（右）
 *         page 4 (y32-39): 低电量 "LOW" 警告（仅 soc_low 时显示）
 *         page 6 (y48-55): 操作提示页脚
 *
 *         电池图标绘制：外框（上下左右边框）+ 正极突起 + 内部填充条
 *         低电量时内部填充条闪烁（每帧交替显示/隐藏）
 *         翻转放在绘制之后，确保首帧填充条可见
 */
static void UI_ShowSOC(int soc_percent, uint8_t soc_low)
{
  char txt[8];
  if (soc_percent < 0)   soc_percent = 0;
  if (soc_percent > 100) soc_percent = 100;

  static uint8_t blink_phase = 0;  /* 闪烁相位：0=显示填充，1=隐藏填充 */

  /* page 0: 标题 */
  UI_DrawHeader("SOC");

  /* page 2 (y16-22): 电池图标外框 */
  SSD1306_FillRect(&oled, BAT_X,         BAT_Y,           BAT_W, 1);       /* 顶边 y16 */
  SSD1306_FillRect(&oled, BAT_X,         BAT_Y+BAT_H-1,   BAT_W, 1);       /* 底边 y22 */
  SSD1306_FillRect(&oled, BAT_X,         BAT_Y,           1,     BAT_H);    /* 左边 */
  SSD1306_FillRect(&oled, BAT_X+BAT_W-1, BAT_Y,           1,     BAT_H);    /* 右边 */
  /* 正极突起：居中偏移 (BAT_H-BAT_NUB_H)/2 = 2 → y18-20 */
  SSD1306_FillRect(&oled, BAT_X+BAT_W, BAT_Y+2, BAT_NUB_W, BAT_NUB_H);

  /* 内部填充条（代表 SOC 水平）
   * 低电量时按 blink_phase 闪烁（交替隐藏/显示），翻转在渲染后确保首帧可见
   * 内部可用宽：BAT_W - 4（左右边框和内边距各 1px）= 82px
   * 内部可用高：BAT_H - 4 = 3px */
  if (!(soc_low && blink_phase)) {
    uint8_t fill_w = (uint8_t)((uint32_t)soc_percent * 82u / 100u);
    if (fill_w > 82u) fill_w = 82u;
    if (fill_w > 0u)
      SSD1306_FillRect(&oled, BAT_X+2, BAT_Y+2, fill_w, BAT_H-4);
  }
  /* 翻转放在渲染后：保证低电量首帧填充条可见 */
  if (soc_low) blink_phase ^= 1u;
  else         blink_phase  = 0u;

  /* page 2: 百分比文字，与电池图标同行（x=92 在图标右侧） */
  snprintf(txt, sizeof(txt), "%d%%", soc_percent);
  SSD1306_DrawString(&oled, 92, 2, txt);

  /* page 4: 低电量警告文字（仅 soc_low 时显示） */
  if (soc_low)
    SSD1306_DrawString(&oled, 92, 4, "LOW");

  /* page 6: 操作提示页脚 */
  UI_DrawFooter();
}

/**
 * @brief  绘制 SOC 历史曲线页面
 */
static void UI_ShowSOCCurvePage(void)
{
  UI_DrawHeader("SOC HIST");
  UI_DrawSOCCurve();
  UI_DrawFooter();
}

/**
 * @brief  绘制温度页面
 * @param  tC  温度值（°C），NAN 表示传感器故障
 * @note   显示格式："-25.3 C" 或 "--.- C"（无效值）
 */
static void UI_ShowTemp(float tC)
{
  char l2[24];
  UI_DrawHeader("TEMP");

  if (isnan(tC)) {
    strcpy(l2, "--.- C");  /* 无效温度显示占位符 */
  } else {
    const char *sign = (tC < 0.0f) ? "-" : "";  /* 负温度显示负号 */
    float abs_val = fabsf(tC);
    int ti = (int)abs_val;                                     /* 整数部分 */
    int td = (int)((abs_val - (float)ti) * 10.0f);             /* 小数部分（1位） */
    snprintf(l2, sizeof(l2), "%s%d.%d C", sign, ti, td);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);  /* page 3 居中显示 */
  UI_DrawFooter();
}

/**
 * @brief  绘制电压页面
 * @param  v  母线电压（V），NAN 表示传感器故障
 * @note   显示格式："-12.345 V" 或 "---.--- V"（无效值）
 *         3S 锂电池组正常范围约 9.0V ~ 12.6V
 */
static void UI_ShowVolt(float v)
{
  char l2[24];
  UI_DrawHeader("VOLT");

  if (isnan(v)) {
    strcpy(l2, "---.--- V");
  } else {
    const char *sign = (v < 0.0f) ? "-" : "";
    float abs_val = fabsf(v);
    int vi = (int)abs_val;                                     /* 整数部分（V） */
    int vd = (int)((abs_val - (float)vi) * 1000.0f);           /* 小数部分（mV，3位） */
    snprintf(l2, sizeof(l2), "%s%d.%03d V", sign, vi, vd);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);
  UI_DrawFooter();
}

/**
 * @brief  绘制电流页面
 * @param  ia  电流（A），正=放电，负=充电；NAN 表示传感器故障
 * @note   显示格式："-123.4 mA" 或 "---.- mA"（无效值）
 *         使用 mA 为单位更直观（本项目电流范围 0~400mA）
 */
static void UI_ShowCurr(float ia)
{
  char l2[24];
  UI_DrawHeader("CURR");

  if (isnan(ia)) {
    strcpy(l2, "---.- mA");
  } else {
    float mA = ia * 1000.0f;   /* 转换为 mA */
    const char *sign = (mA < 0.0f) ? "-" : "";
    float abs_mA = fabsf(mA);
    int mi = (int)abs_mA;                                     /* 整数部分（mA） */
    int md = (int)((abs_mA - (float)mi) * 10.0f);             /* 小数部分（0.1mA） */
    snprintf(l2, sizeof(l2), "%s%d.%d mA", sign, mi, md);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);
  UI_DrawFooter();
}

/**
 * @brief  计算预估剩余放电时间（TTE，Time-To-Empty）
 * @param  vbus     当前母线电压（V）
 * @param  ia_abs   当前电流绝对值（A）
 * @param  soc_now  当前 SOC（0~1）
 * @return 预估剩余时间（秒）；数据不足或功率过低返回 NAN
 * @note   算法：
 *         1. 用 10s 滑动窗口计算平均功率 P_avg = V_bus × I_abs
 *         2. 剩余能量 E_rem = SOC × V_nom × Capacity（Wh）
 *         3. TTE = E_rem / P_avg（小时）→ 转换为秒
 *         窗口未满 5 个点或平均功率 < 0.05W 时不计算（避免除零和无意义结果）
 */
static float RT_UpdateAndCompute_TTE_sec(float vbus, float ia_abs, float soc_now)
{
  static float p_buf[RT_WIN_N];        /* 功率环形缓冲区（50 点） */
  static uint32_t idx = 0;             /* 写入位置 */
  static uint8_t filled = 0;           /* 是否已写满一轮 */

  /* 计算瞬时功率并写入环形缓冲区 */
  float p = vbus * ia_abs;             /* 瞬时功率（W） */
  p_buf[idx] = p;
  idx++;
  if (idx >= RT_WIN_N) { idx = 0; filled = 1; }  /* 环形回绕 */

  /* 计算滑动窗口内的平均功率 */
  uint32_t n = filled ? RT_WIN_N : idx;
  if (n < 5) return NAN;               /* 数据点不足，不计算 */

  float sum = 0.0f;
  for (uint32_t i = 0; i < n; i++) sum += p_buf[i];
  float p_avg = sum / (float)n;

  if (p_avg < 0.05f) return NAN;       /* 功率过低（<50mW），无意义 */

  /* 计算剩余能量和 TTE */
  float e_nom_Wh = V_NOM_PACK_V * CAPACITY_AH;  /* 标称总能量（Wh）= 11.1V × 2.2Ah = 24.42Wh */
  float e_rem_Wh = soc_now * e_nom_Wh;           /* 剩余能量（Wh） */
  if (e_rem_Wh < 0.001f) return 0.0f;            /* 几乎没电了 */

  float t_hours = e_rem_Wh / p_avg;   /* 剩余时间（小时） */
  float t_sec = t_hours * 3600.0f;    /* 转换为秒 */
  if (t_sec < 0.0f) t_sec = 0.0f;
  return t_sec;
}

/**
 * @brief  绘制预估剩余时间页面
 * @param  tte_sec  TTE（秒），NAN 表示无法计算
 * @note   显示格式："123 min"（四舍五入到分钟）或 "-- min"（无效值）
 */
static void UI_ShowTime(float tte_sec)
{
  char l2[24];
  UI_DrawHeader("TIME");

  if (isnan(tte_sec)) {
    strcpy(l2, "-- min");
  } else {
    int min = (int)(tte_sec / 60.0f + 0.5f);  /* 秒 → 分钟，四舍五入 */
    if (min < 0) min = 0;
    snprintf(l2, sizeof(l2), "%d min", min);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);
  UI_DrawFooter();
}

/* ===== F2: 编辑模式覆盖层 ===== */
/* 在正常页面绘制完成后，覆盖 page 4 (y32-39) 显示当前编辑值
 * 格式根据页面类型自动选择：
 *   TEMP 页 → "[30 C]"（温度阈值）
 *   CURR 页 → "[400 mA]"（过流阈值）
 *   SOC  页 → "[30%]"（低电量阈值）
 * SSD1306_Update 由 Display_Update 统一调用，此处只操作帧缓冲区 */
/**
 * @brief  绘制设置模式编辑值覆盖层
 * @param  pg   当前页面（决定显示格式）
 * @param  val  当前编辑值
 * @note   在正常页面内容之上覆盖显示，用方括号 [] 标识处于编辑模式
 */
static void UI_ShowEditOverlay(UiPage pg, float val)
{
  char buf[20];

  if (pg == UI_TEMP) {
    int v = (int)roundf(val);
    snprintf(buf, sizeof(buf), "[%d C]", v);       /* 如 "[30 C]" */
  } else if (pg == UI_CURR) {
    int ma = (int)roundf(val * 1000.0f);
    snprintf(buf, sizeof(buf), "[%d mA]", ma);     /* 如 "[400 mA]" */
  } else if (pg == UI_SOC) {
    int pct = (int)roundf(val * 100.0f);
    snprintf(buf, sizeof(buf), "[%d%%]", pct);     /* 如 "[30%]" */
  } else {
    return;  /* 不可编辑的页面：不绘制覆盖层 */
  }

  /* 在 page 4 (y32-39) 显示编辑值 */
  SSD1306_DrawString(&oled, 0, 4, buf);
}

/* ===== F6: 统计页面绘制 ===== */

/**
 * @brief  绘制 F6-1 核心统计页面
 * @note   显示格式：
 *         FLASH STATS
 *         BOOT : 13
 *         RUN  : 05h42m
 *         TMAX : 32.8C
 *         ENER : 1.84Wh
 */
static void UI_ShowStats1(void)
{
  char line[22];
  UI_DrawHeader("FLASH STATS");

  /* BOOT 次数 */
  snprintf(line, sizeof(line), "BOOT : %lu", (unsigned long)stats_boot_count);
  SSD1306_DrawString(&oled, 0, 2, line);

  /* 运行时间：HHhMMm 格式 */
  uint32_t hrs = stats_total_runtime_s / 3600U;
  uint32_t mns = (stats_total_runtime_s % 3600U) / 60U;
  snprintf(line, sizeof(line), "RUN  : %02luh%02lum",
           (unsigned long)hrs, (unsigned long)mns);
  SSD1306_DrawString(&oled, 0, 3, line);

  /* 历史最高温度 */
  if (stats_hist_max_temp <= -99.0f) {
    snprintf(line, sizeof(line), "TMAX : --.-C");
  } else {
    int ti = (int)fabsf(stats_hist_max_temp);
    int td = (int)((fabsf(stats_hist_max_temp) - (float)ti) * 10.0f);
    const char *sign = (stats_hist_max_temp < 0.0f) ? "-" : "";
    snprintf(line, sizeof(line), "TMAX : %s%d.%dC", sign, ti, td);
  }
  SSD1306_DrawString(&oled, 0, 4, line);

  /* 累计能量 */
  {
    int ei = (int)stats_total_energy_wh;
    int ed = (int)((stats_total_energy_wh - (float)ei) * 100.0f);
    snprintf(line, sizeof(line), "ENER : %d.%02dWh", ei, ed);
  }
  SSD1306_DrawString(&oled, 0, 5, line);

  UI_DrawFooter();
}

/**
 * @brief  绘制 F6-2 详细统计页面
 * @note   显示格式：
 *         DETAIL STATS
 *         IMAX : 1.28A
 *         PMAX : 14.6W
 *         FTOT : 4
 *         LAST : OC
 */
static void UI_ShowStats2(void)
{
  char line[22];
  UI_DrawHeader("DETAIL STATS");

  /* 最大电流 */
  {
    int ii = (int)(stats_max_current_a);
    int id = (int)((stats_max_current_a - (float)ii) * 100.0f);
    snprintf(line, sizeof(line), "IMAX : %d.%02dA", ii, id);
  }
  SSD1306_DrawString(&oled, 0, 2, line);

  /* 最大功率 */
  {
    int pi_val = (int)stats_max_power_w;
    int pd = (int)((stats_max_power_w - (float)pi_val) * 10.0f);
    snprintf(line, sizeof(line), "PMAX : %d.%dW", pi_val, pd);
  }
  SSD1306_DrawString(&oled, 0, 3, line);

  /* 故障总次数 */
  snprintf(line, sizeof(line), "FTOT : %lu", (unsigned long)stats_fault_total);
  SSD1306_DrawString(&oled, 0, 4, line);

  /* 最后故障类型 */
  {
    const char *ft_str = "NONE";
    if (stats_last_fault == STATS_LASTFAULT_OT) ft_str = "OT";
    else if (stats_last_fault == STATS_LASTFAULT_OC) ft_str = "OC";
    snprintf(line, sizeof(line), "LAST : %s", ft_str);
  }
  SSD1306_DrawString(&oled, 0, 5, line);

  SSD1306_DrawString(&oled, 0, 7, "S:page D:clear");
}

/**
 * @brief  绘制 F6-C 清零确认页面
 * @note   显示格式：
 *         CLEAR STATS?
 *         Press : YES
 *         Wait  : NO
 */
static void UI_ShowClearConfirm(void)
{
  UI_DrawHeader("CLEAR STATS?");
  SSD1306_DrawString(&oled, 0, 3, "Press : YES");
  SSD1306_DrawString(&oled, 0, 4, "Wait  : NO");
}

/**
 * @brief  绘制清零完成提示页面
 */
static void UI_ShowClearDone(void)
{
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 12, 3, "STATS CLEARED");
}

/**
 * @brief  绘制故障页面（SYS_FAULT 状态下独占显示）
 * @param  f  故障类型
 * @note   显示故障原因和 "RESET to clear" 提示
 *         故障状态需硬件复位才能清除
 */
static void UI_ShowFault(FaultType f)
{
  UI_DrawHeader("FAULT");
  if (f == FAULT_WRONG_TEMP) SSD1306_DrawString(&oled, 0, 3, "WRONG TEMP");
  else if (f == FAULT_WRONG_LOAD) SSD1306_DrawString(&oled, 0, 3, "WRONG LOAD");
  else SSD1306_DrawString(&oled, 0, 3, "UNKNOWN");
  SSD1306_DrawString(&oled, 0, 6, "RESET to clear");
}

/* ===== 开机预热阶段 ===== */
/* 上电后等待传感器稳定（约 3 秒），期间采集有效电压用于 OCV 初始化 SOC */

#define WARMUP_SAMPLES      6          /* 采样次数（不含初始显示） */
#define WARMUP_INTERVAL_MS  500U       /* 每次采样间隔 500ms，总预热 ≈ 3s */

/**
 * @brief  绘制预热进度界面
 * @param  step   当前步骤（0~total）
 * @param  total  总步骤数
 * @note   显示：版本号、进度条、"Sensor warming" 提示
 *         进度条区域：x=4~123, y=33~38，内部填充 118×4 像素
 */
static void UI_ShowWarmup(int step, int total)
{
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 16, 0, "BMS  v6.0");       /* 居中显示版本号 */
  SSD1306_DrawString(&oled, 4,  2, "Sensor warming");   /* 提示文字 */

  /* 进度条外框（像素坐标：y=33-38，x=4-123） */
  SSD1306_FillRect(&oled, 4,   33, 120, 1); /* 顶边 */
  SSD1306_FillRect(&oled, 4,   38, 120, 1); /* 底边 */
  SSD1306_FillRect(&oled, 4,   33, 1,   6); /* 左边 */
  SSD1306_FillRect(&oled, 123, 33, 1,   6); /* 右边 */

  /* 进度填充（内部 118px 宽，4px 高） */
  if (step > 0 && total > 0) {
    uint8_t fill_w = (uint8_t)((uint32_t)step * 118u / (uint32_t)total);
    if (fill_w > 118) fill_w = 118;
    if (fill_w > 0) SSD1306_FillRect(&oled, 5, 34, fill_w, 4);
  }

  SSD1306_Update(&oled);  /* 预热期间直接更新屏幕（不经过 Display_Update） */
}

/**
 * @brief  开机预热主函数
 * @note   流程：
 *   1. 循环 6 次（每次 500ms），读取 INA228 母线电压
 *   2. 过滤有效电压（8.4V~13.0V 范围内，排除 I2C 故障值）
 *   3. 用有效电压平均值计算 OCV 初始 SOC
 *   4. 如果全部无效，soc_inited 保持 0，主循环首次采样时兜底初始化
 *
 *   预热期间 MOSFET 保持断开（main() 中初始化为 RESET）
 */
static void Warmup_Phase(void)
{
  float vbus_sum  = 0.0f;
  int   valid_cnt = 0;

  for (int step = 0; step <= WARMUP_SAMPLES; step++) {
    UI_ShowWarmup(step, WARMUP_SAMPLES);  /* 更新进度条 */
    if (step == WARMUP_SAMPLES) break;    /* 最后一步只更新显示 */

    HAL_Delay(WARMUP_INTERVAL_MS);        /* 等待 500ms */

    float v = INA228_Vbus_V();
    /* 3S LiPo 有效电压范围：8.4V（全空 3×2.8V）~ 13.0V（满充上限 3×4.33V） */
    if (v >= 8.4f && v <= 13.0f) {
      vbus_sum += v;
      valid_cnt++;
    }
  }

  /* 用有效电压的平均值进行 OCV 初始 SOC 估算 */
  if (valid_cnt > 0) {
    float vcell = (vbus_sum / (float)valid_cnt) / 3.0f;  /* 母线电压 / 3 = 单节电压 */
    soc        = SOC_From_OCV_Cell(vcell);                /* OCV 查表得到初始 SOC */
    soc_inited = 1;
  }
  /* valid_cnt==0：soc_inited 保持 0，主循环第一次成功采样时兜底 OCV 初始化 */
}

/* ===== F6: 运行时统计更新 ===== */

/**
 * @brief  运行时统计更新（在 Sensor_Update 内部调用，每 200ms）
 * @param  now  当前 tick
 * @param  dt   实际采样间隔（ms）
 * @note   必须在全局变量（sys_state, app_tC, app_ia, app_vbus, ui_fault）
 *         更新之后调用，因此嵌入在 Sensor_Update 尾部
 */
static void Stats_Update(uint32_t now, uint32_t dt)
{
  /* 运行时间累计（仅 SYS_NORMAL 状态） */
  if (sys_state == SYS_NORMAL) {
    stats_runtime_accum_ms += dt;
    while (stats_runtime_accum_ms >= 1000U) {
      stats_runtime_accum_ms -= 1000U;
      stats_total_runtime_s++;
      stats_dirty = 1;
    }
  }

  /* 极值追踪：温度 */
  if (!isnan(app_tC) && app_tC > stats_hist_max_temp) {
    stats_hist_max_temp = app_tC;
    stats_dirty = 1;
  }

  /* 极值追踪：电流 */
  if (!isnan(app_ia)) {
    float abs_ia = fabsf(app_ia);
    if (abs_ia > stats_max_current_a) {
      stats_max_current_a = abs_ia;
      stats_dirty = 1;
    }
    /* 极值追踪：功率 */
    if (!isnan(app_vbus)) {
      float pw = app_vbus * abs_ia;
      if (pw > stats_max_power_w) {
        stats_max_power_w = pw;
        stats_dirty = 1;
      }
    }
  }

  /* 累计能量（Wh）：P × dt / 3600000 */
  if (!isnan(app_vbus) && !isnan(app_ia)) {
    float p_w = app_vbus * fabsf(app_ia);
    float dE  = p_w * ((float)dt / 3600000.0f);
    if (dE > 0.0f) {
      stats_total_energy_wh += dE;
      stats_dirty = 1;
    }
  }

  /* 故障计数（边沿检测：仅在 SYS_FAULT 刚进入时计数一次） */
  if (sys_state == SYS_FAULT && !stats_fault_was_active) {
    stats_fault_total++;
    /* 记录故障类型 */
    if (ui_fault == FAULT_WRONG_TEMP)      stats_last_fault = STATS_LASTFAULT_OT;
    else if (ui_fault == FAULT_WRONG_LOAD) stats_last_fault = STATS_LASTFAULT_OC;
    stats_dirty = 1;
    stats_fault_was_active = 1;
  } else if (sys_state != SYS_FAULT) {
    stats_fault_was_active = 0;
  }

  /* 定期 Flash 保存（15 分钟，仅 dirty 时） */
  if (stats_dirty && (now - stats_last_save_tick) >= STATS_SAVE_INTERVAL_MS) {
    Stats_SaveToFlash();
    stats_last_save_tick = now;
  }
}

/* ===== 核心模块函数：传感器采样 + SOC 更新 + 故障检测 ===== */
/**
 * @brief  周期性传感器采样与 SOC 更新（每 200ms 执行一次）
 * @param  now  当前 tick
 * @note   调用顺序：
 *   1. NTC 温度采样（ADC，不依赖 I2C，始终执行）
 *   2. INA228 电压/电流采样（I2C，失败时跳过相关更新）
 *   3. 电流低通滤波 + 死区处理
 *   4. TTE（预估剩余时间）计算
 *   5. SOC 更新：库仑计数 + 轻量 OCV 校正
 *   6. SOC 历史记录（每 5s）
 *   7. 过流/欠流故障检测
 *   8. 过温故障检测（不依赖 INA228，始终执行）
 */
static void Sensor_Update(uint32_t now)
{
  if ((now - last_sample_tick) < SAMPLE_MS) return;  /* 未到采样周期 */

  uint32_t dt = now - last_sample_tick;  /* 实际采样间隔（ms） */
  last_sample_tick = now;

  /* 1. NTC 温度采样（ADC 本地采样，不依赖 I2C，始终执行）
   *    先通过 VREFINT 刷新实测 VDDA，再读 NTC 节点电压，
   *    分压电路供电 = ADC 参考 = VDDA，传入实测值消除 VDDA 偏差 */
  ADC_UpdateVDDA();
  float vntc = ADC_Channel_Voltage(NTC_ADC_CH);
  app_tC = NTC_TempC_FromDivider(g_vdda_v, vntc);

  /* 2. INA228 电压/电流采样（I2C 远程采样，可能失败） */
  float new_vbus = INA228_Vbus_V();
  float raw_ia   = INA228_Current_A();

  /* INA228 读取成功才更新电压/电流/SOC/过流故障 */
  if (!isnan(new_vbus) && !isnan(raw_ia)) {
    app_vbus = new_vbus;

    /* 3. 电流一阶低通滤波 + 死区处理
     *   滤波公式：filtered = α × raw + (1-α) × prev，α=0.1（较强平滑）
     *   死区：< 2mA 视为 0（消除 INA228 零点偏移噪声） */
    static float filtered_ia = 0.0f;
    static uint8_t ia_first_run = 1;

    if (ia_first_run) {
      filtered_ia  = raw_ia;      /* 首次采样直接使用原始值 */
      ia_first_run = 0;
    } else {
      filtered_ia = 0.1f * raw_ia + 0.9f * filtered_ia;  /* 一阶低通 */
    }
    if (fabsf(filtered_ia) < 0.002f) filtered_ia = 0.0f;  /* 死区消除 */
    app_ia = filtered_ia;

    /* 4. TTE 计算（10s 滑动窗口平均功率 → 剩余时间） */
    app_tte_sec = RT_UpdateAndCompute_TTE_sec(app_vbus, fabsf(app_ia), soc);

    /* 5. SOC 更新 */
    if (!soc_inited) {
      /* 兜底 OCV 初始化（预热阶段全部失败时，用首次成功采样初始化） */
      float vcell = app_vbus / 3.0f;
      soc        = SOC_From_OCV_Cell(vcell);
      soc_inited = 1;
      /* 跳过本次库仑积分：dt 可能包含预热阶段的长时间间隔，不准确 */
    } else {
      /* 5a. 库仑计数：SOC -= |I| × Δt / Q_total
       *   Q_total = 容量(Ah) × 3600(s/h) = 总电荷量(As) */
      float Q_as = CAPACITY_AH * 3600.0f;
      soc = soc - (fabsf(app_ia) * (dt / 1000.0f)) / Q_as;
      soc = clampf(soc, 0.0f, 1.0f);

      /* 5b. 轻量 OCV 校正：当电流 < 50mA 时，电池接近开路状态
       *   此时 OCV 查表较准确，用 2% 权重缓慢校正库仑计数的累积漂移 */
      if (fabsf(app_ia) < 0.05f) {
        float vcell   = app_vbus / 3.0f;
        float soc_ocv = SOC_From_OCV_Cell(vcell);
        soc = 0.98f * soc + 0.02f * soc_ocv;  /* 98% 库仑 + 2% OCV */
        soc = clampf(soc, 0.0f, 1.0f);
      }

      /* 6. SOC 历史记录：每 5 秒存一次到环形缓冲区（用于曲线图显示） */
      if (now - last_soc_hist >= SOC_HIST_INTERVAL_MS) {
        last_soc_hist = now;
        soc_hist[soc_hist_idx] = soc;
        if (++soc_hist_idx >= SOC_HIST_N) {  /* 环形回绕 */
          soc_hist_idx    = 0;
          soc_hist_filled = 1;
        }
      }
    }

    /* 7. WRONG LOAD 故障检测（SOC/温度/电流任一超阈值持续 5s 触发）
     *   SOC 低：soc_inited 后 soc < g_soc_low_thresh（默认 30%）
     *   温度高：app_tC > g_mosfet_off_temp（默认 30°C，MOSFET 断开阈值）
     *   过流：|I| > g_overcurrent_a（默认 400mA）
     *   欠流：MOSFET 接通且 |I| < 6mA（负载异常断开）
     *   任一异常持续超过 5s 触发 FAULT_WRONG_LOAD */
    if (sys_state != SYS_FAULT) {
      float absI = fabsf(app_ia);
      uint8_t mosfet_on = (HAL_GPIO_ReadPin(MOSFET_GPIO_Port, MOSFET_Pin) == GPIO_PIN_SET);
      uint8_t soc_bad  = (soc_inited && soc < g_soc_low_thresh) ? 1 : 0;
      uint8_t temp_bad = (!isnan(app_tC) && app_tC > g_mosfet_off_temp) ? 1 : 0;
      uint8_t curr_bad = (absI > g_overcurrent_a || (mosfet_on && absI < I_UNDERCURRENT_A)) ? 1 : 0;
      uint8_t load_bad = soc_bad || temp_bad || curr_bad;

      if (load_bad) {
        bad_load_ms += dt;                   /* 累计异常持续时间 */
        if (bad_load_ms >= LOAD_FAULT_MS) {  /* 超过 5s 阈值 */
          ui_fault  = FAULT_WRONG_LOAD;
          sys_state = SYS_FAULT;
        }
      } else {
        bad_load_ms = 0;                     /* 恢复正常，重置累计 */
      }
    }
  }

  /* 8. 温度故障检测（不依赖 INA228，始终执行）
   *   温度 > 60°C 立即触发 FAULT_WRONG_TEMP（无延时，安全优先） */
  if (sys_state != SYS_FAULT && !isnan(app_tC) && app_tC > TEMP_HI_C) {
    ui_fault  = FAULT_WRONG_TEMP;
    sys_state = SYS_FAULT;
  }

  /* 9. F6 统计数据更新（运行时间/极值/能量/故障计数/定期 Flash 保存） */
  Stats_Update(now, dt);
}

/**
 * @brief  MOSFET 负载控制（每次主循环迭代执行）
 * @note   断开条件（任一满足即断开 MOSFET）：
 *         - 温度无效（isnan）
 *         - 温度 > 用户设置的断开阈值（默认 30°C）
 *         - 系统处于故障状态
 *         正常条件：温度有效且 ≤ 阈值且非故障 → 接通 MOSFET
 */
static void Control_Update(void)
{
  if (isnan(app_tC) || (app_tC > g_mosfet_off_temp) || (sys_state == SYS_FAULT)) {
    HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);  /* 断开 */
  } else {
    HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_SET);    /* 接通 */
  }
}

/**
 * @brief  UI 事件处理：将按键事件转化为系统状态转移
 * @param  evt  按键事件（来自 Button_Update()）
 * @param  now  当前 tick（用于超时检测）
 * @note   状态转移规则：
 *   SYS_NORMAL:
 *     - 单击：循环翻页（SOC → SOC_HIST → TEMP → VOLT → CURR → TIME → SOC）
 *     - 双击：进入设置模式（仅限可编辑页面：TEMP/CURR/SOC）
 *   SYS_SETTING:
 *     - 单击：递增参数（到达上限后回绕到下限，四舍五入到步长整数倍）
 *     - 双击：保存参数并退出设置（可选写入 Flash）
 *     - 长按：丢弃修改并退出设置
 *     - 无操作 10s：自动退出（不保存）
 *   SYS_FAULT:
 *     - 所有按键事件被忽略（需硬件复位）
 */
static void UI_ProcessEvent(BtnEvent evt, uint32_t now)
{
  /* 按键锁定期间吞掉所有事件
   * btn_lock_until 由清零完成后设置为 now + 800ms
   * 用 (int32_t)(now - btn_lock_until) < 0 判断是否仍在锁定期
   * 这依赖有符号差值在 ~24 天内正确（远超实际使用场景） */
  if (evt != BTN_EVENT_NONE && (int32_t)(now - btn_lock_until) < 0) {
    evt = BTN_EVENT_NONE;
  }

  /* 无事件时：检查各模式超时 */
  if (evt == BTN_EVENT_NONE) {
    if (sys_state == SYS_SETTING) {
      if ((now - edit_last_action) >= EDIT_TIMEOUT_MS) {
        sys_state = SYS_NORMAL;  /* 超时退出，不保存 */
      }
    } else if (sys_state == SYS_CLEAR_CONFIRM) {
      if ((now - clear_confirm_tick) >= CLEAR_CONFIRM_TIMEOUT_MS) {
        sys_state = SYS_NORMAL;  /* 4 秒超时：取消清零，回 STATS2 */
        ui_page   = UI_STATS2;
      }
    } else if (sys_state == SYS_CLEAR_DONE) {
      if ((now - clear_done_tick) >= CLEAR_DONE_DISPLAY_MS) {
        sys_state = SYS_NORMAL;
        ui_page   = UI_STATS1;   /* 返回 F6-1 */
        btn_lock_until = now + CLEAR_BTN_LOCK_MS;  /* 800ms 按键锁定 */
      }
    }
    return;
  }

  /* 故障优先级 > 清零交互：故障发生时强制退出确认/完成状态 */
  if (sys_state == SYS_FAULT) return;
  if (sys_state == SYS_CLEAR_DONE) return;  /* 完成提示期间忽略按键 */

  switch (sys_state) {
  case SYS_NORMAL:
    if (evt == BTN_EVENT_SINGLE) {
      /* 单击：循环翻页（8 页，包括 STATS2） */
      if (ui_page == UI_SOC)            ui_page = UI_SOC_CURVE;
      else if (ui_page == UI_SOC_CURVE) ui_page = UI_TEMP;
      else if (ui_page == UI_TEMP)      ui_page = UI_VOLT;
      else if (ui_page == UI_VOLT)      ui_page = UI_CURR;
      else if (ui_page == UI_CURR)      ui_page = UI_TIME;
      else if (ui_page == UI_TIME)      ui_page = UI_STATS1;
      else if (ui_page == UI_STATS1)    ui_page = UI_STATS2;
      else                              ui_page = UI_SOC;
    } else if (evt == BTN_EVENT_DOUBLE) {
      if (ui_page == UI_STATS2) {
        /* F6-2 双击：进入清零确认页面
         * STATS2 不是可编辑页面（无 EditParamDef），双击空闲可复用
         * 仍有 SYS_CLEAR_CONFIRM 确认步骤 + 4s 超时防误触 */
        sys_state = SYS_CLEAR_CONFIRM;
        clear_confirm_tick = now;
      } else {
        /* 其他页面双击：进入设置模式（仅可编辑页面） */
        const EditParamDef *ep = EditParam_FindByPage(ui_page);
        if (ep != (const EditParamDef *)0) {
          sys_state        = SYS_SETTING;
          edit_val         = *(ep->pval);   /* 复制当前值作为编辑副本 */
          edit_last_action = now;            /* 重置超时计时器 */
        }
      }
    }
    break;

  case SYS_SETTING:
    if (evt == BTN_EVENT_SINGLE) {
      /* 单击：递增参数值（循环回绕 + 步长对齐） */
      const EditParamDef *ep = EditParam_FindByPage(ui_page);
      if (ep != (const EditParamDef *)0) {
        edit_val += ep->step;
        if (edit_val > ep->hi + ep->step * 0.5f) edit_val = ep->lo;  /* 回绕 */
        edit_val = roundf(edit_val / ep->step) * ep->step;           /* 步长对齐 */
        edit_val = clampf(edit_val, ep->lo, ep->hi);                 /* 限幅 */
        edit_last_action = now;  /* 重置超时计时器 */
      }
    } else if (evt == BTN_EVENT_DOUBLE) {
      /* 双击：保存参数并退出设置模式 */
      const EditParamDef *ep = EditParam_FindByPage(ui_page);
      if (ep != (const EditParamDef *)0) {
        *(ep->pval) = clampf(edit_val, ep->lo, ep->hi);  /* 写入运行时变量 */
#if ENABLE_SETTINGS_PERSISTENCE
        Flash_SaveSettings();  /* 可选：持久化到 Flash */
#endif
      }
      sys_state = SYS_NORMAL;
    } else if (evt == BTN_EVENT_LONG) {
      /* 长按：取消编辑，丢弃修改 */
      sys_state = SYS_NORMAL;
    }
    break;

  case SYS_CLEAR_CONFIRM:
    if (evt == BTN_EVENT_SINGLE) {
      /* 单击确认清零 */
      Stats_ClearAndSave();
      sys_state       = SYS_CLEAR_DONE;
      clear_done_tick = now;
    }
    /* 其他事件（双击/长按）在确认页忽略，仅靠超时取消 */
    break;

  case SYS_CLEAR_DONE:
    /* 完成提示期间忽略所有按键（已在上方 return） */
    break;

  default:
    break;
  }
}

/**
 * @brief  显示刷新（每 250ms 执行一次）
 * @param  now  当前 tick
 * @note   绘制流程：
 *   1. 故障状态：直接显示故障页面
 *   2. 正常/设置状态：根据 ui_page 绘制对应页面
 *   3. 设置模式：在正常页面之上叠加编辑值覆盖层
 *   4. 低电量全局警告（仅正常模式）
 *   5. 统一调用 SSD1306_Update 发送帧缓冲区到屏幕（仅一次）
 */
static void Display_Update(uint32_t now)
{
  if ((now - last_draw_tick) < 250U) return;  /* 未到刷新周期 */
  last_draw_tick = now;

  /* 故障状态：独占显示故障信息（优先级最高） */
  if (sys_state == SYS_FAULT) {
    UI_ShowFault(ui_fault);
    SSD1306_Update(&oled);
    return;
  }

  /* F6 清零确认页面 */
  if (sys_state == SYS_CLEAR_CONFIRM) {
    UI_ShowClearConfirm();
    SSD1306_Update(&oled);
    return;
  }

  /* F6 清零完成提示 */
  if (sys_state == SYS_CLEAR_DONE) {
    UI_ShowClearDone();
    SSD1306_Update(&oled);
    return;
  }

  /* 计算显示用的 SOC 百分比和低电量标志 */
  int soc_percent = (int)(soc * 100.0f + 0.5f);
  uint8_t soc_low = (soc < g_soc_low_thresh) ? 1 : 0;

  /* 根据当前页面绘制内容（8 页） */
  if (ui_page == UI_SOC)            UI_ShowSOC(soc_percent, soc_low);
  else if (ui_page == UI_SOC_CURVE) UI_ShowSOCCurvePage();
  else if (ui_page == UI_TEMP)      UI_ShowTemp(app_tC);
  else if (ui_page == UI_VOLT)      UI_ShowVolt(app_vbus);
  else if (ui_page == UI_CURR)      UI_ShowCurr(app_ia);
  else if (ui_page == UI_TIME)      UI_ShowTime(app_tte_sec);
  else if (ui_page == UI_STATS1)    UI_ShowStats1();
  else if (ui_page == UI_STATS2)    UI_ShowStats2();

  /* 设置模式：在正常页面之上叠加编辑值覆盖层 */
  if (sys_state == SYS_SETTING) {
    UI_ShowEditOverlay(ui_page, edit_val);
  }

  /* 低电量全局警告（仅正常模式下显示，避免与编辑覆盖层冲突） */
  if (soc_low && sys_state == SYS_NORMAL) {
    SSD1306_DrawString(&oled, 0, 5, "SOC TOO LOW");
  }

  /* 统一刷新：所有绘制完成后仅调用一次 SSD1306_Update 发送到屏幕 */
  SSD1306_Update(&oled);
}

/* ===== main() 入口 ===== */

/**
 * @brief  程序入口
 * @note   初始化顺序（严格遵守）：
 *   1. HAL 库 + 系统时钟（170 MHz）
 *   2. GPIO（MOSFET PB5）
 *   3. ADC1（NTC 温度采集）
 *   4. I2C2（INA228）+ I2C3（OLED）
 *   5. TIM6（按键扫描定时器，但未启动）
 *   6. OLED 初始化
 *   7. 板载 B1 按钮初始化（GPIO 轮询模式，TIM6 ISR 读 GPIO）
 *   8. MOSFET 初始断开（安全默认）
 *   9. INA228 初始化
 *   10. 编辑参数描述表初始化
 *   11. 可选 Flash 加载（设置 + 统计数据）
 *   11b. 统计数据：load → boot_count++ → save
 *   12. UART 初始化（调试输出）
 *   13. 预热阶段（~3s，OCV 初始化 SOC）
 *   14. 应用状态初始化（NAN 标记）
 *   15. 定时基准对齐
 *   16. 启动 TIM6 中断（按键扫描开始）
 *   17. 进入主循环
 */
int main(void)
{
  /* 1-2. HAL + 时钟 + GPIO */
  HAL_Init();
  SystemClock_Config();

  /* 3-5. 外设初始化 */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();       /* 配置 TIM6 但未启动，预热期间不产生中断 */

  /* 6. OLED 初始化（SSD1306，I2C3，128×64） */
  SSD1306_Init(&oled, &OLED_I2C, SSD1306_ADDR);

  /* 7. 初始化板载 B1 按钮（GPIO 轮询模式）
   * TIM6 ISR 通过 BSP_PB_GetState() 读取 GPIO 电平 */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* 8. MOSFET 初始断开（安全默认：上电时负载不接通） */
  HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);

  /* 9. INA228 电流/电压传感器初始化 */
  INA228_Init_Simple();

  /* 10. 初始化 F2 编辑参数描述表（关联页面 → 可调阈值变量）
   *   TEMP 页 → g_mosfet_off_temp（MOSFET 断开温度，15~50°C，步长 5°C）
   *   CURR 页 → g_overcurrent_a（过流阈值，0.1~0.8A，步长 0.05A）
   *   SOC  页 → g_soc_low_thresh（低电量阈值，5%~50%，步长 5%） */
  g_edit_params[0] = (EditParamDef){
    .page = UI_TEMP, .pval = &g_mosfet_off_temp,
    .def = TEMP_MOSFET_OFF_C_DEFAULT, .lo = 15.0f, .hi = 50.0f, .step = 5.0f
  };
  g_edit_params[1] = (EditParamDef){
    .page = UI_CURR, .pval = &g_overcurrent_a,
    .def = I_OVERCURRENT_A_DEFAULT, .lo = 0.1f, .hi = 0.8f, .step = 0.05f
  };
  g_edit_params[2] = (EditParamDef){
    .page = UI_SOC, .pval = &g_soc_low_thresh,
    .def = SOC_LOW_THRESH_DEFAULT, .lo = 0.05f, .hi = 0.50f, .step = 0.05f
  };

  /* 11. 可选：从 Flash 加载用户上次保存的阈值 */
#if ENABLE_SETTINGS_PERSISTENCE
  Flash_LoadSettings();
#endif

  /* 11b. F6: 从 Flash 加载统计数据 → 递增 boot_count → 立即回写 */
  Stats_LoadFromFlash();
  stats_boot_count++;
  Stats_SaveToFlash();

  /* 12. UART 串口初始化（115200-8N1，调试输出用） */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  /* 13. 预热阶段（~3 秒）：等待传感器稳定，用 OCV 初始化 SOC */
  Warmup_Phase();

  /* 14. 初始化应用状态为无效值（等待首次采样填充） */
  app_tC      = NAN;   /* 温度未知 */
  app_tte_sec = NAN;   /* TTE 未知 */

  /* 15. 将定时基准对齐到预热结束时刻
   *   避免 first dt 包含预热阶段的 ~3 秒，导致库仑积分第一步偏大 */
  last_sample_tick    = HAL_GetTick();
  last_draw_tick      = HAL_GetTick();
  last_soc_hist       = HAL_GetTick();
  stats_last_save_tick = HAL_GetTick();  /* 统计 Flash 保存基准 */

  /* 16. 启动 TIM6 定时器中断（5ms 周期）
   *   预热结束后才启动，避免预热期间的假按键事件。
   *   首次 ISR 时按钮为松开状态（stable=0），边沿标志不会误触发。 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* 17. 主循环：非阻塞周期调度
   *   每次迭代执行 5 个模块函数，各函数内部自行判断是否到达执行周期
   *   Stats_Update 在 Sensor_Update 内部调用（共享 dt） */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    BtnEvent evt = Button_Update(now);   /* 按键事件检测（读取 ISR 边沿标志） */
    Sensor_Update(now);                   /* 传感器采样 + SOC + 故障 + 统计更新（200ms） */
    Control_Update();                     /* MOSFET 控制（温度保护） */
    UI_ProcessEvent(evt, now);            /* 按键事件 → 状态转移（翻页/设置/故障） */
    Display_Update(now);                  /* OLED 显示刷新（250ms） */
  }
}

/* ===== 以下是 CubeIDE 生成的硬件初始化函数 ===== */
/* 这些函数由 STM32CubeMX 根据 .ioc 配置文件自动生成，一般不需要手动修改 */

/**
 * @brief  系统时钟配置（170 MHz）
 * @note   HSI → PLL → 170 MHz SYSCLK
 *         APB1 = APB2 = HCLK = 170 MHz
 *         Flash Latency = 4 WS（BOOST 模式）
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  ADC1 初始化（12-bit，单端，软件触发，通道 1）
 * @note   用于 NTC 热敏电阻温度采集（PA0 / ADC1_CH1）
 *         采样时间 92.5 周期，适合高阻抗源
 *         独立模式（非双核模式）
 */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  I2C2 初始化（连接 INA228 电流/电压传感器）
 * @note   时序寄存器 0x40B285C2 对应标准模式（100kHz）
 *         7 位寻址，模拟滤波使能
 */
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x40B285C2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  I2C3 初始化（连接 SSD1306 OLED 显示屏）
 * @note   配置与 I2C2 相同（标准模式 100kHz）
 */
static void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x40B285C2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief  GPIO 初始化（MOSFET PB5 推挽输出）
 * @note   启用 GPIOA/B/C/F 时钟，PB5 初始为低电平（MOSFET 断开）
 *         板载 B1 按钮由 BSP_PB_Init() 初始化，此处不再配置外部按钮 GPIO
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, MOSFET_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = MOSFET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  /* 推挽输出 */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOSFET_GPIO_Port, &GPIO_InitStruct);
}

/**
 * @brief  TIM6 基本定时器初始化（5ms 按键扫描周期）
 * @note   时钟源：APB1 = 170 MHz
 *         预分频器：170 MHz / (16999+1) = 10 kHz
 *         自动重载值：10 kHz / (49+1) = 200 Hz → 5ms 周期
 *         中断优先级：1（低于 SysTick 的 0，高于其他外设）
 *
 *         如果 CubeIDE 已通过 .ioc 生成该函数，删除此手写版本，保留生成的即可。
 */
static void MX_TIM6_Init(void)
{
  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  htim6.Init.Prescaler   = 16999;     /* 170 MHz / 17000 = 10 kHz */
  htim6.Init.Period       = 49;       /* 10 kHz / 50 = 200 Hz → 5 ms */
  htim6.Init.CounterMode  = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }

  /* 使能 TIM6 全局中断，优先级 = 1（比 SysTick 低） */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief  错误处理函数（不可恢复的初始化故障）
 * @note   安全措施：关全局中断 + 断开 MOSFET + 死循环
 *         需硬件复位或看门狗才能恢复
 */
void Error_Handler(void)
{
  __disable_irq();                                       /* 关全局中断 */
  HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);  /* 断开负载 */
  while (1) { }                                          /* 死循环 */
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  断言失败回调（开发调试用）
 */
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
