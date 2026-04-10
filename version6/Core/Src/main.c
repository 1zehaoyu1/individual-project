#include "main.h"
#include "stm32g4xx_nucleo.h"   /* 改动1：加入 Nucleo BSP 头文件，用来操作板载 B1 */

#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "ssd1306.h"

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim6;   /* v6: 按键扫描定时器 */

SSD1306 oled;

/* I2C mapping */
#define OLED_I2C     hi2c3
#define INA228_I2C   hi2c2

/* OLED address */
#define SSD1306_ADDR         0x78

/* INA228 */
#define INA228_ADDR          (0x40 << 1)
#define INA228_REG_CONFIG    0x00
#define INA228_REG_ADC_CONF  0x01
#define INA228_REG_VSHUNT    0x04
#define INA228_REG_VBUS      0x05

#define INA228_CONFIG_RST      (1U << 15)
#define INA228_CONFIG_ADCRANGE (1U << 4)
#define INA228_ADCRANGE_1      1

#define RSHUNT_OHM             0.015f
#define VBUS_LSB_V             0.0001953125f

/* NTC (ADC1_CH1) */
#define ADC_VDDA_V      3.3f
#define ADC_FULL_SCALE  4095.0f

#define NTC_R0_OHM       10000.0f
#define NTC_BETA_K       3988.0f
#define NTC_T0_K         298.15f
#define NTC_RFIX_OHM     27000.0f
#define NTC_ADC_CH       ADC_CHANNEL_1
#define NTC_SUPPLY_V     3.3f

/* MOSFET (PB5) */
#define MOSFET_GPIO_Port GPIOB
#define MOSFET_Pin       GPIO_PIN_5

/* 按键去抖参数（与 TIM6 ISR 配合） */
#define BTN_DEBOUNCE_CNT  4U    /* 连续一致次数才确认（4×5ms=20ms） */

/* 可选：打开此宏在 UART 上输出按键调试信息（提交时保持注释） */
/* #define BTN_DEBUG_UART */

/* 按键事件 */
typedef enum {
    BTN_EVENT_NONE = 0,
    BTN_EVENT_SINGLE,
    BTN_EVENT_DOUBLE,
    BTN_EVENT_LONG
} BtnEvent;

/* 按键检测参数 */
#define BTN_LONG_MS       1500U
#define BTN_DOUBLE_MS     300U

/* 系统状态机 */
typedef enum {
    SYS_NORMAL = 0,
    SYS_SETTING,
    SYS_FAULT
} SysState;

/* 编辑超时（10 秒无操作自动退出，不保存） */
#define EDIT_TIMEOUT_MS  10000U

/* Pages */
typedef enum { UI_SOC = 0, UI_SOC_CURVE, UI_TEMP, UI_VOLT, UI_CURR, UI_TIME } UiPage;

/* Faults */
typedef enum { FAULT_NONE = 0, FAULT_WRONG_TEMP, FAULT_WRONG_LOAD } FaultType;

/* thresholds */
#define TEMP_HI_C            60.0f
#define TEMP_MOSFET_OFF_C_DEFAULT  30.0f

/* 过流 / 欠流阈值：400mA / 6mA */
#define I_OVERCURRENT_A_DEFAULT    0.4f
#define I_UNDERCURRENT_A     0.006f
#define LOAD_FAULT_MS        10000U
#define CAPACITY_AH      2.2f
#define V_NOM_PACK_V     11.1f

#define SAMPLE_MS        200U
#define RT_WIN_SEC       10U
#define RT_WIN_N         (RT_WIN_SEC*1000U/SAMPLE_MS)

/* SOC */
static float soc = 0.90f;
static uint8_t soc_inited = 0;
#define SOC_LOW_THRESH_DEFAULT  0.30f

/* ===== F2: 运行时可调阈值 ===== */
/* 设为 1 则开机从 Flash 加载上次保存的阈值；设为 0 则每次开机恢复默认值 */
#define ENABLE_SETTINGS_PERSISTENCE  0
static float g_mosfet_off_temp = TEMP_MOSFET_OFF_C_DEFAULT;
static float g_overcurrent_a   = I_OVERCURRENT_A_DEFAULT;
static float g_soc_low_thresh  = SOC_LOW_THRESH_DEFAULT;

/* F2: Flash 设置存储 */
#define SETTINGS_FLASH_ADDR  0x0807F800U  /* 末页，Page 255 */
#define SETTINGS_FLASH_PAGE  255U
#define SETTINGS_MAGIC       0xBEEFCAFEU

/* F2: EditParamDef - 可编辑参数描述 */
typedef struct {
  UiPage page;       /* 关联的显示页 */
  float *pval;       /* 指向运行时变量 */
  float  def;        /* 默认值 */
  float  lo;         /* 最小值 */
  float  hi;         /* 最大值 */
  float  step;       /* 步长 */
} EditParamDef;

static EditParamDef g_edit_params[3];  /* 在 main() 中初始化，避免 non-const initializer */

static const EditParamDef *EditParam_FindByPage(UiPage pg)
{
  for (int i = 0; i < 3; i++) {
    if (g_edit_params[i].page == pg) return &g_edit_params[i];
  }
  return (const EditParamDef *)0;
}

/* F5 SOC 历史采样 */
#define SOC_HIST_N            64U
#define SOC_HIST_INTERVAL_MS  5000U

/* 电池图标像素参数 */
/* BAT_Y=16: 图标顶部对齐 page 2 上沿（y16-23）              */
/* BAT_H=7 : 图标高 7px（y16-22），严格在 page 2 内           */
/* BAT_NUB_H=3: 突起居中偏移 (7-3)/2=2 → y18-20，在框内     */
/* 内部可填充高 BAT_H-4=3（边框+内边距各1px），宽上限仍 82px  */
#define BAT_X       2
#define BAT_Y      16
#define BAT_W      86
#define BAT_H       7
#define BAT_NUB_W   4
#define BAT_NUB_H   3

/* 历史曲线区域 */
#define CURVE_Y_TOP  32
#define CURVE_Y_BOT  47
#define CURVE_H      16

static float    soc_hist[SOC_HIST_N];
static uint32_t soc_hist_idx    = 0;
static uint8_t  soc_hist_filled = 0;
static uint32_t last_soc_hist   = 0;

/* === 应用状态（从 while(1) 内提升到文件作用域，供模块函数访问） === */
static SysState  sys_state = SYS_NORMAL;
static UiPage    ui_page   = UI_SOC;
static FaultType ui_fault  = FAULT_NONE;

static float     app_vbus    = 0.0f;
static float     app_ia      = 0.0f;
static float     app_tC      = 0.0f;    /* main() 中初始化为 NAN */
static float     app_tte_sec = 0.0f;    /* main() 中初始化为 NAN */

static float     edit_val         = 0.0f;
static uint32_t  edit_last_action = 0;

static uint32_t  last_sample_tick = 0;
static uint32_t  last_draw_tick   = 0;
static uint32_t  bad_load_ms      = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM6_Init(void);

/* clampf 前向声明（Flash_LoadSettings 使用） */
static float clampf(float v, float lo, float hi);

#if ENABLE_SETTINGS_PERSISTENCE
/* ===== F2: Flash 设置存储 =====
 * 布局（20 字节，对齐到 8 字节 doubleword 边界）：
 *   偏移 0x00: magic     (uint32_t) 0xBEEFCAFE
 *   偏移 0x04: mosfet_off_temp (float)
 *   偏移 0x08: overcurrent_a   (float)
 *   偏移 0x0C: soc_low_thresh  (float)
 *   偏移 0x10: checksum        (uint32_t) — 三个 float 的位模式异或
 *   偏移 0x14–0x17: padding (0xFFFFFFFF)
 * 共 24 字节 = 3 个 doubleword
 */

typedef struct {
  uint32_t magic;
  float    mosfet_off_temp;
  float    overcurrent_a;
  float    soc_low_thresh;
  uint32_t checksum;
  uint32_t _pad;
} SettingsFlash;

static uint32_t Settings_Checksum(float a, float b, float c)
{
  uint32_t ua, ub, uc;
  memcpy(&ua, &a, sizeof(uint32_t));
  memcpy(&ub, &b, sizeof(uint32_t));
  memcpy(&uc, &c, sizeof(uint32_t));
  return ua ^ ub ^ uc;
}

static void Flash_LoadSettings(void)
{
  const volatile SettingsFlash *p =
      (const volatile SettingsFlash *)SETTINGS_FLASH_ADDR;

  if (p->magic != SETTINGS_MAGIC) return;  /* 未初始化：保持默认值 */

  uint32_t expected = Settings_Checksum(
      p->mosfet_off_temp, p->overcurrent_a, p->soc_low_thresh);
  if (p->checksum != expected) return;      /* 校验失败：保持默认值 */

  /* 防御性边界检查 */
  g_mosfet_off_temp = clampf(p->mosfet_off_temp, 15.0f, 50.0f);
  g_overcurrent_a   = clampf(p->overcurrent_a,   0.1f,  0.8f);
  g_soc_low_thresh  = clampf(p->soc_low_thresh,  0.05f, 0.50f);
}

static uint8_t Flash_SaveSettings(void)
{
  /* 1. 解锁 Flash */
  if (HAL_FLASH_Unlock() != HAL_OK) return 0;

  /* 2. 擦除末页 */
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
  s._pad            = 0xFFFFFFFFU;

  /* 4. 逐 doubleword 写入（用 memcpy 避免 strict-aliasing 和对齐问题） */
  uint32_t addr = SETTINGS_FLASH_ADDR;
  uint8_t ok = 1;
  const uint8_t *raw = (const uint8_t *)&s;

  for (int i = 0; i < 3; i++) {
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
  return ok;
}
#endif /* ENABLE_SETTINGS_PERSISTENCE */

/* ===== INA228 helpers (按位宽读，避免尺度错误) ===== */

/* 20-bit signed in [23:4] -> >>4 then sign-extend 20-bit
 * 返回 INT32_MIN 表示 I2C 通信失败（合法范围 ±(2^19)，远小于 INT32_MIN） */
static int32_t INA228_ReadS20(uint8_t reg)
{
  uint8_t buf[3] = {0};
  if (HAL_I2C_Mem_Read(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK)
    return INT32_MIN;

  uint32_t raw24 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  int32_t v = (int32_t)(raw24 >> 4);

  if (v & 0x80000) v |= 0xFFF00000;
  return v;
}

/* 20-bit unsigned in [23:4] -> >>4
 * 返回 UINT32_MAX 表示 I2C 通信失败（合法范围 0–0xFFFFF，远小于 UINT32_MAX） */
static uint32_t INA228_ReadU20(uint8_t reg)
{
  uint8_t buf[3] = {0};
  if (HAL_I2C_Mem_Read(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK)
    return UINT32_MAX;

  uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  return (raw >> 4) & 0xFFFFFU;
}

static HAL_StatusTypeDef INA228_Write16(uint8_t reg, uint16_t value)
{
  uint8_t data[2];
  data[0] = (uint8_t)((value >> 8) & 0xFF);
  data[1] = (uint8_t)(value & 0xFF);
  return HAL_I2C_Mem_Write(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
}

static void INA228_Init_Simple(void)
{
  if (INA228_Write16(INA228_REG_CONFIG, INA228_CONFIG_RST) != HAL_OK) {
    Error_Handler();
  }
  HAL_Delay(20);

  uint16_t cfg = 0;
  if (INA228_ADCRANGE_1) cfg |= INA228_CONFIG_ADCRANGE;
  if (INA228_Write16(INA228_REG_CONFIG, cfg) != HAL_OK) {
    Error_Handler();
  }

  /* ADC_CONFIG (Address=0x01)
     AVG[2:0]  : 0=1,1=4,2=16,3=64,4=128,5=256,6=512,7=1024
     VBUSCT    : 5 => 1.052ms
     VSHCT     : 5 => 1.052ms
     VTCT      : 5 => 1.052ms
     MODE      : 0xB => Continuous Shunt + Bus
  */
  uint16_t avg    = 0x3;
  uint16_t vbusct = 5;
  uint16_t vshct  = 5;
  uint16_t vtct   = 5;
  uint16_t mode   = 0xB;
  uint16_t adc_conf = (uint16_t)((mode << 12) | (vbusct << 9) | (vshct << 6) | (vtct << 3) | (avg << 0));
  if (INA228_Write16(INA228_REG_ADC_CONF, adc_conf) != HAL_OK) {
    Error_Handler();
  }
}

static float INA228_Vbus_V(void)
{
  uint32_t v = INA228_ReadU20(INA228_REG_VBUS);
  if (v == UINT32_MAX) return NAN;   /* I2C 失败：返回 NAN，调用方用 isnan() 检测 */
  return (float)v * VBUS_LSB_V;
}

/* “PHY current” from VSHUNT / RSHUNT (no SHUNT_CAL dependency) */
static float INA228_Current_A(void)
{
  int32_t vsh = INA228_ReadS20(INA228_REG_VSHUNT);
  if (vsh == INT32_MIN) return NAN;  /* I2C 失败：返回 NAN，调用方用 isnan() 检测 */
  float lsb_V = INA228_ADCRANGE_1 ? 78.125e-9f : 312.5e-9f;
  float v_shunt_V = (float)vsh * lsb_V;
  return v_shunt_V / RSHUNT_OHM;
}

/* ===== ADC + NTC ===== */

static uint16_t ADC_Read_Channel(uint32_t channel)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_92CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;

  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  uint32_t sum = 0;
  const int N = 8;
  for (int i = 0; i < N; i++) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 50);
    sum += (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
  }
  return (uint16_t)(sum / N);
}

static float ADC_Channel_Voltage(uint32_t channel)
{
  uint16_t adc = ADC_Read_Channel(channel);
  return ((float)adc / ADC_FULL_SCALE) * ADC_VDDA_V;
}

static float NTC_TempC_FromDivider(float v_supply, float v_node)
{
  if (v_supply < 0.1f) return NAN;
  if (v_node <= 0.0005f) return NAN;
  if (v_node >= (v_supply - 0.0005f)) return NAN;

  /* 你的原理图：上Rfix，下NTC */
  float rntc = NTC_RFIX_OHM * (v_node / (v_supply - v_node));
  if (rntc < 1.0f) rntc = 1.0f;

  float invT = (1.0f / NTC_T0_K) + (1.0f / NTC_BETA_K) * logf(rntc / NTC_R0_OHM);
  float tK = 1.0f / invT;
  return tK - 273.15f;
}

static float clampf(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* OCV->SOC (per-cell) */
static float SOC_From_OCV_Cell(float v_cell)
{
  static const float s[] = {0.00f,0.05f,0.10f,0.20f,0.30f,0.40f,0.50f,0.60f,0.70f,0.80f,0.90f,1.00f};
  static const float v[] = {3.00f,3.25f,3.40f,3.55f,3.65f,3.72f,3.78f,3.84f,3.90f,3.98f,4.08f,4.20f};

  if (v_cell <= v[0]) return s[0];
  if (v_cell >= v[11]) return s[11];

  for (int i = 0; i < 11; i++) {
    if (v_cell >= v[i] && v_cell <= v[i+1]) {
      float t = (v_cell - v[i]) / (v[i+1] - v[i]);
      return s[i] + t * (s[i+1] - s[i]);
    }
  }
  return 0.5f;
}

/* ===== 按键模块：TIM6 ISR 采样去抖 + 主循环事件状态机 =====
 *
 * v6 改造：将采样+去抖从主循环轮询移到 TIM6 定时器 ISR（5ms 硬件周期），
 * 主循环的 Button_Update() 只做事件状态机（单击/双击/长按判定）。
 *
 * 架构：
 *   TIM6 ISR (5ms) → 读 GPIO → 计数式去抖 → 边沿检测 → volatile 标志
 *   Button_Update() → 读 volatile 标志 → 事件状态机 → BtnEvent
 *
 * 解决的问题：主循环中 OLED 刷新（~100ms）和传感器采样会拉伸轮询间隔，
 * 导致按键扫描不及时、快速按下被漏检。ISR 不受主循环阻塞影响。
 */

/* ISR → 主循环的边沿标志（在 stm32g4xx_it.c 的 TIM6 ISR 中置位） */
extern volatile uint8_t g_btn_press_edge;
extern volatile uint8_t g_btn_release_edge;

typedef enum {
    BS_IDLE = 0,
    BS_HELD,
    BS_WAIT_DOUBLE,
    BS_HELD_2ND,
    BS_WAIT_RELEASE
} BtnInternalState;

/* 主循环侧：从 ISR 边沿标志驱动事件状态机 */
static BtnEvent Button_Update(uint32_t now)
{
  static BtnInternalState state = BS_IDLE;
  static uint32_t state_tick    = 0U;

  /* 读取并清除 ISR 产生的边沿标志 */
  uint8_t press_edge   = g_btn_press_edge;
  uint8_t release_edge = g_btn_release_edge;
  if (press_edge)   g_btn_press_edge   = 0;
  if (release_edge) g_btn_release_edge = 0;

  /* --- 事件状态机 --- */
  switch (state) {
  case BS_IDLE:
    if (press_edge) {
      state      = BS_HELD;
      state_tick = now;
    }
    break;

  case BS_HELD:
    if (release_edge) {
      /* 第一次松手：进入双击等待窗口 */
      state      = BS_WAIT_DOUBLE;
      state_tick = now;
    } else if ((now - state_tick) >= BTN_LONG_MS) {
      /* 长按触发（ISR 保证持续按住时 press_edge 不会重复，
         但持续按住期间 main loop 多次进入，用时间差判定长按） */
      state = BS_WAIT_RELEASE;
      return BTN_EVENT_LONG;
    }
    break;

  case BS_WAIT_DOUBLE:
    if (press_edge) {
      /* 窗口内第二次按下 */
      state      = BS_HELD_2ND;
      state_tick = now;
    } else if ((now - state_tick) >= BTN_DOUBLE_MS) {
      /* 超时：确认为单击 */
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
      state = BS_IDLE;
    }
    break;
  }

  return BTN_EVENT_NONE;
}

/* ===== UI ===== */
static void OLED_DrawPixel(uint8_t x, uint8_t y)
{
  if (x >= SSD1306_W || y >= SSD1306_H) return;
  oled.buf[(y / 8u) * SSD1306_W + x] |= (uint8_t)(1u << (y % 8u));
}

static void UI_DrawHeader(const char *title)
{
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 0, 0, title);
}

/* 页脚：根据 sys_state / ui_page 动态显示操作提示
 * S = Single click, D = Double click */
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

static void UI_DrawSOCCurve(void)
{
  uint32_t n = soc_hist_filled ? SOC_HIST_N : soc_hist_idx;
  if (n == 0) return;

  uint32_t oldest  = soc_hist_filled ? soc_hist_idx : 0u;
  uint32_t x_start = (n < SOC_HIST_N) ? (128u - n * 2u) : 0u;

  for (uint32_t i = 0; i < n; i++) {
    uint32_t ridx   = (oldest + i) % SOC_HIST_N;
    float    sample = soc_hist[ridx];
    if (sample < 0.0f) sample = 0.0f;
    if (sample > 1.0f) sample = 1.0f;

    uint8_t px = (uint8_t)(x_start + i * 2u);
    uint8_t py = (uint8_t)(CURVE_Y_BOT -
                 (uint8_t)(sample * (float)(CURVE_H - 1) + 0.5f));
    OLED_DrawPixel(px,     py);
    OLED_DrawPixel(px + 1, py);
  }
}

static void UI_ShowSOC(int soc_percent, uint8_t soc_low)
{
  char txt[8];
  if (soc_percent < 0)   soc_percent = 0;
  if (soc_percent > 100) soc_percent = 100;

  static uint8_t blink_phase = 0;

  /* page 0: 标题 */
  UI_DrawHeader("SOC");

  /* page 2 (y16-22): 电池图标外框，单行高度 BAT_H=7 */
  SSD1306_FillRect(&oled, BAT_X,         BAT_Y,           BAT_W, 1);       /* 顶边 y16 */
  SSD1306_FillRect(&oled, BAT_X,         BAT_Y+BAT_H-1,   BAT_W, 1);       /* 底边 y22 */
  SSD1306_FillRect(&oled, BAT_X,         BAT_Y,           1,     BAT_H);    /* 左边 */
  SSD1306_FillRect(&oled, BAT_X+BAT_W-1, BAT_Y,           1,     BAT_H);    /* 右边 */
  /* 正极突起：居中偏移 (BAT_H-BAT_NUB_H)/2 = 2 → y18-20 */
  SSD1306_FillRect(&oled, BAT_X+BAT_W, BAT_Y+2, BAT_NUB_W, BAT_NUB_H);

  /* 内部填充（低电量时按 blink_phase 闪烁隐藏，翻转在渲染后以确保首帧可见）
   * 内部可用宽：BAT_W - 左边框(1) - 左内边距(1) - 右内边距(1) - 右边框(1) = 82
   * 内部可用高：BAT_H - 上边框(1) - 上内边距(1) - 下内边距(1) - 下边框(1) = 3 → BAT_H-4 */
  if (!(soc_low && blink_phase)) {
    uint8_t fill_w = (uint8_t)((uint32_t)soc_percent * 82u / 100u);
    if (fill_w > 82u) fill_w = 82u;
    if (fill_w > 0u)
      SSD1306_FillRect(&oled, BAT_X+2, BAT_Y+2, fill_w, BAT_H-4);
  }
  /* 翻转放在渲染后：保证低电量首帧填充条可见 */
  if (soc_low) blink_phase ^= 1u;
  else         blink_phase  = 0u;

  /* page 2: 百分比文字，与图标同行（x=92 在图标右侧，DrawString 第三参数为 page 编号） */
  snprintf(txt, sizeof(txt), "%d%%", soc_percent);
  SSD1306_DrawString(&oled, 92, 2, txt);

  /* page 4: 低电量警告（仅 soc_low 时显示，有足够空白） */
  if (soc_low)
    SSD1306_DrawString(&oled, 92, 4, "LOW");

  /* page 6: 页脚 */
  UI_DrawFooter();
  SSD1306_Update(&oled);
}

static void UI_ShowSOCCurvePage(void)
{
  UI_DrawHeader("SOC HIST");
  UI_DrawSOCCurve();
  UI_DrawFooter();
  SSD1306_Update(&oled);
}

static void UI_ShowTemp(float tC)
{
  char l2[24];
  UI_DrawHeader("TEMP");

  if (isnan(tC)) strcpy(l2, "--.- C");
  else {
    int ti = (int)tC;
    int td = (int)fabsf((tC - (float)ti) * 10.0f);
    snprintf(l2, sizeof(l2), "%d.%d C", ti, td);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);
  UI_DrawFooter();
  SSD1306_Update(&oled);
}

static void UI_ShowVolt(float v)
{
  char l2[24];
  UI_DrawHeader("VOLT");

  if (isnan(v)) {
    strcpy(l2, "---.--- V");
  } else {
    int vi = (int)v;
    int vd = (int)fabsf((v - (float)vi) * 1000.0f);
    snprintf(l2, sizeof(l2), "%d.%03d V", vi, vd);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);
  UI_DrawFooter();
  SSD1306_Update(&oled);
}

static void UI_ShowCurr(float ia)
{
  char l2[24];
  UI_DrawHeader("CURR");

  if (isnan(ia)) {
    strcpy(l2, "---.- mA");
  } else {
    float mA = ia * 1000.0f;
    int mi = (int)mA;
    int md = (int)fabsf((mA - (float)mi) * 10.0f);
    snprintf(l2, sizeof(l2), "%d.%d mA", mi, md);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);
  UI_DrawFooter();
  SSD1306_Update(&oled);
}

static float RT_UpdateAndCompute_TTE_sec(float vbus, float ia_abs, float soc_now)
{
  static float p_buf[RT_WIN_N];
  static uint32_t idx = 0;
  static uint8_t filled = 0;

  float p = vbus * ia_abs;
  p_buf[idx] = p;
  idx++;
  if (idx >= RT_WIN_N) { idx = 0; filled = 1; }

  uint32_t n = filled ? RT_WIN_N : idx;
  if (n < 5) return NAN;

  float sum = 0.0f;
  for (uint32_t i = 0; i < n; i++) sum += p_buf[i];
  float p_avg = sum / (float)n;

  if (p_avg < 0.05f) return NAN;

  float e_nom_Wh = V_NOM_PACK_V * CAPACITY_AH;
  float e_rem_Wh = soc_now * e_nom_Wh;
  if (e_rem_Wh < 0.001f) return 0.0f;

  float t_hours = e_rem_Wh / p_avg;
  float t_sec = t_hours * 3600.0f;
  if (t_sec < 0.0f) t_sec = 0.0f;
  return t_sec;
}

static void UI_ShowTime(float tte_sec)
{
  char l2[24];
  UI_DrawHeader("TIME");

  if (isnan(tte_sec)) {
    strcpy(l2, "-- min");
  } else {
    int min = (int)(tte_sec / 60.0f + 0.5f);
    if (min < 0) min = 0;
    snprintf(l2, sizeof(l2), "%d min", min);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);
  UI_DrawFooter();
  SSD1306_Update(&oled);
}

/* F2: 编辑模式值显示覆盖层
 * 在正常页面绘制完成后（SSD1306_Update 已调用），覆盖 page 4 (y32) 显示编辑值，
 * 然后再次 SSD1306_Update。
 * 格式：TEMP -> "[30 C]"  CURR -> "[400 mA]"  SOC -> "[30%]"
 */
static void UI_ShowEditOverlay(UiPage pg, float val)
{
  char buf[20];

  if (pg == UI_TEMP) {
    int v = (int)roundf(val);
    snprintf(buf, sizeof(buf), "[%d C]", v);
  } else if (pg == UI_CURR) {
    int ma = (int)roundf(val * 1000.0f);
    snprintf(buf, sizeof(buf), "[%d mA]", ma);
  } else if (pg == UI_SOC) {
    int pct = (int)roundf(val * 100.0f);
    snprintf(buf, sizeof(buf), "[%d%%]", pct);
  } else {
    return;  /* 不可编辑的页面 */
  }

  /* 在 page 4 (y32-39) 显示编辑值 */
  SSD1306_DrawString(&oled, 0, 4, buf);
  SSD1306_Update(&oled);
}

static void UI_ShowFault(FaultType f)
{
  UI_DrawHeader("FAULT");
  if (f == FAULT_WRONG_TEMP) SSD1306_DrawString(&oled, 0, 3, "WRONG TEMP");
  else if (f == FAULT_WRONG_LOAD) SSD1306_DrawString(&oled, 0, 3, "WRONG LOAD");
  else SSD1306_DrawString(&oled, 0, 3, "UNKNOWN");
  SSD1306_DrawString(&oled, 0, 6, "RESET to clear");
  SSD1306_Update(&oled);
}

/* ===== 开机预热 ===== */

#define WARMUP_SAMPLES      6
#define WARMUP_INTERVAL_MS  500U

static void UI_ShowWarmup(int step, int total)
{
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 16, 0, "BMS  v4.0");
  SSD1306_DrawString(&oled, 4,  2, "Sensor warming");

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

  SSD1306_Update(&oled);
}

static void Warmup_Phase(void)
{
  float vbus_sum  = 0.0f;
  int   valid_cnt = 0;

  for (int step = 0; step <= WARMUP_SAMPLES; step++) {
    UI_ShowWarmup(step, WARMUP_SAMPLES);
    if (step == WARMUP_SAMPLES) break;

    HAL_Delay(WARMUP_INTERVAL_MS);

    float v = INA228_Vbus_V();
    /* 3S LiPo 有效电压范围 8.4V（全空）~ 13.0V（满充上限） */
    if (v >= 8.4f && v <= 13.0f) {
      vbus_sum += v;
      valid_cnt++;
    }
  }

  if (valid_cnt > 0) {
    float vcell = (vbus_sum / (float)valid_cnt) / 3.0f;
    soc        = SOC_From_OCV_Cell(vcell);
    soc_inited = 1;
  }
  /* valid_cnt==0：soc_inited 保持 0，主循环第一次采样时兜底 OCV 初始化 */
}

/* ===== 模块函数：传感器采样 + SOC + 故障检测 ===== */
static void Sensor_Update(uint32_t now)
{
  if ((now - last_sample_tick) < SAMPLE_MS) return;

  uint32_t dt = now - last_sample_tick;
  last_sample_tick = now;

  /* NTC 温度采样（不依赖 I2C，始终执行） */
  float vntc = ADC_Channel_Voltage(NTC_ADC_CH);
  app_tC = NTC_TempC_FromDivider(NTC_SUPPLY_V, vntc);

  /* INA228 电压/电流 */
  float new_vbus = INA228_Vbus_V();
  float raw_ia   = INA228_Current_A();

  /* INA228 读取成功才更新电压/电流/SOC/过流故障 */
  if (!isnan(new_vbus) && !isnan(raw_ia)) {
    app_vbus = new_vbus;

    /* 一阶低通滤波 + 死区 */
    static float filtered_ia = 0.0f;
    static uint8_t ia_first_run = 1;

    if (ia_first_run) {
      filtered_ia  = raw_ia;
      ia_first_run = 0;
    } else {
      filtered_ia = 0.1f * raw_ia + 0.9f * filtered_ia;
    }
    if (fabsf(filtered_ia) < 0.002f) filtered_ia = 0.0f;
    app_ia = filtered_ia;

    /* TTE 计算 */
    app_tte_sec = RT_UpdateAndCompute_TTE_sec(app_vbus, fabsf(app_ia), soc);

    /* SOC 更新 */
    if (!soc_inited) {
      float vcell = app_vbus / 3.0f;
      soc        = SOC_From_OCV_Cell(vcell);
      soc_inited = 1;
      /* 跳过本次库仑积分：dt = 开机到现在的时间，不代表采样间隔 */
    } else {
      /* Coulomb counting */
      float Q_as = CAPACITY_AH * 3600.0f;
      soc = soc - (fabsf(app_ia) * (dt / 1000.0f)) / Q_as;
      soc = clampf(soc, 0.0f, 1.0f);

      /* Light OCV correction when current small */
      if (fabsf(app_ia) < 0.05f) {
        float vcell   = app_vbus / 3.0f;
        float soc_ocv = SOC_From_OCV_Cell(vcell);
        soc = 0.98f * soc + 0.02f * soc_ocv;
        soc = clampf(soc, 0.0f, 1.0f);
      }

      /* SOC 历史记录：每 5 s 存一次 */
      if (now - last_soc_hist >= SOC_HIST_INTERVAL_MS) {
        last_soc_hist = now;
        soc_hist[soc_hist_idx] = soc;
        if (++soc_hist_idx >= SOC_HIST_N) {
          soc_hist_idx    = 0;
          soc_hist_filled = 1;
        }
      }
    }

    /* 过流/欠流故障检测（依赖 INA228） */
    if (sys_state != SYS_FAULT) {
      float absI = fabsf(app_ia);
      uint8_t load_bad = (absI > g_overcurrent_a || absI < I_UNDERCURRENT_A) ? 1 : 0;

      if (load_bad) {
        bad_load_ms += dt;
        if (bad_load_ms >= LOAD_FAULT_MS) {
          ui_fault  = FAULT_WRONG_LOAD;
          sys_state = SYS_FAULT;
        }
      } else {
        bad_load_ms = 0;
      }
    }
  }

  /* 温度故障检测（不依赖 INA228，始终执行） */
  if (sys_state != SYS_FAULT && !isnan(app_tC) && app_tC > TEMP_HI_C) {
    ui_fault  = FAULT_WRONG_TEMP;
    sys_state = SYS_FAULT;
  }
}

/* ===== 模块函数：MOSFET 控制 ===== */
static void Control_Update(void)
{
  if (isnan(app_tC) || (app_tC > g_mosfet_off_temp) || (sys_state == SYS_FAULT)) {
    HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_SET);
  }
}

/* ===== 模块函数：UI 事件处理（按键事件 → 状态转移） ===== */
static void UI_ProcessEvent(BtnEvent evt, uint32_t now)
{
  /* 无事件：仅检查编辑超时 */
  if (evt == BTN_EVENT_NONE) {
    if (sys_state == SYS_SETTING) {
      if ((now - edit_last_action) >= EDIT_TIMEOUT_MS) {
        sys_state = SYS_NORMAL;
      }
    }
    return;
  }

  /* 故障状态下忽略所有按键 */
  if (sys_state == SYS_FAULT) return;

  switch (sys_state) {
  case SYS_NORMAL:
    if (evt == BTN_EVENT_SINGLE) {
      /* 单击翻页 */
      if (ui_page == UI_SOC)            ui_page = UI_SOC_CURVE;
      else if (ui_page == UI_SOC_CURVE) ui_page = UI_TEMP;
      else if (ui_page == UI_TEMP)      ui_page = UI_VOLT;
      else if (ui_page == UI_VOLT)      ui_page = UI_CURR;
      else if (ui_page == UI_CURR)      ui_page = UI_TIME;
      else                              ui_page = UI_SOC;
    } else if (evt == BTN_EVENT_DOUBLE) {
      /* 双击进入设置（仅可编辑页） */
      const EditParamDef *ep = EditParam_FindByPage(ui_page);
      if (ep != (const EditParamDef *)0) {
        sys_state        = SYS_SETTING;
        edit_val         = *(ep->pval);
        edit_last_action = now;
      }
    }
    break;

  case SYS_SETTING:
    if (evt == BTN_EVENT_SINGLE) {
      /* 单击递增参数（循环回绕） */
      const EditParamDef *ep = EditParam_FindByPage(ui_page);
      if (ep != (const EditParamDef *)0) {
        edit_val += ep->step;
        if (edit_val > ep->hi + ep->step * 0.5f) edit_val = ep->lo;
        edit_val = roundf(edit_val / ep->step) * ep->step;
        edit_val = clampf(edit_val, ep->lo, ep->hi);
        edit_last_action = now;
      }
    } else if (evt == BTN_EVENT_DOUBLE) {
      /* 双击保存并退出 */
      const EditParamDef *ep = EditParam_FindByPage(ui_page);
      if (ep != (const EditParamDef *)0) {
        *(ep->pval) = clampf(edit_val, ep->lo, ep->hi);
#if ENABLE_SETTINGS_PERSISTENCE
        Flash_SaveSettings();
#endif
      }
      sys_state = SYS_NORMAL;
    } else if (evt == BTN_EVENT_LONG) {
      /* 长按取消：丢弃修改，回到正常模式 */
      sys_state = SYS_NORMAL;
    }
    break;

  default:
    break;
  }
}

/* ===== 模块函数：显示刷新（250 ms） ===== */
static void Display_Update(uint32_t now)
{
  if ((now - last_draw_tick) < 250U) return;
  last_draw_tick = now;

  if (sys_state == SYS_FAULT) {
    UI_ShowFault(ui_fault);
    return;
  }

  int soc_percent = (int)(soc * 100.0f + 0.5f);
  uint8_t soc_low = (soc < g_soc_low_thresh) ? 1 : 0;

  if (ui_page == UI_SOC)            UI_ShowSOC(soc_percent, soc_low);
  else if (ui_page == UI_SOC_CURVE) UI_ShowSOCCurvePage();
  else if (ui_page == UI_TEMP)      UI_ShowTemp(app_tC);
  else if (ui_page == UI_VOLT)      UI_ShowVolt(app_vbus);
  else if (ui_page == UI_CURR)      UI_ShowCurr(app_ia);
  else                              UI_ShowTime(app_tte_sec);

  /* 设置模式覆盖层 */
  if (sys_state == SYS_SETTING) {
    UI_ShowEditOverlay(ui_page, edit_val);
  }

  /* 低电量全局警告（仅普通模式） */
  if (soc_low && sys_state == SYS_NORMAL) {
    SSD1306_DrawString(&oled, 0, 5, "SOC TOO LOW");
    SSD1306_Update(&oled);
  }
}

/* ========================================================= */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM6_Init();

  SSD1306_Init(&oled, &OLED_I2C, SSD1306_ADDR);

  /* 改动4：初始化板载 B1。
     用 GPIO 模式即可，因为你现在的翻页逻辑本来就是轮询 + 去抖，不需要中断。 */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* 预热期间 MOSFET 保持断开，由主循环第一次采样后根据温度决定是否接通 */
  HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);

  INA228_Init_Simple();

  /* F2: 初始化编辑参数描述表 */
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

  /* F2: 从 Flash 加载用户保存的阈值（覆盖默认值） */
#if ENABLE_SETTINGS_PERSISTENCE
  Flash_LoadSettings();
#endif

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  Warmup_Phase();

  /* 初始化应用状态 */
  app_tC      = NAN;
  app_tte_sec = NAN;

  /* 将定时基准对齐到当前时刻，避免首次 dt 包含预热阶段的 ~3 秒 */
  last_sample_tick = HAL_GetTick();
  last_draw_tick   = HAL_GetTick();
  last_soc_hist    = HAL_GetTick();

  /* v6: 预热结束后启动 TIM6，ISR 开始采样按键。
     预热期间 TIM6 未启动，不会产生假事件。
     首次 ISR 读取时按钮为松开状态（stable=0），边沿不会误触发。 */
  HAL_TIM_Base_Start_IT(&htim6);

  /* ===== 主循环：非阻塞周期调度 ===== */
  while (1)
  {
    uint32_t now = HAL_GetTick();

    BtnEvent evt = Button_Update(now);
    Sensor_Update(now);
    Control_Update();
    UI_ProcessEvent(evt, now);
    Display_Update(now);
  }
}

/* ===== Below: keep your generated code style ===== */

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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, MOSFET_Pin, GPIO_PIN_RESET);

  /* 改动5：这里删掉了原来外部 PA1 按钮的 GPIO 初始化
     因为现在按钮改成板载 B1，由 BSP_PB_Init() 负责初始化 */

  GPIO_InitStruct.Pin = MOSFET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOSFET_GPIO_Port, &GPIO_InitStruct);
}

/* ===== v6: TIM6 初始化 — 5ms 周期中断用于按键采样 ===== */
static void MX_TIM6_Init(void)
{
  __HAL_RCC_TIM6_CLK_ENABLE();

  htim6.Instance = TIM6;
  /* APB1 timer clock = 170 MHz (HCLK = SYSCLK = 170 MHz, APB1 prescaler = 1) */
  htim6.Init.Prescaler   = 17000 - 1;  /* 170 MHz / 17000 = 10 kHz */
  htim6.Init.Period       = 50 - 1;    /* 10 kHz / 50 = 200 Hz → 5 ms */
  htim6.Init.CounterMode  = TIM_COUNTERMODE_UP;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
    Error_Handler();
  }

  /* 优先级低于 SysTick（=15），但高于应用逻辑即可 */
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  /* 注意：不在此处 HAL_TIM_Base_Start_IT，预热结束后再启动 */
}

void Error_Handler(void)
{
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
