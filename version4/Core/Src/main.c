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

/* Pages */
typedef enum { UI_SOC = 0, UI_SOC_CURVE, UI_TEMP, UI_VOLT, UI_CURR, UI_TIME } UiPage;

/* Faults */
typedef enum { FAULT_NONE = 0, FAULT_WRONG_TEMP, FAULT_WRONG_LOAD } FaultType;

/* thresholds */
#define TEMP_HI_C                60.0f
#define DEFAULT_MOSFET_OFF_TEMP  30.0f
#define MOSFET_OFF_TEMP_MIN      15.0f
#define MOSFET_OFF_TEMP_MAX      50.0f
#define MOSFET_OFF_TEMP_STEP      5.0f
static float g_mosfet_off_temp = DEFAULT_MOSFET_OFF_TEMP;

/* 过流 / 欠流阈值 */
#define DEFAULT_OVERCURRENT_A  0.4f
#define OVERCURRENT_A_MIN      0.1f
#define OVERCURRENT_A_MAX      0.8f
#define OVERCURRENT_A_STEP     0.05f
static float g_overcurrent_a = DEFAULT_OVERCURRENT_A;
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
#define DEFAULT_SOC_LOW_THRESH  0.30f
#define SOC_LOW_MIN             0.05f
#define SOC_LOW_MAX             0.50f
#define SOC_LOW_STEP            0.05f
static float g_soc_low_thresh = DEFAULT_SOC_LOW_THRESH;

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

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);

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

/* ===== 可编辑参数描述表 (F2) ===== */
typedef struct {
    UiPage      page;
    float       def_val, min_val, max_val, step;
    float      *p_runtime;
    const char *unit;       /* 显示单位 */
    uint8_t     is_int;     /* 1=整数显示 0=需换算 */
} EditParamDef;

#define EDIT_PARAM_COUNT 3
static const EditParamDef g_edit_params[EDIT_PARAM_COUNT] = {
    { UI_TEMP, 30.0f,  15.0f, 50.0f, 5.0f,   &g_mosfet_off_temp, "C",  1 },
    { UI_CURR,  0.4f,   0.1f,  0.8f, 0.05f,  &g_overcurrent_a,   "mA", 0 },
    { UI_SOC,   0.30f,  0.05f, 0.50f, 0.05f, &g_soc_low_thresh,  "%",  0 },
};

static int EditParam_FindByPage(UiPage pg)
{
  for (int i = 0; i < EDIT_PARAM_COUNT; i++)
    if (g_edit_params[i].page == pg) return i;
  return -1;
}

/* ===== App mode ===== */
typedef enum { MODE_NORMAL = 0, MODE_EDIT } AppMode;
static AppMode   g_mode               = MODE_NORMAL;
static int       g_edit_idx           = -1;
static float     g_edit_val           = 0.0f;
static uint32_t  g_edit_last_activity = 0U;
#define EDIT_TIMEOUT_MS  10000U

/* ===== Button (multi-click, immediate-first) ===== */
/* 策略：首次松手立即产生 BTN_SINGLE（翻页无延迟），
   后续在 MULTI_CLICK_MS 窗口内继续按则升级为 BTN_DOUBLE / BTN_TRIPLE。
   双击/三连击处理方需回退首次单击产生的翻页副作用。 */
typedef enum { BTN_NONE = 0, BTN_SINGLE, BTN_DOUBLE, BTN_TRIPLE } BtnEvent;
#define BTN_CLICK_MAX_MS   500U    /* 单次点击最长按住时间 */
#define MULTI_CLICK_MS     400U    /* 连击判定窗口 */
static uint32_t g_btn_press_start  = 0U;
static uint8_t  g_btn_pressing     = 0U;
static uint8_t  g_click_count      = 0U;
static uint32_t g_last_release     = 0U;
static BtnEvent g_btn_event        = BTN_NONE;

/* ===== Bottom hint (two layers: flash > default) ===== */
/* 默认提示始终显示；临时闪现提示（如 "Saved"）在 HINT_FLASH_MS 后消退回默认。 */
static char     g_hint_default[24] = {0};     /* 持久：由模式/页面决定 */
static char     g_hint_flash[24]   = {0};     /* 临时：事件触发，带超时 */
static uint32_t g_hint_flash_tick  = 0U;
#define HINT_FLASH_MS  1500U

/* 设置持久默认提示（每帧调用，无超时） */
static void Hint_SetDefault(const char *s) {
  strncpy(g_hint_default, s, sizeof(g_hint_default) - 1U);
  g_hint_default[sizeof(g_hint_default) - 1U] = '\0';
}

/* 设置临时闪现提示（事件触发：Saved / ERR / Cancelled） */
static void Hint_Flash(const char *s) {
  strncpy(g_hint_flash, s, sizeof(g_hint_flash) - 1U);
  g_hint_flash[sizeof(g_hint_flash) - 1U] = '\0';
  g_hint_flash_tick = HAL_GetTick();
}

/* 获取当前应显示的提示：flash 优先，超时后回退到 default */
static const char *Hint_Get(void) {
  if (g_hint_flash[0] != '\0' &&
      (HAL_GetTick() - g_hint_flash_tick) < HINT_FLASH_MS)
    return g_hint_flash;
  if (g_hint_default[0] != '\0') return g_hint_default;
  return NULL;
}

/* ===== Button debounce =====
   改动3：这里不再读外部 PA1，而是改成读板载 B1。
   仍然保留你原来的”去抖 + 按下一次触发一次事件”逻辑。

   Button_Debounce_Sync()：在长时间不轮询按钮之后（例如 Warmup_Phase 结束后）
   调用一次，将状态机的 last/stable/tick 全部对齐到当前物理电平和当前 tick，
   避免因 tick=0 与 HAL_GetTick() 差值过大而在首次轮询时产生假按键事件。
*/
/* 文件级标志：Button_Debounce_Sync() 置位，Button_Update() 消费。
   必须定义在两个函数之前，避免前向引用。                          */
static volatile uint8_t g_btn_sync_needed = 0U;

static void Button_Debounce_Sync(void)
{
  /* 通过文件级 flag 通知 Button_Update() 在下次入口处执行同步。        */
  g_btn_sync_needed = 1U;
}

static void Button_Update(void)
{
  static uint8_t  candidate      = 0U;
  static uint32_t candidate_tick = 0U;
#define BTN_DEBOUNCE_MS  50U

  uint8_t r = (uint8_t)BSP_PB_GetState(BUTTON_USER);

  /* 复用 warmup 同步标志 */
  if (g_btn_sync_needed) {
    g_btn_sync_needed = 0U;
    candidate         = 0U;
    g_click_count     = 0U;
    g_btn_pressing    = (r == 0U) ? 1U : 0U;
    if (g_btn_pressing) g_btn_press_start = HAL_GetTick();
    return;
  }

  if (!g_btn_pressing) {
    if (r == 0U) {
      if (!candidate) {
        candidate      = 1U;
        candidate_tick = HAL_GetTick();
      } else if ((HAL_GetTick() - candidate_tick) >= BTN_DEBOUNCE_MS) {
        g_btn_pressing    = 1U;
        g_btn_press_start = candidate_tick;
        candidate         = 0U;
      }
    } else {
      candidate = 0U;
    }
  } else {
    if (r != 0U) {                          /* 松手 */
      uint32_t dur = HAL_GetTick() - g_btn_press_start;
      g_btn_pressing = 0U;
      candidate      = 0U;
      if (dur < BTN_CLICK_MAX_MS) {         /* 有效点击 */
        g_click_count++;
        g_last_release = HAL_GetTick();
        /* 立即响应：每次有效松手都立即产生/升级事件。
           主循环在 10ms 内消费；若后续松手在窗口内，
           产生更高级别事件，由主循环处理（回退副作用）。 */
        if (g_click_count == 1U)      g_btn_event = BTN_SINGLE;
        else if (g_click_count == 2U) g_btn_event = BTN_DOUBLE;
        else                          g_btn_event = BTN_TRIPLE;
      }
      /* 按住超过 500ms 的不算点击，忽略 */
    }
  }

  /* 连击窗口到期 → 清除计数器，为下一轮连击做准备 */
  if (g_click_count > 0U &&
      (HAL_GetTick() - g_last_release) >= MULTI_CLICK_MS) {
    g_click_count = 0U;
  }
}

static BtnEvent Button_ConsumeEvent(void)
{
  BtnEvent e  = g_btn_event;
  g_btn_event = BTN_NONE;
  return e;
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

static void UI_ShowSOC(int soc_percent, uint8_t soc_low,
                       uint8_t edit_mode, float edit_val)
{
  char txt[24];
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

  /* 内部填充（低电量时按 blink_phase 闪烁隐藏，翻转在渲染后以确保首帧可见） */
  if (!(soc_low && blink_phase)) {
    uint8_t fill_w = (uint8_t)((uint32_t)soc_percent * 82u / 100u);
    if (fill_w > 82u) fill_w = 82u;
    if (fill_w > 0u)
      SSD1306_FillRect(&oled, BAT_X+2, BAT_Y+2, fill_w, BAT_H-4);
  }
  if (soc_low) blink_phase ^= 1u;
  else         blink_phase  = 0u;

  /* page 2: 百分比文字 */
  snprintf(txt, sizeof(txt), "%d%%", soc_percent);
  SSD1306_DrawString(&oled, 92, 2, txt);

  if (edit_mode) {
    /* 编辑中：显示低电量阈值 */
    int ev = (int)(edit_val * 100.0f + 0.5f);
    snprintf(txt, sizeof(txt), "[%d%%]", ev);
    SSD1306_DrawString(&oled, 0, 4, txt);
    SSD1306_DrawString(&oled, 0, 6, "1x:+5 2x:save");
  } else {
    if (soc_low)
      SSD1306_DrawString(&oled, 92, 4, "LOW");
    { const char *_h = Hint_Get(); if (_h) SSD1306_DrawString(&oled, 0, 6, _h); }
  }
  SSD1306_Update(&oled);
}

static void UI_ShowSOCCurvePage(void)
{
  UI_DrawHeader("SOC HIST");
  UI_DrawSOCCurve();
  { const char *_h = Hint_Get(); if (_h) SSD1306_DrawString(&oled, 0, 6, _h); }
  SSD1306_Update(&oled);
}

static void UI_ShowTemp(float tC, uint8_t edit_mode, float edit_val)
{
  char l2[24];
  UI_DrawHeader("TEMP");

  /* 当前温度（实测值） */
  if (isnan(tC)) strcpy(l2, "--.- C");
  else {
    int ti = (int)tC;
    int td = (int)fabsf((tC - (float)ti) * 10.0f);
    snprintf(l2, sizeof(l2), "%d.%d C", ti, td);
  }
  SSD1306_DrawString(&oled, 0, 2, l2);

  if (edit_mode) {
    /* 编辑中：显示目标阈值 */
    int ei = (int)edit_val;
    snprintf(l2, sizeof(l2), "[%d C]", ei);
    SSD1306_DrawString(&oled, 0, 4, l2);
    SSD1306_DrawString(&oled, 0, 6, "1x:+5 2x:save");
  } else {
    /* 正常：显示当前生效阈值 */
    int gi = (int)g_mosfet_off_temp;
    snprintf(l2, sizeof(l2), "set:%d C", gi);
    SSD1306_DrawString(&oled, 0, 4, l2);
    { const char *_h = Hint_Get(); if (_h) SSD1306_DrawString(&oled, 0, 6, _h); }
  }
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
  { const char *_h = Hint_Get(); if (_h) SSD1306_DrawString(&oled, 0, 6, _h); }
  SSD1306_Update(&oled);
}

static void UI_ShowCurr(float ia, uint8_t edit_mode, float edit_val)
{
  char l2[24];
  UI_DrawHeader("CURR");

  float mA = ia * 1000.0f;
  int mi = (int)mA;
  int md = (int)fabsf((mA - (float)mi) * 10.0f);
  snprintf(l2, sizeof(l2), "%d.%d mA", mi, md);
  SSD1306_DrawString(&oled, 0, 2, l2);

  if (edit_mode) {
    /* 编辑中：显示过流阈值（A → mA） */
    int ev = (int)(edit_val * 1000.0f + 0.5f);
    snprintf(l2, sizeof(l2), "[%d mA]", ev);
    SSD1306_DrawString(&oled, 0, 4, l2);
    SSD1306_DrawString(&oled, 0, 6, "1x:+50 2x:save");
  } else {
    { const char *_h = Hint_Get(); if (_h) SSD1306_DrawString(&oled, 0, 6, _h); }
  }
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
  { const char *_h = Hint_Get(); if (_h) SSD1306_DrawString(&oled, 0, 6, _h); }
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

/* ===== Flash 设置存储（F2：3 个阈值 + checksum） ===== */
#define SETTINGS_FLASH_ADDR  0x0807F800UL
#define SETTINGS_MAGIC       0xBEEFCAFEUL

/*  Flash 布局（3 doubleword = 24 字节）：
 *  DW0 (0x00): [31:0] magic        [63:32] mosfet_off_temp
 *  DW1 (0x08): [31:0] overcurrent  [63:32] soc_low_thresh
 *  DW2 (0x10): [31:0] checksum     [63:32] 0xFFFFFFFF  */

static uint32_t Settings_Checksum(const uint32_t *data, uint32_t count)
{
  uint32_t sum = 0U;
  for (uint32_t i = 0; i < count; i++) sum += data[i];
  return sum;
}

static void Flash_LoadSettings(void)
{
  const uint32_t *p = (const uint32_t *)SETTINGS_FLASH_ADDR;
  uint32_t words[5];  /* magic, temp, curr, soc, checksum */
  memcpy(words, p, sizeof(words));

  if (words[0] != SETTINGS_MAGIC) return; /* 首次上电 */
  if (words[4] != Settings_Checksum(words, 4)) return; /* 校验失败 */

  float val;
  memcpy(&val, &words[1], 4);
  if (val >= MOSFET_OFF_TEMP_MIN && val <= MOSFET_OFF_TEMP_MAX)
    g_mosfet_off_temp = val;

  memcpy(&val, &words[2], 4);
  if (val >= OVERCURRENT_A_MIN && val <= OVERCURRENT_A_MAX)
    g_overcurrent_a = val;

  memcpy(&val, &words[3], 4);
  if (val >= SOC_LOW_MIN && val <= SOC_LOW_MAX)
    g_soc_low_thresh = val;
}

static uint8_t Flash_SaveSettings(void)
{
  if (HAL_FLASH_Unlock() != HAL_OK) return 0U;

  FLASH_EraseInitTypeDef er = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Banks     = FLASH_BANK_1,
    .Page      = 255U,
    .NbPages   = 1U
  };
  uint32_t page_err = 0U;
  if (HAL_FLASHEx_Erase(&er, &page_err) != HAL_OK || page_err != 0xFFFFFFFFU) {
    HAL_FLASH_Lock();
    return 0U;
  }

  /* 组装数据 */
  uint32_t raw_magic = SETTINGS_MAGIC;
  uint32_t raw_temp, raw_curr, raw_soc;
  memcpy(&raw_temp, &g_mosfet_off_temp, 4);
  memcpy(&raw_curr, &g_overcurrent_a,   4);
  memcpy(&raw_soc,  &g_soc_low_thresh,  4);

  uint32_t ck_data[4] = { raw_magic, raw_temp, raw_curr, raw_soc };
  uint32_t cksum = Settings_Checksum(ck_data, 4);

  /* DW0: magic + temp */
  uint64_t dw0 = (uint64_t)raw_magic | ((uint64_t)raw_temp << 32);
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTINGS_FLASH_ADDR, dw0) != HAL_OK) {
    HAL_FLASH_Lock(); return 0U;
  }
  /* DW1: curr + soc */
  uint64_t dw1 = (uint64_t)raw_curr | ((uint64_t)raw_soc << 32);
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTINGS_FLASH_ADDR + 8U, dw1) != HAL_OK) {
    HAL_FLASH_Lock(); return 0U;
  }
  /* DW2: checksum + padding */
  uint64_t dw2 = (uint64_t)cksum | ((uint64_t)0xFFFFFFFFU << 32);
  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTINGS_FLASH_ADDR + 16U, dw2) != HAL_OK) {
    HAL_FLASH_Lock(); return 0U;
  }

  HAL_FLASH_Lock();
  return 1U;
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

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();

  SSD1306_Init(&oled, &OLED_I2C, SSD1306_ADDR);

  /* 改动4：初始化板载 B1。
     用 GPIO 模式即可，因为你现在的翻页逻辑本来就是轮询 + 去抖，不需要中断。 */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

  /* 预热期间 MOSFET 保持断开，由主循环第一次采样后根据温度决定是否接通 */
  HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);

  INA228_Init_Simple();
  Flash_LoadSettings();

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  Warmup_Phase();

  /* 预热阶段（约 3000 ms）期间完全不轮询按钮，导致 Button_Update()
     内部 tick=0 与 HAL_GetTick()≈3000 之间存在巨大差值，首次轮询时去抖
     窗口立即过期，若按钮电平与初始假设不符就会产生假按键事件。
     此处请求同步：下次 Button_Update() 调用时对齐到当前物理电平和
     当前时刻，不产生任何事件。                                      */
  Button_Debounce_Sync();

  while (1)
  {
    static float vbus = 0.0f;
    static float ia   = 0.0f;
    static float tC   = NAN;
    static float tte_sec = NAN;
    static UiPage page = UI_SOC;
    static UiPage prev_page = UI_SOC;   /* 翻页前记录，供双击回退 */
    static FaultType fault = FAULT_NONE;

    static uint32_t last_sample = 0;
    static uint32_t last_draw = 0;
    static uint32_t bad_load_ms = 0;

    Button_Update();
    BtnEvent ev = Button_ConsumeEvent();

    uint32_t now = HAL_GetTick();

    if (g_mode == MODE_NORMAL) {
      if (ev == BTN_SINGLE && fault == FAULT_NONE) {
        /* 单击：立即翻页（~150ms 响应，无 400ms 等待窗口） */
        prev_page = page;
        page = (page == UI_TIME) ? UI_SOC : (UiPage)(page + 1);
      } else if (ev == BTN_DOUBLE && fault == FAULT_NONE) {
        /* 双击：进入设置模式。
           因为立即响应策略，首次松手已触发翻页（BTN_SINGLE），
           这里需回退到翻页前的页面再检查是否可编辑。 */
        page = prev_page;
        int idx = EditParam_FindByPage(page);
        if (idx >= 0) {
          g_edit_idx           = idx;
          g_edit_val           = *g_edit_params[idx].p_runtime;
          g_edit_last_activity = now;
          g_mode               = MODE_EDIT;
        }
        /* 不可编辑页：回退翻页，停留在原页 */
      }
    } else {  /* MODE_EDIT */
      /* 故障发生时立即退出编辑，不保存 */
      if (fault != FAULT_NONE) {
        g_mode = MODE_NORMAL;
      } else if (ev == BTN_SINGLE) {
        /* 单击：循环递增编辑值（原三连击功能移至单击，操作更简单） */
        const EditParamDef *def = &g_edit_params[g_edit_idx];
        g_edit_val += def->step;
        if (g_edit_val > def->max_val + def->step * 0.1f)
          g_edit_val = def->min_val;
        /* 消除 float 累积误差：对齐到步长网格 */
        g_edit_val = roundf(g_edit_val / def->step) * def->step;
        g_edit_last_activity = now;
      } else if (ev == BTN_DOUBLE) {
        /* 双击：保存并退出。
           因立即响应策略，首次松手 BTN_SINGLE 已递增一次 edit_val，
           这里回退该次递增，使双击仅执行"保存"语义。 */
        const EditParamDef *def = &g_edit_params[g_edit_idx];
        g_edit_val -= def->step;
        if (g_edit_val < def->min_val - def->step * 0.1f)
          g_edit_val = def->max_val;
        *def->p_runtime = g_edit_val;
        if (Flash_SaveSettings())
          Hint_Flash("Saved");
        else
          Hint_Flash("ERR:Flash");
        g_mode = MODE_NORMAL;
      }
      /* 超时检查 */
      if (g_mode == MODE_EDIT && (now - g_edit_last_activity >= EDIT_TIMEOUT_MS)) {
        g_mode = MODE_NORMAL;
        Hint_Flash("Cancelled");
      }
    }

    if (now - last_sample >= 200U) {
      uint32_t dt = now - last_sample;
      last_sample = now;

      float new_vbus = INA228_Vbus_V();
      float raw_ia   = INA228_Current_A();

      /* NTC 采样不依赖 I2C，始终执行 */
      float vntc = ADC_Channel_Voltage(NTC_ADC_CH);
      tC = NTC_TempC_FromDivider(NTC_SUPPLY_V, vntc);

      /* INA228 读取成功才更新电压/电流/SOC/过流故障 */
      if (!isnan(new_vbus) && !isnan(raw_ia)) {
        vbus = new_vbus;

        static float filtered_ia = 0.0f;
        static uint8_t ia_first_run = 1;

        if (ia_first_run) {
          filtered_ia = raw_ia;
          ia_first_run = 0;
        } else {
          filtered_ia = 0.1f * raw_ia + 0.9f * filtered_ia;
        }

        /* deadband: <2mA 直接归零 */
        if (fabsf(filtered_ia) < 0.002f) {
          filtered_ia = 0.0f;
        }

        ia = filtered_ia;

        tte_sec = RT_UpdateAndCompute_TTE_sec(vbus, fabsf(ia), soc);

        if (!soc_inited) {
          float vcell = vbus / 3.0f;
          soc = SOC_From_OCV_Cell(vcell);
          soc_inited = 1;
          /* 跳过本次库仑积分：dt = 开机到现在的时间，不代表采样间隔 */
        } else {
          /* Coulomb counting */
          float Q_as = CAPACITY_AH * 3600.0f;
          soc = soc - (fabsf(ia) * (dt / 1000.0f)) / Q_as;
          soc = clampf(soc, 0.0f, 1.0f);

          /* Light OCV correction when current small */
          if (fabsf(ia) < 0.05f) {
            float vcell = vbus / 3.0f;
            float soc_ocv = SOC_From_OCV_Cell(vcell);
            soc = 0.98f * soc + 0.02f * soc_ocv;
            soc = clampf(soc, 0.0f, 1.0f);
          }

          /* SOC 历史记录（F5）：每 5 s 存一次 */
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
        if (fault == FAULT_NONE) {
          float absI = fabsf(ia);
          uint8_t load_bad = 0;
          if (absI > g_overcurrent_a) load_bad = 1;
          else if (absI < I_UNDERCURRENT_A) load_bad = 1;

          if (load_bad) {
            bad_load_ms += dt;
            if (bad_load_ms >= LOAD_FAULT_MS) fault = FAULT_WRONG_LOAD;
          } else {
            bad_load_ms = 0;
          }
        }
      }

      /* 温度故障检测（不依赖 INA228，始终执行） */
      if (fault == FAULT_NONE && !isnan(tC) && tC > TEMP_HI_C) {
        fault = FAULT_WRONG_TEMP;
      }

      /* MOSFET 控制始终执行 */
      if (isnan(tC) || (tC > g_mosfet_off_temp) || (fault != FAULT_NONE)) {
        HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_SET);
      }
    }

    if (now - last_draw >= 250U) {
      last_draw = now;

      if (fault != FAULT_NONE) {
        Hint_SetDefault("FAULT");
        UI_ShowFault(fault);
      } else {
        /* 每帧更新默认底部提示 */
        if (g_mode == MODE_NORMAL) {
          if (EditParam_FindByPage(page) >= 0)
            Hint_SetDefault("1x:page 2x:edit");
          else
            Hint_SetDefault("1x:page");
        }
        /* 编辑模式的底部提示由各 UI 函数硬编码（"1x:+N 2x:save"） */

        int soc_percent = (int)(soc * 100.0f + 0.5f);
        uint8_t soc_low = (soc < g_soc_low_thresh) ? 1 : 0;

        uint8_t in_edit = (g_mode == MODE_EDIT);
        if (page == UI_SOC)            UI_ShowSOC(soc_percent, soc_low, in_edit, g_edit_val);
        else if (page == UI_SOC_CURVE) UI_ShowSOCCurvePage();
        else if (page == UI_TEMP)      UI_ShowTemp(tC, in_edit, g_edit_val);
        else if (page == UI_VOLT)      UI_ShowVolt(vbus);
        else if (page == UI_CURR)      UI_ShowCurr(ia, in_edit, g_edit_val);
        else                           UI_ShowTime(tte_sec);


        if (soc_low) {
          SSD1306_DrawString(&oled, 0, 5, "SOC TOO LOW");
          SSD1306_Update(&oled);
        }
      }
    }

    HAL_Delay(10);
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

void Error_Handler(void)
{
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
