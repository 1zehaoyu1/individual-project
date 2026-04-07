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

/* 改动2：删除了原来外部 PA1 按钮的端口/引脚定义，只保留去抖时间 */
#define BTN_DEBOUNCE_MS 50U

/* Pages */
typedef enum { UI_SOC = 0, UI_TEMP, UI_VOLT, UI_CURR, UI_TIME } UiPage;

/* Faults */
typedef enum { FAULT_NONE = 0, FAULT_WRONG_TEMP, FAULT_WRONG_LOAD } FaultType;

/* thresholds */
#define TEMP_HI_C            60.0f
#define TEMP_MOSFET_OFF_C    30.0f

/* 你要的毫安阈值：12mA / 6mA -> 单位换算成 A */
#define I_OVERCURRENT_A      0.4f
#define I_UNDERCURRENT_A     0.006f
#define LOAD_FAULT_MS        10000U
#define CAPACITY_AH      2.2f
#define V_NOM_PACK_V     11.1f

#define SAMPLE_MS        200U
#define RT_WIN_SEC       10U
#define RT_WIN_N         (RT_WIN_SEC*1000U/SAMPLE_MS)

/* SOC */
static float soc = 0.90f;
#define SOC_LOW_THRESH     0.30f

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);

/* ===== INA228 helpers (按位宽读，避免尺度错误) ===== */

/* 20-bit signed in [23:4] -> >>4 then sign-extend 20-bit */
static int32_t INA228_ReadS20(uint8_t reg)
{
  uint8_t buf[3] = {0};
  if (HAL_I2C_Mem_Read(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK)
    return 0;

  uint32_t raw24 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  int32_t v = (int32_t)(raw24 >> 4);

  if (v & 0x80000) v |= 0xFFF00000;
  return v;
}

/* 20-bit unsigned in [23:4] -> >>4 */
static uint32_t INA228_ReadU20(uint8_t reg)
{
  uint8_t buf[3] = {0};
  if (HAL_I2C_Mem_Read(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK)
    return 0;

  uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  return (raw >> 4) & 0xFFFFFU;
}

static void INA228_Write16(uint8_t reg, uint16_t value)
{
  uint8_t data[2];
  data[0] = (uint8_t)((value >> 8) & 0xFF);
  data[1] = (uint8_t)(value & 0xFF);
  HAL_I2C_Mem_Write(&INA228_I2C, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
}

static void INA228_Init_Simple(void)
{
  INA228_Write16(INA228_REG_CONFIG, INA228_CONFIG_RST);
  HAL_Delay(20);

  uint16_t cfg = 0;
  if (INA228_ADCRANGE_1) cfg |= INA228_CONFIG_ADCRANGE;
  INA228_Write16(INA228_REG_CONFIG, cfg);

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
  INA228_Write16(INA228_REG_ADC_CONF, adc_conf);
}

static float INA228_Vbus_V(void)
{
  uint32_t v = INA228_ReadU20(INA228_REG_VBUS);
  return (float)v * VBUS_LSB_V;
}

/* “PHY current” from VSHUNT / RSHUNT (no SHUNT_CAL dependency) */
static float INA228_Current_A(void)
{
  int32_t vsh = INA228_ReadS20(INA228_REG_VSHUNT);
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

/* ===== Button debounce =====
   改动3：这里不再读外部 PA1，而是改成读板载 B1。
   仍然保留你原来的“去抖 + 按下一次触发一次事件”逻辑。
*/
static uint8_t Button_Pressed_Event(void)
{
  static uint8_t last = 1;
  static uint8_t stable = 1;
  static uint32_t tick = 0;

  /* 读取板载 B1 当前状态 */
  uint8_t r = (uint8_t)BSP_PB_GetState(BUTTON_USER);

  if (r != last) {
    last = r;
    tick = HAL_GetTick();
  }

  if ((HAL_GetTick() - tick) >= BTN_DEBOUNCE_MS) {
    if (stable != last) {
      stable = last;

      /* 保持你原来的按下触发方式：稳定到低电平时，认为按下 */
      if (stable == 0) return 1;
    }
  }
  return 0;
}

/* ===== UI ===== */
static void UI_DrawHeader(const char *title)
{
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 0, 0, title);
}

static void UI_DrawFooter(void)
{
  SSD1306_DrawString(&oled, 0, 6, "BTN: NEXT");
}

static void UI_ShowSOC(int soc_percent, uint8_t soc_low)
{
  char l2[24];
  UI_DrawHeader("SOC");

  if (soc_percent < 0) soc_percent = 0;
  if (soc_percent > 100) soc_percent = 100;

  snprintf(l2, sizeof(l2), "%d%%", soc_percent);
  SSD1306_DrawString(&oled, 0, 3, l2);
  if (soc_low) SSD1306_DrawString(&oled, 60, 3, "LOW");

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

  int vi = (int)v;
  int vd = (int)fabsf((v - (float)vi) * 1000.0f);
  snprintf(l2, sizeof(l2), "%d.%03d V", vi, vd);

  SSD1306_DrawString(&oled, 0, 3, l2);
  UI_DrawFooter();
  SSD1306_Update(&oled);
}

static void UI_ShowCurr(float ia)
{
  char l2[24];
  UI_DrawHeader("CURR");

  float mA = ia * 1000.0f;
  int mi = (int)mA;
  int md = (int)fabsf((mA - (float)mi) * 10.0f);
  snprintf(l2, sizeof(l2), "%d.%d mA", mi, md);

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

static void UI_ShowFault(FaultType f)
{
  UI_DrawHeader("FAULT");
  if (f == FAULT_WRONG_TEMP) SSD1306_DrawString(&oled, 0, 3, "WRONG TEMP");
  else if (f == FAULT_WRONG_LOAD) SSD1306_DrawString(&oled, 0, 3, "WRONG LOAD");
  else SSD1306_DrawString(&oled, 0, 3, "UNKNOWN");
  SSD1306_DrawString(&oled, 0, 6, "RESET to clear");
  SSD1306_Update(&oled);
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

  /* Default: MOSFET ON */
  HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_SET);

  INA228_Init_Simple();

  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE) {
    Error_Handler();
  }

  while (1)
  {
    static float vbus = 0.0f;
    static float ia   = 0.0f;
    static float tC   = NAN;
    static float tte_sec = NAN;
    static UiPage page = UI_SOC;
    static FaultType fault = FAULT_NONE;

    static uint32_t last_sample = 0;
    static uint32_t last_draw = 0;
    static uint32_t bad_load_ms = 0;

    static uint8_t soc_inited = 0;

    if (Button_Pressed_Event()) {
      if (fault == FAULT_NONE) {
        if (page == UI_SOC) page = UI_TEMP;
        else if (page == UI_TEMP) page = UI_VOLT;
        else if (page == UI_VOLT) page = UI_CURR;
        else if (page == UI_CURR) page = UI_TIME;
        else page = UI_SOC;
      }
    }

    uint32_t now = HAL_GetTick();

    if (now - last_sample >= 200U) {
      uint32_t dt = now - last_sample;
      last_sample = now;

      vbus = INA228_Vbus_V();
      float raw_ia = INA228_Current_A();

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

      float vntc = ADC_Channel_Voltage(NTC_ADC_CH);
      tC = (NTC_TempC_FromDivider(NTC_SUPPLY_V, vntc) );
      tte_sec = RT_UpdateAndCompute_TTE_sec(vbus, fabsf(ia), soc);

      if (!soc_inited) {
        float vcell = vbus / 3.0f;
        soc = SOC_From_OCV_Cell(vcell);
        soc_inited = 1;
      }

      /* Coulomb counting */
      float Q_as = 2.2f * 3600.0f;
      soc = soc - (fabsf(ia) * (dt / 1000.0f)) / Q_as;
      soc = clampf(soc, 0.0f, 1.0f);

      /* Light OCV correction when current small */
      if (fabsf(ia) < 0.05f) {
        float vcell = vbus / 3.0f;
        float soc_ocv = SOC_From_OCV_Cell(vcell);
        soc = 0.98f * soc + 0.02f * soc_ocv;
        soc = clampf(soc, 0.0f, 1.0f);
      }

      if (fault == FAULT_NONE) {
        if (!isnan(tC) && tC > TEMP_HI_C) {
          fault = FAULT_WRONG_TEMP;
        }

        float absI = fabsf(ia);
        uint8_t load_bad = 0;
        if (absI > I_OVERCURRENT_A) load_bad = 1;
        else if (absI < I_UNDERCURRENT_A) load_bad = 1;

        if (load_bad) {
          bad_load_ms += dt;
          if (bad_load_ms >= LOAD_FAULT_MS) fault = FAULT_WRONG_LOAD;
        } else {
          bad_load_ms = 0;
        }
      }

      if (isnan(tC) || (tC > TEMP_MOSFET_OFF_C) || (fault != FAULT_NONE)) {
        HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);
      } else {
        HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_SET);
      }
    }

    if (now - last_draw >= 250U) {
      last_draw = now;

      if (fault != FAULT_NONE) {
        UI_ShowFault(fault);
      } else {
        int soc_percent = (int)(soc * 100.0f + 0.5f);
        uint8_t soc_low = (soc < SOC_LOW_THRESH) ? 1 : 0;

        if (page == UI_SOC) UI_ShowSOC(soc_percent, soc_low);
        else if (page == UI_TEMP) UI_ShowTemp(tC);
        else if (page == UI_VOLT) UI_ShowVolt(vbus);
        else if (page == UI_CURR) UI_ShowCurr(ia);
        else UI_ShowTime(tte_sec);

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
