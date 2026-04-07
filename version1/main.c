/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include "ssd1306.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */
SSD1306 oled;

#define SSD1306_ADDR         0x78

#define INA228_ADDR          (0x40 << 1)
#define INA228_REG_CONFIG    0x00
#define INA228_REG_ADC_CONF  0x01
#define INA228_REG_SHUNT_CAL 0x02
#define INA228_REG_VBUS      0x05
#define INA228_REG_CURRENT   0x07

#define INA228_CONFIG_RST      (1U << 15)
#define INA228_CONFIG_ADCRANGE (1U << 4)
#define INA228_ADCRANGE_1      1

#define RSHUNT_OHM             0.015f
#define CURRENT_LSB_A          0.000001f
#define VBUS_LSB_V             0.0001953125f

#define ADC_VDDA_V      3.3f
#define ADC_FULL_SCALE  4095.0f

#define NTC_R0_OHM       10000.0f
#define NTC_BETA_K       3988.0f
#define NTC_T0_K         298.15f
#define NTC_RFIX_OHM     27000.0f
#define NTC_ADC_CH       ADC_CHANNEL_1
#define NTC_SUPPLY_V     3.3f

#define MOSFET_GPIO_Port GPIOB
#define MOSFET_Pin       GPIO_PIN_5

#define BTN_GPIO_Port GPIOA
#define BTN_Pin       GPIO_PIN_1
#define BTN_DEBOUNCE_MS 50U

typedef enum { UI_BOOT = 0, UI_SOC, UI_TEMP, UI_VOLT, UI_CURR } UiPage;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t INA228_ComputeShuntCal(void)
{
  float cal = 13107.2e6f * CURRENT_LSB_A * RSHUNT_OHM;
  if (INA228_ADCRANGE_1) cal *= 4.0f;
  if (cal < 0.0f) cal = 0.0f;
  if (cal > 65535.0f) cal = 65535.0f;
  return (uint16_t)(cal + 0.5f);
}

static void INA228_Write16(uint8_t reg, uint16_t value)
{
  uint8_t data[2];
  data[0] = (uint8_t)((value >> 8) & 0xFF);
  data[1] = (uint8_t)(value & 0xFF);
  HAL_I2C_Mem_Write(&hi2c1, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 2, 100);
}

static uint32_t INA228_ReadU20(uint8_t reg)
{
  uint8_t buf[3] = {0};
  if (HAL_I2C_Mem_Read(&hi2c1, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK)
    return 0;
  uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  return (raw >> 4) & 0xFFFFFU;
}

static int32_t INA228_ReadS20(uint8_t reg)
{
  uint8_t buf[3] = {0};
  if (HAL_I2C_Mem_Read(&hi2c1, INA228_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 3, 100) != HAL_OK)
    return 0;
  uint32_t raw = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
  uint32_t a = (raw >> 4) & 0xFFFFFU;
  if (a & 0x80000U) return (int32_t)(a | 0xFFF00000U);
  return (int32_t)a;
}

static void INA228_Init_Demo(void)
{
  INA228_Write16(INA228_REG_CONFIG, INA228_CONFIG_RST);
  HAL_Delay(20);

  uint16_t cfg = 0;
  if (INA228_ADCRANGE_1) cfg |= INA228_CONFIG_ADCRANGE;
  INA228_Write16(INA228_REG_CONFIG, cfg);

  uint16_t shunt_cal = INA228_ComputeShuntCal();
  INA228_Write16(INA228_REG_SHUNT_CAL, shunt_cal);

  uint16_t adc_conf = (uint16_t)((0xF << 12) | (3 << 9) | (7 << 6) | (7 << 3) | (7 << 0));
  INA228_Write16(INA228_REG_ADC_CONF, adc_conf);
}

static float INA228_Vbus_V(void)
{
  uint32_t v = INA228_ReadU20(INA228_REG_VBUS);
  return (float)v * VBUS_LSB_V;
}

static float INA228_Current_A(void)
{
  int32_t c = INA228_ReadS20(INA228_REG_CURRENT);
  return (float)c * CURRENT_LSB_A;
}

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

static float OCV_Cell_From_SOC(float soc)
{
  soc = clampf(soc, 0.0f, 1.0f);

  static const float s[] = {0.00f,0.05f,0.10f,0.20f,0.30f,0.40f,0.50f,0.60f,0.70f,0.80f,0.90f,1.00f};
  static const float v[] = {3.00f,3.25f,3.40f,3.55f,3.65f,3.72f,3.78f,3.84f,3.90f,3.98f,4.08f,4.20f};

  for (int i = 0; i < 11; i++) {
    if (soc >= s[i] && soc <= s[i+1]) {
      float t = (soc - s[i]) / (s[i+1] - s[i]);
      return v[i] + t * (v[i+1] - v[i]);
    }
  }
  return v[11];
}

static float SOC_Update_UKF_Lite(float soc, float I_A, float V_pack, float dt_s,
                                 float Q_nom_Ah, float R0_pack,
                                 float *P, float Qp, float Rp)
{
  float Q_as = Q_nom_Ah * 3600.0f;

  float soc_pred = soc - (I_A * dt_s) / Q_as;
  soc_pred = clampf(soc_pred, 0.0f, 1.0f);

  float V_cell = V_pack / 3.0f;
  float V_pred_cell = OCV_Cell_From_SOC(soc_pred) - (I_A * (R0_pack / 3.0f));
  float e = V_cell - V_pred_cell;

  float eps = 0.001f;
  float H = (OCV_Cell_From_SOC(soc_pred + eps) - OCV_Cell_From_SOC(soc_pred - eps)) / (2.0f * eps);
  if (H < 1e-3f) H = 1e-3f;

  float K = ((*P) * H) / (H*H*(*P) + Rp);

  float soc_new = soc_pred + K * e;
  soc_new = clampf(soc_new, 0.0f, 1.0f);

  *P = (1.0f - K * H) * (*P) + Qp;

  return soc_new;
}

static void UI_ShowBoot(void)
{
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 46, 2, "BMS DEMO");
  SSD1306_DrawString(&oled, 0, 6, "BTN: NEXT");
  SSD1306_Update(&oled);
}

static void UI_ShowSOC(int soc_percent)
{
  char l2[24];
  if (soc_percent < 0) soc_percent = 0;
  if (soc_percent > 100) soc_percent = 100;

  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 0, 0, "SOC");

  snprintf(l2, sizeof(l2), "%d", soc_percent);

  SSD1306_DrawString(&oled, 0, 3, l2);
  SSD1306_DrawString(&oled, 0, 6, "BTN: NEXT");
  SSD1306_Update(&oled);
}

static void UI_ShowTemp(float tC)
{
  char l2[24];
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 0, 0, "TEMP (NTC)");

  if (isnan(tC)) strcpy(l2, "--.- C");
  else {
    int ti = (int)tC;
    int td = (int)fabsf((tC - (float)ti) * 10.0f);
    snprintf(l2, sizeof(l2), "%d.%d C", ti, td);
  }

  SSD1306_DrawString(&oled, 0, 3, l2);
  SSD1306_DrawString(&oled, 0, 6, "BTN: NEXT");
  SSD1306_Update(&oled);
}

static void UI_ShowVolt(float v)
{
  char l2[24];
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 0, 0, "VOLT (VBUS)");

  int vi = (int)v;
  int vd = (int)fabsf((v - (float)vi) * 1000.0f);
  snprintf(l2, sizeof(l2), "%d.%03d V", vi, vd);

  SSD1306_DrawString(&oled, 0, 3, l2);
  SSD1306_DrawString(&oled, 0, 6, "BTN: NEXT");
  SSD1306_Update(&oled);
}

static void UI_ShowCurr(float ia)
{
  char l2[24];
  SSD1306_Clear(&oled);
  SSD1306_DrawString(&oled, 0, 0, "CURR (LOAD)");

  float mA = ia * 1000.0f;
  if (mA > -0.05f && mA < 0.05f) mA = 0.0f;

  int mi = (int)mA;
  int md = (int)fabsf((mA - (float)mi) * 10.0f);
  snprintf(l2, sizeof(l2), "%d.%d mA", mi, md);

  SSD1306_DrawString(&oled, 0, 3, l2);
  SSD1306_DrawString(&oled, 0, 6, "BTN: NEXT");
  SSD1306_Update(&oled);
}

static uint8_t Button_Pressed_Event(void)
{
  static uint8_t last = 1;
  static uint8_t stable = 1;
  static uint32_t tick = 0;

  uint8_t r = (uint8_t)HAL_GPIO_ReadPin(BTN_GPIO_Port, BTN_Pin);

  if (r != last) {
    last = r;
    tick = HAL_GetTick();
  }

  if ((HAL_GetTick() - tick) >= BTN_DEBOUNCE_MS) {
    if (stable != last) {
      stable = last;
      if (stable == 0) return 1;
    }
  }

  return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init(&oled, &hi2c2, SSD1306_ADDR);
  INA228_Init_Demo();

  UI_ShowBoot();
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  static UiPage page = UI_BOOT;
	  static uint32_t boot_t0 = 0;
	  static uint32_t last_draw = 0;

	  static float soc = 0.90f;
	  static float P = 0.02f;
	  static const float Q_nom_Ah = 2.2f;
	  static const float R0_pack = 0.16f;
	  static const float Qp = 1e-6f;
	  static const float Rp = 1e-4f;

	  static int soc_percent = 90;
	  static uint32_t last_soc_tick = 0;

	  if (boot_t0 == 0) {
	    boot_t0 = HAL_GetTick();
	    last_soc_tick = HAL_GetTick();
	  }

	  if (page == UI_BOOT && (HAL_GetTick() - boot_t0) > 800U) page = UI_SOC;

	  if (Button_Pressed_Event()) {
	    if (page == UI_BOOT) page = UI_SOC;
	    else if (page == UI_SOC) page = UI_TEMP;
	    else if (page == UI_TEMP) page = UI_VOLT;
	    else if (page == UI_VOLT) page = UI_CURR;
	    else page = UI_SOC;
	  }

	  if ((HAL_GetTick() - last_draw) >= 250U) {
	    last_draw = HAL_GetTick();

	    float vbus = INA228_Vbus_V();
	    float ia = INA228_Current_A();

	    float vntc_node = ADC_Channel_Voltage(NTC_ADC_CH);

	    float tC = NTC_TempC_FromDivider(3.3f, vntc_node);

	    if (!isnan(tC) && (tC > 30.0f))
	      HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_RESET);
	    else
	      HAL_GPIO_WritePin(MOSFET_GPIO_Port, MOSFET_Pin, GPIO_PIN_SET);

	    if ((HAL_GetTick() - last_soc_tick) >= 1000U) {
	      last_soc_tick = HAL_GetTick();
	      soc = SOC_Update_UKF_Lite(soc, ia, vbus, 1.0f, Q_nom_Ah, R0_pack, &P, Qp, Rp);
	      soc_percent = (int)(soc * 100.0f + 0.5f);
	      if (soc_percent < 0) soc_percent = 0;
	      if (soc_percent > 100) soc_percent = 100;
	    }

	    if (page == UI_BOOT) UI_ShowBoot();
	    else if (page == UI_SOC) UI_ShowSOC(soc_percent);
	    else if (page == UI_TEMP) UI_ShowTemp(tC);
	    else if (page == UI_VOLT) UI_ShowVolt(vbus);
	    else UI_ShowCurr(ia);
	  }

	  HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x40B285C2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x40B285C2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {

  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
