#ifndef SSD1306_DRIVER_H
#define SSD1306_DRIVER_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define SSD1306_W 128
#define SSD1306_H 64

typedef struct {
  I2C_HandleTypeDef *hi2c;
  uint8_t addr;
  uint8_t buf[SSD1306_W * SSD1306_H / 8];
} SSD1306;

HAL_StatusTypeDef SSD1306_Init(SSD1306 *d, I2C_HandleTypeDef *hi2c, uint8_t addr);
HAL_StatusTypeDef SSD1306_Update(SSD1306 *d);
void SSD1306_Clear(SSD1306 *d);

void SSD1306_DrawChar(SSD1306 *d, uint8_t x, uint8_t page, char c);
void SSD1306_DrawString(SSD1306 *d, uint8_t x, uint8_t page, const char *s);

#endif
