#include "ssd1306.h"
#include "font5x7.h"
#include <string.h>

static HAL_StatusTypeDef ssd1306_write(SSD1306 *d, uint8_t val, uint8_t ctrl)
{
  uint8_t p[2] = {ctrl, val};
  return HAL_I2C_Master_Transmit(d->hi2c, d->addr, p, 2, 10);
}

static HAL_StatusTypeDef ssd1306_cmd(SSD1306 *d, const uint8_t *cmds, uint16_t n)
{
  for (uint16_t i = 0; i < n; i++) {
    if (ssd1306_write(d, cmds[i], 0x00) != HAL_OK) return HAL_ERROR;
  }
  return HAL_OK;
}

HAL_StatusTypeDef SSD1306_Init(SSD1306 *d, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
  d->hi2c = hi2c;
  d->addr = addr;
  memset(d->buf, 0, sizeof(d->buf));
  HAL_Delay(100);

  const uint8_t cmds[] = {
    0xAE,0x20,0x00,0xB0,0xC8,0x00,0x10,
    0x40,0x81,0x7F,0xA1,0xA6,0xA8,0x3F,
    0xA4,0xD3,0x00,0xD5,0x80,0xD9,0xF1,
    0xDA,0x12,0xDB,0x40,0x8D,0x14,0xAF
  };

  if (ssd1306_cmd(d, cmds, (uint16_t)sizeof(cmds)) != HAL_OK) return HAL_ERROR;
  return SSD1306_Update(d);
}

void SSD1306_Clear(SSD1306 *d)
{
  memset(d->buf, 0, sizeof(d->buf));
}

HAL_StatusTypeDef SSD1306_Update(SSD1306 *d)
{
  for (uint8_t p = 0; p < 8; p++) {
    if (ssd1306_write(d, (uint8_t)(0xB0 + p), 0x00) != HAL_OK) return HAL_ERROR;
    if (ssd1306_write(d, 0x00, 0x00) != HAL_OK) return HAL_ERROR;
    if (ssd1306_write(d, 0x10, 0x00) != HAL_OK) return HAL_ERROR;

    if (HAL_I2C_Mem_Write(d->hi2c, d->addr, 0x40, 1, &d->buf[SSD1306_W * p], SSD1306_W, 100) != HAL_OK)
      return HAL_ERROR;
  }
  return HAL_OK;
}

void SSD1306_DrawChar(SSD1306 *d, uint8_t x, uint8_t page, char c)
{
  if (page > 7) return;
  if (x > (SSD1306_W - 6)) return;

  if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) c = '?';
  const uint8_t *ptr = Font5x7[(uint8_t)c - 0x20];

  uint16_t base = (uint16_t)page * SSD1306_W + x;
  for (uint8_t i = 0; i < 5; i++) d->buf[base + i] = ptr[i];
}

void SSD1306_DrawString(SSD1306 *d, uint8_t x, uint8_t page, const char *s)
{
  while (*s) {
    SSD1306_DrawChar(d, x, page, *s++);
    x = (uint8_t)(x + 6);
    if (x > (SSD1306_W - 6)) break;
  }
}
