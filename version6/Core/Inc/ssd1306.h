/**
 * @file    ssd1306.h
 * @brief   SSD1306 OLED 显示屏驱动头文件（128×64，I2C 接口）
 *
 * 【硬件连接】SSD1306 通过 I2C3 连接到 STM32G491RETx
 *   - I2C 地址：0x78（7位地址 0x3C 左移1位）
 *   - 分辨率：128×64 像素
 *   - 显示模式：单色（白/黑），page-based 显存
 *
 * 【显存结构】
 *   SSD1306 内部显存按 page 组织，共 8 个 page（0~7），每 page 8 行像素
 *   总显存大小 = 128 × 64 / 8 = 1024 字节
 *   本驱动使用软件帧缓冲区（SSD1306.buf），绘制完成后统一通过 I2C 发送到屏幕
 *
 * 【坐标系】
 *   x: 0~127（列），y: 0~63（行）
 *   DrawString 的 page 参数：0~7（每 page 对应 8 行）
 */

#ifndef SSD1306_DRIVER_H
#define SSD1306_DRIVER_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#define SSD1306_W 128    /* 屏幕宽度（像素） */
#define SSD1306_H 64     /* 屏幕高度（像素） */

/**
 * @brief SSD1306 驱动结构体
 * 包含 I2C 句柄、设备地址和 1024 字节帧缓冲区
 */
typedef struct {
  I2C_HandleTypeDef *hi2c;                    /* I2C 句柄指针 */
  uint8_t addr;                               /* I2C 写地址（0x78） */
  uint8_t buf[SSD1306_W * SSD1306_H / 8];     /* 帧缓冲区（1024 字节） */
} SSD1306;

/**
 * @brief  初始化 SSD1306 显示屏
 * @param  d      SSD1306 结构体指针
 * @param  hi2c   I2C 句柄指针
 * @param  addr   I2C 写地址（0x78）
 * @return HAL_OK=成功，HAL_ERROR=I2C 通信失败
 * @note   初始化序列包括：复位显示模式、设置寻址模式、配置显示参数、开启显示
 */
HAL_StatusTypeDef SSD1306_Init(SSD1306 *d, I2C_HandleTypeDef *hi2c, uint8_t addr);

/**
 * @brief  将帧缓冲区内容发送到 SSD1306 显示
 * @return HAL_OK=成功，HAL_ERROR=I2C 通信失败
 * @note   逐 page 发送，每 page 128 字节，共 8 page = 1024 字节
 *         I2C 超时 100ms
 */
HAL_StatusTypeDef SSD1306_Update(SSD1306 *d);

/**
 * @brief  清空帧缓冲区（所有像素置黑）
 * @param  d  SSD1306 结构体指针
 */
void SSD1306_Clear(SSD1306 *d);

/**
 * @brief  在帧缓冲区中绘制单个 ASCII 字符
 * @param  d     SSD1306 结构体指针
 * @param  x     列坐标（0~122，留 6px 给字符宽度）
 * @param  page  page 编号（0~7）
 * @param  c     ASCII 字符（0x20~0x7E，超出范围显示 '?'）
 * @note   使用 5×7 点阵字体（font5x7），字符间距 1px，总宽 6px
 */
void SSD1306_DrawChar(SSD1306 *d, uint8_t x, uint8_t page, char c);

/**
 * @brief  在帧缓冲区中绘制字符串
 * @param  d     SSD1306 结构体指针
 * @param  x     起始列坐标
 * @param  page  page 编号（0~7）
 * @param  s     要显示的字符串（ASCII）
 * @note   逐字符绘制，自动递增 x 坐标，超出屏幕宽度时停止
 */
void SSD1306_DrawString(SSD1306 *d, uint8_t x, uint8_t page, const char *s);

/**
 * @brief  在帧缓冲区中填充矩形区域（所有像素置白）
 * @param  d  SSD1306 结构体指针
 * @param  x  左上角列坐标
 * @param  y  左上角行坐标（像素，0~63）
 * @param  w  宽度（像素）
 * @param  h  高度（像素）
 * @note   用于绘制电池图标、进度条、边框等图形元素
 *         使用 OR 操作，不清除已有内容（叠加绘制）
 */
void SSD1306_FillRect(SSD1306 *d, uint8_t x, uint8_t y, uint8_t w, uint8_t h);

#endif
