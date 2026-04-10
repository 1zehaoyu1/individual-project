/**
 * @file    ssd1306.c
 * @brief   SSD1306 OLED 显示屏驱动实现（128×64，I2C 接口）
 *
 * 【驱动架构】
 *   所有绘制操作（DrawChar、DrawString、FillRect）只修改帧缓冲区（d->buf），
 *   不直接与 I2C 通信。绘制完成后由上层调用 SSD1306_Update() 统一发送到屏幕。
 *
 * 【I2C 通信协议】
 *   - 控制字节 0x00：后续字节为命令
 *   - 控制字节 0x40：后续字节为显示数据
 *   - 命令使用 I2C_Master_Transmit（每次 2 字节：控制+数据）
 *   - 数据使用 I2C_Mem_Write（高效批量传输）
 */

#include "ssd1306.h"
#include "font5x7.h"
#include <string.h>

/**
 * @brief  向 SSD1306 写入单字节（命令或数据）
 * @param  d     SSD1306 结构体指针
 * @param  val   要写入的字节值
 * @param  ctrl  控制字节（0x00=命令，0x40=数据）
 * @return HAL 状态
 */
static HAL_StatusTypeDef ssd1306_write(SSD1306 *d, uint8_t val, uint8_t ctrl)
{
  uint8_t p[2] = {ctrl, val};
  return HAL_I2C_Master_Transmit(d->hi2c, d->addr, p, 2, 10);  /* 超时 10ms */
}

/**
 * @brief  发送命令序列到 SSD1306
 * @param  d    SSD1306 结构体指针
 * @param  cmds 命令数组
 * @param  n    命令数量
 * @return HAL_OK=全部成功，HAL_ERROR=任一命令失败
 */
static HAL_StatusTypeDef ssd1306_cmd(SSD1306 *d, const uint8_t *cmds, uint16_t n)
{
  for (uint16_t i = 0; i < n; i++) {
    if (ssd1306_write(d, cmds[i], 0x00) != HAL_OK) return HAL_ERROR;
  }
  return HAL_OK;
}

/**
 * @brief  初始化 SSD1306 显示屏
 * @param  d      SSD1306 结构体指针
 * @param  hi2c   I2C 句柄指针
 * @param  addr   I2C 写地址（0x78）
 * @return HAL_OK=成功，HAL_ERROR=失败
 * @note   初始化序列：
 *   - 等待 100ms（SSD1306 上电稳定时间）
 *   - 发送 28 字节初始化命令（显示模式、寻址、对比度、电荷泵等）
 *   - 清空帧缓冲区并更新显示（确保屏幕初始为空）
 */
HAL_StatusTypeDef SSD1306_Init(SSD1306 *d, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
  d->hi2c = hi2c;
  d->addr = addr;
  memset(d->buf, 0, sizeof(d->buf));   /* 清空帧缓冲区 */
  HAL_Delay(100);                       /* 等待 SSD1306 上电稳定 */

  /* SSD1306 初始化命令序列（28 字节）
   * 0xAE          : 关闭显示
   * 0x20,0x00     : 水平寻址模式
   * 0xB0          : 起始 page = 0
   * 0xC8          : COM 扫描方向：反向（从 COM63 到 COM0）
   * 0x00,0x10     : 列地址低4位=0，高4位=0
   * 0x40          : 起始行 = 0
   * 0x81,0x7F     : 对比度 = 127（最大值）
   * 0xA1          : 段映射：列 127 → SEG0（水平翻转）
   * 0xA6          : 正常显示（非反色）
   * 0xA8,0x3F     : 多路复用率 = 64（64 行）
   * 0xA4          : 全局显示：跟随 RAM 内容
   * 0xD3,0x00     : 显示偏移 = 0
   * 0xD5,0x80     : 时钟分频 = 默认值
   * 0xD9,0xF1     : 预充电周期 = VCC 模式
   * 0xDA,0x12     : COM 引脚配置 = 交替模式
   * 0xDB,0x40     : VCOMH 电压 = 默认值
   * 0x8D,0x14     : 电荷泵设置 = 使能（必须开启才能显示）
   * 0xAF          : 开启显示 */
  const uint8_t cmds[] = {
    0xAE,0x20,0x00,0xB0,0xC8,0x00,0x10,
    0x40,0x81,0x7F,0xA1,0xA6,0xA8,0x3F,
    0xA4,0xD3,0x00,0xD5,0x80,0xD9,0xF1,
    0xDA,0x12,0xDB,0x40,0x8D,0x14,0xAF
  };

  if (ssd1306_cmd(d, cmds, (uint16_t)sizeof(cmds)) != HAL_OK) return HAL_ERROR;
  return SSD1306_Update(d);  /* 清空显示 */
}

/**
 * @brief  清空帧缓冲区（所有像素置 0 = 黑色）
 */
void SSD1306_Clear(SSD1306 *d)
{
  memset(d->buf, 0, sizeof(d->buf));
}

/**
 * @brief  将帧缓冲区内容通过 I2C 发送到 SSD1306 显示
 * @param  d  SSD1306 结构体指针
 * @return HAL_OK=成功，HAL_ERROR=任一 page 传输失败
 * @note   逐 page 传输（共 8 page），每 page 128 字节
 *         每个 page 传输前发送 3 个命令设置 page 地址和列地址
 *         数据传输使用 HAL_I2C_Mem_Write（效率高于逐字节发送）
 */
HAL_StatusTypeDef SSD1306_Update(SSD1306 *d)
{
  for (uint8_t p = 0; p < 8; p++) {
    /* 设置当前 page 地址 */
    if (ssd1306_write(d, (uint8_t)(0xB0 + p), 0x00) != HAL_OK) return HAL_ERROR;
    /* 设置列地址低 4 位 = 0 */
    if (ssd1306_write(d, 0x00, 0x00) != HAL_OK) return HAL_ERROR;
    /* 设置列地址高 4 位 = 0 */
    if (ssd1306_write(d, 0x10, 0x00) != HAL_OK) return HAL_ERROR;

    /* 批量发送该 page 的 128 字节数据（控制字节 0x40 = 数据模式） */
    if (HAL_I2C_Mem_Write(d->hi2c, d->addr, 0x40, 1, &d->buf[SSD1306_W * p], SSD1306_W, 100) != HAL_OK)
      return HAL_ERROR;
  }
  return HAL_OK;
}

/**
 * @brief  在帧缓冲区中绘制单个 ASCII 字符
 * @param  d     SSD1306 结构体指针
 * @param  x     列坐标（0~122）
 * @param  page  page 编号（0~7）
 * @param  c     ASCII 字符
 * @note   字体：5×7 点阵（font5x7），字符间距 1px，每个字符占 6 列
 *         字符数据从 Font5x7 查找表获取，索引 = (ASCII - 0x20)
 *         超出可显示范围的字符显示 '?'
 */
void SSD1306_DrawChar(SSD1306 *d, uint8_t x, uint8_t page, char c)
{
  if (page > 7) return;                    /* page 越界检查 */
  if (x > (SSD1306_W - 6)) return;         /* 剩余宽度不足一个字符 */

  if ((uint8_t)c < 0x20 || (uint8_t)c > 0x7E) c = '?';  /* 非可打印字符替换 */
  const uint8_t *ptr = Font5x7[(uint8_t)c - 0x20];       /* 查找字体数据 */

  /* 将 5 列字体数据写入帧缓冲区对应位置 */
  uint16_t base = (uint16_t)page * SSD1306_W + x;
  for (uint8_t i = 0; i < 5; i++) d->buf[base + i] = ptr[i];
  /* 第 6 列留空（字符间距） */
}

/**
 * @brief  在帧缓冲区中绘制字符串
 * @param  d     SSD1306 结构体指针
 * @param  x     起始列坐标
 * @param  page  page 编号（0~7）
 * @param  s     要显示的字符串
 */
void SSD1306_DrawString(SSD1306 *d, uint8_t x, uint8_t page, const char *s)
{
  while (*s) {
    SSD1306_DrawChar(d, x, page, *s++);
    x = (uint8_t)(x + 6);          /* 每字符宽 5px + 1px 间距 = 6px */
    if (x > (SSD1306_W - 6)) break; /* 超出屏幕宽度停止 */
  }
}

/**
 * @brief  在帧缓冲区中填充矩形区域（像素置 1 = 白色/亮）
 * @param  d  SSD1306 结构体指针
 * @param  x  左上角列坐标
 * @param  y  左上角行坐标（像素级，0~63）
 * @param  w  宽度（像素）
 * @param  h  高度（像素）
 * @note   使用按位 OR（|=），不清除已有内容，实现叠加绘制效果
 *         像素位置映射：buf[(y/8)*128 + x] 的 bit (y%8)
 */
void SSD1306_FillRect(SSD1306 *d, uint8_t x, uint8_t y, uint8_t w, uint8_t h)
{
  for (uint8_t py = y; py < y + h; py++) {
    uint8_t page = py / 8;       /* 计算 page 编号 */
    uint8_t bit  = py % 8;       /* 计算 page 内的位偏移 */
    if (page > 7) break;         /* page 越界停止 */
    for (uint8_t px = x; px < x + w; px++) {
      if (px >= SSD1306_W) break; /* 列越界停止 */
      d->buf[page * SSD1306_W + px] |= (uint8_t)(1u << bit);  /* 置位像素 */
    }
  }
}
