#ifndef SSD1306_H
#define SSD1306_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/i2c.h"

// ─────────────────────────────────────────────
// Configuração do display BitDogLab
// ─────────────────────────────────────────────
#define SSD1306_I2C_PORT    i2c1
#define SSD1306_I2C_SDA     14   // GP14
#define SSD1306_I2C_SCL     15   // GP15
#define SSD1306_I2C_ADDR    0x3C
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64

// ─────────────────────────────────────────────
// Comandos do SSD1306
// ─────────────────────────────────────────────
#define SSD1306_SET_CONTRAST        0x81
#define SSD1306_SET_ENTIRE_ON       0xA4
#define SSD1306_SET_NORM_INV        0xA6
#define SSD1306_SET_DISP            0xAE
#define SSD1306_SET_MEM_ADDR        0x20
#define SSD1306_SET_COL_ADDR        0x21
#define SSD1306_SET_PAGE_ADDR       0x22
#define SSD1306_SET_DISP_START_LINE 0x40
#define SSD1306_SET_SEG_REMAP       0xA0
#define SSD1306_SET_MUX_RATIO       0xA8
#define SSD1306_SET_COM_OUT_DIR     0xC0
#define SSD1306_SET_DISP_OFFSET     0xD3
#define SSD1306_SET_COM_PIN_CFG     0xDA
#define SSD1306_SET_DISP_CLK_DIV    0xD5
#define SSD1306_SET_PRECHARGE       0xD9
#define SSD1306_SET_VCOM_DESEL      0xDB
#define SSD1306_SET_CHARGE_PUMP     0x8D

// Buffer do display
extern uint8_t ssd1306_buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

// Funções
void ssd1306_init();
void ssd1306_clear();
void ssd1306_show();
void ssd1306_draw_pixel(int x, int y, bool on);
void ssd1306_draw_char(int x, int y, char c);
void ssd1306_draw_string(int x, int y, const char *str);

#endif
