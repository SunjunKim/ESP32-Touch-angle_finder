#pragma once

// Waveshare ESP32-S3-Touch-LCD-1.69 board pin assignments.
// Matches the vendor demo's Mylibrary/pin_config.h; no Mylibrary install needed.
// Reference: docs/hardware/ESP32-S3-Touch-LCD-1.69_V2.1.pdf
#define LCD_DC 4
#define LCD_CS 5
#define LCD_SCK 6
#define LCD_MOSI 7
#define LCD_RST 8
#define LCD_BL 15
#define LCD_WIDTH 240
#define LCD_HEIGHT 280

#define IIC_SDA 11
#define IIC_SCL 10
#define TP_RST 13
#define TP_INT 14
