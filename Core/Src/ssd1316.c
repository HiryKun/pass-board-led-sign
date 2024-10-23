/**
 * @file ssd1316.c
 * @author HiryKun (1951086367@qq.com)
 * @brief SSD1316驱动库源文件，适用于128*32 OLED屏幕，硬件I2C
 * @version 0.1
 * @date 2024-10-17
 * 
 * @copyright Copyright (c) 2024 HiryKun
 * @attention
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */
#include "ssd1316.h"

extern I2C_HandleTypeDef hi2c1;     //所使用的I2C硬件
#define SSD1316_HI2C hi2c1

static uint8_t SSD1316_Buffer[512];

HAL_StatusTypeDef SSD1316_Write_CMD(uint8_t Date) {
    return HAL_I2C_Mem_Write(&SSD1316_HI2C, SSD1316_ADDR_WRITE, 0x00, 1, &Date, 1, SSD1316_I2C_TIMEOUT);
    
}

HAL_StatusTypeDef SSD1316_Write_DATA(uint8_t *pDate, uint16_t Size) {
    return HAL_I2C_Mem_Write(&SSD1316_HI2C, SSD1316_ADDR_WRITE, 0x40, 1, pDate, Size, SSD1316_I2C_TIMEOUT);
}

void SSD1316_Refresh() {
    SSD1316_Write_DATA(SSD1316_Buffer, sizeof(SSD1316_Buffer));
}

void SSD1316_Clear() {
    memset(SSD1316_Buffer, 0x00, sizeof(SSD1316_Buffer));
    SSD1316_Refresh();
}

void SSD1316_Display_ON() {
    SSD1316_Write_CMD(0x8D);    //使能电荷泵
    SSD1316_Write_CMD(0x14);
    SSD1316_Write_CMD(0xAF);    //使能显示
}

void SSD1316_Display_OFF() {
    SSD1316_Write_CMD(0xAE);    //禁用显示
    SSD1316_Write_CMD(0x8D);    //禁用电荷泵
    SSD1316_Write_CMD(0x10);
}

void SSD1316_Fill_Screen(uint8_t Color) {
    memset(SSD1316_Buffer, Color, sizeof(SSD1316_Buffer));
}

void SSD1316_Draw_Screen(const uint8_t *pData) {
    memcpy(SSD1316_Buffer, pData, sizeof(SSD1316_Buffer));
}

void SSD1316_Draw_Pic(const uint8_t *pData, uint8_t Start_COL, uint8_t Width) {
    uint8_t True_Width;
    /*若图片超过右边界，裁切超出部分*/
    if (Start_COL + Width <= 128) True_Width = Width;
    else True_Width = 128 - Start_COL;
    for (int page = 0; page < 4; ++page) {
        memcpy(SSD1316_Buffer + Start_COL + 128 * page , pData + Width * page, True_Width);
    }
}

void SSD1316_Init() {
    SSD1316_Display_OFF();
    SSD1316_Write_CMD(0x21);
    SSD1316_Write_CMD(0x00);
    SSD1316_Write_CMD(0x7F);
    SSD1316_Write_CMD(0x22);
    SSD1316_Write_CMD(0x00);
    SSD1316_Write_CMD(0x03);
    SSD1316_Write_CMD(0x40);
    SSD1316_Write_CMD(0x81);
    SSD1316_Write_CMD(0xC5);
    SSD1316_Write_CMD(0xA1);
    SSD1316_Write_CMD(0xA6);
    SSD1316_Write_CMD(0xA8);
    SSD1316_Write_CMD(0x1F);
    SSD1316_Write_CMD(0xC0);
    SSD1316_Write_CMD(0xD3);
    SSD1316_Write_CMD(0x00);
    SSD1316_Write_CMD(0xD5);
    SSD1316_Write_CMD(0x80);
    SSD1316_Write_CMD(0xD9);
    SSD1316_Write_CMD(0x22);
    SSD1316_Write_CMD(0xDA);
    SSD1316_Write_CMD(0x12);
    SSD1316_Write_CMD(0xDB);
    SSD1316_Write_CMD(0x20);
    SSD1316_Write_CMD(0x8D);
    SSD1316_Write_CMD(0x15);
    SSD1316_Write_CMD(0x20);
    SSD1316_Write_CMD(0x00);
    SSD1316_Clear();
    SSD1316_Display_ON();
}
