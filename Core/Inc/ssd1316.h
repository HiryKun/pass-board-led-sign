/**
 * @file ssd1316.h
 * @author HiryKun (1951086367@qq.com)
 * @brief SSD1316驱动库头文件，适用于128*32 OLED屏幕，硬件I2C
 * @version 0.1
 * @date 2024-10-17
 * 
 * @copyright Copyright (c) 2024 HiryKun
 * @attention
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */
#ifndef __OLED_H
#define __OLED_H

#include "main.h"
#include <string.h>

#define SSD1316_SCREEN_SIZE     512
#define SSD1316_ADDR_WRITE      0x78    //SSD1316地址，已左移，写模式
#define SSD1316_ADDR_READ       0x79    //SSD1316地址，已左移，读模式
#define SSD1316_I2C_TIMEOUT     100     //I2C超时时间(ms)
#define BLACK                   0x00    //黑色
#define WHITE                   0xFF    //白色

/**
 * @brief 向SSD1316发送命令（D/C 0x00）
 * 
 * @param pDate 命令内容
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef SSD1316_Write_CMD(uint8_t Date);

/**
 * @brief 向SSD1316发送数据（D/C 0x40）
 * 
 * @param pDate 数据地址
 * @param Size 数据长度
 * @return HAL_StatusTypeDef 
 */
HAL_StatusTypeDef SSD1316_Write_DATA(uint8_t *pDate, uint16_t Size);

/**
 * @brief 从绘制缓存写入SSD1316
 * 
 */
void SSD1316_Refresh();

/**
 * @brief 清屏并写入SSD1316寄存器
 * 
 */
void SSD1316_Clear();

/**
 * @brief 使能SSD1316显示
 * 
 */
void SSD1316_Display_ON();

/**
 * @brief 禁用SSD13136显示
 * 
 */
void SSD1316_Display_OFF();

/**
 * @brief 填充单色到绘制缓存
 * 
 * @param Color BLACK:黑色 WHITE:白色
 */
void SSD1316_Fill_Screen(uint8_t Color);

/**
 * @brief 写入完整屏幕到绘制缓存
 * 
 * @param pData 数据地址
 */
void SSD1316_Draw_Screen(const uint8_t *pData);


void SSD1316_Draw_Pic(const uint8_t *pData, uint8_t Start_COL, uint8_t Width);

/**
 * @brief 初始化SSD1316
 * 
 */
void SSD1316_Init();

#endif