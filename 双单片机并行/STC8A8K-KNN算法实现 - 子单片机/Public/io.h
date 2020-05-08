/****************************************Copyright (c)**************************************************
**--------------File Info-------------------------------------------------------------------------------
** File name:			io.h
** Created by:			ZuoYouPaPa
** Last modified Date:  2020-3-10
** Last Version:		1.0
** Descriptions:		IO模式控制
********************************************************************************************************/
#ifndef _IO_H
#define _IO_H

#include "STC8A.h"
#include "KNN.h"
/*
M1 M0
0  0    准双向口（传统8051端口模式，弱上拉） 灌电流可达20mA，拉电流为270～150μA（存在制造误差）
0  1    推挽输出（强上拉输出，可达20mA，要加限流电阻）
1  0    高阻输入（电流既不能流入也不能流出）
1  1    开漏输出（Open-Drain），内部上拉电阻断开 开漏模式既可读外部状态也可对外输出（高电平或低电平）。
        如要正确读外部状态或需要对外输出高电平，需外加上拉电阻，否则读不到外部状态，也对外输不出高电平。
*/


typedef enum
{
    GPIO_Mode_HightZ = 0x04, //高阻输入既不输出电流也不输入电流
    GPIO_Mode_PullUp = 0x28,  //上拉准双向口(默认)
    GPIO_Mode_Out_OD = 0x14,  //Open drain output 开漏输出
    GPIO_Mode_Out_PP = 0x10, //push-pull  推挽输出
} GPIOMode_TypeDef;

//使用枚举类型，在输入参数超出范围时编译器能报错
typedef enum
{
    GPIO_P0=0,
    GPIO_P1=1,
    GPIO_P2=2,
    GPIO_P3=3,
    GPIO_P4=4,
    GPIO_P5=5,
} GPIO_Port;


typedef struct
{
    unsigned char	GPIO_Mode;		//IO Mode|IO模式
    unsigned char	GPIO_Pin;		//Port|端口
} GPIO_InitTypeDef;

#define	GPIO_Pin_0		0x01	//IO pin Px.0|IO引脚 Px.0
#define	GPIO_Pin_1		0x02	//IO pin Px.1|IO引脚 Px.1
#define	GPIO_Pin_2		0x04	//IO pin Px.2|IO引脚 Px.2
#define	GPIO_Pin_3		0x08	//IO pin Px.3|IO引脚 Px.3
#define	GPIO_Pin_4		0x10	//IO pin Px.4|IO引脚 Px.4
#define	GPIO_Pin_5		0x20	//IO pin Px.5|IO引脚 Px.5
#define	GPIO_Pin_6		0x40	//IO pin Px.6|IO引脚 Px.6
#define	GPIO_Pin_7		0x80	//IO pin Px.7|IO引脚 Px.7
#define	GPIO_Pin_All	0xFF	//IO All pin |IO所有引脚

void GPIO_Init(GPIO_Port GPIO_Choice,GPIO_InitTypeDef *GPIOx);
void GPIO_ResetBits(uint8 GPIO_Port,uint8 GPIO_Pin);
void GPIO_SetBits(uint8 GPIO_Port,uint8 GPIO_Pin);
void USART2_Init();
#endif