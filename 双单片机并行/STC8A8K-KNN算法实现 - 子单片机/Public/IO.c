/****************************************Copyright (c)**************************************************
**--------------File Info-------------------------------------------------------------------------------
** File name:			io.c
** Created by:			ZuoYouPaPa
** Last modified Date:  2020-3-10
** Last Version:		1.0
** Descriptions:		IO模式控制
********************************************************************************************************/
#include "io.h"

/*******************************************************************************************************
函数名称：GPIO_Init
输入参数：GPIO_Choice 端口选择
          *GPIOx初始化结构体的地址
输出参数：无
函数功能：设置端口的工作模式
*******************************************************************************************************/
void GPIO_Init(GPIO_Port GPIO_Choice,GPIO_InitTypeDef *GPIOx)
{

    if(GPIO_Choice == GPIO_P0)
    {
        if(GPIOx->GPIO_Mode == GPIO_Mode_PullUp)		P0M1 &= ~GPIOx->GPIO_Pin,	P0M0 &= ~GPIOx->GPIO_Pin;	 //Up-drawing alignment bi-directional port|上拉准双向口
        if(GPIOx->GPIO_Mode == GPIO_Mode_HightZ)		P0M1 |=  GPIOx->GPIO_Pin,	P0M0 &= ~GPIOx->GPIO_Pin;	 //Floating input|浮空输入
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_OD)		P0M1 |=  GPIOx->GPIO_Pin,	P0M0 |=  GPIOx->GPIO_Pin;	 //Open drain output|开漏输出
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_PP)		P0M1 &= ~GPIOx->GPIO_Pin,	P0M0 |=  GPIOx->GPIO_Pin;	 //push-pull|推挽输出
    }
    if(GPIO_Choice == GPIO_P1)
    {
        if(GPIOx->GPIO_Mode == GPIO_Mode_PullUp)		P1M1 &= ~GPIOx->GPIO_Pin,	P1M0 &= ~GPIOx->GPIO_Pin;	 //Up-drawing alignment bi-directional port|上拉准双向口
        if(GPIOx->GPIO_Mode == GPIO_Mode_HightZ)		P1M1 |=  GPIOx->GPIO_Pin,	P1M0 &= ~GPIOx->GPIO_Pin;	 //Floating input|浮空输入
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_OD)		P1M1 |=  GPIOx->GPIO_Pin,	P1M0 |=  GPIOx->GPIO_Pin;	 //Open drain output|开漏输出
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_PP)		P1M1 &= ~GPIOx->GPIO_Pin,	P1M0 |=  GPIOx->GPIO_Pin;	 //push-pull|推挽输出
    }
    if(GPIO_Choice == GPIO_P2)
    {
        if(GPIOx->GPIO_Mode == GPIO_Mode_PullUp)		P2M1 &= ~GPIOx->GPIO_Pin,	P2M0 &= ~GPIOx->GPIO_Pin;	 //Up-drawing alignment bi-directional port|上拉准双向口
        if(GPIOx->GPIO_Mode == GPIO_Mode_HightZ)		P2M1 |=  GPIOx->GPIO_Pin,	P2M0 &= ~GPIOx->GPIO_Pin;	 //Floating input|浮空输入
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_OD)		P2M1 |=  GPIOx->GPIO_Pin,	P2M0 |=  GPIOx->GPIO_Pin;	 //Open drain output|开漏输出
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_PP)		P2M1 &= ~GPIOx->GPIO_Pin,	P2M0 |=  GPIOx->GPIO_Pin;	 //push-pull|推挽输出
    }
    if(GPIO_Choice == GPIO_P3)
    {
        if(GPIOx->GPIO_Mode == GPIO_Mode_PullUp)		P3M1 &= ~GPIOx->GPIO_Pin,	P3M0 &= ~GPIOx->GPIO_Pin;	 //Up-drawing alignment bi-directional port|上拉准双向口
        if(GPIOx->GPIO_Mode == GPIO_Mode_HightZ)		P3M1 |=  GPIOx->GPIO_Pin,	P3M0 &= ~GPIOx->GPIO_Pin;	 //Floating input|浮空输入
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_OD)		P3M1 |=  GPIOx->GPIO_Pin,	P3M0 |=  GPIOx->GPIO_Pin;	 //Open drain output|开漏输出
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_PP)		P3M1 &= ~GPIOx->GPIO_Pin,	P3M0 |=  GPIOx->GPIO_Pin;	 //push-pull|推挽输出
    }
    if(GPIO_Choice == GPIO_P4)
    {
        if(GPIOx->GPIO_Mode == GPIO_Mode_PullUp)		P4M1 &= ~GPIOx->GPIO_Pin,	P4M0 &= ~GPIOx->GPIO_Pin;	 //Up-drawing alignment bi-directional port|上拉准双向口
        if(GPIOx->GPIO_Mode == GPIO_Mode_HightZ)		P4M1 |=  GPIOx->GPIO_Pin,	P4M0 &= ~GPIOx->GPIO_Pin;	 //Floating input|浮空输入
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_OD)		P4M1 |=  GPIOx->GPIO_Pin,	P4M0 |=  GPIOx->GPIO_Pin;	 //Open drain output|开漏输出
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_PP)		P4M1 &= ~GPIOx->GPIO_Pin,	P4M0 |=  GPIOx->GPIO_Pin;	 //push-pull|推挽输出
    }
    if(GPIO_Choice == GPIO_P5)
    {
        if(GPIOx->GPIO_Mode == GPIO_Mode_PullUp)		P5M1 &= ~GPIOx->GPIO_Pin,	P5M0 &= ~GPIOx->GPIO_Pin;	 //Up-drawing alignment bi-directional port|上拉准双向口
        if(GPIOx->GPIO_Mode == GPIO_Mode_HightZ)		P5M1 |=  GPIOx->GPIO_Pin,	P5M0 &= ~GPIOx->GPIO_Pin;	 //Floating input|浮空输入
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_OD)		P5M1 |=  GPIOx->GPIO_Pin,	P5M0 |=  GPIOx->GPIO_Pin;	 //Open drain output|开漏输出
        if(GPIOx->GPIO_Mode == GPIO_Mode_Out_PP)		P5M1 &= ~GPIOx->GPIO_Pin,	P5M0 |=  GPIOx->GPIO_Pin;	 //push-pull|推挽输出
    }
}


/*******************************************************************************************************
函数名称：GPIO_ReadInputDataBit
输入参数：GPIO_Choice 端口选择
          *GPIOx初始化结构体的地址
输出参数：无
函数功能：读取单个IO口状态
说    明：读取某个IO口的状态之前需要先使能内部弱上拉电阻
*******************************************************************************************************/
void GPIO_ReadSingIOStatus(uint8 GPIO_Port,uint8 GPIO_Pin)
{

}

/*******************************************************************************************************
函数名称：GPIO_ResetBits
输入参数：GPIO_Port 端口选择  GPIO_Pin引脚选择
输出参数：无
函数功能：置位IO口，无需使用sbit定义一个端口
*******************************************************************************************************/
void GPIO_ResetBits(uint8 GPIO_Port,uint8 GPIO_Pin)
{
    switch(GPIO_Port)
    {
    case GPIO_P0:
        P0&=~GPIO_Pin;
        break;
    case GPIO_P1:
        P1&=~GPIO_Pin;
        break;
    case GPIO_P2:
        P2&=~GPIO_Pin;
        break;
    case GPIO_P3:
        P3&=~GPIO_Pin;
        break;
    case GPIO_P4:
        P4&=~GPIO_Pin;
        break;
    case GPIO_P5:
        P5&=~GPIO_Pin;
        break;
    default:
        break;
    }

}

/*******************************************************************************************************
函数名称：GPIO_SetBits
输入参数：GPIO_Port 端口选择  GPIO_Pin引脚选择
输出参数：无
函数功能：复位IO口，无需使用sbit定义一个端口
*******************************************************************************************************/
void GPIO_SetBits(uint8 GPIO_Port,uint8 GPIO_Pin)
{
    switch(GPIO_Port)
    {
    case GPIO_P0:
        P0|=GPIO_Pin;
        break;
    case GPIO_P1:
        P1|=GPIO_Pin;
        break;
    case GPIO_P2:
        P2|=GPIO_Pin;
        break;
    case GPIO_P3:
        P3|=GPIO_Pin;
        break;
    case GPIO_P4:
        P4|=GPIO_Pin;
        break;
    case GPIO_P5:
        P5|=GPIO_Pin;
        break;
    default:
        break;
    }
}

//串口2引脚初始化
void USART2_Init()
{
//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_All;
//    GPIO_Init(GPIO_P2,&GPIO_InitStructure);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
    GPIO_Init(GPIO_P1,&GPIO_InitStructure);

}
/*********************************************************************************************************
**                            End Of File
********************************************************************************************************/

