/*基于51单片机的KNN算法的实现*/
/*基于51单片机的KNN算法的实现*/
/*基于51单片机的KNN算法的实现*/
/*基于51单片机的KNN算法的实现*/
#include "STC8A.h"
#include "KNN.h"
#include "train.h"
#define USART2_ReceLenght  2

//注意xdata定义的数据能够掉电保持，并且单片机复位后并不会复位xdata区的数据
//RAM区上电后初始值是随机的，需要重新自己赋初值
char xdata TestData[JZLenght+1];  //接收测试数据暂存，有时候矩阵会多一个数据成1025字节
unsigned char  USART2_RecBuff[USART2_ReceLenght];

unsigned char  USART2_SendBuff[Numtrain*3+3];

int Timer0_Counter=0;
bit USART2_AllowSendFlag=0;

//串口1数据接受完成标志以及接收数据的计数
int ReceiveCounter=0;
uint8 RecOverFlag=0;
sbit LED = P2^6;  //LED指示灯
sbit ReceDataReadly=P1^3;

bit USART1_BUSY;   //串口1繁忙标志
bit USART2_BUSY;   //串口2繁忙标志

//串口2数据接受完成标志以及接收数据的计数
uint8 USART2_RecOverFlag=0;
uint8 USART2_ReceCount=0;

Distance xdata distance[Numtrain]; //距离值暂存,Numtrain训练数据模板长度

void exchange(Distance *in, uint8 index1,uint8 index2);
void selectSort(Distance *in, uint8 length);
void USART1_SendData(char Data);
//128 字节内部直接访问 RAM（DATA）
//128 字节内部间接访问 RAM（IDATA）
//8192 字节内部扩展 RAM（内部 XDATA）
//外部最大可扩展 64K 字节 RAM（外部 XDATA）





void USART1_SendData(uint8 Data)
{
    while (USART1_BUSY);
    USART1_BUSY = 1;
    SBUF = Data;
}



void USART1_SendStr(uint8 *Data,int Lenght)
{
    int i;
    for(i=0; i<Lenght; i++)
    {
        USART1_SendData(Data[i]);
    }
}


//串口2发送数据
void USART2_SendData(uint8 Data)
{
    while (USART2_BUSY);
    USART2_BUSY = 1;
    S2BUF = Data;
}

//串口2发送字符串
void USART2_SendStr(uint8 *USARTData,int Lenght)
{
    int j=0;
    for(j=0; j<Lenght; j++)
    {
        USART2_SendData(USARTData[j]);
    }
}
//求欧式距离
//digit1 测试数据
//digit2 训练数据
//返回值:int类型
int calDistance(char *digit1, char *digit2)
/*求距离*/
{
    int i, squareSum=0.0;
    for(i=0; i<JZLenght; i++)
    {
        //该处这样做的目的是提高运算速度，因为训练数据标本中只有0和1
        //所以这种算法处理只适用于标本是0和1的训练数据
        if(digit1[i]-digit2[i]!=0) squareSum+=1;
    }
    return  squareSum;  //返回距离值
}

//输入in 一个结构体数组
//index1 结构体数组的索引，前一个数
//index2 结构体数组的索引，后一个数
//两个结构体互换
void exchange(Distance *in, uint8 index1,uint8 index2)
{
    Distance tmp;
    tmp = in[index1];
    in[index1] = in[index2];
    in[index2] = tmp;
}


//输入in  一个结构体数组
//lenght 训练数据模板数量
//从小到大排序
void selectSort(Distance *in, uint8 length)
{
    uint8 i, j, min;
    uint8 N = length;
    for(i=0; i<N-1; i++)
    {
        min = i;
        //j=i+1 即该数据后面的数据
        //算法思想:每次循环将比较小数据往前推一次，直到所有数据排序正确
        //先判断前一个数据是否大于后一个数据，如果大于则交换，不大与则不交换
        //最大i*j次可完成排序
        for(j=i+1; j<N; j++)
        {
            if(in[j].distance<in[min].distance) min = j;
        }
        exchange(in,i,min);
    }
}

//in  输入数据矩阵
//train 测试数据矩阵
//K 选出距离目标值最近的数据的个数
//二维数组作为参数传入的时候长度没有加1，导致数据标签全是0
//因为数据标签存储在第1025个字节
void prediction(uint8 K, char *in, char train[][JZLenght+1])
{
    int it;
    /*求取输入digit与训练数据的距离*/
    for(it=0; it<Numtrain; it++)
    {
        distance[it].distance = calDistance(in,train[it]);  //求测试数据与训练数据的距离
        distance[it].label = train[it][JZLenght];  //训练数据的数字编号存放于数组最后一个数据字节中		  
    }			
}

void USART1_Init()
{
    //初次赋值最好别用或运算，有BUG
    //这里不知道是什么原因
    SCON=0x50;        //REN=1允许串行接受状态，串口工作模式1,9位可变波特率
    TMOD = 0x00;       //定时器1为模式0（16位自动重载)
    AUXR=0X40;		 //定时器1开启1T模式
    //9600波特率，注意波特率不能设置太高，之前测试的时候115200波特率的时候有乱码
    TL1=(65536-(11059200/4/9600));    //设置波特率重装值
    TH1=(65536-(11059200/4/9600))>>16;

	  //串口1的发送口设置为推挽输出,手册上有说，这里最好是设置一下
	  //不设置的话虽然也能正常发送，但是按STC手册上的来吧，要不然又会有坑
	  //从机串口1不需要发送数据，所以设置成高阻状态，减少电流消耗以缓解USB发送口的驱动压力
	  //串口1接收口则设置为双向口
    P3M1 &= ~0x01;  //P3.0口设置成双向口
    P3M0 &= ~0x01; 

	  P3M1 |=  0x02; //P3.1口设置成高阻状态
    P3M0 &= ~0x02;
	
    TR1  = 1;        //开启定时器1,,作为波特率发生器
    ES   = 1;        //开串口中断
    EA   = 1;        // 开总中断

    USART1_BUSY=0;   //串口1忙标志初始状态为0
}

void USART2_Init()
{
    S2CON = 0x50;   //模式0,8位可变波特率,允许串口接受数据
    T2L = (65536-(11059200/4/9600));   //设置定时器2初值，用于波特率发生器，该值决定波特率
    T2H = (65536-(11059200/4/9600))>>8;
    AUXR |= 0x14;   //开定时器2,时钟为1T模式，作为波特率发生器
    IE2 = 0x01;  //开启串口2的中断
    EA=1;

 	 
    
	  
    P1M1 &= ~0x01; //P1.0 串口接收口设置成双向口
    P1M0 &= ~0x01;
	  
		P1M1 &= ~0x02;
		P1M0 |=  0x02;    //P1.1发送数据前设置成推挽输出	
	
	  USART2_BUSY=0;   //串口1忙标志初始状态为0
}

void Timer0_Init(void)
{
    AUXR &= ~0x80;	//定时器时钟12T模式,即时钟12分频(FOSC/12)
    TMOD |= 0x00;  //模式0,16位自动重装载
    TL0 = 0x00;		//设置定时初值
	  TH0 = 0x4C;		//设置定时初值
	  TF0 = 0;		//清除TF0标志
	  TR0 = 0;		//初始化时关闭定时器
    ET0 = 1;      //使能定时器中断
    EA = 1;
}

void Clearn_TestData()
{
    int i;
    for(i=0; i<JZLenght; i++)
    {
        TestData[i]=0x00;
    }
}

//串口2向主机发送预测的计算值，将存储在结构体中的数据全部发送过去
//带和校验
void USART2_SendPredictData()
{	
	 uint8 temp1;
	 unsigned int Sum=0;
	 USART2_SendBuff[0]=0x01;  //从机地址
	 	for(temp1=0;temp1<Numtrain;temp1++)
		{ 
			  USART2_SendBuff[temp1*3+1]=distance[temp1].label;  //数据存入发送缓冲区
			  USART2_SendBuff[temp1*3+2]=distance[temp1].distance>>8;  //高8位
			  USART2_SendBuff[temp1*3+3]=distance[temp1].distance;   //低8位			  					 
		}
		for(temp1=0;temp1<Numtrain*3+1;temp1++)
		{
			   Sum+=USART2_SendBuff[temp1];  //求和
		}
		USART2_SendBuff[Numtrain*3+1]=Sum>>8;  //高8位
		USART2_SendBuff[Numtrain*3+2]=Sum;  //低8位	
		
		for(temp1=0;temp1<Numtrain*3+3;temp1++)  
		{
			 USART2_SendData(USART2_SendBuff[temp1]);  //发送数据
		}
}

void main() 			 //主函数
{
	  uint8 ReceErrorCounter=0;  //串口2发送错误次数计数，超过2次发送错误则不在重发数据
    //低于16MHz的时钟频率在自行调节时有一定的误差
    //需要将其调制目标频率的2至3倍，然后再进行分频
    //这里十分重要，否则在串口而通讯的时候会导致波特率不准却，然后出现乱码
    P_SW2 = 0x80;
    CKSEL = 0x00; //选择内部 IRC ( 默认 )
    CLKDIV = 0x02; //2分频
    P_SW2 = 0x00;
	
	  USART1_Init();
    USART2_Init();
	  Timer0_Init();
    while(1)
    {
        if(RecOverFlag==1)
        {      				
            prediction(1, TestData,train1);  //预测值计算
			      LED = ~LED;
					  TR0=1; //开定时器计时
					  while(!USART2_AllowSendFlag);		//等待主机处理完数据		
            USART2_SendPredictData();			 //串口2发送预测值计算的所有数据	  
					  RecOverFlag=0;			  //接收完成标志清除
						ReceiveCounter=0;
					  ReceErrorCounter=0;  //串口2发送错误次数清0
        } 				
				if(USART2_RecOverFlag==1)  //串口2接收数据完成,说明发送过程中数据有错，再发送一次测试数据
				{
					 USART1_SendData(0x01);  //串口1向上位机反馈信息，告知上位机：两个单片机数据交互时通讯有干扰，导致数据发送错误
					 USART1_SendData(0x88);  //通讯错误标志
					 LED = ~LED;
					 if(USART2_RecBuff[0]==0x01&&USART2_RecBuff[1]==0x2C)  //上次发送数据传输过程中出错
					 {
						   ReceErrorCounter++;
							 if(ReceErrorCounter<3)  //发送错误次数小于2次
							 {
								   
									 USART2_SendPredictData();	//数据重发
							 }		
					 }
           USART2_RecOverFlag=0;
					 USART2_ReceCount=0;					 
				}					
    }
}


//串口1中断服务程序
void USART1_Handle(void) interrupt 4
{
    if(RI)
    {
        TestData[ReceiveCounter++]=SBUF;
			  //这里不用定时器20ms做一帧数据结束的判断，那样做有BUG
			  //因为已知传送数据的长度，所以这里直接用长度来判断
        if(ReceiveCounter>=JZLenght)  //大于等于1024，说明矩阵传输完成
				{
					 RecOverFlag=1;
				}
				RI=0;
    }
    if(TI)
    {
        TI=0;
        USART1_BUSY=0;
    }
}


//串口2中断服务程序
void  USART2_Handle(void) interrupt 8
{
	  if(S2CON & S2TI)  //发送标志
    { 
			  USART2_BUSY = 0;               //清忙标志
        S2CON &= ~S2TI;         //清除S3TI位
    }
    if(S2CON & S2RI)  //接收标志
    {		  
        USART2_RecBuff[USART2_ReceCount++]=S2BUF;
        if(USART2_ReceCount>=USART2_ReceLenght)
				{
					 USART2_RecOverFlag=1;
					 USART2_ReceCount=0;
				}
        S2CON &= ~0x01;  //清楚接收中断
    }   
}
//定时器0，用于延时等待从机1数据处理完成，以免2个从机同时发送数据造成冲突
void TIME0_Handle(void) interrupt 1
{
	  Timer0_Counter++;
	  if(Timer0_Counter>=18)  //延时900ms,等待主机处理完数据
		{		 
			 TR0=0;
			 LED = ~LED;
			 Timer0_Counter=0;
			 USART2_AllowSendFlag=1;  //串口2允许发送数据了
		}     
}


