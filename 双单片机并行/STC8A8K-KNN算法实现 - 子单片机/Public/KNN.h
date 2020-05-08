#ifndef _KNN_H_
#define _KNN_H_


typedef unsigned char  uint8;                   /* defined for unsigned 8-bits integer variable 	无符号8位整型变量  */
typedef signed   char  int8;                    /* defined for signed 8-bits integer variable		有符号8位整型变量  */
typedef unsigned short uint16;                  /* defined for unsigned 16-bits integer variable 	无符号16位整型变量 */
typedef signed   short int16;                   /* defined for signed 16-bits integer variable 		有符号16位整型变量 */
typedef struct
{
    int distance;  //距离值
    char label;     //训练数据矩阵标签
} Distance;


#define   Numtrain     30  // 训练数据的数量 
#define   Numpredict   1  //预测数据数量 
#define   Testpredict  1  //测试数据数量
#define   JZLenght  1024  //测试数据长度


#endif