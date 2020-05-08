#include "KNN.h"
Distance distance1;
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
//两个结构体数组互换
void exchange(Distance *in, uint8 index1,uint8 index2)
{
    Distance tmp;
    tmp = in[index1];
    in[index1] = in[index2];
    in[index2] = tmp;
}


//输入in  一个结构体数组
//lenght 训练数据模板数量
//从小到达排序
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
uint8 prediction(uint8 K, char *in, char train[][JZLenght])
{
    int i, it;
    int predict = 0;
    Distance distance[Numtrain]; //距离值暂存,Numtrain训练数据模板长度
    /*求取输入digit与训练数据的距离*/
    for(it=0; it<Numtrain; it++)
    {
        distance[it].distance = calDistance(in,train[it]);  //求测试数据与训练数据的距离
        distance[it].label = it;  //由于数组本就是按0-9模板来做下标的,所以这里直接给就行
    }
    /*给计算的距离排序*/

    selectSort(distance, Numtrain);  //对距离值数组进行排序
    for(i=0; i<K; i++)
    {
        predict += distance[i].label;  //取距离目标值最近的K个值
    }
    return predict/K;  //返回预测值
}

