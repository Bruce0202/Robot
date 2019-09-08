#include "LinkObject.h"
extern CErrorQueue g_ErrQue;
/*********************************************************************************************************
*名称：LINK正解
*功能：根据轴位置/速度，计算LINK正解
*参数：
	Input： 对象成员变量m_Joints[].m_Status
	Output:	对象成员变量m_Status
*返回值：无
*********************************************************************************************************/
void CLink_0::LinkForwardKin()
{ 

}


/*********************************************************************************************************
*名称：LINK联动逆解
*功能：根据LINK联动目标，计算轴位置/速度
*参数：
Iput： 对象成员变量m_Command
Output:	对象成员变量m_Joints[].m_Commd
*返回值：无
*********************************************************************************************************/

//------------------------------------------------------------------------------//
//MotionMode1 自动:直线纠偏(PGV)
//------------------------------------------------------------------------------//
void CLink_0::MotionMode1() {}

//------------------------------------------------------------------------------//
//MotionMode2 自动：原地转角(4组对轮角度相同) phiTarget=0时为reset
//------------------------------------------------------------------------------//
void CLink_0::MotionMode2() {}

//------------------------------------------------------------------------------//
//MotionMode3 圆弧运动 从Motion1到3要清空qr_code
//------------------------------------------------------------------------------//
void CLink_0::MotionMode3() {}

//------------------------------------------------------------------------------//
//MotionMode4 原地转角(圆弧运动前)
//------------------------------------------------------------------------------//
void CLink_0::MotionMode4() {}

//------------------------------------------------------------------------------//
//MotionMode5 直线运动(手动) 输入：LinkVel[0]
//------------------------------------------------------------------------------//
void CLink_0::MotionMode5() {}

//------------------------------------------------------------------------------//
//MotionMode6 调整方向角(手动) 输入：LinkVel[3]
//该函数未使用，方向角手动调整用mode2实现
//------------------------------------------------------------------------------//
void CLink_0::MotionMode6() {}

//------------------------------------------------------------------------------//
//MotionMode7 圆弧运动(手动) 输入：LinkVel[0]对应走圆弧/LinkVel[2]对应原地转;
//------------------------------------------------------------------------------//
void CLink_0::MotionMode7() {}

//------------------------------------------------------------------------------//
//MotionMode8 走圆弧前调整角度(手动) 输入：LinkVel[3]
//由于原地旋转和圆弧运动合并，该函数未使用
//------------------------------------------------------------------------------//
void CLink_0::MotionMode8() {}

