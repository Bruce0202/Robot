#include "Axis.h"

extern CErrorQueue g_ErrQue;//包含Axis.h的文件引用此全局变量，需要extern修饰符
/********************************
*名称：Axis轴类构造函数
*功能：构造Axis轴类对象
*参数介绍：无
*返回值：无
********************************/
CAxis::CAxis()
{
	m_index = NULL;
	m_homed = false;
	memset(&m_Status, 0, sizeof(st_ReadAxis));
	memset(&m_Commd, 0, sizeof(m_Commd));
	memset(&m_SetAction, 0, sizeof(st_SetAxis));

	m_PositiveLimit = 0;
	m_NegtiveLimit = 0;
	m_VelocityLimit = 0;
	m_Direction = 0;
	m_Radio = 0;
	m_EncoderCorr = 0;

}

/********************************
*名称：Axis轴类析构函数
*功能：回收Axis轴类对象
*参数介绍：无
*返回值：无
********************************/
CAxis::~CAxis()
{
}

/********************************
*名称：Axis轴类初始化函数
*功能：初始化轴类变量
*参数介绍：无
*返回值：无
********************************/
ERR_ID CAxis::Init(int index, _IN stJointPara JointPar[])
{
	m_index = index;

	m_PositiveLimit = JointPar[index].Limit_Positive;
	m_NegtiveLimit = JointPar[index].Limit_Negtive;
	m_VelocityLimit = JointPar[index].Max_Vel;
	m_Radio = JointPar[index].Ration;
	m_Direction = JointPar[index].Direction;
	m_EncoderCorr = JointPar[index].EncoderCorr;


	if (m_PositiveLimit < m_NegtiveLimit)
	{
		//等于表示限位有误
		return eErrorJointLimtPar;
	}
	//修改3月12号
	if (fabs_(m_VelocityLimit) <= MIN_VEL_LIMT)
	{
		return eErrorJointMaxVel;
	}
	if (m_Radio < MINIUM)
	{
		return eErrorJointRadio;
	}
	if (m_Direction != 1 && m_Direction != -1)
	{
		return eErrorJointDirection;
	}
	return eNoErr;
}
/********************************
*名称：状态获取函数
*功能：
*参数介绍：无
*返回值：无
********************************/
void CAxis::GetStatus(_IN st_AxisGroupRead* stAxisGroupStatus)
{
	m_Status = stAxisGroupStatus->AxisGoup[m_index];
	
	//减速比换算
	m_Status.Position = m_Status.Position / m_Radio;
	m_Status.Velocity = m_Status.Velocity / m_Radio;

	//单位转换degree->rad
	m_Status.Position = m_Status.Position ;
	m_Status.Velocity = m_Status.Velocity ;
	
	//方向转换
	m_Status.Position = m_Status.Position * m_Direction;
	m_Status.Velocity = m_Status.Velocity* m_Direction;

	//编码器偏置
	m_Status.Position = m_Status.Position + m_EncoderCorr;
	
}

/********************************
*名称：轴状态机
*功能：该处不执行状态机，只负责变量传递;
*参数介绍：无
*返回值：无
********************************/
void CAxis::StateMachine()
{
	//运动模式下，如果下发速度小于限制值，则停止运动
//	if ((fabs_(m_Commd.Velocity) < MIN_VEL_LIMT)
//		&& (m_Status.eState == eAxis_CONTINOUSMOTION || m_Status.eState == eAxis_DISCRETEMOTION))
//	{
//		m_Commd.eMC_Motion = eMC_HALT;
//
	m_SetAction = m_Commd;
	//if (m_Commd.eMC_Motion == eMC_RESET || m_Commd.eMC_Motion == eMC_HOME || m_Commd.eMC_Motion == eMC_POWER
	//	|| m_Commd.eMC_Motion == eMC_STOP || m_Commd.eMC_Motion == eMC_HALT || m_Commd.eMC_Motion == eMC_MOV_RELATIVE
	//	|| m_Commd.eMC_Motion == eMC_MOV_ABS || m_Commd.eMC_Motion == eMC_MOV_CON_ABS)
	if(m_Commd.eMC_Motion != eMC_MOV_VEL)
	{
		m_Commd.eMC_Motion = eMC_NONE;
	}
//	m_Commd.eMC_Motion = eMC_NONE;	
}
/********************************
*名称：轴动作设置函数
*功能：设置轴的运动参数
*参数介绍：无
*返回值：无
********************************/
void CAxis::SetAction(OUT st_AxisGroupSet*	stAxisGroupSet)
{	
	//减速比换算
	m_SetAction.Position = m_SetAction.Position * m_Radio;
	m_SetAction.Velocity = m_SetAction.Velocity* m_Radio;
	m_SetAction.Torque = m_SetAction.Torque / m_Radio;
	m_SetAction.EndVelocity = m_SetAction.EndVelocity * m_Radio;
	m_SetAction.Distance = m_SetAction.Distance * m_Radio;
	
	//单位转换rad->degree
	m_SetAction.Position = m_SetAction.Position;
	m_SetAction.Velocity = m_SetAction.Velocity ;
	m_SetAction.EndVelocity = m_SetAction.EndVelocity;
	m_SetAction.Distance = m_SetAction.Distance;


	//方向转换
	m_SetAction.Position = m_SetAction.Position * m_Direction;
	m_SetAction.Velocity = m_SetAction.Velocity* m_Direction;
	m_SetAction.Torque = m_SetAction.Torque* m_Direction;
	m_SetAction.EndVelocity = m_SetAction.EndVelocity * m_Direction;
	m_SetAction.Distance = m_SetAction.Distance * m_Direction;

	//编码器偏置
	m_SetAction.Position = m_SetAction.Position + m_EncoderCorr;

	stAxisGroupSet->AxisGroup[m_index] = m_SetAction;
}

/********************************
*名称：轴重置
*功能：重置轴
*参数介绍：无
*返回值：无
********************************/
void CAxis::CommdReset()
{
	m_Commd.eMC_Motion = eMC_RESET;
}
/********************************
*名称：轴使能
*功能：重置轴
*参数介绍：无
*返回值：无
********************************/
void CAxis::CommdPower(bool power)
{
	m_Commd.eMC_Motion = eMC_POWER;
	m_Commd.bEnable = power;
}

/********************************
*名称：轴回零
*功能：
*参数介绍：无
*返回值：无
********************************/
void CAxis::CommdHome(double distance)
{
	m_Commd.eMC_Motion = eMC_HOME;
	m_Commd.Distance = distance;
}

/********************************
*名称：轴急停
*功能：
*参数介绍：无
*返回值：无
********************************/
void CAxis::CommdStop()
{
	m_Commd.eMC_Motion = eMC_STOP;
}
/********************************
*名称：轴暂停停
*功能：
*参数介绍：无
*返回值：无
********************************/
void CAxis::CommdHalt()
{
	m_Commd.eMC_Motion = eMC_HALT;
}

/********************************
*名称：轴暂停
*功能：
*参数介绍：无
*返回值：无
********************************/
void CAxis::CommdMove(st_SetAxis Axiscommd)
{/*
	if (false == DealwithLimit(Axiscommd))
	{
		return;
	}
*/
	m_Commd.eMC_Motion = Axiscommd.eMC_Motion;
	m_Commd.Position = Axiscommd.Position;
	m_Commd.Velocity = Axiscommd.Velocity;
	m_Commd.EndVelocity = Axiscommd.EndVelocity;
	m_Commd.Distance = Axiscommd.Distance;

}

/********************************
*名称：轴限位限速处理
*功能：
*参数介绍：无
*返回值：无
********************************/
bool CAxis::DealwithLimit(st_SetAxis Axiscommd)
{
	//当前位置超限，仅仅允许单侧运动Mov_vel
	if (m_Status.Position > m_PositiveLimit )
	{
		if ((eMC_MOV_VEL == Axiscommd.eMC_Motion) && (Axiscommd.Velocity < 0))
		{
			; //允许反向运动
		}	
		else 
		{
			g_ErrQue.Enter_Queue(eErrorAxisPositionLimit);
			return false;
		}
		
	}
	else if (m_Status.Position < m_NegtiveLimit)
	{
		if ( (eMC_MOV_VEL == Axiscommd.eMC_Motion)  && (Axiscommd.Velocity > 0))
		{
			;//允许反向运动
		}
		else
		{
			g_ErrQue.Enter_Queue(eErrorAxisPositionLimit);
			return false;
		}		
	}

	//超速
	if (fabs_(m_Status.Velocity) > m_VelocityLimit ||fabs_(Axiscommd.Velocity) > m_VelocityLimit || fabs_(Axiscommd.EndVelocity) > m_VelocityLimit)
	{
		g_ErrQue.Enter_Queue(eErrorAxisVelocityLimit);
		CommdHalt();
		return false;
	}

	return true;
}

