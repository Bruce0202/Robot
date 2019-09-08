#include "Axis.h"

extern CErrorQueue g_ErrQue;//����Axis.h���ļ����ô�ȫ�ֱ�������Ҫextern���η�
/********************************
*���ƣ�Axis���๹�캯��
*���ܣ�����Axis�������
*�������ܣ���
*����ֵ����
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
*���ƣ�Axis������������
*���ܣ�����Axis�������
*�������ܣ���
*����ֵ����
********************************/
CAxis::~CAxis()
{
}

/********************************
*���ƣ�Axis�����ʼ������
*���ܣ���ʼ���������
*�������ܣ���
*����ֵ����
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
		//���ڱ�ʾ��λ����
		return eErrorJointLimtPar;
	}
	//�޸�3��12��
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
*���ƣ�״̬��ȡ����
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CAxis::GetStatus(_IN st_AxisGroupRead* stAxisGroupStatus)
{
	m_Status = stAxisGroupStatus->AxisGoup[m_index];
	
	//���ٱȻ���
	m_Status.Position = m_Status.Position / m_Radio;
	m_Status.Velocity = m_Status.Velocity / m_Radio;

	//��λת��degree->rad
	m_Status.Position = m_Status.Position ;
	m_Status.Velocity = m_Status.Velocity ;
	
	//����ת��
	m_Status.Position = m_Status.Position * m_Direction;
	m_Status.Velocity = m_Status.Velocity* m_Direction;

	//������ƫ��
	m_Status.Position = m_Status.Position + m_EncoderCorr;
	
}

/********************************
*���ƣ���״̬��
*���ܣ��ô���ִ��״̬����ֻ�����������;
*�������ܣ���
*����ֵ����
********************************/
void CAxis::StateMachine()
{
	//�˶�ģʽ�£�����·��ٶ�С������ֵ����ֹͣ�˶�
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
*���ƣ��ᶯ�����ú���
*���ܣ���������˶�����
*�������ܣ���
*����ֵ����
********************************/
void CAxis::SetAction(OUT st_AxisGroupSet*	stAxisGroupSet)
{	
	//���ٱȻ���
	m_SetAction.Position = m_SetAction.Position * m_Radio;
	m_SetAction.Velocity = m_SetAction.Velocity* m_Radio;
	m_SetAction.Torque = m_SetAction.Torque / m_Radio;
	m_SetAction.EndVelocity = m_SetAction.EndVelocity * m_Radio;
	m_SetAction.Distance = m_SetAction.Distance * m_Radio;
	
	//��λת��rad->degree
	m_SetAction.Position = m_SetAction.Position;
	m_SetAction.Velocity = m_SetAction.Velocity ;
	m_SetAction.EndVelocity = m_SetAction.EndVelocity;
	m_SetAction.Distance = m_SetAction.Distance;


	//����ת��
	m_SetAction.Position = m_SetAction.Position * m_Direction;
	m_SetAction.Velocity = m_SetAction.Velocity* m_Direction;
	m_SetAction.Torque = m_SetAction.Torque* m_Direction;
	m_SetAction.EndVelocity = m_SetAction.EndVelocity * m_Direction;
	m_SetAction.Distance = m_SetAction.Distance * m_Direction;

	//������ƫ��
	m_SetAction.Position = m_SetAction.Position + m_EncoderCorr;

	stAxisGroupSet->AxisGroup[m_index] = m_SetAction;
}

/********************************
*���ƣ�������
*���ܣ�������
*�������ܣ���
*����ֵ����
********************************/
void CAxis::CommdReset()
{
	m_Commd.eMC_Motion = eMC_RESET;
}
/********************************
*���ƣ���ʹ��
*���ܣ�������
*�������ܣ���
*����ֵ����
********************************/
void CAxis::CommdPower(bool power)
{
	m_Commd.eMC_Motion = eMC_POWER;
	m_Commd.bEnable = power;
}

/********************************
*���ƣ������
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CAxis::CommdHome(double distance)
{
	m_Commd.eMC_Motion = eMC_HOME;
	m_Commd.Distance = distance;
}

/********************************
*���ƣ��ἱͣ
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CAxis::CommdStop()
{
	m_Commd.eMC_Motion = eMC_STOP;
}
/********************************
*���ƣ�����ͣͣ
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CAxis::CommdHalt()
{
	m_Commd.eMC_Motion = eMC_HALT;
}

/********************************
*���ƣ�����ͣ
*���ܣ�
*�������ܣ���
*����ֵ����
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
*���ƣ�����λ���ٴ���
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
bool CAxis::DealwithLimit(st_SetAxis Axiscommd)
{
	//��ǰλ�ó��ޣ������������˶�Mov_vel
	if (m_Status.Position > m_PositiveLimit )
	{
		if ((eMC_MOV_VEL == Axiscommd.eMC_Motion) && (Axiscommd.Velocity < 0))
		{
			; //�������˶�
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
			;//�������˶�
		}
		else
		{
			g_ErrQue.Enter_Queue(eErrorAxisPositionLimit);
			return false;
		}		
	}

	//����
	if (fabs_(m_Status.Velocity) > m_VelocityLimit ||fabs_(Axiscommd.Velocity) > m_VelocityLimit || fabs_(Axiscommd.EndVelocity) > m_VelocityLimit)
	{
		g_ErrQue.Enter_Queue(eErrorAxisVelocityLimit);
		CommdHalt();
		return false;
	}

	return true;
}

