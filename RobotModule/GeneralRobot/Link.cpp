#include "Link.h"

extern CErrorQueue g_ErrQue;
/********************************
*���ƣ�Self��е���๹�캯��
*���ܣ�����Self��е�������,������ĸ��������û�е�۵�λ���޷�
*�������ܣ���
*����ֵ����
********************************/
CLink::CLink(int linkindex,int freedom)
{	
	for (int i = 0; i < MAX_FREEDOM_LINK; i++)
	{
		m_Joints[i] = NULL;
	}
	m_Index = linkindex;
	m_Freedom = freedom;
	
	memset(&m_Status, 0, sizeof(stLinkStatus));
	memset(&m_Command, 0, sizeof(stLinkCommand));
	memset(&m_SetAction, 0, sizeof(stAction));
	memset(&m_preCommand, 0, sizeof(stLinkCommand));
}

/********************************
*���ƣ�VSelf��е������������
*���ܣ�����Self��е�������
*�������ܣ���
*����ֵ����
********************************/
CLink::~CLink()
{
	for (int i = 0; i < m_Freedom; i++)
	{
		D_SafeDelete(m_Joints[i]);
	}
}

/********************************
*���ƣ�Self��е�����ʼ������
*���ܣ�����VSelf��е�����������Ĳ���������ʼ����е�۱���Ĳ���
*�������ܣ�
*JointIndex: LInk��һ���������
*����ֵ����
********************************/
ERR_ID CLink::Init(int JointIndex, stJointPara JointPar[])
{
	//��е����Ĳ�����ʼ��
	ERR_ID err;
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i] = new CAxis;
		if (m_Joints[i] == NULL)
		{
			return eErrorJointInstance;
		}
		err = m_Joints[i]->Init(JointIndex, JointPar);
		if (eNoErr != err)
		{
			return err;
		}
		JointIndex++;
	}
	return eNoErr;
}
/********************************
*���ƣ�Linkicle������״̬��ȡ
*���ܣ���ȡ����״̬
*�������ܣ���
*����ֵ����
********************************/
void  CLink::GetStatus(_IN st_AxisGroupRead* stAxisGroupStatus)
{
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i]->GetStatus(stAxisGroupStatus);
	}
	m_ActAction = stAxisGroupStatus->LinkActActions[m_Index];

	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (m_ActAction.bflag[0] == true)
	{


		m_Status.stResverOutput.lReserve[0] = 1;
		if (m_Status.stResverOutput.lReserve[1] != m_ActAction.dData[2])
		{
			m_Status.stResverOutput.lReserve[2]++;
			m_Status.stResverOutput.lReserve[1] = (LONG)m_ActAction.dData[2];

		}

		m_Status.stResverOutput.fReserve[0] = m_ActAction.dData[0];//PGV_x
		m_Status.stResverOutput.fReserve[1] = m_ActAction.dData[10];//PGV_y
		m_Status.stResverOutput.fReserve[2] = (m_ActAction.dData[1]) * 0.1;//PGV_theta
		m_Status.stResverOutput.fReserve[3] = m_ActAction.dData[2];//tag

	}
	else
	{
		m_Status.stResverOutput.lReserve[0] = 0;
	}
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


	LinkForwardKin();

	
	JugeLinkState();
}
/********************************
*���ƣ�Linkicle����״̬��ȡ
*���ܣ�������״̬�жϳ���״̬
*�������ܣ���
*����ֵ����
********************************/
void  CLink::JugeLinkState()
{
	for (int i = 0; i < m_Freedom; i++)
	{
		if (m_Joints[i]->m_Status.eState == eAxis_ERRORSTOP)
		{ 
			m_Status.eLinkActState = eLINK_ERRORSTOP;
			return;
		}
	}
	for (int i = 0; i < m_Freedom; i++)
	{
		if (m_Joints[i]->m_Status.eState == eAxis_DISABLED)
		{
			m_Status.eLinkActState = eLINK_DISABLED;
			return;
		}
	}
	for (int i = 0; i < m_Freedom; i++)
	{
		if (m_Joints[i]->m_Status.eState == eAxis_STOPPING)
		{
			m_Status.eLinkActState = eLINK_STOPPING;
			return;
		}
	}
	for (int i = 0; i < m_Freedom; i++)
	{
		if (m_Joints[i]->m_Status.eState == eAxis_HOMING)
		{
			m_Status.eLinkActState = eLINK_HOMING;
			return;
		}
	}
	for (int i = 0; i < m_Freedom; i++)
	{
		if (m_Joints[i]->m_Status.eState == eAxis_DISCRETEMOTION ||
			m_Joints[i]->m_Status.eState == eAxis_CONTINOUSMOTION ||
			m_Joints[i]->m_Status.eState == eAxis_SYNCHRONIZEDMOTION)
		{
			m_Status.eLinkActState = eLINK_MOVING;
			return;
		}
	}
	m_Status.eLinkActState = eLINK_STANDSTILL;

}
/********************************
*���ƣ�Linkicle��״̬��
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void  CLink::StateMachine()
{
	LinkStateMachine();
	
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i]->StateMachine();
	}
	
}

/********************************
*���ƣ�Link����״̬��
*���ܣ������˶�����
*�������ܣ���
*����ֵ����
********************************/
void  CLink::LinkStateMachine()
{
	switch (m_Status.eLinkActState)
	{
	case eLINK_ERRORSTOP:
		switch (m_Command.eLinkCommd)
		{
		case eLINK_NONE:
			break;
		case eLINK_RESET:
			ActionReset();
			break;			//�޸�
		default:
			//���棬�����ڴ���״̬���޷����иò�����
			m_Command.eLinkCommd = eLINK_NONE;
			g_ErrQue.Enter_Queue(eWarningErrorState);
			break;
		}
		break;
	case eLINK_DISABLED:
		switch (m_Command.eLinkCommd)
		{
		case eLINK_NONE:
			break;
		case eLINK_RESET:
			ActionReset();
			break;			//�޸�
		case eLINK_POWER:
			ActionPower();
			break;
		default:
			m_Command.eLinkCommd = eLINK_NONE;
			g_ErrQue.Enter_Queue(eWarningRobotDisabled);
			break;
		}
		break;
	case eLINK_STANDSTILL:
		switch (m_Command.eLinkCommd)
		{
		case eLINK_NONE:
			break;
		case eLINK_RESET:
			ActionReset();
			break;			//�޸�
		case eLINK_HOME:
			ActionHome();
			break;
		case eLINK_POWER:
			ActionPower();
			break;
		case eLINK_STOP:
			ActionStop();
			break;
		case eLINK_HALT:
			ActionHalt();
			break;
		case eLINK_MOV:
			ActionMove();
			break;
		default:
			m_Command.eLinkCommd = eLINK_NONE;
			g_ErrQue.Enter_Queue(eWarningUnknownCommand);
			break;
		}
		break;
	case eLINK_STOPPING:
		switch (m_Command.eLinkCommd)
		{
		case eLINK_NONE:
			break;
		case eLINK_POWER:
			ActionPower();
			g_ErrQue.Enter_Queue(eWarningDisableWhenStopping);
			break;
		case eLINK_STOP:
			ActionStop();
			break;
		default:
			m_Command.eLinkCommd = eLINK_NONE;
			g_ErrQue.Enter_Queue(eWarningRobotIsStopping);
			break;
		}
		break;
	case eLINK_HOMING:
		switch (m_Command.eLinkCommd)
		{
		case eLINK_NONE:
			break;
		case eLINK_STOP:
			ActionStop();
			g_ErrQue.Enter_Queue(eWarningHomingInterrupted);
			break;
		case eLINK_POWER:
			ActionPower();
			g_ErrQue.Enter_Queue(eWarningDisableWhenHoming);
			break;
		default:
			m_Command.eLinkCommd = eLINK_NONE;
			g_ErrQue.Enter_Queue(eWarningRobotIsHoming);
			break;
		}
		break;
	case eLINK_MOVING:
		switch (m_Command.eLinkCommd)
		{
		case eLINK_NONE:
			break;
		case eLINK_POWER:
			ActionPower();
			g_ErrQue.Enter_Queue(eWarningDisableWhenMoving);
			break;
		case eLINK_STOP:
			ActionStop();
			break;
		case eLINK_HALT:
			ActionHalt();
			break;
		case eLINK_MOV:
			ActionMove();
			break;
		default:
			m_Command.eLinkCommd = eLINK_NONE;
			g_ErrQue.Enter_Queue(eWarningRobotIsMoving);
			break;
		}
		break;	//�޸�
	default:
		break;
	} //end of statemachine
	//���
	//�޸�
	//m_Command = m_preCommand; //����ָ������
	m_preCommand = m_Command;
}

/********************************
*���ƣ�Link�ද���·�
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void  CLink::SetAction(OUT st_AxisGroupSet* stAxisGroupSet)
{
	//�����������
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i]->SetAction(stAxisGroupSet);
	}
	//����Link����
	SetLinkSelfAction();

	stAxisGroupSet->LinkSetActions[m_Index] = m_SetAction;
}

/********************************
*���ƣ�Linkicle��ȫ������
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CLink::ActionReset()
{
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i]->CommdReset();
	}
	m_Command.eLinkCommd = eLINK_NONE;
}
/********************************
*���ƣ�Linkicle����ʹ��
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CLink ::ActionPower()
{
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i]->CommdPower(m_Command.stLinkKinPar.bEnable);
	}
	m_Command.eLinkCommd = eLINK_NONE;
}
/********************************
*���ƣ�Linkicle���
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CLink::ActionHome()
{
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i]->CommdHome();
	}
	m_Command.eLinkCommd = eLINK_NONE;
}
/********************************
*���ƣ�Linkicle��ͣ
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CLink::ActionStop()
{
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i]->CommdStop();
	}
	m_Command.eLinkCommd = eLINK_NONE;
}
/********************************
*���ƣ�Linkicle��ͣ
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CLink::ActionHalt()
{
	for (int i = 0; i < m_Freedom; i++)
	{
		m_Joints[i]->CommdHalt();
	}
	m_Command.eLinkCommd = eLINK_NONE;
	if (m_Command.stResverInput.bReserve[1] == true)
	{
		m_Status.stLinkActKin.bDone = false;
	}
}
/********************************
*���ƣ�Linkicle����
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void CLink::ActionMove()
{
	if (true == m_Status.stLinkActKin.bBusy)
	{
		//��ǰ�Զ�����δ��ɣ�����Ӧ�µ��˶�ָ��
		//m_Command = m_preCommand;
	}
	switch (m_Command.stLinkKinPar.eActMotionMode)
	{
	case eMotionLineAbsolute:
		MotionLineAbsolute();
		break;
	case eMotionLineVelocity:
		MotionLineVelocity();
		break;
	case eMotionLineRelative:
		MotionLineRelative();
		break;
	case eMotionMode1:
		MotionMode1();
		break;
	case eMotionMode2:
		MotionMode2();
		break;
	case eMotionMode3:
		MotionMode3();
		break;
	case eMotionMode4:
		MotionMode4();
		break;
	case eMotionMode5:
		MotionMode5();
		break;
	case eMotionMode6:
		MotionMode6();
		break;
	case eMotionMode7:
		MotionMode7();
		break;
	case eMotionMode8:
		MotionMode8();
		break;
	case eMotionMode9:
		MotionMode9();
		break;
	default:
		g_ErrQue.Enter_Queue(eWarningUnknownCommand);
		break;
	}
}

/********************************
*���ƣ�Linkָ��ӿ�
*���ܣ����ⲿ���ã��޸�LinkCommand��
*�������ܣ���
*����ֵ����
********************************/
void CLink::CommdReset()
{
	m_Command.eLinkCommd = eLINK_RESET;
}
void CLink::CommdPower(bool enable)
{
	m_Command.eLinkCommd = eLINK_POWER;
	m_Command.stLinkKinPar.bEnable = enable;
}
void CLink::CommdHome()
{
	m_Command.eLinkCommd = eLINK_HOME;
}
void CLink::CommdStop()
{
	m_Command.eLinkCommd = eLINK_STOP;
}
void CLink::CommdHalt()
{
	m_Command.eLinkCommd = eLINK_HALT;
}
void CLink::CommdMove()
{
	m_Command.eLinkCommd = eLINK_MOV;
}