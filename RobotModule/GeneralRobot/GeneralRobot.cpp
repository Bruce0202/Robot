#include "GeneralRobot.h"
CErrorQueue g_ErrQue;	//������robot.h���ļ�������ȫ�ֱ���������Ҫextern���η�
/********************************
*���ƣ�AutoVehicleRobot�๹�캯��
*���ܣ���������˶���
*�������ܣ���
*����ֵ����
********************************/
CGeneralRobot::CGeneralRobot()
{
	for (int i = 0; i < 5; i++)
	{
		m_links[i] = NULL;
	}
	m_LinksNum = 0;
	m_JointsNum = 0;	
	memset(&m_SetAction, 0, sizeof(stAction));
	memset(&m_Commd, 0, sizeof(stLinkCommand));
	memset(&m_Status, 0, sizeof(stLinkStatus));

	m_Heartbeat = 0;
	m_Lost = 0;
	memset(&m_stAxisGroupStatus, 0, sizeof(st_AxisGroupRead));
	memset(&m_stAxisGroupSet, 0, sizeof(st_AxisGroupSet));
	memset(&m_RecvData, 0, sizeof(Recvbuff));
	memset(&m_SendData, 0, sizeof(Sendbuff));

	memset(&m_JointPar, 0, MAX_FREEDOM_ROBOT*sizeof(stJointPara));;
}

/********************************
*���ƣ�AutoVehicleRobot����������
*���ܣ�����AutoVehicleRobot�����
*�������ܣ���
*����ֵ����
********************************/
CGeneralRobot::~CGeneralRobot()
{
	//��ȫ�ͷ�ָ��
	for (int i = 0; i < m_LinksNum; i++)
	{
		D_SafeDelete(m_links[i]);
	}
}

/********************************
*���ƣ����ʼ������
*���ܣ��ֱ��ʼ��������󣬻�е�۶���,��������˵����������״̬������״̬������
*�������ܣ���
*����ֵ����
********************************/
ERR_ID CGeneralRobot::Init(OUT st_InitParameter *pParmeter)
{
	ERR_ID err = eNoErr;
	//ʵ����link
	m_LinksNum = 0;
	m_JointsNum = 0;
	for (int cnt = 0; cnt < 6; cnt++)
	{
		if (0 != LINK_FREEDOM[cnt])
		{
			switch (cnt)
			{
			case 0:
				m_links[m_LinksNum] = new CLink_0(m_LinksNum, LINK_FREEDOM[cnt]);
				//��ʼ�������
				for (int i = 0; i < LINK_FREEDOM[cnt]; i++)
				{
					m_JointPar[m_JointsNum].Limit_Positive = LINK_0_JOINT_LIMIT_POS[i];
					m_JointPar[m_JointsNum].Limit_Negtive = LINK_0_JOINT_LIMIT_NEG[i];
					m_JointPar[m_JointsNum].Max_Vel = LINK_0_JOINT_MAX_VEL[i];
					m_JointPar[m_JointsNum].Ration = LINK_0_JOINT_RATIO[i];
					m_JointPar[m_JointsNum].Direction = LINK_0_JOINT_DIRECTION[i];
					m_JointPar[m_JointsNum].EncoderCorr = LINK_0_JOINT_ENCODER_CORR[i];
					m_JointsNum++;
				}
				break;
			case 1:
				m_links[m_LinksNum] = new CLink_1(m_LinksNum, LINK_FREEDOM[cnt]);
				//��ʼ�������
				for (int i = 0; i < LINK_FREEDOM[cnt]; i++)
				{
					m_JointPar[m_JointsNum].Limit_Positive = LINK_1_JOINT_LIMIT_POS[i];
					m_JointPar[m_JointsNum].Limit_Negtive = LINK_1_JOINT_LIMIT_NEG[i];
					m_JointPar[m_JointsNum].Max_Vel = LINK_1_JOINT_MAX_VEL[i];
					m_JointPar[m_JointsNum].Ration = LINK_1_JOINT_RATIO[i];
					m_JointPar[m_JointsNum].Direction = LINK_1_JOINT_DIRECTION[i];
					m_JointPar[m_JointsNum].EncoderCorr = LINK_1_JOINT_ENCODER_CORR[i];
					m_JointsNum++;
				}
				break;
			case 2:
				m_links[m_LinksNum] = new CLink_2(m_LinksNum, LINK_FREEDOM[cnt]);
				//��ʼ�������
				for (int i = 0; i < LINK_FREEDOM[cnt]; i++)
				{
					m_JointPar[m_JointsNum].Limit_Positive = LINK_2_JOINT_LIMIT_POS[i];
					m_JointPar[m_JointsNum].Limit_Negtive = LINK_2_JOINT_LIMIT_NEG[i];
					m_JointPar[m_JointsNum].Max_Vel = LINK_2_JOINT_MAX_VEL[i];
					m_JointPar[m_JointsNum].Ration = LINK_2_JOINT_RATIO[i];
					m_JointPar[m_JointsNum].Direction = LINK_2_JOINT_DIRECTION[i];
					m_JointPar[m_JointsNum].EncoderCorr = LINK_2_JOINT_ENCODER_CORR[i];
					m_JointsNum++;
				}
				break;
			case 3:
				m_links[m_LinksNum] = new CLink_3(m_LinksNum, LINK_FREEDOM[cnt]);
				//��ʼ�������
				for (int i = 0; i < LINK_FREEDOM[cnt]; i++)
				{
					m_JointPar[m_JointsNum].Limit_Positive = LINK_3_JOINT_LIMIT_POS[i];
					m_JointPar[m_JointsNum].Limit_Negtive = LINK_3_JOINT_LIMIT_NEG[i];
					m_JointPar[m_JointsNum].Max_Vel = LINK_3_JOINT_MAX_VEL[i];
					m_JointPar[m_JointsNum].Ration = LINK_3_JOINT_RATIO[i];
					m_JointPar[m_JointsNum].Direction = LINK_3_JOINT_DIRECTION[i];
					m_JointPar[m_JointsNum].EncoderCorr = LINK_3_JOINT_ENCODER_CORR[i];
					m_JointsNum++;
				}
				break;
			case 4:
				m_links[m_LinksNum] = new CLink_4(m_LinksNum, LINK_FREEDOM[cnt]);
				for (int i = 0; i < LINK_FREEDOM[cnt]; i++)
				{
					m_JointPar[m_JointsNum].Limit_Positive = LINK_4_JOINT_LIMIT_POS[i];
					m_JointPar[m_JointsNum].Limit_Negtive = LINK_4_JOINT_LIMIT_NEG[i];
					m_JointPar[m_JointsNum].Max_Vel = LINK_4_JOINT_MAX_VEL[i];
					m_JointPar[m_JointsNum].Ration = LINK_4_JOINT_RATIO[i];
					m_JointPar[m_JointsNum].Direction = LINK_4_JOINT_DIRECTION[i];
					m_JointPar[m_JointsNum].EncoderCorr = LINK_4_JOINT_ENCODER_CORR[i];
					m_JointsNum++;
				}
				break;
			case 5:
				m_links[m_LinksNum] = new CLink_5(m_LinksNum, LINK_FREEDOM[cnt]);
				for (int i = 0; i < LINK_FREEDOM[cnt]; i++)
				{
					m_JointPar[m_JointsNum].Limit_Positive = LINK_5_JOINT_LIMIT_POS[i];
					m_JointPar[m_JointsNum].Limit_Negtive = LINK_5_JOINT_LIMIT_NEG[i];
					m_JointPar[m_JointsNum].Max_Vel = LINK_5_JOINT_MAX_VEL[i];
					m_JointPar[m_JointsNum].Ration = LINK_5_JOINT_RATIO[i];
					m_JointPar[m_JointsNum].Direction = LINK_5_JOINT_DIRECTION[i];
					m_JointPar[m_JointsNum].EncoderCorr = LINK_5_JOINT_ENCODER_CORR[i];
					m_JointsNum++;
				}
				break;
			default:
				break;
			}//end switch
			
			//��ʼ��link
			if (NULL != m_links[m_LinksNum])
			{
				err = m_links[m_LinksNum]->Init(m_JointsNum - LINK_FREEDOM[cnt], m_JointPar);
				if (eNoErr != err)
				{
					return err;
				}
			}
			else
			{
				return eErrorLinkInstance;
			}//end if NULL
			m_LinksNum++;

		}//end if LINK_FREEDOM
	}
	
	pParmeter->JointsNum = m_JointsNum;
	pParmeter->RecvSize = sizeof(DINT) + sizeof(stRobotCommand) + m_LinksNum*sizeof(stLinkCommand) + m_JointsNum*sizeof(st_SetAxis);
	pParmeter->SendSize = sizeof(DINT) + sizeof(stRobotStatus) + m_LinksNum * sizeof(stLinkStatus) + m_JointsNum * sizeof(st_ReadAxis);

	return eNoErr;
}

/********************************
*���ƣ����к���
*���ܣ��ֱ��ʼ��������󣬻�е�۶���,��������˵����������״̬������״̬������
*�������ܣ���
*����ֵ����
********************************/
void CGeneralRobot::Run(st_AxisGroupRead *MC_status, st_AxisGroupSet *MC_SetCmd, Recvbuff *RecvData, Sendbuff *SendData)
{

	memcpy(&m_stAxisGroupStatus, MC_status, sizeof(st_AxisGroupRead));
	memcpy(m_RecvData, *RecvData, sizeof(Recvbuff)); //�޸�
	
	GetStatus();
	
	GetCommd();
	
	StateMachine();
	
	SetFeedback();
	
	//���������û�п����ã�����û��ʹ�õ���
	SetAction();
	
	
	memcpy(MC_SetCmd, &m_stAxisGroupSet,sizeof(st_AxisGroupSet));
	
	memcpy(*SendData, m_SendData, sizeof(Sendbuff)); //�޸�
	
}

/********************************
*���ƣ�Robot״̬��ȡ
*���ܣ�����Link��״̬�ж�Robot��״̬��Link����ײ��Asix����
*�������ܣ���
*����ֵ����
********************************/
void CGeneralRobot::GetStatus()
{
	//��ȡlinks ״̬
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->GetStatus(&m_stAxisGroupStatus);
	}
	m_ActAction = m_stAxisGroupStatus.RobotActAction;
	//����links״̬�ж�robot״̬
	JugeRobotState();
}

/********************************
*���ƣ�Robot״̬��ȡ
*���ܣ�����Link״̬�ж�robot״̬
*�������ܣ���
*����ֵ����
********************************/
void CGeneralRobot::JugeRobotState()
{
	for (int i = 0; i < m_LinksNum; i++)
	{
		if (m_links[i]->m_Status.eLinkActState == eLINK_ERRORSTOP)
		{
			m_Status.eActState = eLINK_ERRORSTOP;
			return;
		}
	}
	for (int i = 0; i < m_LinksNum; i++)
	{
		if (m_links[i]->m_Status.eLinkActState == eLINK_DISABLED)
		{
			m_Status.eActState = eLINK_DISABLED;
			return;
		}
	}
	for (int i = 0; i < m_LinksNum; i++)
	{
		if (m_links[i]->m_Status.eLinkActState == eLINK_STOPPING)
		{
			m_Status.eActState = eLINK_STOPPING;
			return;
		}
	}
	for (int i = 0; i < m_LinksNum; i++)
	{
		if (m_links[i]->m_Status.eLinkActState == eLINK_HOMING)
		{
			m_Status.eActState = eLINK_HOMING;
			return;
		}
	}
	for (int i = 0; i < m_LinksNum; i++)
	{
		if (m_links[i]->m_Status.eLinkActState == eLINK_MOVING)
		{
			m_Status.eActState = eLINK_MOVING;
			return;
		}
	}
	m_Status.eActState = eLINK_STANDSTILL;
}

/********************************
*���ƣ�Robot��ȡָ������
*���ܣ���recvbuff�����ݷַ������ƶ���
*�������ܣ�
	Recvbuff �� [RobotCMD][Link1CMD][Joint1CMD][Joint2CMD][Joint3CMD]..[Link2CMD]..
*����ֵ����
********************************/
void CGeneralRobot::GetCommd()
{
	int length = 0;



	//����״̬�ж�
	DINT HeartbeatNew = 0;
	memcpy(&HeartbeatNew, m_RecvData, sizeof(DINT));//�޸�
	length = length + sizeof(DINT);
#ifndef myDEBUG
	if (HeartbeatNew == 0 || HeartbeatNew == m_Heartbeat) //�����ݴ���
	{
		m_Lost++;
		if (m_Lost > LOST_COMM_THRESHOLD)
		{

			//��ͨѶ��ֹͣ������.Debug��Ч
			ActionStop();

		}
		return;
	}
#endif // myDEBUG
	
#ifdef myDEBUG
	if (HeartbeatNew == 0)
		return;
#endif
	
	m_Heartbeat = HeartbeatNew;
	m_Lost = 0;


	//�����ݴ���ʱ��������Ϣ
	memcpy(&m_Commd, m_RecvData + length, sizeof(stRobotCommand)); //�޸�
	length = length + sizeof(stRobotCommand);
	for (int i = 0; i < m_LinksNum; i++)
	{
		memcpy(&(m_links[i]->m_Command), m_RecvData + length, sizeof(stLinkCommand)); //�޸�
		length = length + sizeof(stLinkCommand);
		for (int j = 0; j < m_links[i]->m_Freedom; j++)
		{
			memcpy(&(m_links[i]->m_Joints[j]->m_Commd), m_RecvData + length, sizeof(st_SetAxis));//�޸�
			length = length + sizeof(st_SetAxis);
		}	
	}
	
}
/********************************
*���ƣ�Robot��״̬��
*���ܣ�
*�������ܣ���
*����ֵ����
********************************/
void  CGeneralRobot::StateMachine()
{
	RobotStateMachine();
	
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->StateMachine();
	}
}
/********************************
*���ƣ�Robot״̬��
*���ܣ������·���Robot��ָ�����״̬������������ָ���robotaction��DA�������
*	   ע�⣡��������ָ��Ḳ����λ�����͵���ָ�
*�������ܣ�
*����ֵ����
********************************/
void  CGeneralRobot::RobotStateMachine()
{
	//��Ӧ���
	Clear();//�޸�
	switch (m_Status.eActState)
	{
	case eROBOT_ERRORSTOP:
		switch (m_Commd.eCommd)
		{
		case eROBOT_NONE: //��ָ��ʱ��Ĭ��ִ�и���䣡��������
			break;
		case eROBOT_RESET:
			ActionReset();
			break;		//�޸�
		default:
			m_Commd.eCommd = eROBOT_NONE;
			g_ErrQue.Enter_Queue(eWarningErrorState);
			break;
		}
		break;
	case eROBOT_DISABLED:
		switch (m_Commd.eCommd)
		{
		case eROBOT_NONE://��ָ��ʱ��Ĭ��ִ�и���䣡��������
			break;
		case eROBOT_RESET:
			ActionReset();
			break;//�޸�
		case eROBOT_POWER:
			ActionPower();
			break;
		default:
			m_Commd.eCommd = eROBOT_NONE;
			g_ErrQue.Enter_Queue(eWarningRobotDisabled);
			break;
		}
		break;
	case eROBOT_STANDSTILL:
		switch (m_Commd.eCommd)
		{
		case eROBOT_NONE://��ָ��ʱ��Ĭ��ִ�и���䣡��������
			break;
		case eROBOT_RESET:
			ActionReset();
			break;//�޸�
		case eROBOT_HOME:
			ActionHome();
			break;
		case eROBOT_POWER:
			ActionPower();
			break;
		case eROBOT_STOP:
			ActionStop();
			break;
		case eROBOT_HALT:
			ActionHalt();
			break;
		case eROBOT_MOV:
			ActionMove();
			break;
		default:
			m_Commd.eCommd = eROBOT_NONE;
			g_ErrQue.Enter_Queue(eWarningUnknownCommand);
			break;
		}
		break;
	case eROBOT_STOPPING:
		switch (m_Commd.eCommd)
		{
		case eROBOT_NONE://��ָ��ʱ��Ĭ��ִ�и���䣡��������
			break;
		case eROBOT_POWER:
			ActionPower();
			g_ErrQue.Enter_Queue(eWarningDisableWhenStopping);
			break;
		case eROBOT_STOP:
			ActionStop();
			break;
		default:
			m_Commd.eCommd = eROBOT_NONE;
			g_ErrQue.Enter_Queue(eWarningRobotIsStopping);
			break;
		}
		break;
	case eROBOT_HOMING:
		switch (m_Commd.eCommd)
		{
		case eROBOT_NONE://��ָ��ʱ��Ĭ��ִ�и���䣡��������
			break;
		case eROBOT_STOP:
			ActionStop();
			g_ErrQue.Enter_Queue(eWarningHomingInterrupted);
			break;
		case eROBOT_POWER:
			ActionPower();
			g_ErrQue.Enter_Queue(eWarningDisableWhenHoming);
			break;
		default:
			m_Commd.eCommd = eROBOT_NONE;
			g_ErrQue.Enter_Queue(eWarningRobotIsHoming);
			break;
		}
		break;
	case eROBOT_MOVING:
		switch (m_Commd.eCommd)
		{
		case eROBOT_NONE://��ָ��ʱ��Ĭ��ִ�и���䣡��������
			break;
		case eROBOT_POWER:
			ActionPower();
			g_ErrQue.Enter_Queue(eWarningDisableWhenMoving);
			break;
		case eROBOT_STOP:
			ActionStop();
			break;
		case eROBOT_HALT:
			ActionHalt();
			break;
		case eROBOT_MOV:
			ActionMove();
			break;
		default:
			m_Commd.eCommd = eROBOT_NONE;
			g_ErrQue.Enter_Queue(eWarningRobotIsMoving);
			break;
		}
		break;
	default:
		break;
	}//End of swith(state)


}
/********************************
*���ƣ�Robot��״̬����
*���ܣ���Robot״̬����״̬�������飬��PLCת������λ����
*�������ܣ���
*����ֵ����
********************************/
void CGeneralRobot::SetFeedback()
{
	int length = 0;
	//д��������Ϣ
	memcpy(m_SendData, &m_Heartbeat,sizeof(DINT)); //�޸�
	length = length + sizeof(DINT);

	//��������Ϣ
	m_Status.ErrNum = g_ErrQue.Out_Queue(m_Status.ErrList); //���´�����Ϣ��
	memcpy(m_SendData, &m_Status,sizeof(stRobotStatus)); //�޸�
	length = length + sizeof(stRobotStatus);
	//Links��Ϣ
	for (int i = 0; i < m_LinksNum; i++)
	{
		memcpy(m_SendData + length, &(m_links[i]->m_Status), sizeof(stLinkStatus)); //�޸�
		length = length + sizeof(stLinkStatus);
		for (int j = 0; j < m_links[i]->m_Freedom; j++)
		{
			memcpy(m_SendData + length, &(m_links[i]->m_Joints[j]->m_Status), sizeof(st_ReadAxis));//�޸�
			length = length + sizeof(st_ReadAxis);
		}
	}
}


/********************************
*���ƣ�Linkicle�ද���·�
*���ܣ���ָ������������m_SetAction��ͨ��PLCִ�ж���
*�������ܣ���
*����ֵ����
********************************/
void  CGeneralRobot::SetAction()
{
	//m_stAxisGroupSet.Number = m_JointsNum; �о�ûʲô�ã��߼���Ҳ��Ӧ���ڴ˴�����ֵ
	//����Link����
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->SetAction(&m_stAxisGroupSet);
	}
	//����Robot����
	SetRobotAction();
	m_stAxisGroupSet.RobotSetAction = m_SetAction;
}


void CGeneralRobot::ActionReset()
{
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->CommdReset();
	}
	m_Commd.eCommd = eROBOT_NONE;
}
void CGeneralRobot::ActionPower()
{
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->CommdPower(m_Commd.stKinPar.bEnable);
	}
	m_Commd.eCommd = eROBOT_NONE;
}
void CGeneralRobot::ActionHome()
{
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->CommdHome();
	}
	m_Commd.eCommd = eROBOT_NONE;
}
void CGeneralRobot::ActionStop()
{
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->CommdStop();
	}
	m_Commd.eCommd = eROBOT_NONE;
}
void CGeneralRobot::ActionHalt()
{
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->CommdHalt();
	}
	m_Commd.eCommd = eROBOT_NONE;
}
void CGeneralRobot::ActionMove()
{
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->CommdMove();
	}
}
/********************************
*���ƣ��������
*���ܣ����Robot���в����Ĵ��󣬲����������
*�������ܣ���
*����ֵ����
********************************/
void CGeneralRobot:: Clear()
{
	g_ErrQue.Clean_Queue(m_Commd.ErrNum);
	m_Commd.ErrNum = 0;
}