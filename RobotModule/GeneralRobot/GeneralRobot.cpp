#include "GeneralRobot.h"
CErrorQueue g_ErrQue;	//包含该robot.h的文件不引用全局变量，不需要extern修饰符
/********************************
*名称：AutoVehicleRobot类构造函数
*功能：构造机器人对象
*参数介绍：无
*返回值：无
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
*名称：AutoVehicleRobot类析构函数
*功能：回收AutoVehicleRobot类对象
*参数介绍：无
*返回值：无
********************************/
CGeneralRobot::~CGeneralRobot()
{
	//安全释放指针
	for (int i = 0; i < m_LinksNum; i++)
	{
		D_SafeDelete(m_links[i]);
	}
}

/********************************
*名称：类初始化函数
*功能：分别初始化车体对象，机械臂对象,清零机器人的命令参数，状态参数，状态机变量
*参数介绍：无
*返回值：无
********************************/
ERR_ID CGeneralRobot::Init(OUT st_InitParameter *pParmeter)
{
	ERR_ID err = eNoErr;
	//实例化link
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
				//初始化轴参数
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
				//初始化轴参数
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
				//初始化轴参数
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
				//初始化轴参数
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
			
			//初始化link
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
*名称：运行函数
*功能：分别初始化车体对象，机械臂对象,清零机器人的命令参数，状态参数，状态机变量
*参数介绍：无
*返回值：无
********************************/
void CGeneralRobot::Run(st_AxisGroupRead *MC_status, st_AxisGroupSet *MC_SetCmd, Recvbuff *RecvData, Sendbuff *SendData)
{

	memcpy(&m_stAxisGroupStatus, MC_status, sizeof(st_AxisGroupRead));
	memcpy(m_RecvData, *RecvData, sizeof(Recvbuff)); //修改
	
	GetStatus();
	
	GetCommd();
	
	StateMachine();
	
	SetFeedback();
	
	//这个东西还没有开发好，或者没有使用到过
	SetAction();
	
	
	memcpy(MC_SetCmd, &m_stAxisGroupSet,sizeof(st_AxisGroupSet));
	
	memcpy(*SendData, m_SendData, sizeof(Sendbuff)); //修改
	
}

/********************************
*名称：Robot状态获取
*功能：根据Link的状态判断Robot的状态，Link由最底层的Asix决定
*参数介绍：无
*返回值：无
********************************/
void CGeneralRobot::GetStatus()
{
	//获取links 状态
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->GetStatus(&m_stAxisGroupStatus);
	}
	m_ActAction = m_stAxisGroupStatus.RobotActAction;
	//根据links状态判断robot状态
	JugeRobotState();
}

/********************************
*名称：Robot状态获取
*功能：根据Link状态判断robot状态
*参数介绍：无
*返回值：无
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
*名称：Robot获取指令内容
*功能：将recvbuff的内容分发到控制对象
*参数介绍：
	Recvbuff ： [RobotCMD][Link1CMD][Joint1CMD][Joint2CMD][Joint3CMD]..[Link2CMD]..
*返回值：无
********************************/
void CGeneralRobot::GetCommd()
{
	int length = 0;



	//连接状态判断
	DINT HeartbeatNew = 0;
	memcpy(&HeartbeatNew, m_RecvData, sizeof(DINT));//修改
	length = length + sizeof(DINT);
#ifndef myDEBUG
	if (HeartbeatNew == 0 || HeartbeatNew == m_Heartbeat) //无数据传入
	{
		m_Lost++;
		if (m_Lost > LOST_COMM_THRESHOLD)
		{

			//无通讯，停止机器人.Debug无效
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


	//有数据传入时，更新信息
	memcpy(&m_Commd, m_RecvData + length, sizeof(stRobotCommand)); //修改
	length = length + sizeof(stRobotCommand);
	for (int i = 0; i < m_LinksNum; i++)
	{
		memcpy(&(m_links[i]->m_Command), m_RecvData + length, sizeof(stLinkCommand)); //修改
		length = length + sizeof(stLinkCommand);
		for (int j = 0; j < m_links[i]->m_Freedom; j++)
		{
			memcpy(&(m_links[i]->m_Joints[j]->m_Commd), m_RecvData + length, sizeof(st_SetAxis));//修改
			length = length + sizeof(st_SetAxis);
		}	
	}
	
}
/********************************
*名称：Robot类状态机
*功能：
*参数介绍：无
*返回值：无
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
*名称：Robot状态机
*功能：根据下发到Robot的指令，运行状态机，输出轴控制指令和robotaction（DA输出）；
*	   注意！产生的轴指令会覆盖上位机发送的轴指令；
*参数介绍：
*返回值：无
********************************/
void  CGeneralRobot::RobotStateMachine()
{
	//响应清错
	Clear();//修改
	switch (m_Status.eActState)
	{
	case eROBOT_ERRORSTOP:
		switch (m_Commd.eCommd)
		{
		case eROBOT_NONE: //无指令时，默认执行该语句！！！！！
			break;
		case eROBOT_RESET:
			ActionReset();
			break;		//修改
		default:
			m_Commd.eCommd = eROBOT_NONE;
			g_ErrQue.Enter_Queue(eWarningErrorState);
			break;
		}
		break;
	case eROBOT_DISABLED:
		switch (m_Commd.eCommd)
		{
		case eROBOT_NONE://无指令时，默认执行该语句！！！！！
			break;
		case eROBOT_RESET:
			ActionReset();
			break;//修改
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
		case eROBOT_NONE://无指令时，默认执行该语句！！！！！
			break;
		case eROBOT_RESET:
			ActionReset();
			break;//修改
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
		case eROBOT_NONE://无指令时，默认执行该语句！！！！！
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
		case eROBOT_NONE://无指令时，默认执行该语句！！！！！
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
		case eROBOT_NONE://无指令时，默认执行该语句！！！！！
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
*名称：Robot类状态返回
*功能：将Robot状态存入状态返回数组，由PLC转发到上位机。
*参数介绍：无
*返回值：无
********************************/
void CGeneralRobot::SetFeedback()
{
	int length = 0;
	//写入心跳信息
	memcpy(m_SendData, &m_Heartbeat,sizeof(DINT)); //修改
	length = length + sizeof(DINT);

	//机器人信息
	m_Status.ErrNum = g_ErrQue.Out_Queue(m_Status.ErrList); //更新错误信息；
	memcpy(m_SendData, &m_Status,sizeof(stRobotStatus)); //修改
	length = length + sizeof(stRobotStatus);
	//Links信息
	for (int i = 0; i < m_LinksNum; i++)
	{
		memcpy(m_SendData + length, &(m_links[i]->m_Status), sizeof(stLinkStatus)); //修改
		length = length + sizeof(stLinkStatus);
		for (int j = 0; j < m_links[i]->m_Freedom; j++)
		{
			memcpy(m_SendData + length, &(m_links[i]->m_Joints[j]->m_Status), sizeof(st_ReadAxis));//修改
			length = length + sizeof(st_ReadAxis);
		}
	}
}


/********************************
*名称：Linkicle类动作下发
*功能：将指令存入输出变量m_SetAction，通过PLC执行动作
*参数介绍：无
*返回值：无
********************************/
void  CGeneralRobot::SetAction()
{
	//m_stAxisGroupSet.Number = m_JointsNum; 感觉没什么用，逻辑上也不应该在此处被赋值
	//设置Link动作
	for (int i = 0; i < m_LinksNum; i++)
	{
		m_links[i]->SetAction(&m_stAxisGroupSet);
	}
	//设置Robot动作
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
*名称：清除错误
*功能：清除Robot运行产生的错误，不包括轴错误。
*参数介绍：无
*返回值：无
********************************/
void CGeneralRobot:: Clear()
{
	g_ErrQue.Clean_Queue(m_Commd.ErrNum);
	m_Commd.ErrNum = 0;
}