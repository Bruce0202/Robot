/************************************************************/
/*机器人类定义
/*                                                          */
/*                                                          */
/************************************************************/
#pragma once
#include "LinkObject.h"
#pragma pack(push)
#pragma pack(8) 
//------------------------------------------------------------------------------//
//Robot class
//------------------------------------------------------------------------------//
class CGeneralRobot 
{
public:
	CGeneralRobot();
	~CGeneralRobot();

	ERR_ID Init(OUT st_InitParameter *pParmeter );															//机器人初始化
	void Run(st_AxisGroupRead *MC_status, st_AxisGroupSet *MC_SetCmd, Recvbuff *RecvData, Sendbuff *SendData);															//机器人运行

protected:
	void GetStatus();					//获取机器人状态
	void GetCommd();					//机器人控制指令
	void StateMachine();				//机器人状态机
	void SetFeedback();					//机器人反馈状态
	void SetAction();					//机器人运动指令

	void JugeRobotState();				//判断机器人状态
	void SetRobotAction() {}
	void RobotStateMachine();

private:
	virtual void ActionReset();
	virtual void ActionPower();
	virtual void ActionHome();
	virtual void ActionStop();
	virtual void ActionHalt();
	virtual void ActionMove();

	virtual void Clear();

public:
	
	CLink*			m_links[6];

	int				m_LinksNum;
	int				m_JointsNum;

	stAction		m_SetAction;
	stAction		m_ActAction;
	stRobotCommand	m_Commd;
	stRobotStatus	m_Status;

protected:
	DINT			m_Heartbeat;
	LONG			m_Lost;
	
	st_AxisGroupRead	m_stAxisGroupStatus;
	st_AxisGroupSet		m_stAxisGroupSet;
	Recvbuff		m_RecvData;
	Sendbuff		m_SendData;

private:
	stJointPara		m_JointPar[MAX_FREEDOM_ROBOT];
	
};

#pragma pack(pop)