/****************************************************************/
/*功能：移动载体类
/*创建者：AT
/*重大改动及时间：
2017.5.14：  创建
*****************************************************************/
#pragma once
#include <stdlib.h>
#include "Axis.h"
#pragma pack(push)
#pragma pack(8) 

class CLink
{
public:
	CLink(int linkindex, int freedom);
	~CLink();
public:
	ERR_ID Init(int JointIndex, stJointPara JointPar[]);											//车体初始化
	void GetStatus(_IN st_AxisGroupRead* stAxisGroupStatus);
	void StateMachine();				 									//状态机（运行函数）
	void SetAction(OUT st_AxisGroupSet*	stAxisGroupSet);					//机器人指令
	
	//LINK操作接口----------------------------------------------------------------------//
	void CommdReset();
	void CommdPower(bool enable);
	void CommdHome();
	void CommdStop();
	void CommdHalt();
	void CommdMove();

	double output1;
	double output2;
	double output3;
	double output4;
	double output5;
	double output6;
	double output7;
	double output8;
	////////////
	double output88;
	////////////
protected:
	void JugeLinkState();
	void LinkStateMachine();
	
	//多态，虚函数----------------------------------------------------------------------//
	//正解
	virtual void LinkForwardKin() {}			//Link 正解
	//逆解
	virtual void MotionLineAbsolute() {}	//沿直线运动到绝对位置	
	virtual void MotionLineVelocity() {}	//末端速度运动
	virtual void MotionLineRelative() {}	//相对位置模式
	virtual void MotionMode1() {}			//自定义1
	virtual void MotionMode2() {}			//自定义2
	virtual void MotionMode3() {}			//自定义3
	virtual void MotionMode4() {}			//自定义4
	virtual void MotionMode5() {}			//自定义5
	virtual void MotionMode6() {}			//自定义6
	virtual void MotionMode7() {}			//自定义7
	virtual void MotionMode8() {}			//自定义8
	virtual void MotionMode9() {}			//自定义9
	
	virtual void SetLinkSelfAction() {}		//Link操作，如末端工具，水炮枪逻辑控制

private:
	virtual void ActionReset();
	virtual void ActionPower();
	virtual void ActionHome();
	virtual void ActionStop();
	virtual void ActionHalt();
	virtual void ActionMove();

public:
	CAxis*			m_Joints[MAX_FREEDOM_LINK];
	int				m_Index;			//按顺序连续编号
	int				m_Freedom;

	stLinkStatus	m_Status;			//Link当前的状态
	stLinkCommand	m_Command;			//需要LINK执行的命令
	stAction		m_SetAction;		//PLC执行
	stAction		m_ActAction;		//PLC当前动作

	int				test;
private:
	stLinkCommand	m_preCommand;

	
};
#pragma pack(pop)