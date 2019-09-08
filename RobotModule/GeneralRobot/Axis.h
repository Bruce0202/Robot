/*************************************************************************/
/*功能：单轴控制类
/*创建者:AT
/*重大改动及时间:
2017 5 12

*************************************************************************/
#pragma once
#include "../RobotModuleServices.h"

#include "DataStruct.h"
#pragma pack(push)
#pragma pack(8) 

class CAxis
{

public:
	CAxis();
	~CAxis();

	ERR_ID Init(int index, _IN stJointPara JointPar[]);							//轴初始化
	void GetStatus(_IN st_AxisGroupRead* stAxisGroupStatus);
	void StateMachine();				 				//轴的状态机由PLC完成
	void SetAction(OUT st_AxisGroupSet*	stAxisGroupSet);								//车体指令

	//Axis操作接口----------------------------------------------------------------------//
	void CommdReset();
	void CommdPower(bool power);
	void CommdHome(double distance = 0);
	void CommdStop();
	void CommdHalt();
	void CommdMove(st_SetAxis Axiscommd);

private:
	bool DealwithLimit(st_SetAxis Axiscommd);


public:
	int				m_index;
	bool			m_homed;
	st_ReadAxis		m_Status;
	st_SetAxis		m_Commd;		//cmd from upper

	st_SetAxis		m_SetAction;	//action to pcl_nc
private:
	double			m_PositiveLimit;
	double			m_NegtiveLimit;
	double			m_VelocityLimit;
	LONG			m_Direction;			//电机方向与轴方向一致性
	double			m_Radio;
	double			m_EncoderCorr;


};
#pragma pack(pop)