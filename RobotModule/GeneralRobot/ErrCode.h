/*********************************************************************/
/*功能：错误代码枚举类型
/*创建者：
/*重大改动及时间：
2016.3.30：创建：
********************************************************************/
#pragma once
#pragma pack(push)
#pragma pack(8) 
#include <stdio.h>//修改
#include <string.h>
#include "../RobotModuleServices.h"

enum ERR_ID :ULONG
{
	eNoErr = 0,//无错误

	eErrorAxisPositionLimit = 10001,
	eErrorAxisVelocityLimit,
	eErrorLinkInstance,
	eErrorJointInstance,
	eErrorUnkownLinkIndex,

	eErrorJointLimtPar,
	eErrorJointMaxVel,
	eErrorJointRadio,
	eErrorJointDirection,

	eErrorLinkCommandAuto1,
	eErrorLinkCommandAuto2,
	eErrorLinkCommandAuto3,
	eErrorLinkCommandAuto4,
	eErrorCoupleSetLimit,			//对轮设定位置超限
	eErrorCoupleLimit,				//对轮位置超限
	eErrorLinkCommandAuto5,
	eErrorLinkCommandAuto6,

	
	eWarningErrorState		= 20001,
	eWarningRobotDisabled,
	eWarningUnknownCommand,
	eWarningDisableWhenStopping,
	eWarningRobotIsStopping,
	eWarningDisableWhenHoming,
	eWarningHomingInterrupted,
	eWarningRobotIsHoming,
	eWarningDisableWhenMoving,
	eWarningRobotIsMoving,
	eWarningDismatch,

	eWarningMissQCode,
	eWarningCoupleLimit			//对轮位置超限


};

//------------------------------------------------------------------------------//
//class 错误队列操作 
//------------------------------------------------------------------------------//
#define ERR_QUE_SIZE 100

class CErrorQueue
{
public:
	CErrorQueue();
	~CErrorQueue();

	bool Enter_Queue(ERR_ID newErr);				//入队

	ULONG Out_Queue(ERR_ID ErrQue[], ULONG len = 10);
	ULONG Clean_Queue(ULONG len = 10);//清除

private:
	void Set_NULL();				//置空队列

	bool Is_NULL();					//判断队列是否为空

private:

	ERR_ID m_ErrQue[ERR_QUE_SIZE];
	LONG m_front;				//第一个有数据的数组元素
	LONG m_rear;				//队尾巴后第一个空位置
};

extern CErrorQueue g_ErrQue;
#pragma pack(pop)