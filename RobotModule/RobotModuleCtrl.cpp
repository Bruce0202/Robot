// RobotModuleCtrl.cpp : Implementation of CTcRobotModuleCtrl
#include "TcPch.h"
#pragma hdrstop

#include "RobotModuleW32.h"
#include "RobotModuleCtrl.h"

/////////////////////////////////////////////////////////////////////////////
// CRobotModuleCtrl

CRobotModuleCtrl::CRobotModuleCtrl() 
	: ITcOCFCtrlImpl<CRobotModuleCtrl, CRobotModuleClassFactory>() 
{
}

CRobotModuleCtrl::~CRobotModuleCtrl()
{
}

