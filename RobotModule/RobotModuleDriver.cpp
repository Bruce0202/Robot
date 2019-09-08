///////////////////////////////////////////////////////////////////////////////
// RobotModuleDriver.cpp
#include "TcPch.h"
#pragma hdrstop

#include "RobotModuleDriver.h"
#include "RobotModuleClassFactory.h"

DECLARE_GENERIC_DEVICE(ROBOTMODULEDRV)

IOSTATUS CRobotModuleDriver::OnLoad( )
{
	TRACE(_T("CObjClassFactory::OnLoad()\n") );
	m_pObjClassFactory = new CRobotModuleClassFactory();

	return IOSTATUS_SUCCESS;
}

VOID CRobotModuleDriver::OnUnLoad( )
{
	delete m_pObjClassFactory;
}

unsigned long _cdecl CRobotModuleDriver::ROBOTMODULEDRV_GetVersion( )
{
	return( (ROBOTMODULEDRV_Major << 8) | ROBOTMODULEDRV_Minor );
}

