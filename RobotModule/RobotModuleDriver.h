///////////////////////////////////////////////////////////////////////////////
// RobotModuleDriver.h

#ifndef __ROBOTMODULEDRIVER_H__
#define __ROBOTMODULEDRIVER_H__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "TcBase.h"

#define ROBOTMODULEDRV_NAME        "ROBOTMODULE"
#define ROBOTMODULEDRV_Major       1
#define ROBOTMODULEDRV_Minor       0

#define DEVICE_CLASS CRobotModuleDriver

#include "ObjDriver.h"

class CRobotModuleDriver : public CObjDriver
{
public:
	virtual IOSTATUS	OnLoad();
	virtual VOID		OnUnLoad();

	//////////////////////////////////////////////////////
	// VxD-Services exported by this driver
	static unsigned long	_cdecl ROBOTMODULEDRV_GetVersion();
	//////////////////////////////////////////////////////
	
};

Begin_VxD_Service_Table(ROBOTMODULEDRV)
	VxD_Service( ROBOTMODULEDRV_GetVersion )
End_VxD_Service_Table


#endif // ifndef __ROBOTMODULEDRIVER_H__