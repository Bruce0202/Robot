///////////////////////////////////////////////////////////////////////////////
// RobotModuleCtrl.h

#ifndef __ROBOTMODULECTRL_H__
#define __ROBOTMODULECTRL_H__

#include <atlbase.h>
#include <atlcom.h>

#define ROBOTMODULEDRV_NAME "ROBOTMODULE"

#include "resource.h"       // main symbols
#include "RobotModuleW32.h"
#include "TcBase.h"
#include "RobotModuleClassFactory.h"
#include "TcOCFCtrlImpl.h"

class CRobotModuleCtrl 
	: public CComObjectRootEx<CComMultiThreadModel>
	, public CComCoClass<CRobotModuleCtrl, &CLSID_RobotModuleCtrl>
	, public IRobotModuleCtrl
	, public ITcOCFCtrlImpl<CRobotModuleCtrl, CRobotModuleClassFactory>
{
public:
	CRobotModuleCtrl();
	virtual ~CRobotModuleCtrl();

DECLARE_REGISTRY_RESOURCEID(IDR_ROBOTMODULECTRL)
DECLARE_NOT_AGGREGATABLE(CRobotModuleCtrl)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CRobotModuleCtrl)
	COM_INTERFACE_ENTRY(IRobotModuleCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl)
	COM_INTERFACE_ENTRY(ITcCtrl2)
END_COM_MAP()

};

#endif // #ifndef __ROBOTMODULECTRL_H__
