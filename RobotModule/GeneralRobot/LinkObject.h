#pragma once
#include "Link.h"
#pragma pack(push)
#pragma pack(8) 

#include <stdio.h>


class CLink_0 : public CLink
{
public:
	CLink_0(int linkindex, int freedom):CLink(linkindex, freedom)
	{


	}
	~CLink_0()
	{}
	//多态====================================================//
	//正解
	virtual void LinkForwardKin()  ;			//Link 正解
	//逆解
	virtual void MotionLineAbsolute() {}		//单轴运动	
	virtual void MotionLineVelocity() {}		//末端位置模式
	virtual void MotionLineRelative() {}		//末端速度模式
	
	virtual void MotionMode1();					//自定义1
	virtual void MotionMode2();					//自定义2
	virtual void MotionMode3();					//自定义3
	virtual void MotionMode4();					//自定义4
	virtual void MotionMode5();					//自定义5
	virtual void MotionMode6();					//自定义6
	virtual void MotionMode7();					//自定义7
	virtual void MotionMode8();					//自定义8
	virtual void MotionMode9() {}				//自定义9

	void SetLinkSelfAction() {}					//Link操作，如末端工具，水炮枪
};

class CLink_1 : public CLink
{
public:
	CLink_1(int linkindex, int freedom):CLink(linkindex, freedom)
	{}
	~CLink_1()
	{}
	//多态====================================================//
	//正解
	virtual void LinkForwardKin() {}			//Link 正解
	//逆解
	virtual void MotionLineAbsolute() {}			//单轴运动	
	virtual void MotionLineVelocity() {}			//末端位置模式
	virtual void MotionLineRelative() {}			//末端速度模式
	
	virtual void MotionMode1()  {}			//自定义1
	virtual void MotionMode2()  {}			//自定义2
	virtual void MotionMode3()  {}			//自定义3
	virtual void MotionMode4()  {}			//自定义4
	virtual void MotionMode5()  {}			//自定义5
	virtual void MotionMode6()  {}			//自定义6
	virtual void MotionMode7()  {}			//自定义7
	virtual void MotionMode8()  {}			//自定义8
	virtual void MotionMode9()  {}			//自定义9

	virtual void SetLinkSelfAction() {}			//Link操作，如末端工具，水炮枪

};
class CLink_2 : public CLink
{
public:
	CLink_2(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_2()
	{}
	//多态====================================================//
	//正解
	virtual void LinkForwardKin() {}			//Link 正解
											//逆解
	virtual void MotionLineAbsolute() {}			//单轴运动	
	virtual void MotionLineVelocity() {}			//末端位置模式
	virtual void MotionLineRelative() {}			//末端速度模式
	
	virtual void MotionMode1()  {}			//自定义1
	virtual void MotionMode2()  {}			//自定义2
	virtual void MotionMode3()  {}			//自定义3
	virtual void MotionMode4()  {}			//自定义4
	virtual void MotionMode5()  {}			//自定义5
	virtual void MotionMode6()  {}			//自定义6
	virtual void MotionMode7()  {}			//自定义7
	virtual void MotionMode8()  {}			//自定义8
	virtual void MotionMode9()  {}			//自定义9

	virtual void SetLinkSelfAction() {}			//Link操作，如末端工具，水炮枪

};
class CLink_3 : public CLink
{
public:
	CLink_3(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_3()
	{}
	//多态====================================================//
	//正解
	virtual void LinkForwardKin() {}			//Link 正解
											//逆解
	virtual void MotionLineAbsolute() {}			//单轴运动	
	virtual void MotionLineVelocity() {}			//末端位置模式
	virtual void MotionLineRelative() {}			//末端速度模式
	
	virtual void MotionMode1()  {}			//自定义1
	virtual void MotionMode2()  {}			//自定义2
	virtual void MotionMode3()  {}			//自定义3
	virtual void MotionMode4()  {}			//自定义4
	virtual void MotionMode5()  {}			//自定义5
	virtual void MotionMode6()  {}			//自定义6
	virtual void MotionMode7()  {}			//自定义7
	virtual void MotionMode8()  {}			//自定义8
	virtual void MotionMode9()  {}			//自定义9

	virtual void SetLinkSelfAction() {}			//Link操作，如末端工具，水炮枪
};
class CLink_4 : public CLink
{
public:
	CLink_4(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_4()
	{}
	//多态====================================================//
	//正解
	virtual void LinkForwardKin() {}			//Link 正解
											//逆解
	virtual void MotionLineAbsolute() {}			//单轴运动	
	virtual void MotionLineVelocity() {}			//末端位置模式
	virtual void MotionLineRelative() {}			//末端速度模式
	
	virtual void MotionMode1()  {}			//自定义1
	virtual void MotionMode2()  {}			//自定义2
	virtual void MotionMode3()  {}			//自定义3
	virtual void MotionMode4()  {}			//自定义4
	virtual void MotionMode5()  {}			//自定义5
	virtual void MotionMode6()  {}			//自定义6
	virtual void MotionMode7()  {}			//自定义7
	virtual void MotionMode8()  {}			//自定义8
	virtual void MotionMode9()  {}			//自定义9

	virtual void SetLinkSelfAction() {}			//Link操作，如末端工具，水炮枪
};
class CLink_5 : public CLink
{
public:
	CLink_5(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_5()
	{}
	//可重载函数====================================================//
	//正解
	virtual void LinkForwardKin() {}		//Link 正解
											//逆解
	virtual void MotionLineAbsolute() {}		//单轴运动	
	virtual void MotionLineVelocity() {}		//末端位置模式
	virtual void MotionLineRelative() {}		//末端速度模式
	
	virtual void MotionMode1()  {}			//自定义1
	virtual void MotionMode2()  {}			//自定义2
	virtual void MotionMode3()  {}			//自定义3
	virtual void MotionMode4()  {}			//自定义4
	virtual void MotionMode5()  {}			//自定义5
	virtual void MotionMode6()  {}			//自定义6
	virtual void MotionMode7()  {}			//自定义7
	virtual void MotionMode8()  {}			//自定义8
	virtual void MotionMode9()  {}			//自定义9

	virtual void SetLinkSelfAction() {}			//Link操作，如末端工具，水炮枪
};

#pragma pack(pop)