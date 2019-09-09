#pragma once
#include "Link.h"
#pragma pack(push)
#pragma pack(8) 
class CLink_0 : public CLink
{
public:
	CLink_0(int linkindex, int freedom):CLink(linkindex, freedom)
	{
		//PID参数
		Kp = 0.5;
		Ki = 0.1;
		Kd = 0.5;
		A = Kp + Ki + Kd;
		B = -2 * Kd - Kp;
		C = Kd;
		for (int i = 0; i < 2; i++) {
			e_pre_1[i] = 0.0;
			e_pre_2[i] = 0.0;
			e[i] = 0.0;
		}
	}
	~CLink_0()
	{}
	//多态====================================================//
	//正解
	void LinkForwardKin()  ;			//Link 正解
	//逆解
	void MotionLineAbsolute() {}		//空的循环，如果跑motion程序在默认状态下一直进这个循环		
	void MotionLineVelocity() {}		//末端位置模式
	void MotionLineRelative() {}		//末端速度模式
	
	void MotionMode1();			//自定义1
	void MotionMode2();			//自定义2
	void MotionMode3();			//自定义3
	void MotionMode4();			//自定义4
	void MotionMode5() {} 		//自定义5
	void MotionMode6() {}		//自定义6
	void MotionMode7() {}		//自定义7
	void MotionMode8() {}		//自定义8
	void MotionMode9() {}		//自定义9

	void SetLinkSelfAction() {}			//Link操作，如末端工具，水炮枪

private:
	void SynControl(int numL, int numR, int count, double TarMoveVel, double posL, double posR, int i = 1);   //对平行的进行同步控制
	void PosLimit(int currAxis = 0, bool normMode = true); //对行程的限制

private:
	// PID参数
	double Kp, Ki, Kd;
	double A, B, C;
	double e_pre_1[2];
	double e_pre_2[2];
	double e[2];

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
	virtual void MotionLineAbsolute() {}			//	
	virtual void MotionLineVelocity() {}			//
	virtual void MotionLineRelative() {}			//
	
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
	void LinkForwardKin() {}			//Link 正解
											//逆解
	void MotionLineAbsolute() {}			//空的循环，如果跑motion程序在默认状态下一直进这个循环	
	void MotionLineVelocity() {}			//0
	void MotionLineRelative() {}			//0
	
	void MotionMode1()  {}			//自定义1
	void MotionMode2()  {}			//自定义2
	void MotionMode3()  {}			//自定义3
	void MotionMode4()  {}			//自定义4
	void MotionMode5()  {}			//自定义5
	void MotionMode6()  {}			//自定义6
	void MotionMode7()  {}			//自定义7
	void MotionMode8()  {}			//自定义8
	void MotionMode9()  {}			//自定义9

	void SetLinkSelfAction() {}			//Link操作，如末端工具，水炮枪
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