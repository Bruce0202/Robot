#pragma once
#include "Link.h"
#pragma pack(push)
#pragma pack(8) 

enum objAsix {
	xAxis = 0,
	xAxisPlus,
	yAxis,
	zAxis
};

class CLink_0 : public CLink
{
public:
	CLink_0(int linkindex, int freedom):CLink(linkindex, freedom)
	{
		//PID����
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
	//��̬====================================================//
	//positive
	void LinkForwardKin()  ;			//Link ����

	//inverse
	void MotionLineAbsolute() {}		//�յ�ѭ���������motion������Ĭ��״̬��һֱ�����ѭ��		
	void MotionLineVelocity() {}		//ĩ��λ��ģʽ
	void MotionLineRelative() {}		//ĩ���ٶ�ģʽ
	
	void MotionMode1();			//λ��ģʽ
	void MotionMode2();			//�Խ�
	void MotionMode3();			//š��
	void MotionMode4() {}		//�Զ���4
	void MotionMode5() {} 		//�Զ���5
	void MotionMode6() {}		//�Զ���6
	void MotionMode7() {}		//�Զ���7
	void MotionMode8() {}		//�Զ���8
	void MotionMode9() {}		//�Զ���9

	void SetLinkSelfAction() {}			//Link��������ĩ�˹��ߣ�ˮ��ǹ

private:
	void SynControl(objAsix currAxis, double TarMoveVel);   //��ƽ�еĽ���ͬ������
	void PosLimit(objAsix currAxis, bool normMode = true); //���г̵�����
	void xAxisMotion(double tarMoveVel, double beSafeMode = 10);
	void yAxisMotion(double tarMoveVel, double beSafeMode = 10);
	void zAxisMotion(double tarMoveVel, double beSafeMode = 10);
	void xPlusAxisMotion(double tarMoveVel, double beSafeMode = 10);

	void XAxisMotion();
	void YAxisMotion();
	void ZAxisMotion();
	void XPlusAxisMotion();
	void NLJAxisMotion();

public:

private:
	// PID����
	double Kp, Ki, Kd;
	double A, B, C;
	double e_pre_1[2];
	double e_pre_2[2];
	double e[2];

	//���Ʋ���
	const double nljMoveVel = 0.05;
	const double delta_t = 2.0;
	const double ACCEPTCURR = 1.0;
	const double TARCURR = 20.0;

	const double XDOWNPOSLIMIT = 0.0;
	const double XUPPOSLIMIT = 28.0;

	const double YDOWNPOSLIMIT = 0.0;
	const double YUPPOSLIMIT = 141;

	const double ZDOWNPOSLIMIT = 0.0;
	const double ZUPPOSLIMIT = 30;

	const double XPLUSDOWNPOSLIMIT = 0.0;
	const double XPLUSUPPOSLIMIT = 200;

	//����λ��ģʽ���˶��ٶȵ�Ĭ��ֵ
	double xPlusMoveVel = 1.0;
	double yMoveVel = 1.0;
	double zMoveVel = 0.5;
};

class CLink_1 : public CLink
{
public:
	CLink_1(int linkindex, int freedom):CLink(linkindex, freedom)
	{}
	~CLink_1()
	{}
	//��̬====================================================//
	//����
	virtual void LinkForwardKin() {}			//Link ����
	//���
	virtual void MotionLineAbsolute() {}			//�����˶�	
	virtual void MotionLineVelocity() {}			//ĩ��λ��ģʽ
	virtual void MotionLineRelative() {}			//ĩ���ٶ�ģʽ
	
	virtual void MotionMode1()  {}			//�Զ���1
	virtual void MotionMode2()  {}			//�Զ���2
	virtual void MotionMode3()  {}			//�Զ���3
	virtual void MotionMode4()  {}			//�Զ���4
	virtual void MotionMode5()  {}			//�Զ���5
	virtual void MotionMode6()  {}			//�Զ���6
	virtual void MotionMode7()  {}			//�Զ���7
	virtual void MotionMode8()  {}			//�Զ���8
	virtual void MotionMode9()  {}			//�Զ���9

	virtual void SetLinkSelfAction() {}			//Link��������ĩ�˹��ߣ�ˮ��ǹ

};
class CLink_2 : public CLink
{
public:
	CLink_2(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_2()
	{}
	//��̬====================================================//
	//����
	virtual void LinkForwardKin() {}			//Link ����
											//���
	virtual void MotionLineAbsolute() {}			//	
	virtual void MotionLineVelocity() {}			//
	virtual void MotionLineRelative() {}			//
	
	virtual void MotionMode1()  {}			//�Զ���1
	virtual void MotionMode2()  {}			//�Զ���2
	virtual void MotionMode3()  {}			//�Զ���3
	virtual void MotionMode4()  {}			//�Զ���4
	virtual void MotionMode5()  {}			//�Զ���5
	virtual void MotionMode6()  {}			//�Զ���6
	virtual void MotionMode7()  {}			//�Զ���7
	virtual void MotionMode8()  {}			//�Զ���8
	virtual void MotionMode9()  {}			//�Զ���9

	virtual void SetLinkSelfAction() {}			//Link��������ĩ�˹��ߣ�ˮ��ǹ

};
class CLink_3 : public CLink
{
public:
	CLink_3(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_3()
	{}
	//��̬====================================================//
	//����
	void LinkForwardKin() {}			//Link ����
											//���
	void MotionLineAbsolute() {}			//�յ�ѭ���������motion������Ĭ��״̬��һֱ�����ѭ��	
	void MotionLineVelocity() {}			//0
	void MotionLineRelative() {}			//0
	
	void MotionMode1()  {}			//�Զ���1
	void MotionMode2()  {}			//�Զ���2
	void MotionMode3()  {}			//�Զ���3
	void MotionMode4()  {}			//�Զ���4
	void MotionMode5()  {}			//�Զ���5
	void MotionMode6()  {}			//�Զ���6
	void MotionMode7()  {}			//�Զ���7
	void MotionMode8()  {}			//�Զ���8
	void MotionMode9()  {}			//�Զ���9

	void SetLinkSelfAction() {}			//Link��������ĩ�˹��ߣ�ˮ��ǹ
};
class CLink_4 : public CLink
{
public:
	CLink_4(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_4()
	{}
	//��̬====================================================//
	//����
	virtual void LinkForwardKin() {}			//Link ����
											//���
	virtual void MotionLineAbsolute() {}			//�����˶�	
	virtual void MotionLineVelocity() {}			//ĩ��λ��ģʽ
	virtual void MotionLineRelative() {}			//ĩ���ٶ�ģʽ
	
	virtual void MotionMode1()  {}			//�Զ���1
	virtual void MotionMode2()  {}			//�Զ���2
	virtual void MotionMode3()  {}			//�Զ���3
	virtual void MotionMode4()  {}			//�Զ���4
	virtual void MotionMode5()  {}			//�Զ���5
	virtual void MotionMode6()  {}			//�Զ���6
	virtual void MotionMode7()  {}			//�Զ���7
	virtual void MotionMode8()  {}			//�Զ���8
	virtual void MotionMode9()  {}			//�Զ���9

	virtual void SetLinkSelfAction() {}			//Link��������ĩ�˹��ߣ�ˮ��ǹ
};
class CLink_5 : public CLink
{
public:
	CLink_5(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_5()
	{}
	//�����غ���====================================================//
	//����
	virtual void LinkForwardKin() {}		//Link ����
											//���
	virtual void MotionLineAbsolute() {}		//�����˶�	
	virtual void MotionLineVelocity() {}		//ĩ��λ��ģʽ
	virtual void MotionLineRelative() {}		//ĩ���ٶ�ģʽ
	
	virtual void MotionMode1()  {}			//�Զ���1
	virtual void MotionMode2()  {}			//�Զ���2
	virtual void MotionMode3()  {}			//�Զ���3
	virtual void MotionMode4()  {}			//�Զ���4
	virtual void MotionMode5()  {}			//�Զ���5
	virtual void MotionMode6()  {}			//�Զ���6
	virtual void MotionMode7()  {}			//�Զ���7
	virtual void MotionMode8()  {}			//�Զ���8
	virtual void MotionMode9()  {}			//�Զ���9

	virtual void SetLinkSelfAction() {}			//Link��������ĩ�˹��ߣ�ˮ��ǹ
};

#pragma pack(pop)