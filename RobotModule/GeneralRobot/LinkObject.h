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
	//��̬====================================================//
	//����
	virtual void LinkForwardKin()  ;			//Link ����
	//���
	virtual void MotionLineAbsolute() {}		//�����˶�	
	virtual void MotionLineVelocity() {}		//ĩ��λ��ģʽ
	virtual void MotionLineRelative() {}		//ĩ���ٶ�ģʽ
	
	virtual void MotionMode1();					//�Զ���1
	virtual void MotionMode2();					//�Զ���2
	virtual void MotionMode3();					//�Զ���3
	virtual void MotionMode4();					//�Զ���4
	virtual void MotionMode5();					//�Զ���5
	virtual void MotionMode6();					//�Զ���6
	virtual void MotionMode7();					//�Զ���7
	virtual void MotionMode8();					//�Զ���8
	virtual void MotionMode9() {}				//�Զ���9

	void SetLinkSelfAction() {}					//Link��������ĩ�˹��ߣ�ˮ��ǹ
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
class CLink_3 : public CLink
{
public:
	CLink_3(int linkindex, int freedom) :CLink(linkindex, freedom)
	{}
	~CLink_3()
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