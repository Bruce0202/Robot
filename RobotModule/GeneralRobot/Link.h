/****************************************************************/
/*���ܣ��ƶ�������
/*�����ߣ�AT
/*�ش�Ķ���ʱ�䣺
2017.5.14��  ����
*****************************************************************/
#pragma once
#include <stdlib.h>
#include "Axis.h"
#pragma pack(push)
#pragma pack(8) 

class CLink
{
public:
	CLink(int linkindex, int freedom);
	~CLink();
public:
	ERR_ID Init(int JointIndex, stJointPara JointPar[]);											//�����ʼ��
	void GetStatus(_IN st_AxisGroupRead* stAxisGroupStatus);
	void StateMachine();				 									//״̬�������к�����
	void SetAction(OUT st_AxisGroupSet*	stAxisGroupSet);					//������ָ��
	
	//LINK�����ӿ�----------------------------------------------------------------------//
	void CommdReset();
	void CommdPower(bool enable);
	void CommdHome();
	void CommdStop();
	void CommdHalt();
	void CommdMove();

	double output1;
	double output2;
	double output3;
	double output4;
	double output5;
	double output6;
	double output7;
	double output8;
	////////////
	double output88;
	////////////
protected:
	void JugeLinkState();
	void LinkStateMachine();
	
	//��̬���麯��----------------------------------------------------------------------//
	//����
	virtual void LinkForwardKin() {}			//Link ����
	//���
	virtual void MotionLineAbsolute() {}	//��ֱ���˶�������λ��	
	virtual void MotionLineVelocity() {}	//ĩ���ٶ��˶�
	virtual void MotionLineRelative() {}	//���λ��ģʽ
	virtual void MotionMode1() {}			//�Զ���1
	virtual void MotionMode2() {}			//�Զ���2
	virtual void MotionMode3() {}			//�Զ���3
	virtual void MotionMode4() {}			//�Զ���4
	virtual void MotionMode5() {}			//�Զ���5
	virtual void MotionMode6() {}			//�Զ���6
	virtual void MotionMode7() {}			//�Զ���7
	virtual void MotionMode8() {}			//�Զ���8
	virtual void MotionMode9() {}			//�Զ���9
	
	virtual void SetLinkSelfAction() {}		//Link��������ĩ�˹��ߣ�ˮ��ǹ�߼�����

private:
	virtual void ActionReset();
	virtual void ActionPower();
	virtual void ActionHome();
	virtual void ActionStop();
	virtual void ActionHalt();
	virtual void ActionMove();

public:
	CAxis*			m_Joints[MAX_FREEDOM_LINK];
	int				m_Index;			//��˳���������
	int				m_Freedom;

	stLinkStatus	m_Status;			//Link��ǰ��״̬
	stLinkCommand	m_Command;			//��ҪLINKִ�е�����
	stAction		m_SetAction;		//PLCִ��
	stAction		m_ActAction;		//PLC��ǰ����

	int				test;
private:
	stLinkCommand	m_preCommand;

	
};
#pragma pack(pop)