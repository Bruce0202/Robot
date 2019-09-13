#include "LinkObject.h"

extern CErrorQueue g_ErrQue;

const double MoveVel = 1.0;
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

/*********************************************************************************************************
Positive kinematics
*********************************************************************************************************/
void CLink_0::LinkForwardKin() {
	//position
	double xNow = m_Joints[0]->m_Status.Position;
	double yNow_l = m_Joints[1]->m_Status.Position;
	double yNow_r = m_Joints[2]->m_Status.Position;
	double zNow_l = m_Joints[3]->m_Status.Position;
	double zNow_r = m_Joints[4]->m_Status.Position;
	double NLJNow = m_Joints[5]->m_Status.Position;
	double xPlusNow = m_Joints[6]->m_Status.Position;

	//velocity
	double xVel = m_Joints[0]->m_Status.Velocity;
	double yVel_l = m_Joints[1]->m_Status.Velocity;
	double yVel_r = m_Joints[2]->m_Status.Velocity;
	double zVel_l = m_Joints[3]->m_Status.Velocity;
	double zVel_r = m_Joints[4]->m_Status.Velocity;
	double NLJVel = m_Joints[5]->m_Status.Velocity;
	double xPlusVel = m_Joints[6]->m_Status.Velocity;

	//current
	double NLJCurrent = m_Joints[5]->m_Status.Current;

	//Positive kinematics,pareper the data for algorithm
	m_Status.stLinkActKin.LinkPos[0] = xNow;
	m_Status.stLinkActKin.LinkPos[1] = yNow_l;
	m_Status.stLinkActKin.LinkPos[2] = yNow_r;
	m_Status.stLinkActKin.LinkPos[3] = zNow_l;
	m_Status.stLinkActKin.LinkPos[4] = zNow_r;
	m_Status.stLinkActKin.LinkPos[5] = NLJNow;
	m_Status.stLinkActKin.LinkPos[6] = xPlusNow;
	m_Status.stLinkActKin.LinkVel[0] = NLJVel;
	m_Status.stLinkActKin.LinkVel[1] = NLJCurrent;
}

//-----------------------------------------------------------------//
//safeMode for X, Y, Z Axis, 1 is X, 2 is Y, 3 is Z
//-------------------------------------------------------------------//
void CLink_0::PosLimit(objAsix currAxis, bool safeMode = true) {
	if (safeMode == true) {
		double currPos = 0.0;
		switch (currAxis) {
		case xAxis:
			if (m_Status.stLinkActKin.LinkPos[0] > XUPPOSLIMIT ||
				m_Status.stLinkActKin.LinkPos[0] < XDOWNPOSLIMIT) {
				m_Joints[0]->m_Commd.eMC_Motion = eMC_HALT;
			}
			break;
		case yAxis:
			currPos = (m_Status.stLinkActKin.LinkPos[1] + m_Status.stLinkActKin.LinkPos[2]) / 2;
			if (currPos > YUPPOSLIMIT || currPos < YDOWNPOSLIMIT) {
				m_Joints[1]->m_Commd.eMC_Motion = eMC_HALT;
				m_Joints[2]->m_Commd.eMC_Motion = eMC_HALT;
			}
			break;
		case zAxis:
			currPos = (m_Status.stLinkActKin.LinkPos[3] + m_Status.stLinkActKin.LinkPos[4]) / 2;
			if (currPos > ZUPPOSLIMIT || currPos < ZDOWNPOSLIMIT) {
				m_Joints[3]->m_Commd.eMC_Motion = eMC_HALT;
				m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
			}
			break;
		case xAxisPlus:
			currPos = m_Status.stLinkActKin.LinkPos[6];
			if (currPos > ZUPPOSLIMIT || currPos < ZDOWNPOSLIMIT) {
				m_Joints[6]->m_Commd.eMC_Motion = eMC_HALT;
			}
			break;
		default:
			for (int i = 0; i < 7; i++) {
				m_Joints[i]->m_Commd.eMC_Motion = eMC_HALT;
			}
			break;
		}
	}
}


//-------------------------------------------------------------------//
//basing PID, control two axis to move
//SynControl(1, 2, 0, MoveVel, Now_YL, Now_YR, 1);
//--------------------------------------------------------------------//
void CLink_0::SynControl(objAsix currAxis, double TarMoveVel) {
	switch (currAxis)
	{
	case yAxis:
		double posL = m_Status.stLinkActKin.LinkPos[1];
		double posR = m_Status.stLinkActKin.LinkPos[2];
		e[0] = posL - posR;
		double u_inc = A * e[0] + B * e_pre_1[0] + C * e_pre_2[0];
		e_pre_2[0] = e_pre_1[0];
		e_pre_1[0] = e[0];
		double AdjVar = fabs_(u_inc) * delta_t;
		AdjVar = min(0.5, AdjVar);
		if (u_inc < 0) {
			int flag1 = 1; int flag2 = 2;
			if (TarMoveVel < 0){
				flag1 = 2; flag2 = 1; AdjVar = -AdjVar;
			}
			m_Joints[flag1]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag1]->m_Commd.Velocity = TarMoveVel;
			m_Joints[flag2]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag2]->m_Commd.Velocity = TarMoveVel - AdjVar;
		}
		else {
			int flag1 = 1; int flag2 = 2;
			if (TarMoveVel < 0) {
				flag1 = 2; flag2 = 1; AdjVar = -AdjVar;
			}
			m_Joints[flag1]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag1]->m_Commd.Velocity = TarMoveVel - AdjVar;
			m_Joints[flag2]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag2]->m_Commd.Velocity = TarMoveVel;
		}
		break;
	case zAxis:
		double posL = m_Status.stLinkActKin.LinkPos[3];
		double posR = m_Status.stLinkActKin.LinkPos[4];
		e[1] = posL - posR;
		double u_inc = A * e[1] + B * e_pre_1[1] + C * e_pre_2[1];
		e_pre_2[1] = e_pre_1[1];
		e_pre_1[1] = e[1];
		double AdjVar = fabs_(u_inc) * delta_t;
		AdjVar = min(0.5, AdjVar);
		if (u_inc < 0) {
			int flag1 = 3; int flag2 = 4;
			if (TarMoveVel < 0) {
				flag1 = 4; flag2 = 3; AdjVar = -AdjVar;
			}
			m_Joints[flag1]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag1]->m_Commd.Velocity = TarMoveVel;
			m_Joints[flag2]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag2]->m_Commd.Velocity = TarMoveVel - AdjVar;
		}
		else {
			int flag1 = 3; int flag2 = 4;
			if (TarMoveVel < 0) {
				flag1 = 4; flag2 = 3; AdjVar = -AdjVar;
			}
			m_Joints[flag1]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag1]->m_Commd.Velocity = TarMoveVel - AdjVar;
			m_Joints[flag2]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag2]->m_Commd.Velocity = TarMoveVel;
		}
		break;
	default:
		printf("using the SynControl function, but do nothing");
		break;
	}
}

/*********************************************************************************************************
Inverse kinematics
*********************************************************************************************************/
//X Axis
void CLink_0::xAxisMotion() {
	//获取控制信息
	int beSafeMode = m_Command.stLinkKinPar.LinkPos[0];
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	//获取速度参数值
	double tarMoveVel = m_Command.stLinkKinPar.LinkVel[0];
	//获取X轴的位置关系
	double xNowPosition = m_Status.stLinkActKin.LinkPos[0];

	m_Joints[0]->m_Commd.Velocity = tarMoveVel;
	m_Joints[0]->m_Commd.eMC_Motion = eMC_MOV_VEL;
	PosLimit(xAxis, safeMode);

	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	}
	return;
}

//Y Axis
void CLink_0::yAxisMotion() {
	//获取控制信息
	int beSafeMode = m_Command.stLinkKinPar.LinkPos[0];
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	//获取速度参数值
	double tarMoveVel = m_Command.stLinkKinPar.LinkVel[0];
	//获取Y轴的位置关系
	double yNowPosition = m_Status.stLinkActKin.LinkPos[1];

	SynControl(yAxis, tarMoveVel);
	PosLimit(yAxis, safeMode);
	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	}
	return;
}

//Z Asix
void CLink_0::zAxisMotion() {
	//获取控制信息
	int beSafeMode = m_Command.stLinkKinPar.LinkPos[0];
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	//获取速度参数值
	double tarMoveVel = m_Command.stLinkKinPar.LinkVel[0];
	//获取Z轴的位置关系
	double yNowPosition = m_Status.stLinkActKin.LinkPos[3];
	SynControl(zAxis, tarMoveVel);
	PosLimit(zAxis, safeMode);
	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	}
	return;
}

//X plus Axis
void CLink_0::xPlusAxisMotion() {
	//获取控制信息
	int beSafeMode = m_Command.stLinkKinPar.LinkPos[0];
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	//获取速度参数值
	double tarMoveVel = m_Command.stLinkKinPar.LinkVel[0];
	//获取x轴的位置关系
	double xPlusNowPosition = m_Status.stLinkActKin.LinkPos[6];

	m_Joints[6]->m_Commd.Velocity = tarMoveVel;
	m_Joints[6]->m_Commd.eMC_Motion = eMC_MOV_VEL;
	PosLimit(xAxisPlus, safeMode);

	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	}
	return;
}



void CLink_0::MotionMode1() {
	double targetPos_X = m_Command.stLinkKinPar.LinkPos[0];
	double targetPos_Y = m_Command.stLinkKinPar.LinkPos[1];
	double targetPos_Z = m_Command.stLinkKinPar.LinkPos[2];

	double Now_X = m_Status.stLinkActKin.LinkPos[0];
	double Now_YL = m_Status.stLinkActKin.LinkPos[1];
	double Now_YR = m_Status.stLinkActKin.LinkPos[2];
	double Now_ZL = m_Status.stLinkActKin.LinkPos[3];
	double Now_ZR = m_Status.stLinkActKin.LinkPos[4];

	double Now_Y = (Now_YL + Now_YR) / 2;
	double Now_Z = (Now_ZL + Now_ZR) / 2;

	double Dis_X = targetPos_X - Now_X;
	double Dis_Y = targetPos_Y - Now_Y;
	double Dis_Z = targetPos_Z - Now_Z;

	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->m_Commd.eMC_Motion = eMC_NONE;
		m_Joints[i]->m_Commd.Velocity = 0;
		m_Joints[i]->m_Commd.Position = 0;
	}

	if ((fabs_(Dis_X) < 0.5) && (fabs_(Dis_Y) < 0.5) && (fabs_(Dis_Z) < 0.5)) {
		for (int i = 0; i < m_Freedom; i++) {
			m_Joints[i]->m_Commd.eMC_Motion = eMC_HALT;
			m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
		}
		return;
	}

	if (fabs_(Dis_X) > 0.5) {
		if (Dis_X > 0) {
			m_Joints[0]->m_Commd.Velocity = MoveVel;
			m_Joints[0]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			PosLimit(1);
		}
		else {
			m_Joints[0]->m_Commd.Velocity = -MoveVel;
			m_Joints[0]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			PosLimit(1);
		}
	}
	else {
		m_Joints[0]->m_Commd.eMC_Motion = eMC_HALT;
	}

	if (fabs_(Dis_Y) > 0.5) {
		if (Dis_Y > 0) {
			SynControl(1, 2, 0, MoveVel, Now_YL, Now_YR, 1);
			PosLimit(2);
		}
		else {
			SynControl(1, 2, 0, MoveVel, Now_YL, Now_YR, -1);
			PosLimit(2);
		}
	}
	else {
		m_Joints[1]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[2]->m_Commd.eMC_Motion = eMC_HALT;
	}

	if (fabs_(Dis_Z) > 0.5) {
		if (Dis_Z > 0) {
			SynControl(3, 4, 1, 0.5, Now_ZL, Now_ZR, 1);
			PosLimit(3);
		}
		else {
			SynControl(3, 4, 1, 0.5, Now_ZL, Now_ZR, -1);
			PosLimit(3);
		}
	}
	else {
		m_Joints[3]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
	}
	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	}
	return;
}

//------------------------------------------------------------------------------//
//���ﶨ�����š�ݶ�ͷ�����ݶ��ԽӵĿ��ƹ���
//------------------------------------------------------------------------------//
void CLink_0::MotionMode2() {
	//��ȡZ���λ�ú�š�ݶ�����ĵ���
	double zNow_l = m_Status.stLinkActKin.LinkPos[3];
	double zNow_r = m_Status.stLinkActKin.LinkPos[4];
	double Now_z = (zNow_l + zNow_r) / 2;
	double NLJVel = m_Status.stLinkActKin.LinkVel[0];
	double Current_nlj = m_Status.stLinkActKin.LinkVel[1];

	//��ȡĿ��λ��
	double tarPos_z = m_Command.stLinkKinPar.LinkPos[0];
	double tarVel_nlj = m_Command.stLinkKinPar.LinkVel[0];

	//���㵱ǰλ�õĲ�ֵ
	double deviDis = tarPos_z - Now_z;

	//��ʼ��ָ��
	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->m_Commd.eMC_Motion = eMC_NONE;
		m_Joints[i]->m_Commd.Velocity = 0;
		m_Joints[i]->m_Commd.Position = 0;
	}

	//�ж��Ƿ��Ѿ��˶�����Ŀ��λֵ���������ӵ��������Ϊ��ȫ������
	if ((fabs_(deviDis) <= 0.5) && (Current_nlj < TARCURR)) {
		m_Joints[3]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[5]->m_Commd.eMC_Motion = eMC_HALT;
		for (int i = 0; i < m_Freedom; i++) {
			m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
		}
		return;
	}
	else if (Current_nlj > TARCURR) {
		m_Joints[3]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[5]->m_Commd.eMC_Motion = eMC_HALT;
		for (int i = 0; i < m_Freedom; i++) {
			m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
		}
		return;
	}
	else if ((fabs_(deviDis) > 0.5) && (Current_nlj < TARCURR)) {
		if (deviDis > 0) {
			e[1] = m_Status.stLinkActKin.LinkPos[3] - m_Status.stLinkActKin.LinkPos[4];
			double u_inc = A * e[1] + B * e_pre_1[1] + C * e_pre_2[1];
			e_pre_2[1] = e_pre_1[1];
			e_pre_1[1] = e[1];
			if (u_inc < 0) {
				m_Joints[3]->m_Commd.eMC_Motion = eMC_MOV_VEL;
				m_Joints[3]->m_Commd.Velocity = MoveVel + fabs_(u_inc) * delta_t;
				m_Joints[4]->m_Commd.eMC_Motion = eMC_MOV_VEL;
				m_Joints[4]->m_Commd.Velocity = MoveVel;
			}
			else {
				m_Joints[3]->m_Commd.eMC_Motion = eMC_MOV_VEL;
				m_Joints[3]->m_Commd.Velocity = MoveVel;
				m_Joints[4]->m_Commd.eMC_Motion = eMC_MOV_VEL;
				m_Joints[4]->m_Commd.Velocity = MoveVel + fabs_(u_inc) * delta_t;
			}
			m_Joints[5]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[5]->m_Commd.Velocity = nljMoveVel;
		}
		else {
			e[1] = m_Status.stLinkActKin.LinkPos[3] - m_Status.stLinkActKin.LinkPos[4];
			double u_inc = A * e[1] + B * e_pre_1[1] + C * e_pre_2[1];
			e_pre_2[1] = e_pre_1[1];
			e_pre_1[1] = e[1];
			if (u_inc < 0) {
				m_Joints[3]->m_Commd.eMC_Motion = eMC_MOV_VEL;
				m_Joints[3]->m_Commd.Velocity = -MoveVel + fabs_(u_inc) * delta_t;
				m_Joints[4]->m_Commd.eMC_Motion = eMC_MOV_VEL;
				m_Joints[4]->m_Commd.Velocity = -MoveVel;
			}
			else {
				m_Joints[3]->m_Commd.eMC_Motion = eMC_MOV_VEL;
				m_Joints[3]->m_Commd.Velocity = -MoveVel;
				m_Joints[4]->m_Commd.eMC_Motion = eMC_MOV_VEL;
				m_Joints[4]->m_Commd.Velocity = -MoveVel + fabs_(u_inc) * delta_t;
			}
			m_Joints[5]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[5]->m_Commd.Velocity = -nljMoveVel;
		}
	}
	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	}
	return;

}

//------------------------------------------------------------------------------//
//š��������˶�ģʽ
//------------------------------------------------------------------------------//
void CLink_0::MotionMode3() {
	//��ȡ���������Ϣ
	double NLJVel = m_Status.stLinkActKin.LinkVel[0];
	double Current_nlj = m_Status.stLinkActKin.LinkVel[1];

	//��ʼ��ָ��
	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->m_Commd.eMC_Motion = eMC_NONE;
		m_Joints[i]->m_Commd.Velocity = 0;
		m_Joints[i]->m_Commd.Position = 0;
	}

	//ƫ��С��һ����ֵ��Ϊ������Ŀ��λ��   λ����Ϣ����Ҫ����
	if (Current_nlj > TARCURR) {
		m_Joints[5]->m_Commd.eMC_Motion = eMC_HALT;
		for (int i = 0; i < m_Freedom; i++) {
			m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
		}
		return;
	}
	else {
		m_Joints[5]->m_Commd.eMC_Motion = eMC_MOV_VEL;
		m_Joints[5]->m_Commd.Velocity = nljMoveVel;
		for (int i = 0; i < m_Freedom; i++) {
			m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
		}
		return;
	}

	//�·�ָ��
	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	}
	return;
}

//------------------------------------------------------------------------------//
//�ֶ�ģʽ
//------------------------------------------------------------------------------//
void CLink_0::MotionMode4() {
	//��ȡ��Ӧ����Ϣ��������һ���ᣬ�Ǹ�����
	int isXYZ = m_Command.stLinkKinPar.LinkPos[0]; // 1 ΪX�ᣬ 2 ΪY�ᣬ 3 ΪZ��, 4 Ϊš��ģʽ
	int isPOSorNEG = m_Command.stLinkKinPar.LinkPos[1]; //1 Ϊ���� 2Ϊ��
	int beNormMode = m_Command.stLinkKinPar.LinkPos[2]; //��Ϊtrue����Ϊfalse
	bool be_NormMode = true;
	if (beNormMode < -1.0) {
		be_NormMode = false;
	}
	double TarMoveVel = m_Command.stLinkKinPar.LinkVel[0];

	//��ȡ��ǰ�����λ��
	double xNow = m_Status.stLinkActKin.LinkPos[0];
	double yNow_l = m_Status.stLinkActKin.LinkPos[1];
	double yNow_r = m_Status.stLinkActKin.LinkPos[2];
	double zNow_l = m_Status.stLinkActKin.LinkPos[3];
	double zNow_r = m_Status.stLinkActKin.LinkPos[4];
	//switch case�������ĸ��ᣬ������д��
	switch (isXYZ) {
	case 1:
		if (isPOSorNEG == 1) {
			m_Joints[0]->m_Commd.Velocity = TarMoveVel;
			m_Joints[0]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			PosLimit(1, be_NormMode);
		}
		else if (isPOSorNEG == 2) {
			m_Joints[0]->m_Commd.Velocity = -TarMoveVel;
			m_Joints[0]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			PosLimit(1, be_NormMode);
		}
		break;
	case 2:
		if (isPOSorNEG == 1) {
			SynControl(1, 2, 0, TarMoveVel, m_Status.stLinkActKin.LinkPos[1], m_Status.stLinkActKin.LinkPos[2]);
			PosLimit(2, be_NormMode);
		}
		else if (isPOSorNEG == 2) {
			SynControl(1, 2, 0, TarMoveVel, m_Status.stLinkActKin.LinkPos[1], m_Status.stLinkActKin.LinkPos[2], -1);
			PosLimit(2, be_NormMode);
		}
		break;
	case 3:
		if (isPOSorNEG == 1) {
			SynControl(3, 4, 1, TarMoveVel, m_Status.stLinkActKin.LinkPos[3], m_Status.stLinkActKin.LinkPos[4]);
			PosLimit(3, be_NormMode);
		}
		else if (isPOSorNEG == 2) {
			SynControl(3, 4, 1, TarMoveVel, m_Status.stLinkActKin.LinkPos[3], m_Status.stLinkActKin.LinkPos[4], -1);
			PosLimit(3, be_NormMode);
		}
		break;
	case 4:
		if (isPOSorNEG == 1) {
			m_Joints[5]->m_Commd.Velocity = nljMoveVel;
			m_Joints[5]->m_Commd.eMC_Motion = eMC_MOV_VEL;
		}
		else if (isPOSorNEG == 2) {
			m_Joints[5]->m_Commd.Velocity = -nljMoveVel;
			m_Joints[5]->m_Commd.eMC_Motion = eMC_MOV_VEL;
		}
		break;
	default:
		break;
	}
	//�·�ָ��
	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	}
	return;

}




