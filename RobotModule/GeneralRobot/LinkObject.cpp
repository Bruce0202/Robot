#include "LinkObject.h"

extern CErrorQueue g_ErrQue;

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

	if (m_Command.stLinkKinPar.eActMotionMode != eMotionMode7) {
		connectionFlag = 0;
	}
	test = connectionFlag;

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
			if (currPos > XPLUSUPPOSLIMIT || currPos < XPLUSDOWNPOSLIMIT) {
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
	double posL = 0.0;
	double posR = 0.0;
	double u_inc = 0.0;
	double AdjVar = 0.0;
	switch (currAxis)
	{
	case yAxis:
		posL = m_Status.stLinkActKin.LinkPos[1];
		posR = m_Status.stLinkActKin.LinkPos[2];
		e[0] = posL - posR;
		u_inc = A * e[0] + B * e_pre_1[0] + C * e_pre_2[0];
		e_pre_2[0] = e_pre_1[0];
		e_pre_1[0] = e[0];
		AdjVar = fabs_(u_inc) * delta_t;
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
		posL = m_Status.stLinkActKin.LinkPos[3];
		posR = m_Status.stLinkActKin.LinkPos[4];
		e[1] = posL - posR;
		u_inc = A * e[1] + B * e_pre_1[1] + C * e_pre_2[1];
		e_pre_2[1] = e_pre_1[1];
		e_pre_1[1] = e[1];
		AdjVar = fabs_(u_inc) * delta_t;
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
//		printf("using the SynControl function, but do nothing");
		break;
	}
}

//X Axis,tarMoveVel是控制的目标速度，beSafeMode判断是否处于安全模式
void CLink_0::xAxisMotion(double tarMoveVel, double beSafeMode = 10) {
	//获取控制信息
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	//获取X轴的位置关系
	//double xNowPosition = m_Status.stLinkActKin.LinkPos[0];

	m_Joints[0]->m_Commd.Velocity = tarMoveVel;
	m_Joints[0]->m_Commd.eMC_Motion = eMC_MOV_VEL;
	PosLimit(xAxis, safeMode);
	return;
}

//Y Axis，tarMoveVel是控制的目标速度，beSafeMode判断是否处于安全模式
void CLink_0::yAxisMotion(double tarMoveVel, double beSafeMode = 10) {
	//获取控制信息
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	//获取Y轴的位置关系
	//double yNowPosition = m_Status.stLinkActKin.LinkPos[1];

	SynControl(yAxis, tarMoveVel);
	PosLimit(yAxis, safeMode);
	return;
}

//Z Asix，tarMoveVel是控制的目标速度，beSafeMode判断是否处于安全模式
void CLink_0::zAxisMotion(double tarMoveVel, double beSafeMode = 10) {
	//获取控制信息
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	//获取Z轴的位置关系
	//double zNowPosition = m_Status.stLinkActKin.LinkPos[3];

	SynControl(zAxis, tarMoveVel);
	PosLimit(zAxis, safeMode);
	//for (int i = 0; i < m_Freedom; i++) {
	//	m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	//}
	return;
}

//X plus Axis，tarMoveVel是控制的目标速度，beSafeMode判断是否处于安全模式
void CLink_0::xPlusAxisMotion(double tarMoveVel, double beSafeMode = 10) {
	//获取控制信息
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	//获取x轴的位置关系
	//double xPlusNowPosition = m_Status.stLinkActKin.LinkPos[6];

	m_Joints[6]->m_Commd.Velocity = tarMoveVel;
	m_Joints[6]->m_Commd.eMC_Motion = eMC_MOV_VEL;
	PosLimit(xAxisPlus, safeMode);
	return;
}

/*********************************************************************************************************
Inverse kinematics
*********************************************************************************************************/
//-----------------------------------------------------------------------------//
//位置模式，主要是控制三个轴的运动，分别是大X轴，y轴和Z轴.
//-----------------------------------------------------------------------------//
void CLink_0::MotionMode6() {
	//读取控制量
	double targetPosXPlus = m_Command.stLinkKinPar.LinkPos[0];
	double targetPosY = m_Command.stLinkKinPar.LinkPos[1];
	double targetPosZ = m_Command.stLinkKinPar.LinkPos[2];

	//得到当前的位置，然后得到当前与目标位置的差距
	//这里就是使用的X大轴、Y以及Z轴
	double nowXPlus = m_Status.stLinkActKin.LinkPos[6];
	double mowY = (m_Status.stLinkActKin.LinkPos[1] + m_Status.stLinkActKin.LinkPos[2]) / 2;
	double mowZ = (m_Status.stLinkActKin.LinkPos[3] + m_Status.stLinkActKin.LinkPos[4]) / 2;
	double Dis_X = targetPosXPlus - nowXPlus;
	double Dis_Y = targetPosY - mowY;
	double Dis_Z = targetPosZ - mowZ;

	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->m_Commd.eMC_Motion = eMC_NONE;
		m_Joints[i]->m_Commd.Velocity = 0;
		m_Joints[i]->m_Commd.Position = 0;
	}

	if ((fabs_(Dis_X) < 0.1) && (fabs_(Dis_Y) < 0.1) && (fabs_(Dis_Z) < 0.1)) {
		for (int i = 0; i < m_Freedom; i++) {
			m_Joints[i]->m_Commd.eMC_Motion = eMC_HALT;
			m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
		}
		return;
	}

	if (fabs_(Dis_X) > 0.1) {
		if (Dis_X > 0)
			xPlusAxisMotion(xPlusMoveVel);
		else 
			xPlusAxisMotion(-xPlusMoveVel);
	}
	else
		m_Joints[6]->m_Commd.eMC_Motion = eMC_HALT;

	if (fabs_(Dis_Y) > 0.1) {
		if (Dis_Y > 0)
			yAxisMotion(yMoveVel);
		else
			yAxisMotion(-yMoveVel);
	}
	else {
		m_Joints[1]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[2]->m_Commd.eMC_Motion = eMC_HALT;
	}

	if (fabs_(Dis_Z) > 0.1) {
		if (Dis_Z > 0)
			zAxisMotion(zMoveVel);
		else 
			zAxisMotion(-zMoveVel);
	}
	else {
		m_Joints[3]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
	}

	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);

	return;
}

//------------------------------------------------------------------------------//
//这是一个控制拧螺钉头接入螺钉的程序，主要是控制Z轴
//------------------------------------------------------------------------------//
void CLink_0::MotionMode7() {
	//读入控制指令
	double tarPosZ = m_Command.stLinkKinPar.LinkPos[0];
	if (connectionFlag == 0) {
		NljObjPosi = m_Command.stLinkKinPar.LinkPos[1] + 2.5;
		connectionFlag++;
	}

	//获取当前的状态
	double nowZ = (m_Status.stLinkActKin.LinkPos[3] + m_Status.stLinkActKin.LinkPos[4]) / 2;
	double NljPosi = m_Status.stLinkActKin.LinkPos[5];
	double currentNlj = m_Status.stLinkActKin.LinkVel[1];

	//位置的偏差
	double deviDis = tarPosZ - nowZ;

	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->m_Commd.eMC_Motion = eMC_NONE;
		m_Joints[i]->m_Commd.Velocity = 0;
		m_Joints[i]->m_Commd.Position = 0;
	}



	//接入操作
	if (fabs_(deviDis) > 0.1) {
		if (deviDis > 0) {
			zAxisMotion(0.5);
		}
		else {
			zAxisMotion(-0.5);
		}
	}
	else {
		m_Joints[3]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
		if (fabs_(NljObjPosi - NljPosi) < 0.1) {
			m_Joints[5]->m_Commd.eMC_Motion = eMC_HALT;
		}
		else {
			m_Joints[5]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[5]->m_Commd.Velocity = nljMoveVel;
		}
	}
	test1 = NljObjPosi;
	test2 = NljPosi;
	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}

//------------------------------------------------------------------------------//
//这是拧紧电机的控制程序
//------------------------------------------------------------------------------//
void CLink_0::MotionMode8() {
	//读当前的状态
	double NLJVel = m_Status.stLinkActKin.LinkVel[0];
	double currentNlj = m_Status.stLinkActKin.LinkVel[1];

	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->m_Commd.eMC_Motion = eMC_NONE;
		m_Joints[i]->m_Commd.Velocity = 0;
		m_Joints[i]->m_Commd.Position = 0;
	}

	if (currentNlj > TARCURR) {
		m_Joints[5]->m_Commd.eMC_Motion = eMC_HALT;
	}
	else {
		m_Joints[5]->m_Commd.eMC_Motion = eMC_MOV_VEL;
		m_Joints[5]->m_Commd.Velocity = nljMoveVel;
	}

	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}

//------------------------------------------------------------------------------//
//手动模式
//------------------------------------------------------------------------------//
//X轴30mm
void CLink_0::MotionMode1() {
	double tarMoveVel = m_Command.stLinkKinPar.LinkVel[0];
	int beSafeMode = int(m_Command.stLinkKinPar.LinkPos[0]);
	xAxisMotion(tarMoveVel, beSafeMode);
	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}

//Y轴140mm
void CLink_0::MotionMode2() {
	double tarMoveVel = m_Command.stLinkKinPar.LinkVel[0];
	int beSafeMode = int(m_Command.stLinkKinPar.LinkPos[0]);
	yAxisMotion(tarMoveVel, beSafeMode);
	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}

//Z轴30mm
void CLink_0::MotionMode3() {
	double tarMoveVel = m_Command.stLinkKinPar.LinkVel[0];
	int beSafeMode = int(m_Command.stLinkKinPar.LinkPos[0]);
	zAxisMotion(tarMoveVel, beSafeMode);
	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}

//X轴200mm
void CLink_0::MotionMode4() {
	double tarMoveVel = m_Command.stLinkKinPar.LinkVel[0];
	int beSafeMode = int(m_Command.stLinkKinPar.LinkPos[0]);
	xPlusAxisMotion(tarMoveVel, beSafeMode);
	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}

//拧螺钉电机
void CLink_0::MotionMode5() {
	m_Joints[5]->m_Commd.Velocity = m_Command.stLinkKinPar.LinkVel[0] > 0 ? nljMoveVel : -nljMoveVel;
	m_Joints[5]->m_Commd.eMC_Motion = eMC_MOV_VEL;
	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}




