#include "LinkObject.h"

extern CErrorQueue g_ErrQue;

/*********************************************************************************************************
Positive kinematics
这个机器人一共有7个电机，一个拧紧电机，两个车轮，两个Z轴，一个Y轴，一个X轴
我们定义轴的编号为0、1代表车轮的两个电机，2代表X轴，3代表Y轴，4、5代表Z轴，6代表拧紧电机
其中有手动模式和自动模式
手动模式：
1.每个轴的运动
自动模式：
1.运动到绝对位置
2.对接
3.拧紧
*********************************************************************************************************/
void CLink_0::LinkForwardKin() {
	//position
	double wheelLPosi = m_Joints[0]->m_Status.Position;
	double wheelRPosi = m_Joints[1]->m_Status.Position;
	double xPosi = m_Joints[2]->m_Status.Position;
	double yPosi = m_Joints[3]->m_Status.Position;
	double zLPosi = m_Joints[4]->m_Status.Position;
	double zRPosi= m_Joints[5]->m_Status.Position;
	double NLJPosi = m_Joints[6]->m_Status.Position;

	//velocity
	double wheelLVel = m_Joints[0]->m_Status.Velocity;
	double wheelRVel = m_Joints[1]->m_Status.Velocity;
	double xVel = m_Joints[2]->m_Status.Velocity;
	double yVel = m_Joints[3]->m_Status.Velocity;
	double zLVel = m_Joints[4]->m_Status.Velocity;
	double zRVel = m_Joints[5]->m_Status.Velocity;
	double NLJVel = m_Joints[6]->m_Status.Velocity;

	//current
	double NLJCurrent = m_Joints[6]->m_Status.Current;

	//Positive kinematics,pareper the data for algorithm
	m_Status.stLinkActKin.LinkPos[0] = wheelLPosi;
	m_Status.stLinkActKin.LinkPos[1] = wheelRPosi;
	m_Status.stLinkActKin.LinkPos[2] = xPosi;
	m_Status.stLinkActKin.LinkPos[3] = yPosi;
	m_Status.stLinkActKin.LinkPos[4] = zLPosi;
	m_Status.stLinkActKin.LinkPos[5] = zRPosi;
	m_Status.stLinkActKin.LinkPos[6] = NLJPosi;
	m_Status.stLinkActKin.LinkVel[0] = NLJVel;
	m_Status.stLinkActKin.LinkVel[1] = NLJCurrent;
}

//-----------------------------------------------------------------//
//safeMode for X, Y, Z Axis, 0 is X, 1 is Y, 2 is Z
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
//wheelAxis is 3, zAxis is 2;
//SynControl(1, 2)
//--------------------------------------------------------------------//
void CLink_0::SynControl(objAsix currAxis, double TarMoveVel) {
	double posL = 0.0;
	double posR = 0.0;
	double u_inc = 0.0;
	double AdjVar = 0.0;
	switch (currAxis)
	{
	case wheelAxis:
		posL = m_Status.stLinkActKin.LinkPos[1];
		posR = m_Status.stLinkActKin.LinkPos[2];
		e[0] = posL - posR;
		u_inc = A * e[0] + B * e_pre_1[0] + C * e_pre_2[0];
		e_pre_2[0] = e_pre_1[0];
		e_pre_1[0] = e[0];
		AdjVar = fabs_(u_inc) * delta_t;
		AdjVar = min(0.5, AdjVar);
		if (u_inc < 0) {
			int flag1 = 0; int flag2 = 1;
			if (TarMoveVel < 0){
				flag1 = 1; flag2 = 0; AdjVar = -AdjVar;
			}
			m_Joints[flag1]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag1]->m_Commd.Velocity = TarMoveVel;
			m_Joints[flag2]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag2]->m_Commd.Velocity = TarMoveVel - AdjVar;
		}
		else {
			int flag1 = 0; int flag2 = 1;
			if (TarMoveVel < 0) {
				flag1 = 1; flag2 = 0; AdjVar = -AdjVar;
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
			int flag1 = 4; int flag2 = 5;
			if (TarMoveVel < 0) {
				flag1 = 5; flag2 = 4; AdjVar = -AdjVar;
			}
			m_Joints[flag1]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag1]->m_Commd.Velocity = TarMoveVel;
			m_Joints[flag2]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[flag2]->m_Commd.Velocity = TarMoveVel - AdjVar;
		}
		else {
			int flag1 = 4; int flag2 = 5;
			if (TarMoveVel < 0) {
				flag1 = 5; flag2 = 4; AdjVar = -AdjVar;
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

	SynControl(zAxis, tarMoveVel);
	PosLimit(zAxis, safeMode);
	return;
}

//X plus Axis，tarMoveVel是控制的目标速度，beSafeMode判断是否处于安全模式
void CLink_0::xWheelMotion(double tarMoveVel, double beSafeMode = 10) {
	//获取控制信息
	bool safeMode = true;
	if (beSafeMode < 0.0) {
		safeMode = false;
	}
	SynControl(wheelAxis, tarMoveVel);
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
	xWheelMotion(tarMoveVel, beSafeMode);
	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}

//拧螺钉电机
void CLink_0::MotionMode5() {
	m_Joints[6]->m_Commd.Velocity = m_Command.stLinkKinPar.LinkVel[0] > 0 ? nljMoveVel : -nljMoveVel;
	m_Joints[6]->m_Commd.eMC_Motion = eMC_MOV_VEL;
	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}

/*********************************************************************************************************
Inverse kinematics
*********************************************************************************************************/
//-----------------------------------------------------------------------------//
//位置模式，主要是控制三个轴的运动，分别是X轴，y轴和Z轴.
//-----------------------------------------------------------------------------//
void CLink_0::MotionMode6() {
	//读取控制量
	double targetPosX = m_Command.stLinkKinPar.LinkPos[0];
	double targetPosY = m_Command.stLinkKinPar.LinkPos[1];
	double targetPosZ = m_Command.stLinkKinPar.LinkPos[2];

	//得到当前的位置，然后得到当前与目标位置的差距
	//这里就是使用的X大轴、Y以及Z轴
	double nowX = m_Status.stLinkActKin.LinkPos[2];
	double mowY = m_Status.stLinkActKin.LinkPos[3];
	double mowZ = (m_Status.stLinkActKin.LinkPos[4] + m_Status.stLinkActKin.LinkPos[5]) / 2;
	double Dis_X = targetPosX - nowX;
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
			xAxisMotion(xPlusMoveVel);
		else 
			xAxisMotion(-xPlusMoveVel);
	}
	else
		m_Joints[2]->m_Commd.eMC_Motion = eMC_HALT;

	if (fabs_(Dis_Y) > 0.1) {
		if (Dis_Y > 0)
			yAxisMotion(yMoveVel);
		else
			yAxisMotion(-yMoveVel);
	}
	else {
		m_Joints[3]->m_Commd.eMC_Motion = eMC_HALT;
	}

	if (fabs_(Dis_Z) > 0.1) {
		if (Dis_Z > 0)
			zAxisMotion(zMoveVel);
		else 
			zAxisMotion(-zMoveVel);
	}
	else {
		m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[5]->m_Commd.eMC_Motion = eMC_HALT;
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
	double tarVelNlj = m_Command.stLinkKinPar.LinkVel[0];

	//获取当前的状态
	double nowZ = (m_Status.stLinkActKin.LinkPos[4] + m_Status.stLinkActKin.LinkPos[5]) / 2;
	double NLJVel = m_Status.stLinkActKin.LinkVel[0];
	double currentNlj = m_Status.stLinkActKin.LinkVel[1];

	//位置的偏差
	double deviDis = tarPosZ - nowZ;

	for (int i = 0; i < m_Freedom; i++) {
		m_Joints[i]->m_Commd.eMC_Motion = eMC_NONE;
		m_Joints[i]->m_Commd.Velocity = 0;
		m_Joints[i]->m_Commd.Position = 0;
	}

	//接入操作
	if ((fabs_(deviDis) <= 0.2) && (currentNlj < TARCURR)) {
		m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[5]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[6]->m_Commd.eMC_Motion = eMC_HALT;
		for (int i = 0; i < m_Freedom; i++)
			m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
		return;
	}
	else if (currentNlj > TARCURR) {
		m_Joints[4]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[5]->m_Commd.eMC_Motion = eMC_HALT;
		m_Joints[6]->m_Commd.eMC_Motion = eMC_HALT;
		for (int i = 0; i < m_Freedom; i++)
			m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
		return;
	}
	else if ((fabs_(deviDis) > 0.2) && (currentNlj < TARCURR)) {
		if (deviDis > 0) {
			zAxisMotion(NLJVel);
			m_Joints[6]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[6]->m_Commd.Velocity = nljMoveVel;
		}
		else {
			//e[1] = m_Status.stLinkActKin.LinkPos[3] - m_Status.stLinkActKin.LinkPos[4];
			//double u_inc = A * e[1] + B * e_pre_1[1] + C * e_pre_2[1];
			//e_pre_2[1] = e_pre_1[1];
			//e_pre_1[1] = e[1];
			//if (u_inc < 0) {
			//	m_Joints[3]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			//	m_Joints[3]->m_Commd.Velocity = -MoveVel + fabs_(u_inc) * delta_t;
			//	m_Joints[4]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			//	m_Joints[4]->m_Commd.Velocity = -MoveVel;
			//}
			//else {
			//	m_Joints[3]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			//	m_Joints[3]->m_Commd.Velocity = -MoveVel;
			//	m_Joints[4]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			//	m_Joints[4]->m_Commd.Velocity = -MoveVel + fabs_(u_inc) * delta_t;
			//}
			zAxisMotion(0);
			m_Joints[6]->m_Commd.eMC_Motion = eMC_MOV_VEL;
			m_Joints[6]->m_Commd.Velocity = nljMoveVel;
		}
	}
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
		m_Joints[6]->m_Commd.eMC_Motion = eMC_HALT;
	}
	else {
		m_Joints[6]->m_Commd.eMC_Motion = eMC_MOV_VEL;
		m_Joints[6]->m_Commd.Velocity = nljMoveVel;
	}

	for (int i = 0; i < m_Freedom; i++)
		m_Joints[i]->CommdMove(m_Joints[i]->m_Commd);
	return;
}






