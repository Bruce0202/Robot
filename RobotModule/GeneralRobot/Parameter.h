#pragma once
#pragma pack(push)
#pragma pack(8) 
#include <stdio.h>//修改
#include <string.h>

const LONG MAX_FREEDOM_LINK = 10;			//Link 最大自由度
const LONG MAX_FREEDOM_ROBOT = 20;			//robot最大自由度
const LONG LOST_COMM_THRESHOLD = 500;		//通讯超时周期数
const double MIN_VEL_LIMT = 0.001;			//单位 °/s
const double MINIUM = exp_(-10);
const double MAX = 1.7*exp_(308);
const double MIN = 1.7*exp_(308);
//------------------------------------------------------------------------------//
//LINK自由度配置
//------------------------------------------------------------------------------//
const LONG LINK_FREEDOM[6] = { 6, 0, 0, 0, 0, 0 };

//------------------------------------------------------------------------------//
//轴限位
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { MAX, MAX, MAX, MAX, MAX, MAX, 0, 0,0,0 };
const double LINK_0_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -MIN,-MIN,-MIN,-MIN,-MIN,-MIN,0,0,0,0 };


//const double LINK_0_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
//const double LINK_0_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_1_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_1_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_2_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_2_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_3_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_3_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_4_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_4_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_5_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_5_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };

//------------------------------------------------------------------------------//
//最大速度
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 10,10,10,10,10,10,0,0,0,0 };
//const double LINK_0_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_1_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_2_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_3_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_4_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_5_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };

//------------------------------------------------------------------------------//
//减速比
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_RATIO[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,0,0,0,0 };
//const double LINK_0_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_1_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_2_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_3_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_4_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_5_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };

//------------------------------------------------------------------------------//
//电机方向
//------------------------------------------------------------------------------//
const LONG LINK_0_JOINT_DIRECTION[MAX_FREEDOM_LINK] = {1,1,1,1,1,1,0,0,0,0 };
//const LONG LINK_0_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const LONG LINK_1_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const LONG LINK_2_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const LONG LINK_3_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const LONG LINK_4_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const LONG LINK_5_JOINT_DIRECTION[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };

//------------------------------------------------------------------------------//
//编码器偏差
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_1_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_2_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_3_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_4_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_5_JOINT_ENCODER_CORR[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };

#pragma pack(pop)