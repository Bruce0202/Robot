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
const LONG LINK_FREEDOM[6] = { 8, 0, 0, 0, 0, 0 };

//------------------------------------------------------------------------------//
//轴限位
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_LIMIT_POS[MAX_FREEDOM_LINK] = { MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX,0,0 };
const double LINK_0_JOINT_LIMIT_NEG[MAX_FREEDOM_LINK] = { -MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,-MIN,0,0 };


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
const double LINK_0_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 10,10,10,10,10,10,10,10,0,0 };
//const double LINK_0_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_1_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_2_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_3_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_4_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_5_JOINT_MAX_VEL[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };

//------------------------------------------------------------------------------//
//减速比
//------------------------------------------------------------------------------//
const double LINK_0_JOINT_RATIO[MAX_FREEDOM_LINK] = { 1,1,1,1,1,1,1,1,0,0 };
//const double LINK_0_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_1_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_2_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_3_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_4_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };
const double LINK_5_JOINT_RATIO[MAX_FREEDOM_LINK] = { 0,0,0,0,0,0,0,0,0,0 };

//------------------------------------------------------------------------------//
//电机方向
//------------------------------------------------------------------------------//
const LONG LINK_0_JOINT_DIRECTION[MAX_FREEDOM_LINK] = {1,1,1,1,1,1,1,1,0,0 };
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

//地面二维码列表
const LONG PGV_CODE[83][3] = { {-1,-1,0},
 {0,0,0},{1,0,0},{2,0,11},{3,0,10},{4,0,20},{5,0,9},{6,0,15},{7,0,9},{8,0,5},{9,0,4},{10,0,3}, {11,0,7},{12,0,4},{13,0,11},{14,0,17},{15,0,11},{16,0,15},{17,0,5},{18,0,1},{19,0,4},{20,0,11}, {21,0,8},{22,0,6},{23,0,14},{24,0,-10},{25,0,20},{26,0,10},{27,0,0},{28,0,0},
{29,1},{29,2},{29,3},{29,4},{29,5},{29,6},{29,7},{29,8},{29,9}, {29,10},{29,11},{29,12},
 {28,12},{27,12},{26,12},{25,12},{24,12},{23,12},{22,12},{21,12},{20,12},{19,12},{18,12},{17,12},{16,12},{15,12},{14,12},{13,12},{12,12},{11,12},{10,12},{9,12},{8,12},{7,12},{6,12},{5,12},{4,12},{3,12},{2,12},{1,12},{0,12},
 {0,11},{0,10},{0,9},{0,8},{0,7},{0,6},{0,5},{0,4},{0,3},{0,2},{0,1}
};

#pragma pack(pop)