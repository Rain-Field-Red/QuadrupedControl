// runCoppeliaSimulation.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

//#include "pch.h"

#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#include "quadruped.hpp"

#include "PID_v1.h"


extern "C" {
#include "extApi.h"
}

int bodyHandle;

/*
与coppelia的通信接口
*/

struct JointHandles {
	int joint_HS1;
	int joint_HF1;
	int joint_K1;
	int joint_A1;

	int joint_HS2;
	int joint_HF2;
	int joint_K2;
	int joint_A2;

	int joint_HS3;
	int joint_HF3;
	int joint_K3;
	int joint_A3;

	int joint_HS4;
	int joint_HF4;
	int joint_K4;
	int joint_A4;

	void init() {
		joint_HS1 = 0;
		joint_HF1 = 0;
		joint_K1 = 0;
		joint_A1 = 0;

		joint_HS2 = 0;
		joint_HF2 = 0;
		joint_K2 = 0;
		joint_A2 = 0;

		joint_HS3 = 0;
		joint_HF3 = 0;
		joint_K3 = 0;
		joint_A3 = 0;

		joint_HS4 = 0;
		joint_HF4 = 0;
		joint_K4 = 0;
		joint_A4 = 0;
	}
};

JointHandles getJointHandles(int clientID) {
	
	JointHandles jointHandles;
	jointHandles.init();

	//获取关节句柄
	simxGetObjectHandle(clientID, "joint_hip_side1", &jointHandles.joint_HS1, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_hip_front1", &jointHandles.joint_HF1, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_knee1", &jointHandles.joint_K1, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_ankle1", &jointHandles.joint_A1, simx_opmode_oneshot_wait);

	simxGetObjectHandle(clientID, "joint_hip_side2", &jointHandles.joint_HS2, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_hip_front2", &jointHandles.joint_HF2, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_knee2", &jointHandles.joint_K2, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_ankle2", &jointHandles.joint_A2, simx_opmode_oneshot_wait);

	simxGetObjectHandle(clientID, "joint_hip_side3", &jointHandles.joint_HS3, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_hip_front3", &jointHandles.joint_HF3, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_knee3", &jointHandles.joint_K3, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_ankle3", &jointHandles.joint_A3, simx_opmode_oneshot_wait);

	simxGetObjectHandle(clientID, "joint_hip_side4", &jointHandles.joint_HS4, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_hip_front4", &jointHandles.joint_HF4, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_knee4", &jointHandles.joint_K4, simx_opmode_oneshot_wait);
	simxGetObjectHandle(clientID, "joint_ankle4", &jointHandles.joint_A4, simx_opmode_oneshot_wait);

	return jointHandles;
}

int getJointThetas(int clientID, JointHandles jointHandles, float jointthetas[4][4]) {
	
	simxGetJointPosition(clientID, jointHandles.joint_HS1, (jointthetas[0] + 3), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_HF1, (jointthetas[0] + 2), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_K1, (jointthetas[0] + 1), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_A1, (jointthetas[0] + 0), simx_opmode_oneshot);

	simxGetJointPosition(clientID, jointHandles.joint_HS2, (jointthetas[1] + 3), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_HF2, (jointthetas[1] + 2), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_K2, (jointthetas[1] + 1), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_A2, (jointthetas[1] + 0), simx_opmode_oneshot);

	simxGetJointPosition(clientID, jointHandles.joint_HS3, (jointthetas[2] + 3), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_HF3, (jointthetas[2] + 2), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_K3, (jointthetas[2] + 1), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_A3, (jointthetas[2] + 0), simx_opmode_oneshot);

	simxGetJointPosition(clientID, jointHandles.joint_HS4, (jointthetas[3] + 3), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_HF4, (jointthetas[3] + 2), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_K4, (jointthetas[3] + 1), simx_opmode_oneshot);
	simxGetJointPosition(clientID, jointHandles.joint_A4, (jointthetas[3] + 0), simx_opmode_oneshot);

	return 0;
}

int getJointThetaDots(int clientID, JointHandles jointHandles, float jointThetaDots[4][4]) {
	
	simxGetObjectFloatParameter(clientID, jointHandles.joint_HS1, 2012, (jointThetaDots[0] + 3), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_HF1, 2012, (jointThetaDots[0] + 2), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_K1, 2012, (jointThetaDots[0] + 1), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_A1, 2012, (jointThetaDots[0] + 0), simx_opmode_oneshot);

	simxGetObjectFloatParameter(clientID, jointHandles.joint_HS2, 2012, (jointThetaDots[1] + 3), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_HF2, 2012, (jointThetaDots[1] + 2), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_K2, 2012, (jointThetaDots[1] + 1), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_A2, 2012, (jointThetaDots[1] + 0), simx_opmode_oneshot);

	simxGetObjectFloatParameter(clientID, jointHandles.joint_HS3, 2012, (jointThetaDots[2] + 3), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_HF3, 2012, (jointThetaDots[2] + 2), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_K3, 2012, (jointThetaDots[2] + 1), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_A3, 2012, (jointThetaDots[2] + 0), simx_opmode_oneshot);

	simxGetObjectFloatParameter(clientID, jointHandles.joint_HS4, 2012, (jointThetaDots[3] + 3), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_HF4, 2012, (jointThetaDots[3] + 2), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_K4, 2012, (jointThetaDots[3] + 1), simx_opmode_oneshot);
	simxGetObjectFloatParameter(clientID, jointHandles.joint_A4, 2012, (jointThetaDots[3] + 0), simx_opmode_oneshot);

	return 0;
}

int setJointCmd(int clientID, int jointHandle, float torque) {

	float setVelocity;
	float setTorque;
	if (torque > 0) {
		setVelocity = 9999;
		setTorque = torque;
	}
	else {
		setVelocity = -9999;
		setTorque = -torque;
	}

	simxSetJointTargetVelocity(clientID, jointHandle, setVelocity, simx_opmode_oneshot);
	simxSetJointForce(clientID, jointHandle, setTorque, simx_opmode_oneshot);

	return 0;
}


/*
模拟NED指向的IMU信息
*/

struct BodyInfo {
	float nedPos[3];
	float nedEuler[3];
	float nedLinearVel[3];
	float nedAngularVel[3];

	void init() {
		for (int idx = 0; idx < 3; idx++) {
			nedPos[idx] = 0;
			nedEuler[idx] = 0;
			nedLinearVel[idx] = 0;
			nedAngularVel[idx] = 0;
		}
	}

	int getBodyInfo(float bodyPos[3], float eulerAngle[3], float linearVel[3], float angularVel[3]) {
		nedPos[0] = bodyPos[0];
		nedPos[1] = -bodyPos[1];
		nedPos[2] = -bodyPos[2];

		nedLinearVel[0] = linearVel[0];
		nedLinearVel[1] = -linearVel[1];
		nedLinearVel[2] = -linearVel[2];

		nedEuler[0] = eulerAngle[0];
		nedEuler[1] = -eulerAngle[1];
		nedEuler[2] = -eulerAngle[2];

		nedAngularVel[0] = angularVel[0];
		nedAngularVel[1] = -angularVel[1];
		nedAngularVel[2] = -angularVel[2];

		return 0;
	}
};


using namespace std;
#define PI 3.14

int main()
{
	bool VERBOSE = true;
	int clientID = 0;
	int p_time[2];					//用于单步仿真

	JointHandles jointHandles;
	jointHandles.init();

	float bodyPos[3];
	float eulerAngle[3];
	float linearVel[3];
	float angularVel[3];

	BodyInfo bodyInfo;
	bodyInfo.init();

	float jointThetas[4][4];
	float jointThetaDots[4][4];

	float jointTorques[4][4];

	quadruped robot;

	//int counter = 0;
	/*
	连接仿真器
	*/
	bool WORK = true;
	simxFinish(-1);                                                     //! Close any previously unfinished business
	clientID = simxStart((simxChar*)"127.0.0.1", 19997, true, true, 5000, 5);  //!< Main connection to V-REP
	if (clientID != -1) {
		cout << " Connection status to VREP: SUCCESS" << endl;
	}
	else {
		cout << " Connection status to VREP: FAILED " << endl;
		return -1;
	}
	
	Sleep(10);

	/*
	打开同步模式
	设置仿真步长为0.001s
	*/
	simxSynchronous(clientID, TRUE);
	simxSetFloatingParameter(clientID, sim_floatparam_simulation_time_step, 0.001, simx_opmode_blocking);
	
	/*
	启动仿真过程
	首先读取空的状态数据，以便控制环中读取的数值为真值
	*/
	int start = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
	
	//获取关节句柄
	jointHandles = getJointHandles(clientID);
	simxGetObjectHandle(clientID, "body", &bodyHandle, simx_opmode_oneshot_wait);

	//本体位置、速度和姿态
	simxGetObjectPosition(clientID, bodyHandle, -1, bodyPos, simx_opmode_oneshot);
	//simxGetObjectFloatParameter(clientID, bodyHandle, 14, bodyVel, simx_opmode_oneshot);
	simxGetObjectOrientation(clientID, bodyHandle, -1, eulerAngle, simx_opmode_oneshot);
	simxGetObjectVelocity(clientID, bodyHandle, linearVel, angularVel, simx_opmode_oneshot);

	//得到NED坐标系下的本体信息
	bodyInfo.getBodyInfo(bodyPos, eulerAngle, linearVel, angularVel);

	//关节角度和角速度
	getJointThetas(clientID, jointHandles, jointThetas);
	getJointThetaDots(clientID, jointHandles, jointThetaDots);
	

	// 单步执行一次
	simxSynchronousTrigger(clientID);
	simxGetPingTime(clientID, p_time);


	Sleep(1000);

	/*
	控制环主流程
	*/
	//loop
	for (int idx = 0; idx < 1000; idx++) {

		/*
		获取传感器信息
		*/
		//本体位置、姿态和速度
		simxGetObjectPosition(clientID, bodyHandle, -1, bodyPos, simx_opmode_oneshot);
		simxGetObjectOrientation(clientID, bodyHandle, -1, eulerAngle, simx_opmode_oneshot);
		simxGetObjectVelocity(clientID, bodyHandle, linearVel, angularVel, simx_opmode_oneshot);

		//得到NED坐标系下的本体信息
		bodyInfo.getBodyInfo(bodyPos, eulerAngle, linearVel, angularVel);

		//关节角度
		getJointThetas(clientID, jointHandles, jointThetas);
		getJointThetaDots(clientID, jointHandles, jointThetaDots);

		//cout << jointthetas[0][0] << "|" << jointthetas[0][1] << "|" << jointthetas[0][2] << "|" << jointthetas[0][3] << endl;


		/*
		模型解算
		*/
		robot.setRobotAngle(jointThetas);
		robot.setRobotAngleVelocity(jointThetaDots);

		robot.calBiasedFootPos();

		robot.calBiasedJacobian(1);
		robot.calBiasedJeT(1);

		robot.calBiasedJacobian(2);
		robot.calBiasedJeT(2);

		robot.calBiasedJacobian(3);
		robot.calBiasedJeT(3);

		robot.calBiasedJacobian(4);
		robot.calBiasedJeT(4);

		robot.calBiasedFootVel(1);
		robot.calBiasedFootVel(2);
		robot.calBiasedFootVel(3);
		robot.calBiasedFootVel(4);


		/*
		控制计算
		*/
		//1腿
		float xInput = robot.biasedFootPos[0][0];
		float yInput = robot.biasedFootPos[0][1];
		float zInput = robot.biasedFootPos[0][2];

		float xOutput = 1000 * (0 - xInput) + 100 * (0 - robot.biasedFootVel[0][0]);
		float yOutput = 1000 * (-0.7 - yInput) + 100 * (0 - robot.biasedFootVel[0][1]);
		float zOutput = 1000 * (-0.1 - zInput) + 100 * (0 - robot.biasedFootVel[0][2]);

		robot.footForce1[0] = xOutput;
		robot.footForce1[1] = yOutput;
		robot.footForce1[2] = zOutput;

		robot.calJointTao(1);

		//2腿
		xInput = robot.biasedFootPos[1][0];
		yInput = robot.biasedFootPos[1][1];
		zInput = robot.biasedFootPos[1][2];

		xOutput = 1000 * (0 - xInput) + 100 * (0 - robot.biasedFootVel[1][0]);
		yOutput = 1000 * (-0.7 - yInput) + 100 * (0 - robot.biasedFootVel[1][1]);
		zOutput = 1000 * (0.1 - zInput) + 100 * (0 - robot.biasedFootVel[1][2]);

		robot.footForce2[0] = xOutput;
		robot.footForce2[1] = yOutput;
		robot.footForce2[2] = zOutput;

		robot.calJointTao(2);

		//3腿
		xInput = robot.biasedFootPos[2][0];
		yInput = robot.biasedFootPos[2][1];
		zInput = robot.biasedFootPos[2][2];

		xOutput = 1000 * (0 - xInput) + 100 * (0 - robot.biasedFootVel[2][0]);
		yOutput = 1000 * (-0.7 - yInput) + 100 * (0 - robot.biasedFootVel[2][1]);
		zOutput = 1000 * (-0.1 - zInput) + 100 * (0 - robot.biasedFootVel[2][2]);

		robot.footForce3[0] = xOutput;
		robot.footForce3[1] = yOutput;
		robot.footForce3[2] = zOutput;

		robot.calJointTao(3);

		//4腿
		xInput = robot.biasedFootPos[3][0];
		yInput = robot.biasedFootPos[3][1];
		zInput = robot.biasedFootPos[3][2];

		xOutput = 1000 * (0 - xInput) + 100 * (0 - robot.biasedFootVel[3][0]);
		yOutput = 1000 * (-0.7 - yInput) + 100 * (0 - robot.biasedFootVel[3][1]);
		zOutput = 1000 * (0.1 - zInput) + 100 * (0 - robot.biasedFootVel[3][2]);

		robot.footForce4[0] = xOutput;
		robot.footForce4[1] = yOutput;
		robot.footForce4[2] = zOutput;

		robot.calJointTao(4);


		/*
		输出
		*/
		float jointTorques[4][4];
		robot.getJointTorques(jointTorques);

		simxPauseCommunication(clientID, 1);

		//1腿
		setJointCmd(clientID, jointHandles.joint_HS1, jointTorques[0][3]);
		setJointCmd(clientID, jointHandles.joint_HF1, jointTorques[0][2]);
		setJointCmd(clientID, jointHandles.joint_K1, jointTorques[0][1]);
		//setJointCmd(clientID, jointHandles.joint_A1, jointTorques[0][0]);

		simxSetJointTargetPosition(clientID, jointHandles.joint_A1, (-PI / 3), simx_opmode_oneshot);

		//2腿
		setJointCmd(clientID, jointHandles.joint_HS2, jointTorques[1][3]);
		setJointCmd(clientID, jointHandles.joint_HF2, jointTorques[1][2]);
		setJointCmd(clientID, jointHandles.joint_K2, jointTorques[1][1]);
		//setJointCmd(clientID, jointHandles.joint_A2, jointTorques[1][0]);

		simxSetJointTargetPosition(clientID, jointHandles.joint_A2, (-PI / 3), simx_opmode_oneshot);

		//3腿
		setJointCmd(clientID, jointHandles.joint_HS3, jointTorques[2][3]);
		setJointCmd(clientID, jointHandles.joint_HF3, jointTorques[2][2]);
		setJointCmd(clientID, jointHandles.joint_K3, jointTorques[2][1]);
		//setJointCmd(clientID, jointHandles.joint_A3, jointTorques[2][0]);

		simxSetJointTargetPosition(clientID, jointHandles.joint_A3, (-PI / 3), simx_opmode_oneshot);

		//4腿
		setJointCmd(clientID, jointHandles.joint_HS4, jointTorques[3][3]);
		setJointCmd(clientID, jointHandles.joint_HF4, jointTorques[3][2]);
		setJointCmd(clientID, jointHandles.joint_K4, jointTorques[3][1]);
		//setJointCmd(clientID, jointHandles.joint_A4, jointTorques[3][0]);

		simxSetJointTargetPosition(clientID, jointHandles.joint_A4, (-PI / 3), simx_opmode_oneshot);

		simxPauseCommunication(clientID, 0);


		/*
		测试信息
		*/
		if (0 == idx % 10) {
			cout << "running" << endl;
			if (VERBOSE) {
				cout << "angle" << endl;
				for (int m = 0; m < 4; m++) {
					cout << robot.jointAngles[m][0] << "|" << robot.jointAngles[m][1] \
						<< "|" << robot.jointAngles[m][2] << "|" << robot.jointAngles[m][3] << endl;
				}
				cout << "pos" << endl;
				for (int m = 0; m < 4; m++) {
					cout << robot.biasedFootPos[m][0] << "|" << robot.biasedFootPos[m][1] \
						<< "|" << robot.biasedFootPos[m][2] << endl;
				}
				cout << "force" << endl;
				cout << robot.footForce1[0] << "|" << robot.footForce1[1] << "|" << robot.footForce1[2] << endl;
				/*
				cout << robot.jointAngles[0][0] << "|" << robot.jointAngles[0][1] \
					<< "|" << robot.jointAngles[0][2] << "|" << robot.jointAngles[0][3] << endl;
				cout << robot.jointAngles[1][0] << "|" << robot.jointAngles[1][1] \
					<< "|" << robot.jointAngles[1][2] << "|" << robot.jointAngles[1][3] << endl;
				cout << robot.jointAngles[2][0] << "|" << robot.jointAngles[2][1] \
					<< "|" << robot.jointAngles[2][2] << "|" << robot.jointAngles[2][3] << endl;
				cout << robot.jointAngles[3][0] << "|" << robot.jointAngles[3][1] \
					<< "|" << robot.jointAngles[3][2] << "|" << robot.jointAngles[3][3] << endl;

				cout << "pos" << endl;
				cout << robot.biasedFootPos[0][0] << "|" << robot.biasedFootPos[0][1] \
					<< "|" << robot.biasedFootPos[0][2] << endl;
				cout << robot.biasedFootPos[1][2] << endl;
				cout << robot.biasedFootPos[2][2] << endl;
				cout << robot.biasedFootPos[3][2] << endl;
				*/
			}

		}

		//simxSetJointTargetVelocity(clientID, jointHandles.joint_HS1, 0.1, simx_opmode_oneshot);
		//simxSetJointTargetVelocity(clientID, jointHandles.joint_HS2, 0.1, simx_opmode_oneshot);
		//simxSetJointTargetVelocity(clientID, jointHandles.joint_HS3, 0.1, simx_opmode_oneshot);
		//simxSetJointTargetVelocity(clientID, jointHandles.joint_HS4, 0.1, simx_opmode_oneshot);

		simxSynchronousTrigger(clientID);
		
		simxGetPingTime(clientID, p_time);
	}


	simxStopSimulation(clientID, simx_opmode_oneshot_wait);

	simxFinish(clientID);
	return clientID;
}
