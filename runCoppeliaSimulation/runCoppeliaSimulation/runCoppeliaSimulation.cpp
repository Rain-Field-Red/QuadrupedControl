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


using namespace std;
#define PI 3.14

int main()
{
	bool VERBOSE = true;
	int clientID = 0;
	int p_time[2];					//用于单步仿真

	JointHandles jointHandles;
	jointHandles.init();

	float jointThetas[4][4];
	float jointThetaDots[4][4];

	float jointTorques[4][4];

	quadruped robot;

	//int counter = 0;

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
	simxSynchronous(clientID, TRUE);
	simxSetFloatingParameter(clientID, sim_floatparam_simulation_time_step, 0.01, simx_opmode_blocking);
	
	int start = simxStartSimulation(clientID, simx_opmode_oneshot_wait);
	
	//获取关节句柄
	jointHandles = getJointHandles(clientID);


	//关节角度和角速度
	getJointThetas(clientID, jointHandles, jointThetas);
	getJointThetaDots(clientID, jointHandles, jointThetaDots);
	

	// 单步执行一次
	simxSynchronousTrigger(clientID);
	simxGetPingTime(clientID, p_time);


	Sleep(1000);

	robot.setRobotAngle(jointThetas);
	robot.setRobotAngleVelocity(jointThetaDots);

	robot.calBiasedFootPos();

	float x0_d = 0.0;
	float y0_d = -0.8;
	float z0_d = 0.0;

	//1腿三个方向的pid控制器
	//x方向pid
	double xSetpoint, xInput, xOutput;
	double Kp, Ki, Kd;
	xSetpoint = x0_d;
	xInput = robot.biasedFootPos[0][0];
	Kp = 30;
	Ki = 0;
	Kd = 3;
	PID xPID(&xInput, &xOutput, &xSetpoint, Kp, Ki, Kd, DIRECT);
	xPID.SetMode(AUTOMATIC);
	//y方向pid
	double ySetpoint, yInput, yOutput;
	ySetpoint = y0_d;
	yInput = robot.biasedFootPos[0][1];
	Kp = 30;
	Ki = 0;
	Kd = 3;
	PID yPID(&yInput, &yOutput, &ySetpoint, Kp, Ki, Kd, DIRECT);
	yPID.SetMode(AUTOMATIC);
	//z方向pid
	double zSetpoint, zInput, zOutput;
	zSetpoint = z0_d;
	zInput = robot.biasedFootPos[0][2];
	Kp = 30;
	Ki = 0;
	Kd = 3;
	PID zPID(&zInput, &zOutput, &zSetpoint, Kp, Ki, Kd, DIRECT);
	zPID.SetMode(AUTOMATIC);


	for (int idx = 0; idx < 200; idx++) {

		/*
		获取传感器信息
		*/
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

		robot.calBiasedFootVel(1);


		/*
		控制计算
		*/
		xInput = robot.biasedFootPos[0][0];
		yInput = robot.biasedFootPos[0][1];
		zInput = robot.biasedFootPos[0][2];

		//xPID.Compute();
		//yPID.Compute();
		//zPID.Compute();

		xOutput = 200 * (0 - xInput) + 50 * (0 - robot.biasedFootVel[0][0]);
		yOutput = 1000 * (-0.7 - yInput) + 100 * (0 - robot.biasedFootVel[0][1]);
		zOutput = 200 * (0 - zInput) + 50 * (0 - robot.biasedFootVel[0][2]);

		robot.footForce1[0] = xOutput;
		robot.footForce1[1] = yOutput;
		robot.footForce1[2] = zOutput;

		robot.calJointTao(1);

		float jointTorque1[4];
		robot.getJointTorques(jointTorque1);

		setJointCmd(clientID, jointHandles.joint_HS1, jointTorque1[3]);
		setJointCmd(clientID, jointHandles.joint_HF1, jointTorque1[2]);
		setJointCmd(clientID, jointHandles.joint_K1, jointTorque1[1]);
		//setJointCmd(clientID, jointHandles.joint_A1, jointTorque1[0]);

		simxSetJointTargetPosition(clientID, jointHandles.joint_A1, (-PI / 3), simx_opmode_oneshot);


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


		//        simxPauseCommunication(clientID,0);


		//        float joint2 = 1;
		//        //simxSetJointTargetVelocity(clientID, lbrJoint2, 0.1, simx_opmode_oneshot_wait);
		//        while (simxGetConnectionId(clientID)!=-1  && WORK)              ///**<  while we are connected to the server.. */
		//        {
		////            simxSetJointTargetVelocity(clientID, leftmotorHandle, 0.2, simx_opmode_oneshot_wait);
		////            simxSetJointTargetVelocity(clientID, rightmotorHandle, 0.2, simx_opmode_oneshot_wait);
		//                TEST3 = simxSetJointTargetVelocity(clientID, lbrJoint2, -0.1, simx_opmode_oneshot);
		////            TEST3 = simxSetJointTargetPosition(clientID, lbrJoint2, joint2 * (PI/180), simx_opmode_oneshot);
		//
		//            if(counter>1000)
		//            {
		//                simxSetJointTargetVelocity(clientID, leftmotorHandle, 0.0, simx_opmode_oneshot_wait);
		//                simxSetJointTargetVelocity(clientID, rightmotorHandle, 0.0, simx_opmode_oneshot_wait);
		//                simxSetJointTargetVelocity(clientID, lbrJoint2, 0, simx_opmode_oneshot_wait);
		//                break;
		//            }
		//            cout<<counter<< "  "<< TEST3<<endl;
		//            counter++;
		//            joint2 = joint2 + 0.08;
		//        }
	simxStopSimulation(clientID, simx_opmode_oneshot_wait);

	simxFinish(clientID);
	return clientID;
}
