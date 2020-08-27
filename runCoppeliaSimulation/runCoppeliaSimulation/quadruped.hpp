
#ifndef _QUADRUPED_H_
#define _QUADRUPED_H_


#define PI 3.1416

class quadruped{



public:
	float jointAngles[4][4];
	float jointAngleVels[4][4];
	float jointTaos[4][4];
	float footForces[4][3];

	//��ʱ���ڲ���
	float footForce1[3];
	float jointTao1[4];
	float footForce2[3];
	float jointTao2[4];
	float footForce3[3];
	float jointTao3[4];
	float footForce4[3];
	float jointTao4[4];

	//float footForce_H[4][3];
	//float jointTaos[4][4];

	float biasedFootPos[4][3];
	float biasedFootVel[4][3];

	float com2footVectors[4][3];

	float biasedJacobian1[3][4];
	float biasedJeT1[4][3];
	float biasedJacobian2[3][4];
	float biasedJeT2[4][3];
	float biasedJacobian3[3][4];
	float biasedJeT3[4][3];
	float biasedJacobian4[3][4];
	float biasedJeT4[4][3];


public:
	quadruped();
	int setRobotAngle(float jointThetas[4][4]);
	int setRobotAngleVelocity(float jointThetaDots[4][4]);

	//�����������������Ų�����ϵ�����λ��
	int calBiasedFootPos();

	//�������ģ����弸�����ģ�����˵��������ڱ���ϵ�µı�ʾ��
	int calCom2FootVectors();

	//�������������Ų�����ϵ���ſ˱Ⱦ���
	int calBiasedJacobian(int leg);
	int calBiasedJeT(int leg);

	int calBiasedFootVel(int leg);

	int calJointTao(int leg);

	int getJointTorques(float jointTorques[4][4]);



private:
	float _H0 = 0.05950;
	float _R3 = 0.26997;
	float _R2 = 0.33526;
	float _R1 = 0.34606;
	float _R0 = 0.05500;

	//����ĳ����
	float Length = 2 * 0.3465;
	float Width = 2 * 0.18;
	float Height = 0.2;					//�߶ȿ�����ʵ�ʲ���

};




#endif
