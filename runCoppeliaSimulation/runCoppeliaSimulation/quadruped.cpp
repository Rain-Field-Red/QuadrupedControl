
#include "quadruped.hpp"

#include <cmath>

quadruped::quadruped() {
	for (int m = 0; m < 4; m++) {
		for (int n = 0; n < 4; n++) {
			jointAngles[m][n] = 0;
		}
	}
}

int quadruped::setRobotAngle(float jointThetas[4][4]) {
	
	// 1腿
	jointAngles[0][0] = PI + jointThetas[0][0];
	jointAngles[0][1] = PI - jointThetas[0][1];
	jointAngles[0][2] = 0.5*PI + jointThetas[0][2];
	jointAngles[0][3] = -jointThetas[0][3];

	// 2腿
	jointAngles[1][0] = PI + jointThetas[1][0];
	jointAngles[1][1] = PI - jointThetas[1][1];
	jointAngles[1][2] = 0.5*PI + jointThetas[1][2];
	jointAngles[1][3] = jointThetas[1][3];
		
	// 3腿
	jointAngles[2][0] = PI - jointThetas[2][0];
	jointAngles[2][1] = PI + jointThetas[2][1];
	jointAngles[2][2] = 0.5*PI - jointThetas[2][2];
	jointAngles[2][3] = -jointThetas[2][3];

	// 4腿
	jointAngles[3][0] = PI - jointThetas[3][0];
	jointAngles[3][1] = PI + jointThetas[3][1];
	jointAngles[3][2] = 0.5*PI - jointThetas[3][2];
	jointAngles[3][3] = jointThetas[3][3];
	

	return 0;
}


int quadruped::setRobotAngleVelocity(float jointThetaDots[4][4]) {

	jointAngleVels[0][0] = jointThetaDots[0][0];
	jointAngleVels[0][1] = -jointThetaDots[0][1];
	jointAngleVels[0][2] = jointThetaDots[0][2];
	jointAngleVels[0][3] = -jointThetaDots[0][3];

	jointAngleVels[1][0] = jointThetaDots[1][0];
	jointAngleVels[1][1] = -jointThetaDots[1][1];
	jointAngleVels[1][2] = jointThetaDots[1][2];
	jointAngleVels[1][3] = jointThetaDots[1][3];

	jointAngleVels[2][0] = -jointThetaDots[2][0];
	jointAngleVels[2][1] = jointThetaDots[2][1];
	jointAngleVels[2][2] = -jointThetaDots[2][2];
	jointAngleVels[2][3] = -jointThetaDots[2][3];

	jointAngleVels[3][0] = -jointThetaDots[3][0];
	jointAngleVels[3][1] = jointThetaDots[3][1];
	jointAngleVels[3][2] = -jointThetaDots[3][2];
	jointAngleVels[3][3] = jointThetaDots[3][3];

	return 0;
}


int quadruped::calBiasedFootPos() {
	float temp;
	
	//足端位置,相对于髋，统一按本体坐标系方向，没有考虑姿态角。
	// 1腿
	biasedFootPos[0][0] = _R3 * cos(jointAngles[0][2]) \
		- _R2 * cos(jointAngles[0][1] - jointAngles[0][2]) \
		+ _R1 * cos(jointAngles[0][0] - jointAngles[0][1] + jointAngles[0][2]);
	temp = _R3 * sin(jointAngles[0][2]) \
		+ _R2 * sin(jointAngles[0][1] - jointAngles[0][2]) \
		+ _R1 * sin(jointAngles[0][0] - jointAngles[0][1] + jointAngles[0][2]);
	biasedFootPos[0][1] = -(_H0 + temp)*cos(jointAngles[0][3]);
	biasedFootPos[0][2] = -(_H0 + temp)*sin(jointAngles[0][3]);
	//2腿
	biasedFootPos[1][0] = _R3 * cos(jointAngles[1][2]) \
		- _R2 * cos(jointAngles[1][1] - jointAngles[1][2]) \
		+ _R1 * cos(jointAngles[1][0] - jointAngles[1][1] + jointAngles[1][2]);
	temp = _R3 * sin(jointAngles[1][2]) \
		+ _R2 * sin(jointAngles[1][1] - jointAngles[1][2]) \
		+ _R1 * sin(jointAngles[1][0] - jointAngles[1][1] + jointAngles[1][2]);
	biasedFootPos[1][1] = -(_H0 + temp)*cos(jointAngles[1][3]);
	biasedFootPos[1][2] = (_H0 + temp)*sin(jointAngles[1][3]);
	//3腿
	biasedFootPos[2][0] = -_R3 * cos(jointAngles[2][2]) \
		+ _R2 * cos(jointAngles[2][1] - jointAngles[2][2]) \
		- _R1 * cos(jointAngles[2][0] - jointAngles[2][1] + jointAngles[2][2]);
	temp = _R3 * sin(jointAngles[2][2]) \
		+ _R2 * sin(jointAngles[2][1] - jointAngles[2][2]) \
		+ _R1 * sin(jointAngles[2][0] - jointAngles[2][1] + jointAngles[2][2]);
	biasedFootPos[2][1] = -(_H0 + temp)*cos(jointAngles[2][3]);
	biasedFootPos[2][2] = -(_H0 + temp)*sin(jointAngles[2][3]);
	//4腿
	biasedFootPos[3][0] = -_R3 * cos(jointAngles[3][2]) \
		+ _R2 * cos(jointAngles[3][1] - jointAngles[3][2]) \
		- _R1 * cos(jointAngles[3][0] - jointAngles[3][1] + jointAngles[3][2]);
	temp = _R3 * sin(jointAngles[3][2]) \
		+ _R2 * sin(jointAngles[3][1] - jointAngles[3][2]) \
		+ _R1 * sin(jointAngles[3][0] - jointAngles[3][1] + jointAngles[3][2]);
	biasedFootPos[3][1] = -(_H0 + temp)*cos(jointAngles[3][3]);
	biasedFootPos[3][2] = (_H0 + temp)*sin(jointAngles[3][3]);
		
	return 0;
}


int quadruped::calBiasedJacobian(int leg) {

	if (1 == leg) {
		float theta00 = jointAngles[0][0];
		float theta01 = jointAngles[0][1];
		float theta02 = jointAngles[0][2];
		float theta03 = jointAngles[0][3];

		float j11 = -_R1 * sin(theta00 - theta01 + theta02);
		float j12 = _R1 * sin(theta00 - theta01 + theta02) + _R2 * sin(theta01 - theta02);
		float j13 = -_R1 * sin(theta00 - theta01 + theta02) - _R2 * sin(theta01 - theta02) - _R3 * sin(theta02);
		float j14 = 0;

		float j21 = -_R1 * cos(theta03)*cos(theta00 - theta01 + theta02);
		float j22 = (_R1*cos(theta00 - theta01 + theta02) + _R2 * cos(theta01 - theta02))*cos(theta03);
		float j23 = (-_R1 * cos(theta00 - theta01 + theta02) - _R2 * cos(theta01 - theta02) - _R3 * cos(theta02))*cos(theta03);
		float j24 = -(-_H0 - _R1 * sin(theta00 - theta01 + theta02) + _R2 * sin(theta01 - theta02) - _R3 * sin(theta02))*sin(theta03);

		float j31 = -_R1 * sin(theta03)*cos(theta00 - theta01 + theta02);
		float j32 = (_R1*cos(theta00 - theta01 + theta02) + _R2 * cos(theta01 - theta02))*sin(theta03);
		float j33 = (-_R1 * cos(theta00 - theta01 + theta02) - _R2 * cos(theta01 - theta02) - _R3 * cos(theta02))*sin(theta03);
		float j34 = (-_H0 - _R1 * sin(theta00 - theta01 + theta02) + _R2 * sin(theta01 - theta02) - _R3 * sin(theta02))*cos(theta03);

		biasedJacobian1[0][0] = j11;
		biasedJacobian1[0][1] = j12;
		biasedJacobian1[0][2] = j13;
		biasedJacobian1[0][3] = j14;

		biasedJacobian1[1][0] = j21;
		biasedJacobian1[1][1] = j22;
		biasedJacobian1[1][2] = j23;
		biasedJacobian1[1][3] = j24;

		biasedJacobian1[2][0] = j31;
		biasedJacobian1[2][1] = j32;
		biasedJacobian1[2][2] = j33;
		biasedJacobian1[2][3] = j34;

		return 1;
	}

	if (2 == leg) {
		float theta10 = jointAngles[1][0];
		float theta11 = jointAngles[1][1];
		float theta12 = jointAngles[1][2];
		float theta13 = jointAngles[1][3];

		float j11 = -_R1 * sin(theta10 - theta11 + theta12);
		float j12 = _R1 * sin(theta10 - theta11 + theta12) + _R2 * sin(theta11 - theta12);
		float j13 = -_R1 * sin(theta10 - theta11 + theta12) - _R2 * sin(theta11 - theta12) - _R3 * sin(theta12);
		float j14 = 0;

		float j21 = -_R1 * cos(theta13)*cos(theta10 - theta11 + theta12);
		float j22 = (_R1*cos(theta10 - theta11 + theta12) + _R2 * cos(theta11 - theta12))*cos(theta13);
		float j23 = (-_R1 * cos(theta10 - theta11 + theta12) - _R2 * cos(theta11 - theta12) - _R3 * cos(theta12))*cos(theta13);
		float j24 = -(-_H0 - _R1 * sin(theta10 - theta11 + theta12) + _R2 * sin(theta11 - theta12) - _R3 * sin(theta12))*sin(theta13);

		float j31 = _R1 * sin(theta13)*cos(theta10 - theta11 + theta12);
		float j32 = (-_R1*cos(theta10 - theta11 + theta12) - _R2 * cos(theta11 - theta12))*sin(theta13);
		float j33 = (_R1 * cos(theta10 - theta11 + theta12) + _R2 * cos(theta11 - theta12) + _R3 * cos(theta12))*sin(theta13);
		float j34 = (_H0 + _R1 * sin(theta10 - theta11 + theta12) - _R2 * sin(theta11 - theta12) + _R3 * sin(theta12))*cos(theta13);

		biasedJacobian2[0][0] = j11;
		biasedJacobian2[0][1] = j12;
		biasedJacobian2[0][2] = j13;
		biasedJacobian2[0][3] = j14;

		biasedJacobian2[1][0] = j21;
		biasedJacobian2[1][1] = j22;
		biasedJacobian2[1][2] = j23;
		biasedJacobian2[1][3] = j24;

		biasedJacobian2[2][0] = j31;
		biasedJacobian2[2][1] = j32;
		biasedJacobian2[2][2] = j33;
		biasedJacobian2[2][3] = j34;

		return 2;
	}

	if (3 == leg) {
		float theta00 = jointAngles[2][0];
		float theta01 = jointAngles[2][1];
		float theta02 = jointAngles[2][2];
		float theta03 = jointAngles[2][3];

		float j11 = _R1 * sin(theta00 - theta01 + theta02);
		float j12 = -_R1 * sin(theta00 - theta01 + theta02) - _R2 * sin(theta01 - theta02);
		float j13 = _R1 * sin(theta00 - theta01 + theta02) + _R2 * sin(theta01 - theta02) + _R3 * sin(theta02);
		float j14 = 0;

		float j21 = -_R1 * cos(theta03)*cos(theta00 - theta01 + theta02);
		float j22 = (_R1*cos(theta00 - theta01 + theta02) + _R2 * cos(theta01 - theta02))*cos(theta03);
		float j23 = (-_R1 * cos(theta00 - theta01 + theta02) - _R2 * cos(theta01 - theta02) - _R3 * cos(theta02))*cos(theta03);
		float j24 = -(-_H0 - _R1 * sin(theta00 - theta01 + theta02) + _R2 * sin(theta01 - theta02) - _R3 * sin(theta02))*sin(theta03);

		float j31 = -_R1 * sin(theta03)*cos(theta00 - theta01 + theta02);
		float j32 = (_R1*cos(theta00 - theta01 + theta02) + _R2 * cos(theta01 - theta02))*sin(theta03);
		float j33 = (-_R1 * cos(theta00 - theta01 + theta02) - _R2 * cos(theta01 - theta02) - _R3 * cos(theta02))*sin(theta03);
		float j34 = (-_H0 - _R1 * sin(theta00 - theta01 + theta02) + _R2 * sin(theta01 - theta02) - _R3 * sin(theta02))*cos(theta03);

		biasedJacobian3[0][0] = j11;
		biasedJacobian3[0][1] = j12;
		biasedJacobian3[0][2] = j13;
		biasedJacobian3[0][3] = j14;

		biasedJacobian3[1][0] = j21;
		biasedJacobian3[1][1] = j22;
		biasedJacobian3[1][2] = j23;
		biasedJacobian3[1][3] = j24;

		biasedJacobian3[2][0] = j31;
		biasedJacobian3[2][1] = j32;
		biasedJacobian3[2][2] = j33;
		biasedJacobian3[2][3] = j34;

		return 3;
	}

	if (4 == leg) {
		float theta10 = jointAngles[3][0];
		float theta11 = jointAngles[3][1];
		float theta12 = jointAngles[3][2];
		float theta13 = jointAngles[3][3];

		float j11 = _R1 * sin(theta10 - theta11 + theta12);
		float j12 = -_R1 * sin(theta10 - theta11 + theta12) - _R2 * sin(theta11 - theta12);
		float j13 = _R1 * sin(theta10 - theta11 + theta12) + _R2 * sin(theta11 - theta12) + _R3 * sin(theta12);
		float j14 = 0;

		float j21 = -_R1 * cos(theta13)*cos(theta10 - theta11 + theta12);
		float j22 = (_R1*cos(theta10 - theta11 + theta12) + _R2 * cos(theta11 - theta12))*cos(theta13);
		float j23 = (-_R1 * cos(theta10 - theta11 + theta12) - _R2 * cos(theta11 - theta12) - _R3 * cos(theta12))*cos(theta13);
		float j24 = -(-_H0 - _R1 * sin(theta10 - theta11 + theta12) + _R2 * sin(theta11 - theta12) - _R3 * sin(theta12))*sin(theta13);

		float j31 = _R1 * sin(theta13)*cos(theta10 - theta11 + theta12);
		float j32 = (-_R1 * cos(theta10 - theta11 + theta12) - _R2 * cos(theta11 - theta12))*sin(theta13);
		float j33 = (_R1 * cos(theta10 - theta11 + theta12) + _R2 * cos(theta11 - theta12) + _R3 * cos(theta12))*sin(theta13);
		float j34 = (_H0 + _R1 * sin(theta10 - theta11 + theta12) - _R2 * sin(theta11 - theta12) + _R3 * sin(theta12))*cos(theta13);

		biasedJacobian4[0][0] = j11;
		biasedJacobian4[0][1] = j12;
		biasedJacobian4[0][2] = j13;
		biasedJacobian4[0][3] = j14;

		biasedJacobian4[1][0] = j21;
		biasedJacobian4[1][1] = j22;
		biasedJacobian4[1][2] = j23;
		biasedJacobian4[1][3] = j24;

		biasedJacobian4[2][0] = j31;
		biasedJacobian4[2][1] = j32;
		biasedJacobian4[2][2] = j33;
		biasedJacobian4[2][3] = j34;

		return 4;
	}

	return -1;
	
}


int quadruped::calBiasedJeT(int leg) {

	if (1 == leg) {

		for (int m = 0; m < 4; m++) {
			for (int n = 0; n < 3; n++) {
				biasedJeT1[m][n] = biasedJacobian1[n][m];
			}
		}

		return 1;
	}

	if (2 == leg) {

		for (int m = 0; m < 4; m++) {
			for (int n = 0; n < 3; n++) {
				biasedJeT2[m][n] = biasedJacobian2[n][m];
			}
		}

		return 2;
	}

	if (3 == leg) {

		for (int m = 0; m < 4; m++) {
			for (int n = 0; n < 3; n++) {
				biasedJeT3[m][n] = biasedJacobian3[n][m];
			}
		}

		return 3;
	}

	if (4 == leg) {

		for (int m = 0; m < 4; m++) {
			for (int n = 0; n < 3; n++) {
				biasedJeT4[m][n] = biasedJacobian4[n][m];
			}
		}

		return 4;
	}

	return -1;
}


int quadruped::calJointTao(int leg) {

	if (1 == leg) {
		jointTao1[0] = biasedJeT1[0][0] * footForce1[0] \
			+ biasedJeT1[0][1] * footForce1[1] \
			+ biasedJeT1[0][2] * footForce1[2];
		jointTao1[1] = biasedJeT1[1][0] * footForce1[0] \
			+ biasedJeT1[1][1] * footForce1[1] \
			+ biasedJeT1[1][2] * footForce1[2];
		jointTao1[2] = biasedJeT1[2][0] * footForce1[0] \
			+ biasedJeT1[2][1] * footForce1[1] \
			+ biasedJeT1[2][2] * footForce1[2];
		jointTao1[3] = biasedJeT1[3][0] * footForce1[0] \
			+ biasedJeT1[3][1] * footForce1[1] \
			+ biasedJeT1[3][2] * footForce1[2];

		return 1;
	}

	if (2 == leg) {
		jointTao2[0] = biasedJeT2[0][0] * footForce2[0] \
			+ biasedJeT2[0][1] * footForce2[1] \
			+ biasedJeT2[0][2] * footForce2[2];
		jointTao2[1] = biasedJeT2[1][0] * footForce2[0] \
			+ biasedJeT2[1][1] * footForce2[1] \
			+ biasedJeT2[1][2] * footForce2[2];
		jointTao2[2] = biasedJeT2[2][0] * footForce2[0] \
			+ biasedJeT2[2][1] * footForce2[1] \
			+ biasedJeT2[2][2] * footForce2[2];
		jointTao2[3] = biasedJeT2[3][0] * footForce2[0] \
			+ biasedJeT2[3][1] * footForce2[1] \
			+ biasedJeT2[3][2] * footForce2[2];

		return 2;
	}

	if (3 == leg) {
		jointTao3[0] = biasedJeT3[0][0] * footForce3[0] \
			+ biasedJeT3[0][1] * footForce3[1] \
			+ biasedJeT3[0][2] * footForce3[2];
		jointTao3[1] = biasedJeT3[1][0] * footForce3[0] \
			+ biasedJeT3[1][1] * footForce3[1] \
			+ biasedJeT3[1][2] * footForce3[2];
		jointTao3[2] = biasedJeT3[2][0] * footForce3[0] \
			+ biasedJeT3[2][1] * footForce3[1] \
			+ biasedJeT3[2][2] * footForce3[2];
		jointTao3[3] = biasedJeT3[3][0] * footForce3[0] \
			+ biasedJeT3[3][1] * footForce3[1] \
			+ biasedJeT3[3][2] * footForce3[2];

		return 3;
	}

	if (4 == leg) {
		jointTao4[0] = biasedJeT4[0][0] * footForce4[0] \
			+ biasedJeT4[0][1] * footForce4[1] \
			+ biasedJeT4[0][2] * footForce4[2];
		jointTao4[1] = biasedJeT4[1][0] * footForce4[0] \
			+ biasedJeT4[1][1] * footForce4[1] \
			+ biasedJeT4[1][2] * footForce4[2];
		jointTao4[2] = biasedJeT4[2][0] * footForce4[0] \
			+ biasedJeT4[2][1] * footForce4[1] \
			+ biasedJeT4[2][2] * footForce4[2];
		jointTao4[3] = biasedJeT4[3][0] * footForce4[0] \
			+ biasedJeT4[3][1] * footForce4[1] \
			+ biasedJeT4[3][2] * footForce4[2];

		return 4;
	}

	return -1;
}


int quadruped::calBiasedFootVel(int leg) {

	if (1 == leg) {

		biasedFootVel[0][0] = biasedJacobian1[0][0] * jointAngleVels[0][0] \
			+ biasedJacobian1[0][1] * jointAngleVels[0][1] \
			+ biasedJacobian1[0][2] * jointAngleVels[0][2] \
			+ biasedJacobian1[0][3] * jointAngleVels[0][3];

		biasedFootVel[0][1] = biasedJacobian1[1][0] * jointAngleVels[0][0] \
			+ biasedJacobian1[1][1] * jointAngleVels[0][1] \
			+ biasedJacobian1[1][2] * jointAngleVels[0][2] \
			+ biasedJacobian1[1][3] * jointAngleVels[0][3];

		biasedFootVel[0][2] = biasedJacobian1[2][0] * jointAngleVels[0][0] \
			+ biasedJacobian1[2][1] * jointAngleVels[0][1] \
			+ biasedJacobian1[2][2] * jointAngleVels[0][2] \
			+ biasedJacobian1[2][3] * jointAngleVels[0][3];

		return 1;
	}

	if (2 == leg) {

		biasedFootVel[1][0] = biasedJacobian2[0][0] * jointAngleVels[1][0] \
			+ biasedJacobian2[0][1] * jointAngleVels[1][1] \
			+ biasedJacobian2[0][2] * jointAngleVels[1][2] \
			+ biasedJacobian2[0][3] * jointAngleVels[1][3];

		biasedFootVel[1][1] = biasedJacobian2[1][0] * jointAngleVels[1][0] \
			+ biasedJacobian2[1][1] * jointAngleVels[1][1] \
			+ biasedJacobian2[1][2] * jointAngleVels[1][2] \
			+ biasedJacobian2[1][3] * jointAngleVels[1][3];

		biasedFootVel[1][2] = biasedJacobian2[2][0] * jointAngleVels[1][0] \
			+ biasedJacobian2[2][1] * jointAngleVels[1][1] \
			+ biasedJacobian2[2][2] * jointAngleVels[1][2] \
			+ biasedJacobian2[2][3] * jointAngleVels[1][3];

		return 2;
	}

	if (3 == leg) {

		biasedFootVel[2][0] = biasedJacobian3[0][0] * jointAngleVels[2][0] \
			+ biasedJacobian3[0][1] * jointAngleVels[2][1] \
			+ biasedJacobian3[0][2] * jointAngleVels[2][2] \
			+ biasedJacobian3[0][3] * jointAngleVels[2][3];

		biasedFootVel[2][1] = biasedJacobian3[1][0] * jointAngleVels[2][0] \
			+ biasedJacobian3[1][1] * jointAngleVels[2][1] \
			+ biasedJacobian3[1][2] * jointAngleVels[2][2] \
			+ biasedJacobian3[1][3] * jointAngleVels[2][3];

		biasedFootVel[2][2] = biasedJacobian3[2][0] * jointAngleVels[2][0] \
			+ biasedJacobian3[2][1] * jointAngleVels[2][1] \
			+ biasedJacobian3[2][2] * jointAngleVels[2][2] \
			+ biasedJacobian3[2][3] * jointAngleVels[2][3];

		return 3;
	}

	if (4 == leg) {

		biasedFootVel[3][0] = biasedJacobian4[0][0] * jointAngleVels[3][0] \
			+ biasedJacobian4[0][1] * jointAngleVels[3][1] \
			+ biasedJacobian4[0][2] * jointAngleVels[3][2] \
			+ biasedJacobian4[0][3] * jointAngleVels[3][3];

		biasedFootVel[3][1] = biasedJacobian4[1][0] * jointAngleVels[3][0] \
			+ biasedJacobian4[1][1] * jointAngleVels[3][1] \
			+ biasedJacobian4[1][2] * jointAngleVels[3][2] \
			+ biasedJacobian4[1][3] * jointAngleVels[3][3];

		biasedFootVel[3][2] = biasedJacobian4[2][0] * jointAngleVels[3][0] \
			+ biasedJacobian4[2][1] * jointAngleVels[3][1] \
			+ biasedJacobian4[2][2] * jointAngleVels[3][2] \
			+ biasedJacobian4[2][3] * jointAngleVels[3][3];

		return 1;
	}

	return -1;
}

int quadruped::getJointTorques(float jointTorques[4][4]) {
	jointTorques[0][0] = jointTao1[0];
	jointTorques[0][1] = -jointTao1[1];
	jointTorques[0][2] = jointTao1[2];
	jointTorques[0][3] = -jointTao1[3];

	jointTorques[1][0] = jointTao2[0];
	jointTorques[1][1] = -jointTao2[1];
	jointTorques[1][2] = jointTao2[2];
	jointTorques[1][3] = jointTao2[3];

	jointTorques[2][0] = -jointTao3[0];
	jointTorques[2][1] = jointTao3[1];
	jointTorques[2][2] = -jointTao3[2];
	jointTorques[2][3] = -jointTao3[3];

	jointTorques[3][0] = -jointTao4[0];
	jointTorques[3][1] = jointTao4[1];
	jointTorques[3][2] = -jointTao4[2];
	jointTorques[3][3] = jointTao4[3];

	return 0;
}


int quadruped::calCom2FootVectors() {
	/*
	需要注意的是：
	（1）本体坐标系中，前向为x方向，右侧为y方向，z方向按右手坐标系确定；
	（2）髋部坐标系中，前向为x方向，右侧为z方向，y方向按右手坐标系确定；
	*/
	//1腿
	com2footVectors[0][0] = 0.5*Length + biasedFootPos[0][0];
	com2footVectors[0][1] = -0.5*Width + biasedFootPos[0][2];
	com2footVectors[0][2] = 0.5*Height - biasedFootPos[0][1];

	//2腿
	com2footVectors[1][0] = 0.5*Length + biasedFootPos[1][0];
	com2footVectors[1][1] = 0.5*Width + biasedFootPos[1][2];
	com2footVectors[1][2] = 0.5*Height - biasedFootPos[1][1];

	//3腿
	com2footVectors[2][0] = -0.5*Length + biasedFootPos[0][0];
	com2footVectors[2][1] = -0.5*Width + biasedFootPos[0][2];
	com2footVectors[2][2] = 0.5*Height - biasedFootPos[0][1];

	//4腿
	com2footVectors[3][0] = -0.5*Length + biasedFootPos[0][0];
	com2footVectors[3][1] = 0.5*Width + biasedFootPos[0][2];
	com2footVectors[3][2] = 0.5*Height - biasedFootPos[0][1];
}