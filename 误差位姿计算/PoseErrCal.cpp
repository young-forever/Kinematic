#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

/*
�������ƣ�λ�������⺯��
������������������λ�ˣ���̬��xyz˳��ŷ���Ǳ�ʾ������������ڿ��Ƶ����λ�ˡ��ο����ĸ���������ѧ����3��P25��
����������P1��P2��ʾ�����λ�ˣ������ַָ�룩��dPΪ�����λ���������ַָ�룩��
��    ����������Ҫ����Eigen�⣬λ�˵�λΪm��rad��
*/
void PoseErrCal(double* P1, double* P2, double* dP)
{
	Matrix<double, 6, 1>  v_P1, v_P2;
	for (int i = 0; i < 6; i++)
	{
		v_P1(i, 0) = P1[i];
		v_P2(i, 0) = P2[i];
	}

	Matrix<double, 6, 1> d_EulerPose;
	d_EulerPose = v_P1 - v_P2;
	Matrix<double, 6, 6> J_Euler;
	J_Euler <<
		MatrixXd::Zero(6, 6);
	J_Euler(0, 0) = 1;
	J_Euler(1, 1) = 1;
	J_Euler(2, 2) = 1;
	J_Euler(3, 3) = 1;
	J_Euler(3, 5) = sin(v_P2(4, 0));
	J_Euler(4, 4) = cos(v_P2(3, 0));
	J_Euler(4, 5) = -sin(v_P2(3, 0))*cos(v_P2(4, 0));
	J_Euler(5, 4) = sin(v_P2(3, 0));
	J_Euler(5, 5) = cos(v_P2(3, 0))*cos(v_P2(4, 0));

	Matrix<double, 6, 1> d_Pose;
	d_Pose = J_Euler*d_EulerPose;

	for (int i = 0; i < 6; i++)
	{
		dP[i] = d_Pose(i, 0);
	}
}