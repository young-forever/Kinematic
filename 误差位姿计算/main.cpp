#include"iostream"
#include"PoseErrCal.h"
using namespace std;

//���Ժ���
int main(int argc, char *argv[])
{
	double Pc[6] = {2,3,4,0.1,0.3,0.2};//��ǰλ��
	double Pd[6] = {5,7,9,0.1,0,0};//����λ��
	double dP[6] = {0.0};

	PoseErrCal(Pd,Pc,dP);

	for (int i = 0; i < 6;i++)
	{
		if (i == 0)
		{
			cout << "dP=[";
		}
		cout << dP[i];
		if (i < 5)
		{
			cout << ",";
		}
		else
		{
			cout << "].";
		}
	}

	while (1)
		;
	return 0;
}