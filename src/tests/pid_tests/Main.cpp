
#include "pid/pid.h"
#include <iostream>

namespace pid
{
typedef Eigen::Matrix<float, 1, 1> vec1_t;
typedef Eigen::Matrix<float, 5, 1> vec5_t;
typedef pid<float, float, 1, vec1_t> pid1_t;
typedef pid<float, float, 5, vec5_t> pid5_t;
}

using namespace std;
using namespace pid;

void PidCreationTest(bool& error)
{
	vec1_t kp, ki, kd;
	kp(0) = 1; ki(0) = 2; kd(0) = 3;
	pid1_t pid1(kp, ki, kd);
	pid5_t pid5(1, 2, 3);
}

void PidRunTest(bool& error)
{
	vec5_t input, output, result, setPoint;
	input << 1 , 2 , 3 , 4 , 5;
	setPoint << 5, 4, 3, 2, 1;
	output << 24 , 12 , 0 , -12 , -24;
	pid5_t pid5(1, 2, 3);
	result = pid5(1.f, input, setPoint);
	if(output != result)
	{
		error = true;
		std::cout << "In PidRunTest, expected output different than result : \nexpected \n" << output.transpose() << "\ngot \n"  << result.transpose() << std::endl;
	}
}

int main(int argc, char *argv[])
{
	std::cout << "performing tests... \n";
	bool error = false;
	PidCreationTest(error);
	PidRunTest(error);
	if(error)
	{
		std::cout << "There were some errors\n";
		return -1;
	}
	else
	{
		std::cout << "no errors found \n";
		return 0;
	}
}

