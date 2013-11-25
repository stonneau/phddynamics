#include "kinematics/joint.h"
#include "kinematics/joint_io.h"

#include "tools.h"


namespace kinematics
{
typedef joint<float, float, 2, 5, true> joint_2_t;
typedef joint<float, float, 3, 5, true> joint_3_t;
}

using namespace std;
using namespace tools;
using namespace kinematics;

void JointCreationTest(bool& error)
{
	joint_2_t joint2;
	joint_3_t joint;
	joint_3_t jointChild;
	joint.add_child(&jointChild);
}

void JointSaveTest(bool& error)
{
	std::string targetFile("./bin/Robot0.txt");
	std::string wrongFile("./wrongFolder/Robot0.txt");
	joint_3_t j00, j10, j11, j12, j110, j111, j120;
	j00.add_child(&j10);
	j00.add_child(&j11);
	j00.add_child(&j12);
	j11.add_child(&j110);
	j11.add_child(&j111);
	j12.add_child(&j120);
	if(!kinematics::WriteTree<joint_3_t,3,true>(j00, targetFile))
	{
		std::cout << "In JointSaveTest: can not write file Robot0.txt."<< std::endl;
		error = true;
		return;
	}
	if(!CompareFiles(targetFile, std::string("./tests/kinematics/io/Robot0.txt")))
	{
		error = true;
		std::cout << "In JointSaveTest: template file and resulting files have different content."<< std::endl;
	}
	if(!(remove(targetFile.c_str()) == 0))
	{
		error = true;
		std::cout << "In JointSaveTest: can not remove Robot0.txt."<< std::endl;
	}	
	try
	{
		kinematics::WriteTree<joint_3_t,3,false>(j00, wrongFile);
	}
	catch(std::exception e)
	{
		error = true;
		std::cout << "In JointSaveTest: unexpected exception for wrong file opening when Safe parameter is false"<< std::endl;
	}
	try
	{
		kinematics::WriteTree<joint_3_t,3,true>(j00, wrongFile);
	}
	catch(std::exception e)
	{
		return;
	}
	error = true;
	std::cout << "In JointSaveTest: unraised exception for wrong file opening"<< std::endl;
}

void JointLoadTest(bool& error)
{
	std::string targetFile("./tests/kinematics/io/RobotLoad.txt");
	std::string comparedFile("./RobotLoad.txt");
	joint_3_t* root = ReadTree<float, float, 3, 5, true>(targetFile);
	WriteTree<joint_3_t,3,false>(*root, comparedFile);
	if(!CompareFiles(targetFile, comparedFile))
	{
		error = true;
		std::cout << "In JointLoadTest: template file and resulting files have different content."<< std::endl;
	}
	if(!(remove(comparedFile.c_str()) == 0))
	{
		error = true;
		std::cout << "In JointLoadTest: can not remove RobotLoad.txt."<< std::endl;
	}
	delete root;
}

void JointTagTest(bool& error)
{
	std::string targetFile("./tests/kinematics/io/RobotTag.txt");
	joint_3_t* root = ReadTree<float, float, 3, 5, true>(targetFile);
	std::string tag(root->tag);
	if(tag != std::string("tagTest"))
	{
		error = true;
		std::cout << "In JointTagTest: can not read expected tag 'tagTest' for root."<< std::endl;
	}
	delete root;
}


int main(int argc, char *argv[])
{
	std::cout << "performing tests... \n";
	bool error = false;
	JointCreationTest(error);
	JointSaveTest(error);
	JointLoadTest(error);
	JointTagTest(error);
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
