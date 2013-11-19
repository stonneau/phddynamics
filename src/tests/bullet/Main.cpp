
#include "bullet/joint_bullet.h"
#include <iostream>

namespace kinematics
{

}

using namespace std;
using namespace kinematics::bullet;

//int main(int argc, char *argv[])
//{
//	std::cout << "performing tests... \n";
//	bool error = false;
//	if(error)
//	{
//		std::cout << "There were some errors\n";
//		return -1;
//	}
//	else
//	{
//		std::cout << "no errors found \n";
//		return 0;
//	}
//}
#include "TestApplication.h"
#include "GlutStuff.h"

int main(int argc,char* argv[])
{
        TestApplication demoApp;

        demoApp.initPhysics();
       

        return glutmain(argc, argv,640,480,"Bullet Physics Demo. http://bullet.sf.net",&demoApp);
}

