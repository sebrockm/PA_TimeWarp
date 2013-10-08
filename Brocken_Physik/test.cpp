#include "Queue.h"
#include "Movable.h"
#include "Heap.h"
#include "TimeWarpKernel.h"
#include "GLManager.h"
#include "MyExceptions.h"

#include <iostream>


using namespace std;



int main(int argc, char** argv){
	std::cout.precision(3);
	GLManager& glmgr = GLManager::instance();
	glmgr.init(argc, argv);
	glmgr.createShaderProgram("vs.glsl", "fs.glsl");
	

	TimeWarpManager& cmgr = glmgr.cmgr;

	cmgr.addPlane();
	cmgr.planes[0].set(Vector3f(0,-1,0).getNormalized(),Vector3f(), steel);

	cmgr.addSphere(50);
	for(int i=0;i<10;i++)
		cmgr.spheres[i].set(Vector3f(0, 4*(i+1), 0), 1);
	

	glmgr.cam.pos = Vector3f(0,5,25);

	try{
		glmgr.enterMainLoop();
	}
	catch(cuda_exception& cu){
		clog << cu.what() << endl;
		system("pause");
	}
	catch(std::exception& e){
		clog << e.what() << endl;
		system("pause");
	}
	catch(char* s){
		clog << s << endl;
		system("pause");
	}

	return 0;
}