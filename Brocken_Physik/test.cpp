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

	int dim;
	cout << "Wie viele (x*x*x/4)?\nx:";
	cin >> dim;
	cout << endl;
	cmgr.addSphere(dim*dim*dim/4);
	for(int i=0; i<cmgr.sphereCount; i++){
		cmgr.spheres[i].set(Vector3f(i/dim%dim,5+i/(dim*dim)%dim,i%dim)*2.5f+
			Vector3f(rand()%10,rand()%10,rand()%10)/100.f, 
			.99f, (Material)(rand()%material_N));
	}

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