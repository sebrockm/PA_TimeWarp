#include "Queue.h"
#include "Movable.h"
#include "Heap.h"
#include "TimeWarpKernel.h"
#include "GLManager.h"
#include "MyExceptions.h"

#include <iostream>
#include <cstdlib>
#include <ctime>


using namespace std;



int main(int argc, char** argv){
	std::cout.precision(3);
	srand((u32)time(0));
	GLManager& glmgr = GLManager::instance();
	glmgr.init(argc, argv);
	glmgr.createShaderProgram("vs.glsl", "fs.glsl");
	

	TimeWarpManager& cmgr = glmgr.cmgr;

	cmgr.addPlane();
	cmgr.planes[0].set(Vector3f(0,1,0).getNormalized(),Vector3f(), steel);

	int dim;
	cin>>dim;
	//cmgr.addSphere(dim*dim*(dim/2));
	//for(int i=0; i<dim; i++){
	//	for(int j=0; j<dim; j++){
	//		for(int k=0; k<dim/2; k++){
	//			cmgr.spheres[i*dim*(dim/2)+j*(dim/2)+k].set(
	//				Vector3f(
	//					(rand()%100-50)*.005f + i - dim/3.f, 
	//					(k+5), 
	//					(rand()%100-50)*.005f + j - dim/3.f
	//				) * 3.f, 
	//				.5f /*-(rand()%100)/150.f*/, 
	//				Material(rand()%material_N)
	//			);
	//		}
	//	}
	//}
	cmgr.addSphere(dim);
	for(int i=0;i<dim;i++)
		cmgr.spheres[i].set(Vector3d(0, 4+i*3, 2.2*i), 1);

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
