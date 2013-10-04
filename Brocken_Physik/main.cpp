
#include "GLManager.h"
#include "MyExceptions.h"


#include <iostream>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>
#include <exception>


int maina(int argc, char** argv){

	//std::cout.precision(3);

	//srand((u32)time(0));
	//GLManager& glmgr = GLManager::instance();
	//glmgr.init(argc, argv);
	//glmgr.createShaderProgram("vs.glsl", "fs.glsl");
	//

	//CollisionManager& cmgr = glmgr.cmgr;

	///*cmgr.addPlane(2);
	//cmgr.planes[0].set(Vector3f(1,1,1).getNormalized(), Vector3f(), wood);
	//cmgr.planes[1].set(Vector3f(1,1,-1).getNormalized(), Vector3f(), wood);*/
	//cmgr.addPlane();
	//cmgr.planes[0].set(Vector3f(0,1,0).getNormalized(),Vector3f(), steel);
	////cmgr.planes[1].set(Vector3f(0,1,-0.2f).getNormalized(),Vector3f(0,0,200), steel);

	////cmgr.addFixCuboid(5);
	////for(int i=0; i<cmgr.fixCuboidCount; i++){
	////	cmgr.fixCuboids[i].set(3,1.99f,1.99f,Vector3f(0,2*i+1,-2.00f*i+1));
	////}
	//int dim;
	//cout << "Wie viele (x*x*x/4)?\nx:";
	//cin >> dim;
	//cout << endl;
	///*cmgr.addSphere(dim*dim*dim);
	//for(int i=0; i<cmgr.sphereCount; i++){
	//	cmgr.spheres[i].set(Vector3f(i%dim,5+i/dim%dim,i/(dim*dim)%dim)*2.5f+
	//		Vector3f(rand()%10,rand()%10,rand()%10)/100.f, 
	//		.99f, (Material)(rand()%material_N));
	//}*/

	////cmgr.addSphere(dim*dim);
	////for(int i=0; i<cmgr.sphereCount; i++){
	////	cmgr.spheres[i].set(Vector3f(i%dim,0,i/dim), .49f, rubber, .5f);
	////	cmgr.spheres[i].v[0] = 2;
	////}

	////int stren = 10;
	////for(int i=0; i<dim; i++){
	////	for(int j=0; j<dim;j++){
	////		/*if(j<dim-1){
	////			cmgr.connectors[cmgr.addConnector()].set(j+dim*i,j+1+dim*i,Vector3f(1,0,0),Vector3f(-1,0,0),stren);
	////		}
	////		if(i<dim-1){
	////			cmgr.connectors[cmgr.addConnector()].set(j+dim*i,j+dim*(i+1),Vector3f(0,0,1),Vector3f(0,0,-1),stren);
	////		}*/
	////		//if(i==0||j==0||i==dim-1||j==dim-1)
	////		cmgr.connectors[cmgr.addConnector()].set(i+dim*j,-1,Vector3f(0,1,0),cmgr.spheres[i+dim*j].x+Vector3f(0,10,0),stren);
	////	}
	////}
	//cmgr.addSphere(dim*dim*(dim/4));
	//for(int i=0; i<dim; i++){
	//	for(int j=0; j<dim; j++){
	//		for(int k=0; k<dim/4; k++){
	//			cmgr.spheres[i*dim*(dim/4)+j*(dim/4)+k].set(
	//				Vector3f(
	//					(rand()%100-50)*.005f + i - dim/3.f, 
	//					(k+5), 
	//					(rand()%100-50)*.005f + j - dim/3.f
	//				) * 3.f, 
	//				1-(rand()%100)/150.f, 
	//				Material(rand()%material_N)
	//			);
	//		}
	//	}
	//}
	//dim = 3;
	//cmgr.addFixCuboid(dim*dim);
	//for(int i=0; i<dim; i++){
	//	for(int j=0; j<dim; j++){
	//		cmgr.fixCuboids[i*dim+j].set(rand()%3+1,rand()%3+1,rand()%3+1, Vector3f(3*i-dim/3.f,3,3*j-dim/3.f), wood);
	//		cmgr.fixCuboids[i*dim+j].phi = createRotationQuaternion(i+j,Vector3f(i,j,i*j).getNormalized());
	//	}
	//}


	//glmgr.cam.pos = Vector3f(0,10,25);

	////KDTreeNode::set();

	//try{
	//	glmgr.enterMainLoop();
	//}
	//catch(cuda_exception& cu){
	//	clog << cu.what() << endl;
	//	system("pause");
	//}
	//catch(std::exception& e){
	//	clog << e.what() << endl;
	//	system("pause");
	//}
	//catch(char* s){
	//	clog << s << endl;
	//	system("pause");
	//}

	return 0;
}