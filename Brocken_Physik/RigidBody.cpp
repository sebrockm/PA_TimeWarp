#include "RigidBody.h"
#include "GLManager.h"
#include "Geometry.h"


void RigidBody::calculateProperties(vector<Particle>& particles){
	m = 0;
	barycenter = Vector3f();
	for(u32 i = particleOffset; i <= particleOffset+particleCount; ++i){
		m += particles[i].m;
		barycenter += particles[i].x * particles[i].m;
	}
	barycenter /= m; //Schwerpunkt
		
	//Schwerpunkt in Ursprung des Modelkoordinatensystems setzen
	for(u32 i = particleOffset; i <= particleOffset+particleCount; ++i){
		particles[i].x -= barycenter;
	}

	//Trägheitstensor
	theta = Matrix3f();
	for(auto i = particles.begin(); i!= particles.end(); ++i){
		Vector3f ri = i->x - barycenter;
		theta[0][0] += i->m * (ri[1]*ri[1]+ri[2]*ri[2]);
		theta[1][1] += i->m * (ri[0]*ri[0]+ri[2]*ri[2]);
		theta[2][2] += i->m * (ri[0]*ri[0]+ri[1]*ri[1]);
		theta[0][1] -= i->m * ri[0]*ri[1];
		theta[0][2] -= i->m * ri[0]*ri[2];
		theta[1][2] -= i->m * ri[1]*ri[2];
	}
	theta[1][0] = theta[0][1];
	theta[2][0] = theta[0][2];
	theta[2][1] = theta[1][2];

	//Inverser Trägheitstensor
	inverseTheta = Matrix3f();
	f32 det = theta[0][0]*(theta[1][1]*theta[2][2]-theta[1][2]*theta[1][2])
			- theta[0][1]*theta[0][1]*theta[2][2] + 2*theta[0][1]*theta[0][2]*theta[1][2]
			- theta[0][2]*theta[0][2]*theta[1][1];

	inverseTheta[0][0] = (theta[1][1]*theta[2][2]-theta[1][2]*theta[1][2])/det;
	inverseTheta[1][1] = (theta[0][0]*theta[2][2]-theta[0][2]*theta[0][2])/det;
	inverseTheta[2][2] = (theta[0][0]*theta[1][1]-theta[0][1]*theta[0][1])/det;
	inverseTheta[0][1] = inverseTheta[1][0] = (theta[0][2]*theta[2][1]-theta[0][1]*theta[2][2])/det;
	inverseTheta[0][2] = inverseTheta[2][0] = (theta[0][1]*theta[2][1]-theta[0][2]*theta[1][1])/det;
	inverseTheta[1][2] = inverseTheta[2][1] = (theta[0][2]*theta[0][1]-theta[0][0]*theta[1][2])/det;
}


//void createRigidBody(const RigidBody& body, const vector<Particle>& particles){
//	vector<GLfloat> fb;
//	fb.reserve(body.particleCount * (3+3+3) * (14+1)*(14+1));
//	vector<GLuint> ib;
//	ib.reserve(body.particleCount * 14*(2*(14+1)+1));
//
//	glPrimitiveRestartIndex(RESTART_INDEX);
//	glEnable(GL_PRIMITIVE_RESTART);
//    
//	for(u32 it = body.particleOffset; it < body.particleOffset+body.particleCount; ++it){
//		GLfloat dTheta = PI/14;
//		GLfloat dPhi = PI/7;
//		GLfloat theta = 0;
//		for(int j=0; j <= 14; ++j) {
//			GLfloat sinTheta = sin(theta);
//			GLfloat cosTheta = cos(theta);
//			GLfloat phi = 0;
//			for(int i=0; i <= 14; ++i) {
//				GLfloat sinPhi = sin(phi);
//				GLfloat cosPhi = cos(phi);
//                
//				// position
//				fb.push_back(particles[it].r*sinTheta*cosPhi + particles[it].x[0]);  
//				fb.push_back(particles[it].r*cosTheta + particles[it].x[1]);
//				fb.push_back(particles[it].r*sinTheta*sinPhi + particles[it].x[2]);
//                
//				// normal
//				fb.push_back(sinTheta*cosPhi);    
//				fb.push_back(cosTheta);
//				fb.push_back(sinTheta*sinPhi);
//
//				// color
//				fb.push_back((f32)(rand()%2));    
//				fb.push_back((f32)(rand()%2));
//				fb.push_back((f32)(rand()%2));
//                
//				phi += dPhi;
//			}
//			theta += dTheta;
//		}
//        
//		
//	}
//
//	for(u32 k=0; k<body.particleCount; k++){
//		for(int j=0; j < 14; ++j) {
//			for(int i=0; i <= 14; ++i) {
//				ib.push_back((14+1)*(14+1)*k + (j+1)*(14+1) + i);
//				ib.push_back((14+1)*(14+1)*k + j*(14+1) + i);
//			}
//			ib.push_back(RESTART_INDEX);
//		}
//	}
//        
//    Geometry<RigidBody>* geo = new Geometry<RigidBody>();
//	/*geo->setIndices(ib);
//	geo->setVertices(fb);*/
//	geo->model = &body;
//
//	GLManager::instance().add(geo);
//}