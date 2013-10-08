#ifndef _RIGIDBODY_H_
#define _RIGIDBODY_H_

#include "Vector.h"
#include "Matrix.h"
#include "Movable.h"
#include "Particle.h"
#include "cuda_macro.h"
#include "material.h"

#include <vector>

using namespace std;


class RigidBody : public Movable {

public:
	u32 particleOffset, particleCount;
	Matrix3f theta, inverseTheta;
	Vector3f barycenter; //Schwerpunkt in Modelkoordinaten
	Material k;

public:
	CUDA_CALLABLE_MEMBER RigidBody(u32 particleCount)
		:Movable(),	particleCount(particleCount), theta(), inverseTheta(), barycenter(){}

	CUDA_CALLABLE_MEMBER ~RigidBody(){}

	CUDA_CALLABLE_MEMBER void calculateProperties(vector<Particle>& particles);

	CUDA_CALLABLE_MEMBER Matrix4f getModel2World() const {
		return createTranslationMatrix((Vector3f)x) * phi.getMatrix4();
	}

	CUDA_CALLABLE_MEMBER Matrix4f getm2w() const {
		return Matrix4f(1,	0,	0,	(f32)x[0],
						0,	1,	0,	(f32)x[1],
						0,	0,	1,	(f32)x[2],
						0,	0,	0,	1		) * phi.getMatrix4();
	}
};


void createRigidBody(const RigidBody& body, const vector<Particle>& particles);



#endif