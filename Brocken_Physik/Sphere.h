#ifndef _SPHERE_H_
#define _SPHERE_H_

#include "Vector.h"
#include "Matrix.h"
#include "material.h"
#include "Movable.h"
#include "cuda_macro.h"


using namespace std;


class Sphere : public Movable{
public:
	f32 r;//Radius

public:
	CUDA_CALLABLE_MEMBER Sphere()
		:Movable(), r(1) {}

	CUDA_CALLABLE_MEMBER ~Sphere(){}

	CUDA_CALLABLE_MEMBER void set(const Vector3d& pos, const f32& r, Material k = rubber, const f32& m = 1);

	CUDA_CALLABLE_MEMBER Matrix4f getModel2World() const {
		return createTranslationMatrix((Vector3f)x) * phi.getMatrix4() * createScalarMatrix((f32)r,(f32)r,(f32)r);
	}

	CUDA_CALLABLE_MEMBER void moveWithoutA(f64 dt){
		Movable::moveWithoutA(dt);

		//der ultimative Hack:
		x[1] = max((f64)r + EPSILON, x[1]);
	}
};




#endif