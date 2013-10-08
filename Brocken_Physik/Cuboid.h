#ifndef _CUBOID_H_
#define _CUBOID_H_


#include "Vector.h"
#include "Matrix.h"
#include "material.h"
#include "Movable.h"
#include "Quaternion.h"
#include "cuda_macro.h"

#include <vector>

using namespace std;


/**
	  3--------2
	 /:       /|
	7--------6 | lb
	| 0 - - -|-1
	|,       |/ lc
	4--------5
	    la
*/
class Cuboid : public Movable{
public:
	f32 la, lb, lc; 
	
	Matrix3f theta; //Trägheitstensor

public:
	CUDA_CALLABLE_MEMBER Cuboid():Movable(){};

	CUDA_CALLABLE_MEMBER ~Cuboid(){}

	CUDA_CALLABLE_MEMBER void set(f32 a, f32 b, f32 c, const Vector3f& center = Vector3f(), f32 m = 1, Material k = steel);

	CUDA_CALLABLE_MEMBER Array<Vector3f, 8> getCorners() const {
		Array<Vector3f, 8> c;
		Matrix4f m2w = getModel2World();
		for(int i=0; i<8; i++)
			c[i] = m2w * Cuboid::CUBE_CORNERS()[i];

		return c;
	}

	CUDA_CALLABLE_MEMBER Array<Vector3f, 3> getMainAxes() const {
		const Array<Vector3f, 3> axes = {
			Vector3f(la, 0, 0),
			Vector3f(0, lb, 0),
			Vector3f(0, 0, lc)	};
	
		Array<Vector3f, 3> res = {
			phi * axes[0],
			phi * axes[1],
			phi * axes[2]	};

		return res;
	}

	CUDA_CALLABLE_MEMBER void rotate(const Quaternionf& dPhi);

	CUDA_CALLABLE_MEMBER Matrix4f getModel2World() const {
		return createTranslationMatrix((Vector3f)x) * phi.getMatrix4() * createScalarMatrix(la, lb, lc);
	}

	CUDA_CALLABLE_MEMBER Matrix4f getWorld2Model() const {
		return createScalarMatrix(1/la, 1/lb, 1/lc) * phi.getConjugate().getMatrix4() *
				createTranslationMatrix(-(Vector3f)x);
	}

	CUDA_CALLABLE_MEMBER Matrix3f getInertia() const {
		return phi.getMatrix3() * theta * phi.getConjugate().getMatrix3();
	}

	CUDA_CALLABLE_MEMBER Matrix3f getInverseInertia() const {
		return phi.getMatrix3() * 
			Matrix3f(1/theta[0][0], 0, 0, 0, 1/theta[1][1], 0, 0, 0, 1/theta[2][2]) * 
			phi.getConjugate().getMatrix3();
	}

	CUDA_CALLABLE_MEMBER f32 getProjectionOnNormal(const Vector3f& n) const {
		auto axes = getMainAxes();	
		return abs(axes[0]*n) + abs(axes[1]*n) + abs(axes[2]*n);
	}

	CUDA_CONST_VAR static Array<Vector4f, 8> CUBE_CORNERS(){
#ifndef __CUDA_ARCH__
		static
#endif
		const Array<Vector4f, 8> CORNERS = {
				Vector4f(-.5, -.5, -.5, 1),
				Vector4f(.5, -.5, -.5, 1),
				Vector4f(.5, .5, -.5, 1),
				Vector4f(-.5, .5, -.5, 1),
				Vector4f(-.5, -.5, .5, 1),
				Vector4f(.5, -.5, .5, 1),
				Vector4f(.5, .5, .5, 1),
				Vector4f(-.5, .5, .5, 1)	};

		return CORNERS;
	}
};



//void createCuboid(const Cuboid& cube);



#endif