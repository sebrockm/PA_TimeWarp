#ifndef _FIXCUBOID_H_
#define _FIXCUBOID_H_

#include "Vector.h"
#include "types.h"
#include "Quaternion.h"
#include "material.h"
#include "Matrix.h"
#include "cuda_macro.h"


class FixCuboid{
public:
	f32 la, lb, lc;
	Vector3f x;
	Quaternionf phi;
	Material k;

public:
	CUDA_CALLABLE_MEMBER FixCuboid(const Vector3f& pos = Vector3f(), const Quaternionf& phi = Quaternionf(1,0,0,0), f32 a=1, f32 b=1, f32 c=1, Material k=steel);

	CUDA_CALLABLE_MEMBER Matrix4f getModel2World() const {
		return createTranslationMatrix(x) * phi.getMatrix4() * createScalarMatrix(la, lb, lc);
	}

	CUDA_CALLABLE_MEMBER Matrix4f getWorld2Model() const {
		return createScalarMatrix(1/la, 1/lb, 1/lc) * phi.getConjugate().getMatrix4() * createTranslationMatrix(-x);
	}

	CUDA_CALLABLE_MEMBER Array<Vector3f, 8> getCorners() const;

	CUDA_CALLABLE_MEMBER Array<Vector3f, 3> getMainAxes() const {
		const Array<Vector3f, 3> axes = {
			Vector3f(la, 0, 0),
			Vector3f(0, lb, 0),
			Vector3f(0, 0, lc)
		};
	
		Array<Vector3f, 3> res = {
			phi * axes[0],
			phi * axes[1],
			phi * axes[2]
		};

		return res;
	}

	CUDA_CALLABLE_MEMBER f32 getProjectionOnNormal(const Vector3f& n) const {
		auto axes = getMainAxes();
		return abs(axes[0]*n) + abs(axes[1]*n) + abs(axes[2]*n);
	}

	CUDA_CALLABLE_MEMBER void set(f32 a, f32 b, f32 c, const Vector3f& center = Vector3f(), Material k = steel);

	CUDA_CALLABLE_MEMBER void rotate(const Vector3f& dPhi);
};



//void createFixCuboid(const FixCuboid& cube);



#endif