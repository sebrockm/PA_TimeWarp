#ifndef _PLANE_H_
#define _PLANE_H_


#include "Vector.h"
#include "material.h"
#include "Movable.h"
#include "LineSegment.h"
#include "cuda_macro.h"



/**
Ebenenklasse. Wird in Hesse-Normalform gespeichert.
*/
class Plane{
public:
	Vector3f n; // Normaleneinheitsvektor
	f32 d; // Abstand vom Ursprung
	Material k;

public:
	CUDA_CALLABLE_MEMBER Plane():n(0,1,0), d(0), k(rubber){}

	CUDA_CALLABLE_MEMBER Plane(const Vector3f& n, const Vector3f& x, Material k = rubber){
		set(n, x, k);
	}

	CUDA_CALLABLE_MEMBER ~Plane(){}

	CUDA_CALLABLE_MEMBER void set(const Vector3f& n, const Vector3f& x, Material k = rubber){
		this->n = n.getNormalized();
		this->k = k;
		this->d = this->n*x;
		if(this->d < 0){
			this->n = -this->n;
			d = -d;
		}
	}

	CUDA_CALLABLE_MEMBER Plane(const Vector3f& n, f32 d, Material k):n(n.getNormalized()), d(d), k(k){}

	
	CUDA_CALLABLE_MEMBER f32 distanceTo(const Vector3f& point) const {
		return fabs(n*point - d);
	}

	/**
	Identisch mit distanceTo(point), wenn der Punkt auf der Seite der
	Ebene liegt, in die auch der Normaleneinheitsvektor der Ebene zeigt.
	Identisch mit -distanceTo(point) sonst.
	@param point Punkt
	@return orientierter Abstand des Punktes zur Ebene
	*/
	CUDA_CALLABLE_MEMBER f32 orientatedDistanceTo(const Vector3f& point) const {
		return n*point - d;
	}

	template <class T>
	CUDA_CALLABLE_MEMBER bool includes(const Vector<T, 3>& point) const {
		return fEqual(n*point-d, (T)0);
	}

	CUDA_CALLABLE_MEMBER Matrix4f getModel2World() const {
		const Vector3f y(0,1,0);
		return createTranslationMatrix(d*n) * 
			createRotationQuaternion(acos(y*n),crossProduct(y,n).getNormalized()).getMatrix4() * 
			createScalarMatrix(500,0,500);
	}
};


//void createPlane(const Plane& dest);





#endif