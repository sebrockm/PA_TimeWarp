#ifndef _MOVABLE_H_
#define _MOVABLE_H_

#include "Vector.h"
#include "Quaternion.h"
#include "material.h"
#include "cuda_macro.h"



class Movable{
public:
	Vector3d x; //Schwerpunkt
	Vector3d v; //Geschwindigkeit des Schwerpunkts

	//Rotationselemente
	Quaternionf phi; //Rotationsstatus
	Vector3f omega; //Winkelgschwindigkeit

	f32 m; //Masse

	Material k;

	f64 timestamp;
	int partner;

	CUDA_CALLABLE_MEMBER bool operator < (const Movable& other) const {
		return timestamp < other.timestamp;
	}

	CUDA_CALLABLE_MEMBER Movable(const Vector3f& pos = Vector3f(), const f32& m = 1)
	:x(pos),v(),/*a(0,-9.81f,0),*/m(m),phi(1,0,0,0),omega(),k(rubber),timestamp(0),partner(-1){}

	CUDA_CALLABLE_MEMBER void move(f64 dt){
		v[1] -= 9.81*dt;
		x += v*dt;

		phi = createRotationQuaternion(omega.length()*(f32)dt, omega.getNormalized()) * phi;
	}

	CUDA_CALLABLE_MEMBER void moveWithoutA(f64 dt){
		if(false/*partner <= -2 && v.lengthSqr() < EPSILON && omega.lengthSqr() < EPSILON*/){
			v = Vector3d();
			omega = Vector3d();
		}
		else{
			x += v*dt;
			//x[1] -= .5f*9.81f*dt*dt;
			v[1] -= 9.81*dt;
			phi = createRotationQuaternion(omega.length()*(f32)dt, omega.getNormalized()) * phi;
		}

		timestamp += dt;
	}

	CUDA_CALLABLE_MEMBER void moveOnlyA(f64 dt){
		v[1] -= 9.81*dt;
	}

	CUDA_CALLABLE_MEMBER bool isStill() const {
		return partner <= -2 && v.lengthSqr() < EPSILON && omega.lengthSqr() < EPSILON;
	}
};



#endif