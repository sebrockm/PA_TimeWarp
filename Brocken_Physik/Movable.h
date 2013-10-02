#ifndef _MOVABLE_H_
#define _MOVABLE_H_

#include "Vector.h"
#include "Quaternion.h"
#include "material.h"
#include "cuda_macro.h"



class Movable{
public:
	Vector3f x; //Schwerpunkt
	Vector3f v; //Geschwindigkeit des Schwerpunkts

	//Rotationselemente
	Quaternionf phi; //Rotationsstatus
	Vector3f omega; //Winkelgschwindigkeit

	f32 m; //Masse

	Material k;

	f32 timestamp;
	int partner;

	bool operator < (const Movable& other) const {
		return timestamp < other.timestamp;
	}

	CUDA_CALLABLE_MEMBER Movable(const Vector3f& pos = Vector3f(), const f32& m = 1)
	:x(pos),v(),/*a(0,-9.81f,0),*/m(m),phi(1,0,0,0),omega(),k(rubber),timestamp(0),partner(-1){}

	CUDA_CALLABLE_MEMBER void move(f32 dt){
		v[1] -= 9.81f*dt;
		x += v*dt;

		phi = createRotationQuaternion(omega.length()*dt, omega.getNormalized()) * phi;
	}

	CUDA_CALLABLE_MEMBER void moveWithoutA(f32 dt){
		x += v*dt - .5f*9.81f*dt*dt;
		v[1] -= 9.81f*dt;

		phi = createRotationQuaternion(omega.length()*dt, omega.getNormalized()) * phi;

		timestamp += dt;
	}

	CUDA_CALLABLE_MEMBER void moveOnlyA(f32 dt){
		v[1] -= 9.81f*dt;
	}
};



#endif