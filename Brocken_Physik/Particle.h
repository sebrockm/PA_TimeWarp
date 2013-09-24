#ifndef _PARTICLE_H_
#define _PARTICLE_H_

#include "Vector.h"

struct Particle{
	Vector3f x; // in Modelkoordinaten
	f32 r;
	f32 m;
	u32 id;

	Particle(){}
	Particle(const Vector3f& x, f32 r, f32 m, u32 id)
		:x(x), r(r), m(m), id(id) {}
};




#endif