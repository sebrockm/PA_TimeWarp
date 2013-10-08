#include "Sphere.h"

#include "Quaternion.h"



void Sphere::set(const Vector3d& pos, const f32& r, Material k, const f32& m){
	x = pos;
	this->r = r;
	this->k = k;
	this->m = m;
	partner = -1;
	timestamp = 0;
}


