#include "Cuboid.h"
#include "GLManager.h"
#include "Geometry.h"


void Cuboid::set(f32 a, f32 b, f32 c, const Vector3f& center, f32 m, Material k){
	la = a;
	lb = b;
	lc = c;
	x = center;
	this->m = m;
	this->k = k;
	theta[0][0] = m/12*(b*b+c*c);
	theta[1][1] = m/12*(a*a+c*c);
	theta[2][2] = m/12*(a*a+b*b);
}






void Cuboid::rotate(const Quaternionf& dPhi){
	phi = dPhi * phi;
}












//void createCuboid(const Cuboid& cube){
//
//	Geometry<Cuboid>* geo = new Geometry<Cuboid>();
//	geo->model = &cube;
//
//	GLManager::instance().add(geo);
//}