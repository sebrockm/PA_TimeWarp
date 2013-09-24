#include "FixCuboid.h"

#include "Geometry.h"
#include "GLManager.h"
#include "Cuboid.h"

#include <vector>

using std::vector;


FixCuboid::FixCuboid(const Vector3f& pos, const Quaternionf& phi, f32 a, f32 b, f32 c, Material k)
	:x(pos), phi(phi), la(a), lb(b), lc(c), k(k)
{}





Array<Vector3f, 8> FixCuboid::getCorners() const {
	Matrix4f m2w = getModel2World();

	Array<Vector3f, 8> cc = {
		m2w*Cuboid::CUBE_CORNERS()[0],
		m2w*Cuboid::CUBE_CORNERS()[1],
		m2w*Cuboid::CUBE_CORNERS()[2],
		m2w*Cuboid::CUBE_CORNERS()[3],
		m2w*Cuboid::CUBE_CORNERS()[4],
		m2w*Cuboid::CUBE_CORNERS()[5],
		m2w*Cuboid::CUBE_CORNERS()[6],
		m2w*Cuboid::CUBE_CORNERS()[7]
	};

	return cc;
}





void FixCuboid::set(f32 a, f32 b, f32 c, const Vector3f& center, Material k){
	la = a;
	lb = b;
	lc = c;
	x = center;
	this->k = k;
}

void FixCuboid::rotate(const Vector3f& dPhi){	
	if(dPhi){
		phi = createRotationQuaternion(dPhi.length(), dPhi.getNormalized()) * phi;
	}
}





//void createFixCuboid(const FixCuboid& cube){
//
//	Geometry<FixCuboid>* geo = new Geometry<FixCuboid>();
//	geo->model = &cube;
//
//	GLManager::instance().add(geo);
//}