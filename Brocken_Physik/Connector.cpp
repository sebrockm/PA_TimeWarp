#include "Connector.h"
#include "GLManager.h"




void Connector::set(u32 id1, u32 id2, const Vector3f& p1, const Vector3f& p2, f32 k, f32 l){

	this->id1 = id1;
	this->id2 = id2;
	this->p1 = Vector4f(p1,1);
	this->p2 = Vector4f(p2,1);
	this->k = k;
	this->l = l;

	if(l < 0){
		Vector3f w1, w2;
		auto& cmgr = GLManager::instance().cmgr;
		
		if(id1 >= cmgr.getOffset<Vector3f>()){
			w1 = this->p1;
		}
		else if(id1 >= cmgr.getOffset<FixCuboid>()){
			w1 = cmgr.fixCuboids[id1-cmgr.getOffset<FixCuboid>()].getModel2World() * this->p1;
		}
		else if(id1 >= cmgr.getOffset<Cuboid>()){
			w1 = cmgr.cuboids[id1-cmgr.getOffset<Cuboid>()].getModel2World() * this->p1;
		}
		else if(id1 >= cmgr.getOffset<Sphere>()){
			w1 = cmgr.spheres[id1-cmgr.getOffset<Sphere>()].getModel2World() * this->p1;
		}


		if(id2 >= cmgr.getOffset<Vector3f>()){
			w2 = this->p2;
		}
		else if(id2 >= cmgr.getOffset<FixCuboid>()){
			w2 = cmgr.fixCuboids[id2-cmgr.getOffset<FixCuboid>()].getModel2World() * this->p2;
		}
		else if(id2 >= cmgr.getOffset<Cuboid>()){
			w2 = cmgr.cuboids[id2-cmgr.getOffset<Cuboid>()].getModel2World() * this->p2;
		}
		else if(id2 >= cmgr.getOffset<Sphere>()){
			w2 = cmgr.spheres[id2-cmgr.getOffset<Sphere>()].getModel2World() * this->p2;
		}

		this->l = (w1-w2).length();
	}
}

