#ifndef _COLLISIONMANAGER_H_
#define _COLLISIONMANAGER_H_


#include "Sphere.h"
#include "Plane.h"
#include "Cuboid.h"
#include "Connector.h"
#include "Particle.h"
#include "RigidBody.h"
#include "FixCuboid.h"
#include "KDTree.h"

#include <vector>


using std::vector;


class CollisionManager{
public:

	Plane *cuPlanes, *planes;
	const u32 MAX_PLANES;
	u32 planeCount;

	Sphere *cuSpheres, *spheres;
	const u32 MAX_SPHERES;
	u32 sphereCount;

	Cuboid *cuCuboids, *cuboids;
	const u32 MAX_CUBOIDS;
	u32 cuboidCount;

	FixCuboid *cuFixCuboids, *fixCuboids;
	const u32 MAX_FIXCUBOIDS;
	u32 fixCuboidCount;

	Connector *cuConnectors, *connectors;
	const u32 MAX_CONNECTORS;
	u32 connectorCount;

	KDTreeNode *cuTreeArray, *treeArray;

	u32 *leafIndices, *cuLeafIndices;
	const u32 MAX_LEAFINDICES;
	u32 leafIndexCount;

	Vector3f* dBuffer;
	Vector3f *dSpherePos, *dSphereSpeed, *dSphereRot;
	Vector3f *dCuboidPos, *dCuboidSpeed, *dCuboidRot;
	u32 *colBuffer, *colCountSphere, *colCountCuboid;
	
public:
	CollisionManager(u32 maxPlane = 16, 
		u32 maxSpheres = 1<<14, u32 maxCuboids = 1<<10, u32 maxFixCuboids = 1<<10, 
		u32 maxLeafIndices = 1<<19, u32 maxConnectors = 3*(1<<15));

	~CollisionManager();

	u32 addSphere(int n = 1);
	
	u32 addPlane(int n = 1);

	u32 addCuboid(int n = 1);

	//u32 addRigidBody(u32 pCount, int n = 1);

	u32 addFixCuboid(int n = 1);

	u32 addConnector(int n = 1);


	void calculateTime(f32 dt, f32 div = 1);


	template <class T>
	u32 getOffset() const {
		throw std::exception("ungültige Klasse");
	}

	template <>
	u32 getOffset<Sphere>() const {
		return 0;
	}

	template <>
	u32 getOffset<Cuboid>() const {
		return getOffset<Sphere>() + sphereCount;
	}

	template <>
	u32 getOffset<FixCuboid>() const {
		return getOffset<Cuboid>() + cuboidCount;
	}

	template <>
	u32 getOffset<Vector3f>() const {
		return 0xffffffff;
	}
	
};



#endif