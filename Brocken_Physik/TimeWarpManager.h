#ifndef _TIMEWARPMANAGER_H_
#define _TIMEWARPMANAGER_H_

#include "TimeWarpKernel.h"

#include "Cuboid.h"
#include "FixCuboid.h"
#include "Connector.h"
#include "KDTree.h"


class TimeWarpManager{
public:
	Queue<Message, QL> *outputQs, *cuOutputQs;
	Queue<Sphere, QL> *stateQs, *cuStateQs;
	f64 *lvts, *cuLvts;
	Queue<Message, QL> *mailboxes, *cuMailboxes;
	Sphere *pendings, *cuPendings;
	Heap<Message, QL> *inputQs, *cuInputQs;
	
	Sphere *cuSpheres, *spheres;
	const u32 MAX_SPHERES;
	u32 sphereCount;

	Plane *cuPlanes, *planes;
	const u32 MAX_PLANES;
	u32 planeCount;


//muell
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
//muell ende



public:
	TimeWarpManager(u32 maxPlane = 4, u32 maxSpheres = 1<<10);
	
	~TimeWarpManager();

	u32 addSphere(int n = 1);
	
	u32 addPlane(int n = 1);

	void calculateTime(f64 dt, f64 div = 1);

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
		return getOffset<Cuboid>();
	}

	template <>
	u32 getOffset<Vector3f>() const {
		return 0xffffffff;
	}
};


#endif