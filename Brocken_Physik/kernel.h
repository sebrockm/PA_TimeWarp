#ifndef _KERNEL_H_
#define _KERNEL_H_

#include "Plane.h"
#include "Sphere.h"
#include "Cuboid.h"
#include "FixCuboid.h"
#include "KDTree.h"
#include "Connector.h"

#include <cuda.h>
#include <cuda_runtime.h>

const u32 BLOCKSIZE = 1 << 8;


__global__ void cuboidKernel(Plane* planes, u32 planeCount, 
	Sphere* spheres, u32 sphereCount, 
	Cuboid* cuboids, u32 cuboidCount,
	FixCuboid* fixCuboids, u32 fixCuboidCount,
	u32* leafIndices, u32 leafIndexCount,
	KDTreeNode* treeArray, u32 treeArrayCount,
	Vector3f* dPos, Vector3f* dSpeed, Vector3f* dRot, u32* colCount);


__global__ void sphereKernel(Plane* planes, u32 planeCount, 
	Sphere* spheres, u32 sphereCount, 
	Cuboid* cuboids, u32 cuboidCount,
	FixCuboid* fixCuboids, u32 fixCuboidCount,
	u32* leafIndices, u32 leafIndexCount,
	KDTreeNode* treeArray, u32 treeArrayCount,
	Vector3f* dPos, Vector3f* dSpeed, Vector3f* dRot, u32* colCount);


__global__ void moveKernel(Sphere* spheres, u32 sphereCount, f32 dt,
	Vector3f* dPos, Vector3f* dSpeed, Vector3f* dRot, u32* colCount);


__global__ void moveKernel(Cuboid* cuboids, u32 cuboidCount, f32 dt,
	Vector3f* dPos, Vector3f* dSpeed, Vector3f* dRot, u32* colCount);


__global__ void connectorKernel(	Sphere* spheres, u32 sphereOffset, u32 sphereCount,
									Connector* connectors, u32 connectorCount,
									f32 dt);

__global__ void connectorKernel(	Sphere* spheres, u32 sphereOffset, u32 sphereCount,
									u32 pointOffset,
									Connector* connectors, u32 connectorCount,
									f32 dt);


__global__ void connectorKernel(	Cuboid* cuboids, u32 cuboidOffset, u32 cuboidCount,
									Connector* connectors, u32 connectorCount,
									f32 dt);

__global__ void connectorKernel(	Cuboid* cuboids, u32 cuboidOffset, u32 cuboidCount,
									u32 pointOffset,
									Connector* connectors, u32 connectorCount,
									f32 dt);

__global__ void testKernel(	Plane* planes, u32 planeCount,
							Sphere* spheres, u32 sphereCount,
							f32 dt);

#endif