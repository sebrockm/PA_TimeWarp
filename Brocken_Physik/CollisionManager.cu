#include "CollisionManager.h"

#include "GLManager.h"
#include "CollisionHandler.h"
#include "CollisionDetector.h"
#include "kernel.h"
#include "MyExceptions.h"
#include <cuda.h>
#include <cuda_runtime.h>


CollisionManager::CollisionManager(
	u32 maxPlane, u32 maxSpheres, u32 maxCuboids, u32 maxFixCuboids, 
	u32 maxLeafIndices, u32 maxConnectors)
	:MAX_PLANES(maxPlane), MAX_SPHERES(maxSpheres), MAX_CUBOIDS(maxCuboids), 
	MAX_FIXCUBOIDS(maxFixCuboids), planeCount(0), sphereCount(0), cuboidCount(0),
	MAX_LEAFINDICES(maxLeafIndices), leafIndexCount(0), 
	MAX_CONNECTORS(maxConnectors), connectorCount(0)
{
	cudaSetDeviceFlags(cudaDeviceMapHost);
	cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);


	throwCudaError(cudaHostAlloc(&planes, MAX_PLANES*sizeof(Plane), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuPlanes, planes, 0));

	throwCudaError(cudaHostAlloc(&spheres, MAX_SPHERES*sizeof(Sphere), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuSpheres, spheres, 0));

	throwCudaError(cudaHostAlloc(&cuboids, MAX_CUBOIDS*sizeof(Cuboid), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuCuboids, cuboids, 0));

	throwCudaError(cudaHostAlloc(&fixCuboids, MAX_FIXCUBOIDS*sizeof(FixCuboid), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuFixCuboids, fixCuboids, 0));

	throwCudaError(cudaHostAlloc(&connectors, MAX_CONNECTORS*sizeof(Connector), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuConnectors, connectors, 0));

	throwCudaError(cudaHostAlloc(&treeArray, MAX_NODES*sizeof(KDTreeNode), 
		/*cudaHostAllocWriteCombined | */cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuTreeArray, treeArray, 0));

	throwCudaError(cudaHostAlloc(&leafIndices, MAX_LEAFINDICES*sizeof(u32), 
		/*cudaHostAllocWriteCombined | */cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuLeafIndices, leafIndices, 0));

	throwCudaError(cudaHostAlloc(&dBuffer, (MAX_SPHERES+MAX_CUBOIDS)*3*sizeof(Vector3f), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&dSpherePos, dBuffer, 0));
	dSphereSpeed = dSpherePos + MAX_SPHERES;
	dSphereRot = dSphereSpeed + MAX_SPHERES;
	dCuboidPos = dSpherePos + 3*MAX_SPHERES;
	dCuboidSpeed = dSphereSpeed + 3*MAX_SPHERES;
	dCuboidRot = dSphereRot + 3*MAX_SPHERES;

	throwCudaError(cudaHostAlloc(&colBuffer, (MAX_SPHERES+MAX_CUBOIDS)*sizeof(u32), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&colCountSphere, colBuffer, 0));
	colCountCuboid = colCountSphere + MAX_SPHERES;


	for(u32 i=0; i<(MAX_SPHERES+MAX_CUBOIDS)*3; i++){
		dBuffer[i] = Vector3f();
	}
	for(u32 i=0; i<(MAX_SPHERES+MAX_CUBOIDS); i++){
		colBuffer[i] = 0;
	}

	//KDTree zeichnen
	treeArray[0] = KDTreeNode();
	GLManager::instance().add(treeArray[0]);
}

CollisionManager::~CollisionManager(){
	cudaFreeHost(planes);
	cudaFreeHost(spheres);
	cudaFreeHost(cuboids);
	cudaFreeHost(fixCuboids);
	cudaFreeHost(connectors);
	cudaFreeHost(treeArray);
	cudaFreeHost(leafIndices);
	cudaFreeHost(dBuffer);
	cudaFreeHost(colBuffer);
}

u32 CollisionManager::addSphere(int n){
	u32 begin = sphereCount;
	sphereCount += n;
	if(sphereCount >= MAX_SPHERES){
		sphereCount = MAX_SPHERES;
	}

	for(u32 i = begin; i < sphereCount; i++){
		spheres[i] = Sphere();
		GLManager::instance().add(spheres[i]);
	}
	return begin;
}

	
u32 CollisionManager::addPlane(int n){
	u32 begin = planeCount;
	planeCount += n;
	if(planeCount >= MAX_PLANES){
		planeCount = MAX_PLANES;
	}

	for(u32 i = begin; i < planeCount; i++){
		planes[i] = Plane();
		GLManager::instance().add(planes[i]);
	}
	return begin;
}


u32 CollisionManager::addCuboid(int n){
	u32 begin = cuboidCount;
	cuboidCount += n;
	if(cuboidCount >= MAX_CUBOIDS){
		cuboidCount = MAX_CUBOIDS;
	}

	for(u32 i = begin; i < cuboidCount; i++){
		cuboids[i] = Cuboid();
		GLManager::instance().add(cuboids[i]);
	}
	return begin;
}

//u32 CollisionManager::addRigidBody(u32 pCount, int n){
//	u32 begin = (u32)rigids.size();
//	for(int i = 0; i < n; i++){
//		rigids.push_back(RigidBody(pCount));
//		//GLManager::add(&rigids.back());
//		rigids.back().particleOffset = (u32)particles.size();
//		
//		//neue Partikel erzeugen mit passender id
//		particles.insert(particles.end(), pCount, Particle(Vector3f(), 0, 0, (u32)rigids.size()-1));
//	}
//	return begin;
//}

u32 CollisionManager::addFixCuboid(int n){
	u32 begin = fixCuboidCount;
	fixCuboidCount += n;
	if(fixCuboidCount >= MAX_FIXCUBOIDS){
		fixCuboidCount = MAX_FIXCUBOIDS;
	}

	for(u32 i = begin; i < fixCuboidCount; i++){
		fixCuboids[i] = FixCuboid();
		GLManager::instance().add(fixCuboids[i]);
	}
	return begin;
}


u32 CollisionManager::addConnector(int n){
	u32 begin = connectorCount;
	connectorCount += n;
	if(connectorCount >= MAX_CONNECTORS){
		connectorCount = MAX_CONNECTORS;
	}

	for(u32 i = begin; i < connectorCount; i++){
		connectors[i] = Connector();
		GLManager::instance().add(connectors[i]);
	}
	return begin;
}


void CollisionManager::calculateTime(f32 dt, f32 div){

	dt /= div;

	//std::cout<<treeArray[0]<<endl<<endl;

		
	KDTreeNode::set();


	sphereKernel<<<sphereCount/BLOCKSIZE+1, BLOCKSIZE>>>(
		cuPlanes, planeCount, 
		cuSpheres, sphereCount, 
		cuCuboids, cuboidCount,
		cuFixCuboids, fixCuboidCount,
		cuLeafIndices, leafIndexCount, 
		cuTreeArray, KDTreeNode::size,
		dSpherePos, dSphereSpeed, dSphereRot, colCountSphere);


	cuboidKernel<<<cuboidCount/BLOCKSIZE+1, BLOCKSIZE>>>(
		cuPlanes, planeCount, 
		cuSpheres, sphereCount, 
		cuCuboids, cuboidCount,
		cuFixCuboids, fixCuboidCount,
		cuLeafIndices, leafIndexCount, 
		cuTreeArray, KDTreeNode::size,
		dCuboidPos, dCuboidSpeed, dCuboidRot, colCountCuboid);


	throwCudaError(cudaDeviceSynchronize());


	moveKernel<<<sphereCount/BLOCKSIZE+1, BLOCKSIZE>>>(
		cuSpheres, sphereCount,
		dt,
		dSpherePos, dSphereSpeed, dSphereRot, colCountSphere);

	moveKernel<<<cuboidCount/BLOCKSIZE+1, BLOCKSIZE>>>(
		cuCuboids, cuboidCount,
		dt,
		dCuboidPos, dCuboidSpeed, dCuboidRot, colCountCuboid);


	throwCudaError(cudaDeviceSynchronize());


	connectorKernel<<<connectorCount/BLOCKSIZE+1, BLOCKSIZE>>>(
		cuSpheres, getOffset<Sphere>(), getOffset<Sphere>()+sphereCount,
		cuConnectors, connectorCount,
		dt);

	connectorKernel<<<connectorCount/BLOCKSIZE+1, BLOCKSIZE>>>(
		cuSpheres, getOffset<Sphere>(), getOffset<Sphere>()+sphereCount,
		getOffset<Vector3f>(),
		cuConnectors, connectorCount,
		dt);


	throwCudaError(cudaDeviceSynchronize());


	throwCudaError(cudaGetLastError());

}
