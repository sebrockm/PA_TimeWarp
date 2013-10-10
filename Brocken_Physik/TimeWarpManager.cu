#include "TimeWarpManager.h"

#include "MyExceptions.h"
#include "GLManager.h"

#include <thrust\reduce.h>
#include <thrust\functional.h>
#include <iostream>
#include <algorithm>
#include <boost\timer.hpp>

using namespace std;


TimeWarpManager::TimeWarpManager(u32 maxPlane, u32 maxSpheres)
	:MAX_PLANES(maxPlane), MAX_SPHERES(maxSpheres), MAX_CUBOIDS(0), 
	MAX_FIXCUBOIDS(0), planeCount(0), sphereCount(0), cuboidCount(0),
	MAX_LEAFINDICES(0), leafIndexCount(0), 
	MAX_CONNECTORS(0), connectorCount(0)
{
	cudaSetDeviceFlags(cudaDeviceMapHost);
	cudaDeviceSetCacheConfig(cudaFuncCachePreferL1);


	throwCudaError(cudaHostAlloc(&planes, MAX_PLANES*sizeof(Plane), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuPlanes, planes, 0));

	throwCudaError(cudaHostAlloc(&spheres, MAX_SPHERES*sizeof(Sphere), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuSpheres, spheres, 0));

	throwCudaError(cudaHostAlloc(&outputQs, MAX_SPHERES*sizeof(Queue<Message, QL>), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuOutputQs, outputQs, 0));

	throwCudaError(cudaHostAlloc(&stateQs, MAX_SPHERES*sizeof(Queue<Sphere, QL>), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuStateQs, stateQs, 0));

	throwCudaError(cudaHostAlloc(&lvts, MAX_SPHERES*sizeof(f32), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuLvts, lvts, 0));
	
	throwCudaError(cudaHostAlloc(&mailboxes, MAX_SPHERES*sizeof(Queue<Message, QL>), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuMailboxes, mailboxes, 0));

	throwCudaError(cudaHostAlloc(&pendings, MAX_SPHERES*sizeof(Sphere), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuPendings, pendings, 0));
	
	throwCudaError(cudaHostAlloc(&inputQs, MAX_SPHERES*sizeof(Heap<Message, QL>), 
		cudaHostAllocMapped));
	throwCudaError(cudaHostGetDevicePointer(&cuInputQs, inputQs, 0));

}


TimeWarpManager::~TimeWarpManager(){
	cudaFreeHost(planes);
	cudaFreeHost(spheres);
	cudaFreeHost(outputQs);
	cudaFreeHost(stateQs);
	cudaFreeHost(lvts);
	cudaFreeHost(mailboxes);
	cudaFreeHost(pendings);
	cudaFreeHost(inputQs);
}

u32 TimeWarpManager::addSphere(int n){
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

	
u32 TimeWarpManager::addPlane(int n){
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


void TimeWarpManager::calculateTime(f64 dt, f64 div){
	boost::timer timer;

	dt /= div;

	cpToStateQs<<<sphereCount/BSIZE+1, BSIZE>>>(cuSpheres, cuPendings, cuStateQs, sphereCount);
	throwCudaError(cudaDeviceSynchronize());

	f64 gvt = 0;

	while(gvt < dt){
		//initiale Kollisionen bestimmen und entsprechende Nachrichten verschicken
		detectCollisions<<<sphereCount/BSIZE+1, BSIZE>>>(cuPlanes, planeCount, cuMailboxes, cuPendings, cuOutputQs, cuStateQs, sphereCount, dt);
		throwCudaError(cudaDeviceSynchronize());

		/*if(!stateQs[0].back().r)
			cout << "bullshit" << endl;*/

		//Nachrichten in die inputQs stecken
		receiveFromMailboxes<<<sphereCount/BSIZE+1, BSIZE>>>(cuInputQs, cuMailboxes, sphereCount);
		throwCudaError(cudaDeviceSynchronize());
		removeFromMailboxes<<<sphereCount/BSIZE+1, BSIZE>>>(cuMailboxes, sphereCount);
		throwCudaError(cudaDeviceSynchronize());

		/*if(!stateQs[0].back().r)
			cout << "bullshit" << endl;*/

		//inputQs abarbeiten
		handleNextMessages<<<sphereCount/BSIZE+1, BSIZE>>>(cuStateQs, cuInputQs, cuOutputQs, cuPendings, cuMailboxes, sphereCount);
		throwCudaError(cudaDeviceSynchronize());

		/*if(!stateQs[0].back().r)
			cout << "bullshit" << endl;*/

		//verschickte antimesseges in die inputQs stecken
		receiveFromMailboxes<<<sphereCount/BSIZE+1, BSIZE>>>(cuInputQs, cuMailboxes, sphereCount);
		throwCudaError(cudaDeviceSynchronize());
		removeFromMailboxes<<<sphereCount/BSIZE+1, BSIZE>>>(cuMailboxes, sphereCount);
		throwCudaError(cudaDeviceSynchronize());

		/*if(!stateQs[0].back().r)
			cout << "bullshit" << endl;*/

		//rollbacks etc durchfuehren
		handleNextMessages<<<sphereCount/BSIZE+1, BSIZE>>>(cuStateQs, cuInputQs, cuOutputQs, cuPendings, cuMailboxes, sphereCount);
		throwCudaError(cudaDeviceSynchronize());

		/*if(!stateQs[0].back().r)
			cout << "bullshit" << endl;*/

		//mailbox leeren
		receiveFromMailboxes<<<sphereCount/BSIZE+1, BSIZE>>>(cuInputQs, cuMailboxes, sphereCount);
		throwCudaError(cudaDeviceSynchronize());
		removeFromMailboxes<<<sphereCount/BSIZE+1, BSIZE>>>(cuMailboxes, sphereCount);
		throwCudaError(cudaDeviceSynchronize());

		/*if(!stateQs[0].back().r)
			cout << "bullshit" << endl;*/

		//neue gvt berechnen
		calculateLVT<<<sphereCount/BSIZE+1, BSIZE>>>(cuInputQs, cuStateQs, cuLvts, sphereCount);
		throwCudaError(cudaDeviceSynchronize());
		gvt = thrust::reduce(lvts, lvts+sphereCount, 1000000., thrust::min<f64>);

		/*if(!stateQs[0].back().r)
			cout << "bullshit" << endl;*/
		
		//alte Sachen loeschen
		deleteOlderThanGVT<<<sphereCount/BSIZE+1, BSIZE>>>(cuOutputQs, cuStateQs, sphereCount, gvt);
		throwCudaError(cudaDeviceSynchronize());

		/*if(!stateQs[0].back().r)
			cout << "bullshit" << endl;*/

		//cout << "max Q length: " << max_element(stateQs, stateQs+sphereCount)->length() << endl;;
	}
	/*if(gvt > dt)
		cout << "gvt > dt" << endl;*/
	
	cpFromStateQs<<<sphereCount/BSIZE+1, BSIZE>>>(cuSpheres, cuStateQs, sphereCount);
	throwCudaError(cudaDeviceSynchronize());

	accelerate<<<sphereCount/BSIZE+1, BSIZE>>>(cuSpheres, sphereCount, gvt);
	throwCudaError(cudaDeviceSynchronize());

	cout << fixed << "Berechnungszeit fuer einen Frame: " << (int)(timer.elapsed()*1000) << "ms\t\r";
	//cout << "Kugel 0 pos: ("<<spheres[0].x[0]<<", "<<spheres[0].x[1]<<", "<<spheres[0].x[2]<<")\r";
}