#ifndef _TIMEWARPKERNEL_H_
#define _TIMEWARPKERNEL_H_

#include "Plane.h"
#include "Sphere.h"

#include "Queue.h"
#include "Heap.h"
#include "MessageControllSystem.h"

#include <cuda.h>
#include <cuda_runtime.h>


const u32 BSIZE = 1 << 8;

__global__ void removeFromMailboxes(
	Queue<Message, QL>* mailboxes,
	u32 sphereCount);

__global__ void deleteOlderThanGVT(
	Queue<Message, QL>* outputQs,
	Queue<Sphere, QL>* stateQs, 
	u32 sphereCount,
	f64 gvt);

__global__ void cpToStateQs(
	Sphere* spheres,
	Sphere* pendings,
	Queue<Sphere, QL>* stateQs,
	u32 sphereCount);

__global__ void cpFromStateQs(
	Sphere* spheres,
	Queue<Sphere, QL>* stateQs,
	u32 sphereCount);

__global__ void calculateLVT(
	Heap<Message, QL>* inputQs,
	Queue<Sphere, QL>* stateQs,
	f64* lvts,
	u32 sphereCount);


__global__ void detectCollisions(
	Plane* planes, u32 planeCount,
	Queue<Message, QL>* mailboxes,//nur fuers Senden
	Sphere* pendings,
	Queue<Message, QL>* outputQs,
	Queue<Sphere, QL>* stateQs, u32 sphereCount, 
	f64 tmin);

__global__ void receiveFromMailboxes( 
	Heap<Message, QL>* inputQs, 
	Queue<Message, QL>* mailboxes,//nur fuers Empfangen
	u32 sphereCount);

__global__ void handleNextMessages(
	Queue<Sphere, QL>* stateQs, 
	Heap<Message, QL>* inputQs,
	Queue<Message, QL>* outputQs,
	Sphere* pendings,
	Queue<Message, QL>* mailboxes,//nur fuers Senden
	u32 sphereCount);

__global__ void accelerate(
	Sphere* spheres,
	u32 sphereCount,
	f64 dt);

#endif