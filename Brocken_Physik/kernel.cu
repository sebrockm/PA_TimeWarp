#include "kernel.h"

#include "CollisionDetector.h"
#include "CollisionHandler.h"

#include "Queue.h"
#include "Heap.h"
#include "MessageControllSystem.h"


__global__ void handleNextMessage(
	Queue<Sphere, QL>* stateQs, 
	Heap<Message, 20>* inputQs, 
	//Queue<Message, 20>* doneQs,
	Queue<Message, 20>* outputQs,
	Queue<Message, 20>* mailboxes,//nur fuers Senden
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	f32 lvt = stateQs[id].back().timestamp;

	if(id >= sphereCount)
		return;

	Message msg;
	msg.type = Message::mull;

	//naechste Message suchen, die sich nicht irgendwie aufloest
	while(inputQs[id].length() > 0){
		msg = inputQs[id].peek();
		if(inputQs[id].length() > 0){
			if(msg.checkPair(inputQs[id].top())){
				inputQs[id].peek();
				if(inputQs[id].length() > 0){
					msg = inputQs[id].peek();
				}
				else{
					msg.type = Message::mull;
				}
			}
			else if(msg == inputQs[id].top()){
				inputQs[id].peek();
			}
			else{
				break;
			}
		}
	}

	switch(msg.type)
	{
	case Message::event:
	{
		if(msg.timestamp < lvt){
			lvt = msg.timestamp;
			
			//rollback
			stateQs[id].deleteAllGreaterThan(lvt);
			//doneQs[id].deleteAllGreaterThan(lvt);
			MessageControllSystem mcs(inputQs, mailboxes, sphereCount);

			u32 pos = outputQs[id].searchFirstBefore(msg) + 1;
			for(int i = pos; i < outputQs[id].length(); i++){
				mcs.send(outputQs[id][i]);
			}
			outputQs[id].deleteAllAfterEq(pos);

		}
		
		//neuer state		
		Sphere neu = stateQs[id].back();
		neu.x += neu.v * (msg.timestamp-neu.timestamp);
		neu.phi = createRotationQuaternion(neu.omega.length()*(msg.timestamp-neu.timestamp), neu.omega.getNormalized()) * neu.phi;
		neu.timestamp = msg.timestamp - EPSILON; //FIXME ist das so sinnvoll?
		neu.partner = msg.src;
		stateQs[id].insert(neu); //Zustand unmittelbar vor der Kollision

		neu.timestamp = msg.timestamp;
		int sid = stateQs[msg.src].searchFirstBefore(neu);
		if(sid < 0){
			printf("Kollisionspartner ist weg :(\n");
			break;
		}

		Sphere other = stateQs[msg.src][sid];

		CollisionHandler ch;
		ch(neu, other);//neu ist jetzt kollidiert
		stateQs[id].insert(neu); //Zustand unmittelbar nach der Kollision

		break;
	}

	case Message::antievent:
	{
		int sid = stateQs[id].searchFirstBefore(msg.timestamp) + 1;
		while(stateQs[id][sid].timestamp == msg.timestamp){//suche state der antimessage
			if(stateQs[id][sid].partner == msg.src && stateQs[id][sid+1].timestamp == msg.timestamp){//swape gefundenen state mit gleichen timestamps darueber
				Sphere tmp = stateQs[id][sid];
				stateQs[id][sid] = stateQs[id][sid+1];
				stateQs[id][sid+1] = tmp;
			}
			sid++;
		}
		sid--;

		if(stateQs[id][sid].timestamp == msg.timestamp && stateQs[id][sid].partner == msg.src){
			stateQs[id].deleteAllAfterEq(sid);

			MessageControllSystem mcs(inputQs, mailboxes, sphereCount);

			u32 pos = outputQs[id].searchFirstBefore(msg) + 1;
			for(int i = pos; i < outputQs[id].length(); i++){
				mcs.send(outputQs[id][i]);
			}
			outputQs[id].deleteAllAfterEq(pos);
		}
		else{
			//printf("state zum antievent nicht vorhanden\n");
		}
		break;
	}

	case Message::eventAck:break;
	case Message::antieventAck:break;
	}

	//doneQs[id].insert(msg);
}


__global__ void receiveFromMailboxes( 
	Heap<Message, 20>* inputQs, 
	Queue<Message, 20>* mailboxes,//nur fuers Empfangen
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if(id >= sphereCount)
		return;

	MessageControllSystem mcs(inputQs, mailboxes, sphereCount);

	mcs.recv();
}



__global__ void detectCollisions(
	Plane* planes, u32 planeCount,
	Queue<Message, 20>* mailboxes,//nur fuers Senden
	Queue<Sphere, QL>* stateQs, u32 sphereCount, 
	f32 tmin)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	CollisionDetector cd;
	f32 t;
	f32 lvt = stateQs[id].back().timestamp;

	u32 partner;
	bool nextIsPlane = false;
	for(u32 i = 0; i < planeCount; i++){
		if(cd(stateQs[id].back(), planes[i], t)){
			if(t < tmin){
				partner = i;
				tmin = t;
				nextIsPlane = true;
			}
		}
	}

	for(u32 i = 0; i < sphereCount; i++){
		for(u32 j = stateQs[i].searchFirstBefore(stateQs[id].back()); j < stateQs[i].length(); j++){
			if(cd(stateQs[id].back(), stateQs[i][j], t)){
				if(t < tmin){
					partner = i;
					tmin = t;
					nextIsPlane = false;
				}
			}
		}
	}


	//TODO senden
}













__global__ void cuboidKernel(Plane* planes, u32 planeCount, 
	Sphere* spheres, u32 sphereCount, 
	Cuboid* cuboids, u32 cuboidCount,
	FixCuboid* fixCuboids, u32 fixCuboidCount,
	u32* leafIndices, u32 leafIndexCount,
	KDTreeNode* treeArray, u32 treeArrayCount, 
	Vector3f* dPos, Vector3f* dSpeed, Vector3f* dRot, u32* colCount)
{
	int global_id = threadIdx.x + blockIdx.x*blockDim.x;
	//int local_id = threadIdx.x;
		
	CollisionDetector cd;
	CollisionHandler ch;
	Vector3f n, pt;

	
	if(global_id < cuboidCount){

		//Kollision mit Ebenen
		for(u32 i=0; i<planeCount; i++){
			if(cd(cuboids[global_id], planes[i], pt)){
				ch(cuboids[global_id], planes[i], pt, 
					dPos[global_id], dSpeed[global_id], dRot[global_id]);
				colCount[global_id]++;
			}
		}

		//passende Bl�tter im KDTree suchen
		u32 kdId = 0;
		u32 needRight = 0;//steht rechter Sohn noch aus?
		u32 both = 0;//beide S�hne besucht?
		while(true){

			if(treeArray[kdId].axNo == noAxis){//Blatt
				for(u32 i = treeArray[kdId].indexStart; 
					i < treeArray[kdId].indexEnd; 
					i++)
				{
					if(leafIndices[i] < sphereCount){
						if(cd(cuboids[global_id], spheres[leafIndices[i]], pt, n)){
							ch(cuboids[global_id], spheres[leafIndices[i]], 
								pt, n, dPos[global_id], dSpeed[global_id], dRot[global_id]);
							colCount[global_id]++;
						}
					}
					else if(leafIndices[i] < sphereCount + cuboidCount){
						u32 cid = leafIndices[i] - sphereCount;
						if(global_id != cid &&
							cd(cuboids[global_id], cuboids[cid], pt, n))
						{
							ch(cuboids[global_id], cuboids[cid], 
								pt, n, dPos[global_id], dSpeed[global_id], dRot[global_id]);
							colCount[global_id]++;
						}
					}
					else if(leafIndices[i] < sphereCount + cuboidCount + fixCuboidCount){
						u32 cid = leafIndices[i] - sphereCount - cuboidCount;
						if(cd(cuboids[global_id], fixCuboids[cid], pt, n)){
							ch(cuboids[global_id], fixCuboids[cid], 
								pt, n, dPos[global_id], dSpeed[global_id], dRot[global_id]);
							colCount[global_id]++;
						}
					}
				}

				//if(kdId == 0)//Baum komplett durchsucht
				//	break;

				// eine Ebene nach oben
				if(!(needRight >>= 1))
					break;

				both >>= 1;
				both |= 1;
				kdId = treeArray[kdId].father;
			}
			else{//kein Blatt, weiter absteigen
				if(needRight & 1){//linker Sohn ist schon durchsucht,
					kdId = treeArray[kdId].right;//rechter Sohn als n�chstes
					needRight--;
				
					//N�chste Baumebene
					both <<= 1;
					needRight <<= 1;
				}
				else if(!(both & 1)){//erster Besuch dieses Knotens
					f32 d = cuboids[global_id].x[treeArray[kdId].axNo] 
						- treeArray[kdId].axis;

					Vector3f n;
					n[treeArray[kdId].axNo] = 1;
					f32 r = cuboids[global_id].getProjectionOnNormal(n)/2;

					if(d < -r){
						kdId = treeArray[kdId].left;//linker Sohn als n�chstes
					}
					else if(d > r){
						kdId = treeArray[kdId].right;//rechter Sohn als n�chstes
					}
					else{//schneidet die Ebene
						both |= 1;
						kdId = treeArray[kdId].left;//linken Sohn als n�chstes und
						needRight |= 1;//merken, dass rechter Sohn noch aussteht
					}
				
					//N�chste Baumebene
					both <<= 1;
					needRight <<= 1;
				}
				else{ //Knoten komplett abgearbeitet -> eine Ebene rauf
					//if(kdId == 0)//Baum komplett durchsucht
					//	break;

					if(!(needRight >>= 1))
						break;

					both >>= 1;
					both |= 1;
					kdId = treeArray[kdId].father;
				}
			}
		}

		/*__syncthreads();

		if(colCount > 0){
			cuboids[global_id].x += dPos[global_id];
			cuboids[global_id].v += dSpeed[global_id]/colCount;
			cuboids[global_id].omega += dRot[global_id]/colCount;
		}
		
		cuboids[global_id].move(dt);*/
	}
}





__global__ void sphereKernel(Plane* planes, u32 planeCount, 
	Sphere* spheres, u32 sphereCount, 
	Cuboid* cuboids, u32 cuboidCount,
	FixCuboid* fixCuboids, u32 fixCuboidCount,
	u32* leafIndices, u32 leafIndexCount,
	KDTreeNode* treeArray, u32 treeArrayCount, 
	Vector3f* dPos, Vector3f* dSpeed, Vector3f* dRot, u32* colCount)
{
	int global_id = threadIdx.x + blockIdx.x*blockDim.x;
	//int local_id = threadIdx.x;
		
	CollisionDetector cd;
	CollisionHandler ch;
	Vector3f n, pt;

	
	if(global_id < sphereCount){

		//Kollision mit Ebenen
		for(u32 i=0; i<planeCount; i++){
			if(cd(spheres[global_id], planes[i], pt)){
				ch(spheres[global_id], planes[i], pt, 
					dPos[global_id], dSpeed[global_id], dRot[global_id]);
				colCount[global_id]++;
			}
		}

		//passende Bl�tter im KDTree suchen
		u32 kdId = 0;
		u32 needRight = 0;//steht rechter Sohn noch aus?
		u32 both = 0;//beide S�hne besucht?
		//u32 counter = 0;
		while(true){
			//if(counter++ > 10) break;
			if(treeArray[kdId].axNo == noAxis){//Blatt
				for(u32 i = treeArray[kdId].indexStart; 
					i < treeArray[kdId].indexEnd; 
					i++)
				{
					if(leafIndices[i] < sphereCount){
						if(global_id != leafIndices[i] &&
							cd(spheres[global_id], spheres[leafIndices[i]], pt, n))
						{
							ch(spheres[global_id], spheres[leafIndices[i]], 
								pt, n, dPos[global_id], dSpeed[global_id], dRot[global_id]);
							colCount[global_id]++;
						}
					}
					else if(leafIndices[i] < sphereCount + cuboidCount){
						u32 cid = leafIndices[i] - sphereCount;
						if(cd(spheres[global_id], cuboids[cid], pt, n)){
							ch(spheres[global_id], cuboids[cid], 
								pt, n, dPos[global_id], dSpeed[global_id], dRot[global_id]);
							colCount[global_id]++;
						}
					}
					else if(leafIndices[i] < sphereCount + cuboidCount + fixCuboidCount){
						u32 cid = leafIndices[i] - sphereCount - cuboidCount;
						if(cd(spheres[global_id], fixCuboids[cid], pt, n)){
							ch(spheres[global_id], fixCuboids[cid], 
								pt, n, dPos[global_id], dSpeed[global_id], dRot[global_id]);
							colCount[global_id]++;
						}
					}
				}

				//if(kdId == 0)//Baum komplett durchsucht
				//	break;

				// eine Ebene nach oben
				if(!(needRight >>= 1))
					break;

				both >>= 1;
				both |= 1;
				kdId = treeArray[kdId].father;
			}
			else{//kein Blatt, weiter absteigen
				if(needRight & 1){//linker Sohn ist schon durchsucht,
					kdId = treeArray[kdId].right;//rechter Sohn als n�chstes
					needRight--;
				
					//N�chste Baumebene
					both <<= 1;
					needRight <<= 1;
				}
				else if(!(both & 1)){//erster Besuch dieses Knotens
					f32 d = spheres[global_id].x[treeArray[kdId].axNo] 
						- treeArray[kdId].axis;

					if(d < -spheres[global_id].r){
						kdId = treeArray[kdId].left;//linker Sohn als n�chstes
					}
					else if(d > spheres[global_id].r){
						kdId = treeArray[kdId].right;//rechter Sohn als n�chstes
					}
					else{//schneidet die Ebene
						both |= 1;
						kdId = treeArray[kdId].left;//linken Sohn als n�chstes und
						needRight |= 1;//merken, dass rechter Sohn noch aussteht
					}
				
					//N�chste Baumebene
					both <<= 1;
					needRight <<= 1;
				}
				else{ //Knoten komplett abgearbeitet -> eine Ebene rauf
					//if(kdId == 0)//Baum komplett durchsucht
					//	break;

					if(!(needRight >>= 1))
						break;

					both >>= 1;
					both |= 1;
					kdId = treeArray[kdId].father;
				}
			}
		}

		//__syncthreads();

		////printf("Kugel%d\tGeschwindigkeit: %f \n", global_id, spheres[global_id].v[1]);

		//if(colCount > 0){
		//	spheres[global_id].x += dPos;
		//	spheres[global_id].v += dSpeed/colCount;
		//	spheres[global_id].omega += dRot/colCount;
		//}
		//
		//spheres[global_id].move(dt);
	}
}


__global__ void moveKernel(Sphere* spheres, u32 sphereCount, f32 dt,
	Vector3f* dPos, Vector3f* dSpeed, Vector3f* dRot, u32* colCount)
{
	u32 global_id = threadIdx.x + blockIdx.x*blockDim.x;

	if(global_id < sphereCount){
		if(colCount[global_id] > 0){
			spheres[global_id].x += dPos[global_id];
			spheres[global_id].v += dSpeed[global_id]/colCount[global_id];
			spheres[global_id].omega += dRot[global_id]/colCount[global_id];
		}
	
		spheres[global_id].move(dt);
	}

	dPos[global_id] = dSpeed[global_id] = dRot[global_id] = Vector3f();
	colCount[global_id] = 0;
}


__global__ void moveKernel(Cuboid* cuboids, u32 cuboidCount, f32 dt,
	Vector3f* dPos, Vector3f* dSpeed, Vector3f* dRot, u32* colCount)
{
	u32 global_id = threadIdx.x + blockIdx.x*blockDim.x;
	
	if(global_id < cuboidCount){
		if(colCount[global_id] > 0){
			cuboids[global_id].x += dPos[global_id];
			cuboids[global_id].v += dSpeed[global_id]/colCount[global_id];
			cuboids[global_id].omega += dRot[global_id]/colCount[global_id];
		}
	
		cuboids[global_id].move(dt);
	}
	
	dPos[global_id] = dSpeed[global_id] = dRot[global_id] = Vector3f();
	colCount[global_id] = 0;
}


__global__ void connectorKernel(	Sphere* spheres, u32 sphereOffset, u32 sphereCount,
									Connector* connectors, u32 connectorCount,
									f32 dt)
{
	u32 global_id = threadIdx.x + blockIdx.x*blockDim.x;

	if(global_id < connectorCount){
		Connector& con = connectors[global_id];

		if(sphereOffset <= con.id1 && con.id1 < sphereOffset+sphereCount &&
			sphereOffset <= con.id2 && con.id2 < sphereOffset+sphereCount)
		{

			Sphere& s1 = spheres[con.id1];
			Sphere& s2 = spheres[con.id2];
			
			Vector3f w1 = s1.getModel2World() * con.p1;
			Vector3f w2 = s2.getModel2World() * con.p2;

			//Kraftsto� aus Richtung s1 in Richtung s2 (bei gedehnter Feder)
			Vector3f p = w2-w1;
			f32 pLen = p.length();
			p *= (1 - con.l/pLen) * con.k * dt;//wirkt auf s1 positiv, auf s2 negativ

				
			Vector3f r = w1 - s1.x;
			Vector3f L = crossProduct(r, p);//Drehimpuls
			f32 theta = 2.f/5 * s1.m * s1.r * s1.r;

			atomicAdd(&s1.omega, L/theta);

			if(r){
				atomicAdd(&s1.v, p.getParallelPartTo(r) / s1.m);
			}
			else{
				atomicAdd(&s1.v, p / s1.m);
			}
			

			p = -p;
			r = w2 - s2.x;
			L = crossProduct(r, p);//Drehimpuls
			theta = 2.f/5 * s2.m * s2.r * s2.r;

			atomicAdd(&s2.omega, L/theta);

			if(r){
				atomicAdd(&s2.v, p.getParallelPartTo(r) / s2.m);
			}
			else{
				atomicAdd(&s2.v, p / s2.m);
			}
		}

	}
}


__global__ void connectorKernel(	Sphere* spheres, u32 sphereOffset, u32 sphereCount,
									u32 pointOffset,
									Connector* connectors, u32 connectorCount,
									f32 dt)
{
	u32 global_id = threadIdx.x + blockIdx.x*blockDim.x;

	if(global_id < connectorCount){
		Connector& con = connectors[global_id];

		if(sphereOffset <= con.id1 && con.id1 < sphereOffset+sphereCount &&
			pointOffset <= con.id2)
		{

			Sphere& s1 = spheres[con.id1];
			
			Vector3f w1 = s1.getModel2World() * con.p1;
			Vector3f w2 = con.p2;

			//Kraftsto� aus Richtung s1 in Richtung s2 (bei gedehnter Feder)
			Vector3f p = w2-w1;
			f32 pLen = p.length();
			p *= (1 - con.l/pLen) * con.k * dt;//wirkt auf s1 positiv, auf s2 negativ

				
			Vector3f r = w1 - s1.x;
			Vector3f L = crossProduct(r, p);//Drehimpuls
			f32 theta = 2.f/5 * s1.m * s1.r * s1.r;

			atomicAdd(&s1.omega, L/theta);

			if(r){
				atomicAdd(&s1.v, p.getParallelPartTo(r) / s1.m);
			}
			else{
				atomicAdd(&s1.v, p / s1.m);
			}
		}

	}
}





__global__ void connectorKernel(	Cuboid* cuboids, u32 cuboidOffset, u32 cuboidCount,
									Connector* connectors, u32 connectorCount,
									f32 dt)
{
	u32 global_id = threadIdx.x + blockIdx.x*blockDim.x;

	if(global_id < connectorCount){
		Connector& con = connectors[global_id];

		if(cuboidOffset <= con.id1 && con.id1 < cuboidOffset+cuboidCount &&
			cuboidOffset <= con.id2 && con.id2 < cuboidOffset+cuboidCount)
		{

			Cuboid& s1 = cuboids[con.id1];
			Cuboid& s2 = cuboids[con.id2];
			
			Vector3f w1 = s1.getModel2World() * con.p1;
			Vector3f w2 = s2.getModel2World() * con.p2;

			//Kraftsto� aus Richtung s1 in Richtung s2 (bei gedehnter Feder)
			Vector3f p = w2-w1;
			f32 pLen = p.length();
			p *= (1 - con.l/pLen) * con.k * dt;//wirkt auf s1 positiv, auf s2 negativ

				
			Vector3f r = w1 - s1.x;
			Vector3f L = crossProduct(r, p);//Drehimpuls

			atomicAdd(&s1.omega, s1.getInverseInertia()*L);

			if(r){
				atomicAdd(&s1.v, p.getParallelPartTo(r) / s1.m);
			}
			else{
				atomicAdd(&s1.v, p / s1.m);
			}
			

			p = -p;
			r = w2 - s2.x;
			L = crossProduct(r, p);//Drehimpuls

			atomicAdd(&s2.omega, s1.getInverseInertia()*L);

			if(r){
				atomicAdd(&s2.v, p.getParallelPartTo(r) / s2.m);
			}
			else{
				atomicAdd(&s2.v, p / s2.m);
			}
		}

	}
}

__global__ void connectorKernel(	Cuboid* cuboids, u32 cuboidOffset, u32 cuboidCount,
									u32 pointOffset,
									Connector* connectors, u32 connectorCount,
									f32 dt)
{
	u32 global_id = threadIdx.x + blockIdx.x*blockDim.x;

	if(global_id < connectorCount){
		Connector& con = connectors[global_id];

		if(cuboidOffset <= con.id1 && con.id1 < cuboidOffset+cuboidCount &&
			pointOffset <= con.id2)
		{

			Cuboid& s1 = cuboids[con.id1];
			
			Vector3f w1 = s1.getModel2World() * con.p1;
			Vector3f w2 = con.p2;

			//Kraftsto� aus Richtung s1 in Richtung s2 (bei gedehnter Feder)
			Vector3f p = w2-w1;
			f32 pLen = p.length();
			p *= (1 - con.l/pLen) * con.k * dt;//wirkt auf s1 positiv, auf s2 negativ

				
			Vector3f r = w1 - s1.x;
			Vector3f L = crossProduct(r, p);//Drehimpuls

			atomicAdd(&s1.omega, s1.getInverseInertia()*L);

			if(r){
				atomicAdd(&s1.v, p.getParallelPartTo(r) / s1.m);
			}
			else{
				atomicAdd(&s1.v, p / s1.m);
			}
		}

	}
}



__global__ void testKernel(	Plane* planes, u32 planeCount,
							Sphere* spheres, u32 sphereCount,
							f32 dt)
{
	CollisionDetector cd;
	CollisionHandler ch;

	u32 id = threadIdx.x + blockIdx.x*blockDim.x;

	Vector3f dPos, dSpeed, dRot, pt, n;
	u32 colCount = 0;

	if(id < sphereCount){
		for(u32 i = 0; i < planeCount; i++){
			if(cd(spheres[id], planes[i], pt)){
				colCount++;
				ch(spheres[id], planes[i], pt, dPos, dSpeed, dRot);
			}
		}

		for(u32 i = 0; i < sphereCount; i++){
			if(i != id && cd(spheres[id], spheres[i], pt, n)){
				colCount++;
				ch(spheres[id], spheres[i], pt, n, dPos, dSpeed, dRot);
			}
		}

		__syncthreads();

		if(colCount > 0){
			spheres[id].x += dPos;
			spheres[id].v += dSpeed/colCount;
			spheres[id].omega += dRot/colCount;
		}
		
		spheres[id].move(dt);
	}
}