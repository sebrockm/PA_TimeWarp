#include "kernel.h"

#include "CollisionDetector.h"
#include "CollisionHandler.h"

void test(int a){
	calculateGVT<<<100, 32>>>(0, 0, a);
}


__device__ void rollback(Queue<Sphere, QL>* stateQs, 
	Heap<Message, QL>* inputQs,
	Queue<Message, QL>* outputQs,
	Sphere* pendings,
	Queue<Message, QL>* mailboxes,
	int delId,
	const Message& msg,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	MessageControllSystem mcs(mailboxes, sphereCount);

	stateQs[id].deleteAllAfterEq(delId);// states loeschen

	while(!outputQs[id].empty() && outputQs[id].back().timestamp >= msg.timestamp){ // antimessages verschicken
		mcs.send(outputQs[id].peekBack());
	}

	pendings[id].partner = -1;//pending loeschen

	while(!inputQs[id].empty()){//nacks fuer alle verbleibenden Nachrichten schicken
		Message nmsg = inputQs[id].peek();
		if(nmsg.type == Message::event || nmsg.type == Message::eventAck){
			nmsg.dest = msg.src;
			nmsg.src = id;
			nmsg.type = Message::eventNack;
			mcs.send(nmsg);
		}
	}
}

__global__ void handleNextMessages(
	Queue<Sphere, QL>* stateQs, 
	Heap<Message, QL>* inputQs,
	Queue<Message, QL>* outputQs,
	Sphere* pendings,
	Queue<Message, QL>* mailboxes,//nur fuers Senden
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;

	
	MessageControllSystem mcs(inputQs, mailboxes, sphereCount);

	Message msg;
	msg.type = Message::mull;

	while(!inputQs[id].empty()){
		//naechste Message suchen, die sich nicht irgendwie aufloest
		while(inputQs[id].length() > 0){
			msg = inputQs[id].peek();
			if(inputQs[id].length() > 0){
				if(msg.checkAntiPair(inputQs[id].top())){
					inputQs[id].peek();
					if(inputQs[id].length() > 0){
						msg = inputQs[id].peek();
					}
					else{
						return;
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
			int sid = stateQs[id].searchFirstBefore(msg.timestamp);
			if(sid < 0){
				printf("siddest < 0\n");
				return;
			}
			if(sid < stateQs[id].length() - 1){//rollback
				rollback(stateQs, inputQs, outputQs, pendings, mailboxes, sid+1, msg, sphereCount);
			}

			stateQs[id].insertBack(msg.newState); //Zustand unmittelbar vor der Kollision

			//Ack senden, damit anderer weiss, dass diese msg tatsaechlich verarbeitet wurde
			mcs.send(msg.createAck());
			if(msg.src != id){
				outputQs[id].insertBack(Message(Message::antievent, msg.timestamp, id, msg.src));//antievent als output speichern, damit der andere das Einfuegen des states rueckgaengig machen kann
			}

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

			//rollback
			if(stateQs[id][sid].timestamp == msg.timestamp && stateQs[id][sid].partner == msg.src){
				rollback(stateQs, inputQs, outputQs, pendings, mailboxes, sid, msg, sphereCount);
			}
			else{
				//printf("state zum antievent nicht vorhanden\n");
			}
			break;
		}

		case Message::eventAck:
		{
			if(pendings[id].partner != -1 && pendings[id].timestamp == msg.timestamp && msg.src == pendings[id].partner){
				int sid = stateQs[id].searchFirstBefore(msg.timestamp) + 1;
				if(sid < stateQs[id].length()){//rollback
					rollback(stateQs, inputQs, outputQs, pendings, mailboxes, sid, msg, sphereCount);
				}

				stateQs[id].insertBack(pendings[id]);
				pendings[id].partner = -1;
			}
			else{
				printf("pending passt nicht oder ist nicht da (ack)\n");
			}

			break;
		}

		case Message::eventNack: //unser gesendetes Event wurde nicht bearbeitet
		{
			if(pendings[id].partner != -1 && pendings[id].timestamp == msg.timestamp && msg.src == pendings[id].partner){
				pendings[id].partner = -1;
			}
			else{
				printf("pending passt nicht oder ist nicht da (nack)\n");
			}
		}

		}//switch ende
	}//while ende
}


__global__ void receiveFromMailboxes( 
	Heap<Message, QL>* inputQs, 
	Queue<Message, QL>* mailboxes,//nur fuers Empfangen
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
	Queue<Message, QL>* mailboxes,//nur fuers Senden
	Sphere* pendings,
	Queue<Message, QL>* outputQs,
	Queue<Sphere, QL>* stateQs, u32 sphereCount, 
	f32 tmin)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if(id >= sphereCount)
		return;

	if(pendings[id].partner >= 0){
		printf("pending ist nicht leer...\n");
		return;
	}

	CollisionDetector cd;
	f32 t;
	f32 lvt = stateQs[id].back().timestamp;

	u32 partner;
	int nextCol = 0;
	for(u32 i = 0; i < planeCount; i++){
		if(cd(stateQs[id].back(), planes[i], t)){
			t += stateQs[id].back().timestamp;
			if(t < tmin){
				partner = i;
				tmin = t;
				nextCol = 1;
			}
		}
	}

	u32 stateId;
	for(u32 i = 0; i < sphereCount; i++){
		for(u32 j = max(0, stateQs[i].searchFirstBefore(stateQs[id].back())); j < stateQs[i].length(); j++){
			if(cd(stateQs[id].back(), stateQs[i][j], t)){//t ist Kollisionszeitpunkt mit s1.timestamp als Nullpunkt
				t += stateQs[id].back().timestamp;
				if(t < tmin && (j == stateQs[i].length()-1 || t < stateQs[i][j+1].timestamp)){
					partner = i;
					stateId = j;
					tmin = t;
					nextCol = 2;
					break;
				}
			}
		}
	}

	MessageControllSystem mcs(mailboxes, sphereCount);
	CollisionHandler ch;

	if(nextCol == 1){//Kollision mit Ebene
		pendings[id] = stateQs[id].back();
		pendings[id].moveWithoutA(t-pendings[id].timestamp);
		pendings[id].partner = -2;
		ch(pendings[id], planes[partner]);
		
		Message msg(Message::event, pendings[id].timestamp, id, id);
		msg.newState = pendings[id];
		mcs.send(msg);
	}
	else if(nextCol == 2){//Kollision mit Sphere
		pendings[id] = stateQs[id].back(); //Zustand unmittelbar vor der Kollision
		pendings[id].moveWithoutA(t-pendings[id].timestamp);
		pendings[id].partner = partner;
		Sphere cp = stateQs[partner][stateId];
		cp.moveWithoutA(t-cp.timestamp);
		cp.partner = id;
		Sphere tmp = cp;

		ch(cp, pendings[id]); // cp ist jetzt neuer state des Partners
		ch(pendings[id], tmp);//kann sich nicht mit (*) in die Quere kommen

		//event an partner senden
		Message msg(Message::event, pendings[id].timestamp, id, partner);
		msg.newState = cp;
		mcs.send(msg);
		outputQs[id].insertBack(msg.createAnti());
	}
	else{ //keine Kollision
		pendings[id].partner = -1;
	}
}


__global__ void calculateGVT(
	Heap<Message, QL>* inputQs,
	Queue<Sphere, QL>* stateQs,
	f32* gvts,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;

	gvts[id] = min(inputQs[id].top().timestamp, stateQs[id].back().timestamp);

	for(int stride = 2; stride/2 < sphereCount; stride *= 2){
		if(id*stride < sphereCount){
			gvts[id*stride] = min(gvts[id*stride], gvts[min(id*stride+stride/2, sphereCount-1)]);
		}
	}
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