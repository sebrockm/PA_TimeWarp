#include "TimeWarpKernel.h"



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

			if(msg.src != id){//unsere eigenen events nicht in die outputQ stecken, die kommen von Kollisionen mit Ebenen oder nicht vorhandenen Kollisionen	
				mcs.send(msg.createAck());//Ack senden, damit anderer weiss, dass diese msg tatsaechlich verarbeitet wurde
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
			if(pendings[id].partner != -1 && pendings[id].timestamp == msg.timestamp && (msg.src == pendings[id].partner || pendings[id].partner < -1)){
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

	if(pendings[id].partner != -1){
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
		pendings[id].moveWithoutA(tmin-pendings[id].timestamp);
		pendings[id].partner = -2;
		ch(pendings[id], planes[partner]);
		
		Message msg(Message::event, pendings[id].timestamp, id, id);
		msg.newState = pendings[id];
		mcs.send(msg);
	}
	else if(nextCol == 2){//Kollision mit Sphere
		pendings[id] = stateQs[id].back(); //Zustand unmittelbar vor der Kollision
		pendings[id].moveWithoutA(tmin-pendings[id].timestamp);
		pendings[id].partner = partner;
		Sphere cp = stateQs[partner][stateId];
		cp.moveWithoutA(tmin-cp.timestamp);
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
		pendings[id] = stateQs[id].back();
		pendings[id].moveWithoutA(tmin-pendings[id].timestamp);
		pendings[id].partner = -3;
		
		Message msg(Message::event, tmin, id, id);
		msg.newState = pendings[id];
		mcs.send(msg);
	}
}


__global__ void calculateLVT(
	Heap<Message, QL>* inputQs,
	Queue<Sphere, QL>* stateQs,
	f32* lvts,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id < sphereCount)
		lvts[id] = min(inputQs[id].top().timestamp, stateQs[id].back().timestamp);
}


__global__ void deleteOlderThanGVT(
	Queue<Message, QL>* outputQs,
	Queue<Sphere, QL>* stateQs, 
	u32 sphereCount,
	f32 gvt)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;

	Message msg;
	while(!outputQs[id].empty() && outputQs[id].front().timestamp < gvt){
		msg = outputQs[id].peekFront();
	}
	if(outputQs[id].empty()){
		printf("outputQ ist leer, darf nicht sein\n");
		return;
	}
	if(outputQs[id].front().timestamp > gvt)
		outputQs[id].insertFront(msg);

	Sphere s;
	while(!stateQs[id].empty() && stateQs[id].front().timestamp < gvt){
		s = stateQs[id].peekFront();
	}
	if(stateQs[id].empty()){
		printf("stateQ ist leer, darf nicht sein\n");
		return;
	}
	if(stateQs[id].front().timestamp > gvt)
		stateQs[id].insertFront(s);
}


__global__ void cpToStateQs(
	Sphere* spheres,
	Queue<Sphere, QL>* stateQs,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;

	stateQs[id].insertBack(spheres[id]);
}


__global__ void cpFromStateQs(
	Sphere* spheres,
	Queue<Sphere, QL>* stateQs,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;
	if(stateQs[id].length() != 1){
		printf("laenge der stateQ ist nicht 1\n");
	}
	spheres[id] = stateQs[id].front();
}



