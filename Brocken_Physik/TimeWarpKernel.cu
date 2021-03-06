#include "TimeWarpKernel.h"

#include "CollisionDetector.h"
#include "CollisionHandler.h"

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
	
	//printf("%d macht einen Rollback\n", id);
	if(stateQs[id].empty()){
#ifdef DOPRINT
		printf("stateQ ist vor rollback leer, darf nicht sein\n");
#endif
	}
	stateQs[id].deleteAllAfterEq(delId);// states loeschen
	if(stateQs[id].empty()){
#ifdef DOPRINT
		printf("stateQ ist nach rollback leer durch delId==%d\n", delId);
#endif
	}

	while(!outputQs[id].empty() && fGreaterEq(outputQs[id].back().timestamp, msg.timestamp)){ // antimessages verschicken
#ifdef DOPRINT
		if(outputQs[id].back().newState.r == 0 && outputQs[id].back().type == Message::event)
			printf("hier2\n");
#endif
		mcs.send(outputQs[id].peekBack());
	}

	//pendings[id].partner = -1;//pending loeschen

	while(!inputQs[id].empty()){//nacks fuer alle verbleibenden Nachrichten schicken
		Message nmsg = inputQs[id].peek();
		if(nmsg.type == Message::event){
			if(nmsg.dest != nmsg.src)
			{
				nmsg.dest = msg.src;
				nmsg.src = id;
				nmsg.type = Message::eventNack;
				mcs.send(nmsg);
			}
		} else if(nmsg.type == Message::eventAck) {
			if(pendings[id].partner != -1 && pendings[id].timestamp == nmsg.timestamp && (nmsg.src == pendings[id].partner || pendings[id].partner < -1)){
				pendings[id].partner = -1;
			} else {
#ifdef DOPRINT
				printf("Fehler: Nack und pending passen nicht zusammen \n");
#endif
			}
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
		//printf("while ende\n");

		switch(msg.type)
		{
		case Message::event:
		{//printf("event\n");
			int sid = stateQs[id].searchFirstBeforeEq(msg.timestamp);
			if(sid < 0){
#ifdef DOPRINT
				printf("sid < 0, denn msg.timestamp == %f und stateQs[%d].back().timestamp == %f\n", msg.timestamp, id, stateQs[id].back().timestamp);
#endif
				return;
			}
			if(sid < stateQs[id].length() - 1){//rollback
				//printf("im rollback\n");
				rollback(stateQs, inputQs, outputQs, pendings, mailboxes, sid+1, msg, sphereCount);
			}
#ifdef DOPRINT
			if(id==0 && msg.newState.r == 0)
				printf("HOELLE von %d\n", msg.src);
#endif
				
			stateQs[id].insertBack(msg.newState); //Zustand unmittelbar nach der Kollision
			//if(msg.newState.partner == -2)
			//	printf("event: stateQs[%d].insert collision plane \n", id);

			if(msg.src != id){//unsere eigenen events nicht in die outputQ stecken, die kommen von Kollisionen mit Ebenen oder nicht vorhandenen Kollisionen
#ifdef DOPRINT
				if(msg.createAck().newState.r == 0 && msg.createAck().type == Message::event)
					printf("hier1\n");
#endif
				mcs.send(msg.createAck());//Ack senden, damit anderer weiss, dass diese msg tatsaechlich verarbeitet wurde
				outputQs[id].insertBack(Message(Message::antievent, msg.timestamp, id, msg.src));//antievent als output speichern, damit der andere das Einfuegen des states rueckgaengig machen kann
			}
			//printf("kurz vor break\n");
			break;
		}

		case Message::antievent:
		{
			int sid = stateQs[id].searchFirstBeforeEq(msg.timestamp) + 1;
#ifdef DOPRINT
			if(sid == 0)
				printf("1 grosses kaka bei Kugel %d\n", id);
#endif

			int schleife = 0;
			while(stateQs[id][sid].timestamp == msg.timestamp){//suche state der antimessage
				schleife = 1;
				//printf("Schleife durchlaufen id==%d, sid==%d\n", id, sid);
				if(stateQs[id][sid].partner == msg.src && stateQs[id][sid+1].timestamp == msg.timestamp){//swape gefundenen state mit gleichen timestamps darueber
					Sphere tmp = stateQs[id][sid];
					stateQs[id][sid] = stateQs[id][sid+1];
					stateQs[id][sid+1] = tmp;
					//printf("geswapt id==%d, sid==%d\n", id, sid);
				}
				sid++;
			}
			if(schleife == 1)
				sid--;

			//rollback
#ifdef DOPRINT
			if(sid == 0)
				printf("2 grosses kaka bei Kugel %d: sid==%d\n", id, sid);
#endif
			if(stateQs[id][sid].timestamp == msg.timestamp && stateQs[id][sid].partner == msg.src){
				rollback(stateQs, inputQs, outputQs, pendings, mailboxes, sid, msg, sphereCount);
			}
			else if(stateQs[id].back().timestamp > msg.timestamp){
				//printf("state zum antievent nicht vorhanden\n");
			}
			else{
				//printf("alles ok, war schon geloescht\n");
			}
			break;
		}

		case Message::eventAck:
		{
			if(pendings[id].partner != -1 && pendings[id].timestamp == msg.timestamp && msg.src == pendings[id].partner){
				int sid = stateQs[id].searchFirstBeforeEq(msg.timestamp) + 1;
				if(sid < stateQs[id].length()){//rollback
					rollback(stateQs, inputQs, outputQs, pendings, mailboxes, sid, msg, sphereCount);
				}

				stateQs[id].insertBack(pendings[id]);
				//printf("eventAck: stateQs[%d].insert \n", id);
				pendings[id].partner = -1;
			}
			else{
#ifdef DOPRINT
				printf("pendings[%d].timestamp == %.16f && msg.timestamp == %.16f \n", id, pendings[id].timestamp, msg.timestamp);
				printf("pendings[%d].partner == %d && msg.src == %d \n", id, pendings[id].partner, msg.src);
				printf("pending passt nicht oder ist nicht da (ack)\n");
#endif
			}

			break;
		}

		case Message::eventNack: //unser gesendetes Event wurde nicht bearbeitet
		{
			if(pendings[id].partner != -1 && pendings[id].timestamp == msg.timestamp && (msg.src == pendings[id].partner || pendings[id].partner < -1)){
				pendings[id].partner = -1;
			}
			else{
#ifdef DOPRINT
				printf("(nack): pendings[%d].partner == %d && msg.src == %d pendings[%d].timestamp == %.16f && msg.timestamp == %.16f \n", id, pendings[id].partner, msg.src, id, pendings[id].timestamp, msg.timestamp);
#endif
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
	f64 tmin)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;
	if(id >= sphereCount)
		return;

	if(pendings[id].partner != -1){
#ifdef DOPRINT
		printf("pending ist nicht leer bei id %d\n", id);
#endif
		//return;
	}

	CollisionDetector cd;
	f64 t;
	f64 lvt = stateQs[id].back().timestamp;

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
		if(i == id)
			continue;

		for(u32 j = max(0, stateQs[i].searchFirstBeforeEq(stateQs[id].back())); j < stateQs[i].length(); j++){
			if(cd(stateQs[id].back(), stateQs[i][j], t)){//t ist Kollisionszeitpunkt mit s1.timestamp als Nullpunkt
				t += stateQs[id].back().timestamp;

				if(t < tmin && (j == stateQs[i].length()-1 || t < stateQs[i][j+1].timestamp)){
					Sphere tmp1 = stateQs[id].back();
					tmp1.moveWithoutA(t-tmp1.timestamp);
					Sphere tmp2 = stateQs[i][j];
					tmp2.moveWithoutA(t-tmp2.timestamp);
					//printf("Kollision von %d mit %d an stateQ-id %d, timestamp: %f, Abstand: %.16f\n", id, i, j, t, (tmp1.x - tmp2.x).length() - (tmp1.r + tmp2.r));
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
		//pendings[id].moveWithoutA(EPSILON);
		
		Message msg(Message::event, pendings[id].timestamp, id, id);
		msg.newState = pendings[id];
		mcs.send(msg);
		pendings[id].partner = -1;
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
		//cp.moveWithoutA(EPSILON);
		ch(pendings[id], tmp);
		//pendings[id].moveWithoutA(EPSILON);

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
		pendings[id].partner = -1;
	}
}


__global__ void calculateLVT(
	Heap<Message, QL>* inputQs,
	Queue<Sphere, QL>* stateQs,
	f64* lvts,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id < sphereCount)
	{
		//printf("inputQs[%d].top().timestamp == %f && stateQs[%d].back().timestamp == %f \n", id, inputQs[id].top().timestamp, id, stateQs[id].back().timestamp);
		lvts[id] = min(inputQs[id].top().timestamp, stateQs[id].back().timestamp);
	}
}


__global__ void deleteOlderThanGVT(
	Queue<Message, QL>* outputQs,
	Queue<Sphere, QL>* stateQs, 
	u32 sphereCount,
	f64 gvt)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;

	while(outputQs[id].length() > 0 && outputQs[id].front().timestamp < gvt){
		outputQs[id].peekFront();
	}

	while(stateQs[id].length() > 1 && stateQs[id][1].timestamp <= gvt){
		stateQs[id].peekFront();
	}
	if(stateQs[id].empty()){
#ifdef DOPRINT
		printf("stateQ ist leer, darf nicht sein\n");
		return;
#endif
	}
}


__global__ void cpToStateQs(
	Sphere* spheres,
	Sphere* pendings,
	Queue<Sphere, QL>* stateQs,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;
	
	if(!stateQs[id].empty())
	{
#ifdef DOPRINT
		printf("stateQs[%d] ist nicht leer \n", id);
#endif
	}
	pendings[id].partner = -1;
	stateQs[id].insertBack(spheres[id]);
	stateQs[id].back().timestamp = 0;
}


__global__ void cpFromStateQs(
	Sphere* spheres,
	Queue<Sphere, QL>* stateQs,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;
	while(stateQs[id].length() > 1)
		stateQs[id].peekFront();

	spheres[id] = stateQs[id].peekFront();
}


__global__ void removeFromMailboxes(
	Queue<Message, QL>* mailboxes,
	u32 sphereCount)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;

	mailboxes[id].removeAll();
}


__global__ void accelerate(
	Sphere* spheres,
	u32 sphereCount,
	f64 dt)
{
	int id = threadIdx.x + blockIdx.x*blockDim.x;

	if(id >= sphereCount)
		return;

	spheres[id].moveOnlyA(dt);
}