#ifndef _MESSAGECONTROLLSYSTEM_H_
#define _MESSAGECONTROLLSYSTEM_H_


#include "types.h"
#include "cuda_macro.h"
#include "Queue.h"
#include "Heap.h"
#include "Sphere.h"

struct Message{
	enum MsgType{
		event,
		antievent,
		eventAck,
		eventNack,
		mull
	} type;
	f64 timestamp;
	u32 src, dest;
	Sphere newState;

	CUDA_CALLABLE_MEMBER Message():src(55555),type(mull){};
	CUDA_CALLABLE_MEMBER Message(MsgType ty, f64 t, u32 src, u32 dest):type(ty),timestamp(t),src(src),dest(dest){}

	CUDA_CALLABLE_MEMBER bool operator < (const Message& b) const {
		if(timestamp < b.timestamp)
			return true;
		if(timestamp == b.timestamp){
			if(src < b.src)
				return true;
			if(src == b.src){
				return (type == event && b.type == antievent) || 
					(type == eventAck && b.type == eventNack);
			}
		}
		return false;
	}

	CUDA_CALLABLE_MEMBER bool checkAntiPair(const Message& other) const {
		if(timestamp == other.timestamp && src == other.src && dest == other.dest){
			return type == event && other.type == antievent || type == antievent && other.type == event;
		}
		return false;
	}

	CUDA_CALLABLE_MEMBER bool checkAckPair(const Message& other) const {
		if(timestamp == other.timestamp && src == other.src && dest == other.dest){
			return type == event && other.type == eventAck || type == eventAck && other.type == event ||
				type == antievent && other.type == eventNack || type == eventNack && other.type == antievent;
		}
		return false;
	}

	CUDA_CALLABLE_MEMBER Message createAnti() const {
		Message anti = *this;
		switch(anti.type){
		case event: anti.type = antievent; break;
		//case antievent: anti.type = event; break;
		//case eventAck: anti.type = eventNack; break;
		//case eventNack: anti.type = eventAck; break;
#ifdef DOPRINT
		default: printf("falsche antimessage erzeugt\n");
#endif
		}
		return anti;
	}

	CUDA_CALLABLE_MEMBER Message createAck() const {
		Message ack = *this;
		ack.src = this->dest;
		ack.dest = this->src;
		switch(ack.type){
		case event: ack.type = eventAck; break;
		case antievent: ack.type = eventNack; break;
		}
		return ack;
	}

	CUDA_CALLABLE_MEMBER bool operator == (const Message& other) const {
		return timestamp == other.timestamp && src == other.src && dest == other.dest && type == other.type;
	}

	CUDA_CALLABLE_MEMBER bool operator != (const Message& other) const {
		return ! (*this == other);
	}
};


class MessageControllSystem{

private:
	Heap<Message, QL>* inputQueues;
	Queue<Message, QL>* mailboxes;
	u32 sphereCount;

public:
	CUDA_CALLABLE_MEMBER MessageControllSystem(Heap<Message, QL>* q, Queue<Message, QL>* mb, u32 sc):inputQueues(q), mailboxes(mb), sphereCount(sc) {}

	CUDA_CALLABLE_MEMBER MessageControllSystem(Queue<Message, QL>* mb, u32 sc):inputQueues(0), mailboxes(mb), sphereCount(sc) {}

	CUDA_CALLABLE_MEMBER void send(const Message& msg){
		if(msg.newState.r == 0 && msg.type == Message::event){
#ifdef DOPRINT
			printf("komische msg gesendet\n");
#endif
		}
		else
			mailboxes[msg.src].insertBack(msg);
	}

#ifdef __CUDACC__
	CUDA_CALLABLE_MEMBER void recv(){
		u32 id = threadIdx.x + blockIdx.x*blockDim.x;
		for(u32 i = 0; i < sphereCount; i++){
			for(u32 j = 0; j < mailboxes[i].length(); j++){
				if(mailboxes[i][j].type != Message::mull && mailboxes[i][j].dest == id){
					inputQueues[id].insert(mailboxes[i][j]);
					if(mailboxes[i][j].newState.r == 0){
#ifdef DOPRINT
						printf("komische msg geliefert\n");
#endif
					}
				}
			}
		}
	}
#endif
};


#endif