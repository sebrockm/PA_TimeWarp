#include "types.h"
#include "cuda_macro.h"
#include "Queue.h"
#include "Heap.h"

struct Message{
	enum MsgType{
		event,
		antievent,
		eventAck,
		antieventAck,
		mull
	} type;
	f32 timestamp;
	u32 src, dest;

	CUDA_CALLABLE_MEMBER Message(){};
	CUDA_CALLABLE_MEMBER Message(MsgType ty, f32 t, u32 src, u32 dest):type(ty),timestamp(t),src(src),dest(dest){}

	CUDA_CALLABLE_MEMBER bool operator < (const Message& b) const {
		if(timestamp < b.timestamp)
			return true;
		if(timestamp == b.timestamp){
			if(src < b.src)
				return true;
			if(src == b.src){
				return (type == event && b.type == antievent) || 
					(type == eventAck && b.type == antieventAck);
			}
		}
		return false;
	}

	CUDA_CALLABLE_MEMBER bool checkAntiPair(const Message& other) const {
		if(timestamp == other.timestamp && src == other.src && dest == other.dest){
			return type == event && other.type == antievent || type == antievent && other.type == event ||
				type == eventAck && other.type == antieventAck || type == antieventAck && other.type == eventAck;
		}
		return false;
	}

	CUDA_CALLABLE_MEMBER bool checkAckPair(const Message& other) const {
		if(timestamp == other.timestamp && src == other.src && dest == other.dest){
			return type == event && other.type == eventAck || type == eventAck && other.type == event ||
				type == antievent && other.type == antieventAck || type == antieventAck && other.type == antievent;
		}
		return false;
	}

	CUDA_CALLABLE_MEMBER Message createAnti() const {
		Message anti = *this;
		switch(anti.type){
		case event: anti.type = antievent; break;
		case antievent: anti.type = event; break;
		case eventAck: anti.type = antieventAck; break;
		case antieventAck: anti.type = eventAck; break;
		}
		return anti;
	}

	CUDA_CALLABLE_MEMBER Message createAck() const {
		Message ack = *this;
		ack.src = this->dest;
		ack.dest = this->src;
		switch(ack.type){
		case event: ack.type = eventAck; break;
		case antievent: ack.type = antieventAck; break;
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
	Heap<Message, 20>* inputQueues;
	Queue<Message, 20>* mailboxes;
	u32 sphereCount;

public:
	CUDA_CALLABLE_MEMBER MessageControllSystem(Heap<Message, 20>* q, Queue<Message, 20>* mb, u32 sc):inputQueues(q), mailboxes(mb), sphereCount(sc) {}

	CUDA_CALLABLE_MEMBER MessageControllSystem(Queue<Message, 20>* mb, u32 sc):inputQueues(0), mailboxes(mb), sphereCount(sc) {}

	CUDA_CALLABLE_MEMBER void send(const Message& msg){
		mailboxes[msg.src].insert(msg);
	}

	CUDA_CALLABLE_MEMBER void recv(){
		u32 id = threadIdx.x + blockIdx.x*blockDim.x;
		for(u32 i = 0; i < sphereCount; i++){
			for(u32 j = 0; j < mailboxes[i].length(); j++){
				if(mailboxes[i][j].type != Message::mull && mailboxes[i][j].dest == id){
					inputQueues[id].insert(mailboxes[i][j]);
					//mailboxes[i][j].type = Message::mull;
				}
			}
		}
	}
};