#include "types.h"
#include "cuda_macro.h"
#include "StateQueue.h"
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

	CUDA_CALLABLE_MEMBER bool operator == (const Message& other) const {
		return timestamp == other.timestamp && src == other.src && dest == other.dest;
	}

	CUDA_CALLABLE_MEMBER bool operator != (const Message& other) const {
		return ! (*this == other);
	}
};


class MessageControllSystem{

private:
	Heap<Message, 20>* inputQueues;
	Message* mailboxes;
	u32 sphereCount;

public:
	CUDA_CALLABLE_MEMBER MessageControllSystem(Heap<Message, 20>* q, Message* mb, u32 sc):inputQueues(q), mailboxes(mb), sphereCount(sc) {}

	CUDA_CALLABLE_MEMBER void send(const Message& msg){
		mailboxes[msg.src] = msg;
	}

	CUDA_CALLABLE_MEMBER void recv(){
		u32 id = threadIdx.x + blockIdx.x*blockDim.x;
		for(u32 i = 0; i < sphereCount; i++){
			if(mailboxes[i].type != Message::mull && mailboxes[i].dest == id){
				inputQueues[id].insert(mailboxes[i]);
			}
		}
	}
};