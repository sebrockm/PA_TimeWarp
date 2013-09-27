#include "Queue.h"
#include "Movable.h"
#include "Heap.h"

#include <iostream>
using namespace std;

struct Test{
	f32 timestamp;
};

int main(){
	Heap<f32, 10> q;
	for(int i=0;i<10;i++){
		q.insert(10-i);
	}
	cout << q.full() << endl;

	while(!q.empty()){
		cout << q.peek() << endl;
	}

	system("pause");
}