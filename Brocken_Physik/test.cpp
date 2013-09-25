#include "StateQueue.h"
#include "Movable.h"

#include <iostream>
using namespace std;

struct Test{
	f32 timestamp;
};

int main(){
	StateQueue<Test, 10> q;
	for(int i=0;i<10;i++){
		Test m;
		m.timestamp = 2*i;
		q.insert(m);
		q.peek();
		m.timestamp = 2*i+1;
		q.insert(m);
	}
	cout << q.full() << endl;
	u32 id = q.searchNext(.45f);

	cout << q.get(id).timestamp << endl;

	system("pause");
}