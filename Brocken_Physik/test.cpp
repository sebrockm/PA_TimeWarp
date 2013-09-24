#include "StateQueue.h"
#include "Movable.h"

#include <iostream>
using namespace std;

int main(){
	StateQueue<Movable, 10> q;
	for(int i=0;i<9;i++){
		Movable m(Vector3f(0,i,0));
		m.timestamp = i/10.f;
		q.push(m);
		q.pop();
		m.timestamp = i/9.f;
		q.push(m);
	}
	cout << q.full() << endl;
	u32 id = q.searchNext(.45f);

	cout << q.get(id).timestamp << endl;

	system("pause");
}