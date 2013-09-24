#ifndef _KDTREE_H_
#define _KDTREE_H_

//#define DRAW_KDTREE



#include "CollisionDetector.h"
#include "cuda_macro.h"

#include <vector>
#include <utility>
#include <ostream>


const u32 MAX_DEPTH = 12;
const u32 MAX_NODES = 1 << MAX_DEPTH+1;


using std::vector;


enum Axes{
	xAxis,
	yAxis,
	zAxis,
	noAxis
};


struct IsObjectPartOf{
	Axes axNo;
	f32 axis;
	bool less;

	IsObjectPartOf(Axes axNo, f32 axis, bool less)
		:axNo(axNo), axis(axis), less(less){}

	bool operator () (u32 id);
};


std::pair<f32, Axes> findGoodAxis(u32* first, u32* last);



struct AxComparator{
	Axes axNo;
	bool less;

	AxComparator(Axes axNo, bool less):axNo(axNo), less(less){}

	bool operator () (u32 s1, u32 s2) const;
};


struct KDTreeNode{
	static u32 size;

	u32 indexStart, indexEnd;

	u32 father, left, right;

	f32 axis;
	Axes axNo;

#ifdef DRAW_KDTREE
	Vector3f aabb1, aabb2;
#endif

	KDTreeNode();

	static void set();

	void set(vector<u32>::iterator first, 
		vector<u32>::iterator last, int depth, u32 father);

	void set(vector<u32>::reverse_iterator first, 
		vector<u32>::reverse_iterator last, int depth, u32 father);

	~KDTreeNode();

	CUDA_CALLABLE_MEMBER bool isLeaf() const {
		return axNo == noAxis;
	}
};


std::ostream& operator << (std::ostream& st, const KDTreeNode& tree);



#endif