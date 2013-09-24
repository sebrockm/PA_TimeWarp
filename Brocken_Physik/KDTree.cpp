#include "KDTree.h"
#include "GLManager.h"

#include <cstdlib>
#include <algorithm>
#include <boost\tuple\tuple.hpp>


const u32 ELEMENTS_IN_LEAF = 12;


using namespace std;


bool IsObjectPartOf::operator () (u32 id){
	if(axNo != noAxis){
		Vector3f n;

		n[axNo] = 1;

		Plane p(n, axis, (Material)0);
		auto& cmgr = GLManager::instance().cmgr;

		if(id < cmgr.getOffset<Sphere>() + cmgr.sphereCount){
			auto& s = cmgr.spheres[id-cmgr.getOffset<Sphere>()];
			if((p.orientatedDistanceTo(s.x) >= 0) ^ less){
				return true;
			}
			return p.distanceTo(s.x) <= s.r;
		}
		else if(id < cmgr.getOffset<Cuboid>() + cmgr.cuboidCount){
			auto& c = cmgr.cuboids[id-cmgr.getOffset<Cuboid>()];
			if((p.orientatedDistanceTo(c.x) >= 0) ^ less){
				return true;
			}
			return p.distanceTo(c.x) <= c.getProjectionOnNormal(p.n)/2;
		}
		else if(id < cmgr.getOffset<FixCuboid>() + cmgr.fixCuboidCount){
			auto& c = cmgr.fixCuboids[id-cmgr.getOffset<FixCuboid>()];
			if((p.orientatedDistanceTo(c.x) >= 0) ^ less){
				return true;
			}
			return p.distanceTo(c.x) <= c.getProjectionOnNormal(p.n)/2;
		}
	}

	return false;
}

bool AxComparator::operator () (u32 s1, u32 s2) const {
	auto& cmgr = GLManager::instance().cmgr;
	Vector3f x1, x2, r1, r2;

	if(s1 < cmgr.getOffset<Sphere>() + cmgr.sphereCount){
		auto& s = cmgr.spheres[s1-cmgr.getOffset<Sphere>()];
		x1 = s.x;
		r1[0] = r1[1] = r1[2] = s.r;
	}
	else if(s1 < cmgr.getOffset<Cuboid>() + cmgr.cuboidCount){
		auto& c = cmgr.cuboids[s1-cmgr.getOffset<Cuboid>()];
		auto axes = c.getMainAxes();
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				r1[i] += abs(axes[j][i]);
			}
		}
		r1 /= 2;
		x1 = c.x;
	}
	else if(s1 < cmgr.getOffset<FixCuboid>() + cmgr.fixCuboidCount){
		auto& c = cmgr.fixCuboids[s1-cmgr.getOffset<FixCuboid>()];
		auto axes = c.getMainAxes();
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				r1[i] += abs(axes[j][i]);
			}
		}
		r1 /= 2;
		x1 = c.x;
	}

	if(s2 < cmgr.getOffset<Sphere>() + cmgr.sphereCount){
		auto& s = cmgr.spheres[s2-cmgr.getOffset<Sphere>()];
		x2 = s.x;
		r2[0] = r2[1] = r2[2] = s.r;
	}
	else if(s2 < cmgr.getOffset<Cuboid>() + cmgr.cuboidCount){
		auto& c = cmgr.cuboids[s2-cmgr.getOffset<Cuboid>()];
		auto axes = c.getMainAxes();
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				r2[i] += abs(axes[j][i]);
			}
		}
		r2 /= 2;
		x2 = c.x;
	}
	else if(s2 < cmgr.getOffset<FixCuboid>() + cmgr.fixCuboidCount){
		auto& c = cmgr.fixCuboids[s2-cmgr.getOffset<FixCuboid>()];
		auto axes = c.getMainAxes();
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				r2[i] += abs(axes[j][i]);
			}
		}
		r2 /= 2;
		x2 = c.x;
	}

	if(less)
		return x1[axNo] - r1[axNo] < x2[axNo] - r2[axNo];
	else
		return x1[axNo] + r1[axNo] < x2[axNo] + r2[axNo];
}


std::pair<f32, Axes> findGoodAxis(u32* first, u32* last){
	auto& cmgr = GLManager::instance().cmgr;

	// mins und maxs bilden eine AABBox
	Vector3f mins(INFINITY);
	Vector3f maxs(-INFINITY);

	Vector3f sum;
	auto dif = last - first;


	while(first != last){
		if(*first < cmgr.getOffset<Sphere>() + cmgr.sphereCount){
			auto& s = cmgr.spheres[*first-cmgr.getOffset<Sphere>()];
			for(int i=0; i<3; i++){
				if(s.x[i] - s.r < mins[i]){
					mins[i] = s.x[i] - s.r;
				}

				if(s.x[i] + s.r > maxs[i]){
					maxs[i] = s.x[i] + s.r;
				}

				sum[i] += s.x[i];
			}
		}
		else if(*first < cmgr.getOffset<Cuboid>() + cmgr.cuboidCount){
			auto& c = cmgr.cuboids[*first-cmgr.getOffset<Cuboid>()];
			auto axes = c.getMainAxes();
			Vector3f r;
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					r[i] += abs(axes[j][i]);
				}
			}
			r /= 2;

			for(int i=0; i<3; i++){
				if(c.x[i] - r[i] < mins[i]){
					mins[i] = c.x[i] - r[i];
				}

				if(c.x[i] + r[i] > maxs[i]){
					maxs[i] = c.x[i] + r[i];
				}

				sum[i] += c.x[i];
			}
		}
		else if(*first < cmgr.getOffset<FixCuboid>() + cmgr.fixCuboidCount){
			auto& c = cmgr.fixCuboids[*first-cmgr.getOffset<FixCuboid>()];
			auto axes = c.getMainAxes();
			Vector3f r;
			for(int i=0; i<3; i++){
				for(int j=0; j<3; j++){
					r[i] += abs(axes[j][i]);
				}
			}
			r /= 2;

			for(int i=0; i<3; i++){
				if(c.x[i] - r[i] < mins[i]){
					mins[i] = c.x[i] - r[i];
				}

				if(c.x[i] + r[i] > maxs[i]){
					maxs[i] = c.x[i] + r[i];
				}

				sum[i] += c.x[i];
			}
		}

		++first;
	}

	//sum /= static_cast<f32>(dif); //arithmetische Mittel (ideal wären Mediane)
	maxs -= mins; //Seitenlängen der AABBox

	//entlang der Längsten Seite teilen, möglichst in der Mitte
	if(maxs[0] > maxs[1] && maxs[0] > maxs[2]){
		//createPlane(Plane(Vector3f(1,0,0),sum[0]/dif,glass));
		return std::make_pair(sum[0]/dif, xAxis);
	}
	if(maxs[1] > maxs[0] && maxs[1] > maxs[2]){
		//createPlane(Plane(Vector3f(0,1,0),sum[1]/dif,glass));
		return std::make_pair(sum[1]/dif, yAxis);
	}
	//cout<<"z...";
	//createPlane(Plane(Vector3f(0,0,1),sum[2]/dif,glass));
	return std::make_pair(sum[2]/dif, zAxis);
	
}


u32 KDTreeNode::size = 0;


KDTreeNode::KDTreeNode():indexStart(0), indexEnd(0), axis(0), axNo(noAxis),
	father(0), left(0), right(0)
{}


void KDTreeNode::set() 
{
	auto& cmgr = GLManager::instance().cmgr;
	size = 0;
	
	//initiale Indexmenge
	static vector<u32> indices;
	if(indices.capacity() < cmgr.getOffset<FixCuboid>() + cmgr.fixCuboidCount){
		indices.reserve(cmgr.getOffset<FixCuboid>() + cmgr.fixCuboidCount);
		for(u32 i = (u32)indices.size(); i < (u32)indices.capacity(); i++){
			indices.push_back(i);
		}
	}

	cmgr.leafIndexCount = 0;

#ifdef DRAW_KDTREE
	Vector3f& aabb1 = cmgr.treeArray[0].aabb1;
	Vector3f& aabb2 = cmgr.treeArray[0].aabb2;
	for(int i=0; i<3; i++){
		u32 id1 = *min_element(indices.begin(),indices.end(),AxComparator((Axes)i,true));
		u32 id2 = *max_element(indices.begin(),indices.end(),AxComparator((Axes)i,false));

		if(id1 < cmgr.getOffset<Sphere>() + cmgr.sphereCount){
			const Sphere& s1 = cmgr.spheres[id1-cmgr.getOffset<Sphere>()];
			aabb1[i] = s1.x[i] - s1.r;
		}
		else if(id1 < cmgr.getOffset<Cuboid>() + cmgr.cuboidCount){
			const Cuboid& s1 = cmgr.cuboids[id1-cmgr.getOffset<Cuboid>()];
			Vector3f n;
			n[i] = 1;
			aabb1[i] = s1.x[i] - s1.getProjectionOnNormal(n)/2;
		}
		else if(id1 < cmgr.getOffset<FixCuboid>() + cmgr.fixCuboidCount){
			const FixCuboid& s1 = cmgr.fixCuboids[id1-cmgr.getOffset<FixCuboid>()];
			Vector3f n;
			n[i] = 1;
			aabb1[i] = s1.x[i] - s1.getProjectionOnNormal(n)/2;
		}

		if(id2 < cmgr.getOffset<Sphere>() + cmgr.sphereCount){
			const Sphere& s2 = cmgr.spheres[id2-cmgr.getOffset<Sphere>()];
			aabb2[i] = s2.x[i] + s2.r;
		}
		else if(id2 < cmgr.getOffset<Cuboid>() + cmgr.cuboidCount){
			const Cuboid& s2 = cmgr.cuboids[id2-cmgr.getOffset<Cuboid>()];
			Vector3f n;
			n[i] = 1;
			aabb2[i] = s2.x[i] + s2.getProjectionOnNormal(n)/2;
		}
		else if(id2 < cmgr.getOffset<FixCuboid>() + cmgr.fixCuboidCount){
			const FixCuboid& s2 = cmgr.fixCuboids[id2-cmgr.getOffset<FixCuboid>()];
			Vector3f n;
			n[i] = 1;
			aabb2[i] = s2.x[i] + s2.getProjectionOnNormal(n)/2;
		}
	}
#endif

	cmgr.treeArray[0].set(indices.begin(), indices.end(), 0, 0);
	size++;
}


void KDTreeNode::set(vector<u32>::iterator first, 
	vector<u32>::iterator last, int depth, u32 father)
{
	u32 myId = size;
	this->father = father;
	auto& cmgr = GLManager::instance().cmgr;
	
	indexStart = cmgr.leafIndexCount;

#ifdef DRAW_KDTREE
	if(myId != 0){
		aabb1 = cmgr.treeArray[father].aabb1;
		aabb2 = cmgr.treeArray[father].aabb2;
		aabb2[cmgr.treeArray[father].axNo] = cmgr.treeArray[father].axis;
	}
#endif
	
	if(last > first + ELEMENTS_IN_LEAF && depth < MAX_DEPTH){

		boost::tie(axis, axNo) = findGoodAxis(&*first, &*last);

		if(size+1 < MAX_NODES){
			auto middle = partition(first, last, IsObjectPartOf(axNo, axis, true));
			left = ++size;
			cmgr.treeArray[left].set(first, middle, depth+1, myId);
		}
		else throw std::overflow_error("Zu viele Nodes");

		vector<u32>::reverse_iterator rfirst(last), rlast(first);

		if(size+1 < MAX_NODES){
			auto rmiddle = partition(rfirst, rlast, IsObjectPartOf(axNo, axis, false));
			right = ++size;
			cmgr.treeArray[right].set(rfirst, rmiddle, depth+1, myId);
		}
		else throw std::overflow_error("Zu viele Nodes");
	}
	else{ //Dieser Knoten ist ein Blatt
		if(cmgr.leafIndexCount + (last - first) > cmgr.MAX_LEAFINDICES)
			throw std::overflow_error("Zu viele Leaves");

		while(first != last){
			cmgr.leafIndices[cmgr.leafIndexCount++] = *first++;
		}
		axNo = noAxis;
	}
	
	indexEnd = cmgr.leafIndexCount;
	//sort(&cmgr.leafIndices[indexStart], &cmgr.leafIndices[indexEnd]);
}


void KDTreeNode::set(vector<u32>::reverse_iterator first, 
	vector<u32>::reverse_iterator last, int depth, u32 father)
{
	this->father = father;
	u32 myId = size;
	auto& cmgr = GLManager::instance().cmgr;
		
	indexStart = cmgr.leafIndexCount;

#ifdef DRAW_KDTREE
	if(myId != 0){
		aabb1 = cmgr.treeArray[father].aabb1;
		aabb1[cmgr.treeArray[father].axNo] = cmgr.treeArray[father].axis;
		aabb2 = cmgr.treeArray[father].aabb2;
	}
#endif
	
	if(last - first > ELEMENTS_IN_LEAF && depth < MAX_DEPTH){

		vector<u32>::iterator rfirst(last.base()), rlast(first.base());

		boost::tie(axis, axNo) = findGoodAxis(&*rfirst, &*rlast);

		if(size+1 < MAX_NODES){
			auto middle = partition(first, last, IsObjectPartOf(axNo, axis, true));
			left = ++size;
			cmgr.treeArray[left].set(first, middle, depth+1, myId);
		}
		else throw std::overflow_error("Zu viele Nodes");

		if(size+1 < MAX_NODES){
			auto rmiddle = partition(rfirst, rlast, IsObjectPartOf(axNo, axis, false));
			right = ++size;
			cmgr.treeArray[right].set(rfirst, rmiddle, depth+1, myId);
		}
		else throw std::overflow_error("Zu viele Nodes");
	}
	else{ //Dieser Knoten ist ein Blatt
		if(cmgr.leafIndexCount + (last - first) > cmgr.MAX_LEAFINDICES)
			throw std::overflow_error("Zu viele Leaves");

		while(first != last){
			cmgr.leafIndices[cmgr.leafIndexCount++] = *first++;
		}
		axNo = noAxis;
	}
	
	indexEnd = cmgr.leafIndexCount;
	//sort(&cmgr.leafIndices[indexStart], &cmgr.leafIndices[indexEnd]);
}


KDTreeNode::~KDTreeNode(){
	//cout<<"weg"<<endl;
}



std::ostream& operator << (std::ostream& st, const KDTreeNode& node){
	auto& cmgr = GLManager::instance().cmgr;
	if(node.isLeaf()){
		st << " ";
		for(u32 i = node.indexStart; i < node.indexEnd; i++){
			st << cmgr.leafIndices[i] << (i<node.indexEnd-1?",":" ");
		}
		return st << endl;
	}
	st << "l" << node.axis << cmgr.treeArray[node.left];
	st << "r" << node.axis << cmgr.treeArray[node.right];
	return st;
}