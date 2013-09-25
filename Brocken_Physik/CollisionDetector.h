#ifndef _COLLISIONDETECTOR_H_
#define _COLLISIONDETECTOR_H_


#include "Sphere.h"
#include "Plane.h"
#include "Cuboid.h"
#include "RigidBody.h"
#include "FixCuboid.h"
#include "cuda_macro.h"




class CollisionDetector{
public:

	CUDA_CALLABLE_MEMBER bool operator () (const Sphere& s1, const Sphere& s2, /*Vector3f& pt, Vector3f& n,*/ f32& t) const{
		/* Berechnung der notwendigen Variablen */
		Vector3f p12 = s1.x - s2.x;
		Vector3f v12 = s1.v - s2.v;
		f32 p12v12 = p12 * v12;
		f32 v12v12 = v12 * v12;
		f32 p12p12 = p12 * p12;
		f32 r = s1.r + s2.r;
		f32 wurzel = p12v12 * p12v12 + v12v12 * r - v12v12 * p12p12;

		/* Kontrolle */
		if(wurzel < 0)
		{
			return false;
		}
		wurzel = sqrt(wurzel);

		/* Berechnung des Zeitpunkts der Kollision */
		f32 t1 = (-p12v12 + wurzel)/v12v12;
	    f32 t2 = (-p12v12 - wurzel)/v12v12;

		/* Ausgabe */
		if(t1 < 0 && t2 > 0)
		{
			t = t2;
		}
		else if(t2 < 0 && t1 > 0)
		{
			t = t1;
		}
		else if(t2 < 0 && t1 < 0)
		{
			return false;
		}
		else if(t2 > 0 && t1 > 0)
		{
			t = min(t1, t2);
		}

		/* TODO */
		//wenn der Abstand der nächsten Mittelpunkte kleiner ist als die beiden Radien,
		//dann hat eine Kollision stattgefunden
		//f32 len = p12.length();
		//if(fLess(len, r)){
		//	//darauf achten, dass die Kugeln sich aufeinander zu bewegen
		//	n = (-p12).getNormalized();
		//	Vector3f v = s2.v - s1.v;
		//	//v = v.getParallelPartToNormal(n);
		//	if(fLess(v*n, 0)){
		//		pt = .5f*(s1.x+s2.x + (s1.r-s2.r)*n);
		//		return true;
		//	}
		//}

		//return false;
		return true;
	}


	CUDA_CALLABLE_MEMBER bool operator () (const Sphere& s, const Plane& p, /*Vector3f& pt*/ f32& t) const{
		
		//f32 d = p.orientatedDistanceTo(s.x);
		//if(abs(d) < s.r){
		//	//darauf achten, dass sich die Kugel auf die Ebene zu bewegt
		//	//Vector3f v = s.v.getParallelPartToNormal(p.n);
		//	if(fLess(d * (s.v*p.n), 0)){
		//		pt = s.x - d*p.n;
		//		return true;
		//	}
		//}
		//
		//return false;

		t = ((p.orientatedDistanceTo(s.x)>0 ? s.r : -s.r) + p.d - s.x*p.n) / (s.v*p.n);
		return t > 0;
	}

	CUDA_CALLABLE_MEMBER bool operator () (const Plane& p, const Sphere& s, /*Vector3f& pt*/ f32& t) const{
		return operator () (s, p, t);
	}


	CUDA_CALLABLE_MEMBER bool operator () (const Cuboid& c, const Plane& p, Vector3f& pt) const{
		Array<Vector3f, 8> corners = c.getCorners();

		//Annahme, dass der Schwerpunkt des Quaders noch auf der "richtigen" Seite der Ebene liegt
		f32 sign = p.orientatedDistanceTo(c.x);
		if(sign == 0.f){//notfalls einen vorherigen Punkt wählen
			sign = p.orientatedDistanceTo(c.x - c.v);
		}

		//for(int i = 0; i < 8; i++){
		//	f32 tmp = p.orientatedDistanceTo(corners[i]);
		//	if(tmp*sign <= 0){ // unterschiedliche Vorzeichen => Quader schneidet Ebene
		//		// Ecke muss sich auf Ebene zubewegen
		//		Vector3f vCorner = c.v + crossProduct(c.omega, corners[i]-c.x);
		//		if(sign * (vCorner*p.n) < 0){
		//			weight += abs(tmp);
		//			contactPt += corners[i]*abs(tmp);
		//		}
		//	}
		//}

		//if(weight > 0){ //Kontakt
		//	pt = contactPt/weight;
		//	return true;
		//}

		f32 in = -INFINITY;
		pt = Vector3f();
		int count = 0;
		for(auto it = corners.begin(); it != corners.end(); ++it){
			if(p.orientatedDistanceTo(*it)*sign <= 0){
				Vector3f vCorner = c.v + crossProduct(c.omega, *it-c.x);
				if(sign * (vCorner*p.n) < 0 && p.distanceTo(*it) > in){
					in = max(in, p.distanceTo(*it));
					pt += *it;
					count++;
				}
			}
		}
		pt /= count;

		if(in >= 0)
			return true;

		return false;
	}

	CUDA_CALLABLE_MEMBER bool operator () (const Plane& p, const Cuboid& c, Vector3f& pt) const{
		return operator () (c, p, pt);
	}


	CUDA_CALLABLE_MEMBER bool operator () (const Cuboid& c, const Sphere& s, Vector3f& pt, Vector3f& n) const{

		//Kugel in Koordinatensystem vom Quader darstellen
		Vector4f x = c.getWorld2Model() * Vector4f(s.x, 1);
		
		//x zu dem Punkt von c machen, der s.x am nächsten ist (falls s.x außerhalb von c)
		for(int i=0; i<3; i++){
			if(x[i] < -.5f){
				x[i] = -.5f;
			}
			else if(x[i] > .5f){
				x[i] = .5f;
			}
		}

		//pt ist jetzt Punkt auf c mit kleinstem Abstand zu s.x
		pt = c.getModel2World() * x;
		n = pt - s.x;
		if(n.length() > s.r){
			return false;
		}

		n = n.getNormalized();
		Vector3f v = c.v + crossProduct(c.omega, pt-c.x) - s.v;
		if(v*n <= 0) //wenn sie sich aufeinander zubewegen
			return true;
		return false;
	}

	CUDA_CALLABLE_MEMBER bool operator () (const Sphere& s, const Cuboid& c, Vector3f& pt, Vector3f& n) const{
		return operator () (c, s, pt, n);
	}


	CUDA_CALLABLE_MEMBER bool operator () (const Cuboid& c1, const Cuboid& c2, Vector3f& pt, Vector3f& n) const{
		Vector3f dist = c2.x - c1.x;
		Array<Vector3f, 3> axes1 = c1.getMainAxes();
		Array<Vector3f, 3> axes2 = c2.getMainAxes();

		Array<Vector3f, 3> normAxes1;
		for(int i=0; i<3; i++){
			normAxes1[i] = axes1[i].getNormalized();
		}
		f32 smallestLap = INFINITY;

		//testen, ob die Achsen von c1 separierend sind
		for(int i=0; i<3; i++){
			f32 projSum1 = axes1[i].length();

			f32 projSum2 = abs(axes2[0] * normAxes1[i])
						+ abs(axes2[1] * normAxes1[i])
						+ abs(axes2[2] * normAxes1[i]);		
		
			f32 lap = .5f*(projSum1+projSum2) - abs(dist * normAxes1[i]);
			if(lap < 0){
				return false; // separierende Achse axes1[i]
			}
			else{
				if(lap < smallestLap){
					smallestLap = lap;
					n = normAxes1[i];
				}
			}
		}

		Array<Vector3f, 3> normAxes2;
		for(int i=0; i<3; i++){
			normAxes2[i] = axes2[i].getNormalized();
		}

		//testen, ob die Achsen von c2 separierend sind
		for(int i=0; i<3; i++){
			f32 projSum2 = axes2[i].length();

			f32 projSum1 = abs(axes1[0] * normAxes2[i])
						+ abs(axes1[1] * normAxes2[i])
						+ abs(axes1[2] * normAxes2[i]);		
		
			f32 lap = .5f*(projSum1+projSum2) - abs(dist * normAxes2[i]);
			if(lap < 0){
				return false; // separierende Achse axes2[i]
			}
			else{
				if(lap < smallestLap){
					smallestLap = lap;
					n = normAxes2[i];
				}
			}
		}


		//testen, ob die Orthogonalen der Kantenpaare separierende Achsen sind
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				Vector3f orth = crossProduct(axes1[i], axes2[j]).getNormalized();

				if(orth){
					f32 projSum1 = abs(axes1[0] * orth)
								+ abs(axes1[1] * orth)
								+ abs(axes1[2] * orth);

					f32 projSum2 = abs(axes2[0] * orth)
								+ abs(axes2[1] * orth)
								+ abs(axes2[2] * orth);

					f32 lap = .5f*(projSum1+projSum2) - abs(dist * orth);
					if(lap < 0){
						return false; // separierende Achse orth
					}
					else{
						if(lap < smallestLap){
							smallestLap = lap;
							n = orth;
						}
					}
				}
			}
		}


		//Berührpunkt finden
		Matrix4f c1m2w = c1.getModel2World();
		Matrix4f c2m2w = c2.getModel2World();
		Matrix4f c1w2m = c1.getWorld2Model();
		Matrix4f c2w2m = c2.getWorld2Model();
		Matrix4f m12 = c2w2m * c1m2w;
		Matrix4f m21 = c1w2m * c2m2w;
		Array<Vector3f, 8> polygon;
		int polyIndex = 0;

		Vector3f vRel = c1.v - c2.v;
		f32 signX = c1.x*n;


		f32 dif = INFINITY;

		//Ecken von c1 dargestellt in Koordinatensystem von c2
		//und kleinsten Abstand suchen
		for(int i=0; i<8; i++){
			Vector4f tmp = m12*Cuboid::CUBE_CORNERS()[i];
			Vector4f tmp2 = tmp;
			for(int j=0; j<3; j++){
				if(tmp[j] < -.5f){
					tmp[j] = -.5f;
				}
				else if(tmp[j] > .5f){
					tmp[j] = .5f;
				}
			}
			//Abstand in Weltkoordinaten
			Vector3f d = tmp-tmp2;
			d[0] *= c2.la;
			d[1] *= c2.lb;
			d[2] *= c2.lc;
			f32 len = d.length();

			//Ecke zum Schnittpolygon hinzufügen
			if(fEqual(len, dif)){
				Vector3f p = c1m2w * Cuboid::CUBE_CORNERS()[i];
				//relative Geschwindigkeit dieser Ecke
				Vector3f vP = vRel + crossProduct(c1.omega, p-c1.x)
					- crossProduct(c2.omega, p-c2.x);
				if(signX * (vP*n) <= 0){//bewegen sich aufeinander zu
					polygon[polyIndex++] = p;
				}
			}
			else if(len < dif){
				Vector3f p = c1m2w * Cuboid::CUBE_CORNERS()[i];
				//relative Geschwindigkeit dieser Ecke
				Vector3f vP = vRel + crossProduct(c1.omega, p-c1.x)
					- crossProduct(c2.omega, p-c2.x);
				if(signX * (vP*n) <= 0){//bewegen sich aufeinander zu
					dif = len;
					polyIndex=1;
					polygon[0] = p;
				}
			}
		}


		dif = INFINITY;
		int polyIndex2 = polyIndex;

		signX = c2.x*n;
		vRel = -vRel;

		//Ecken von c2 dargestellt in Koordinatensystem von c1
		//und kleinsten Abstand suchen
		for(int i=0; i<8; i++){
			Vector3f tmp = m21*Cuboid::CUBE_CORNERS()[i];
			Vector3f tmp2 = tmp;
			for(int j=0; j<3; j++){
				if(tmp[j] < -.5f){
					tmp[j] = -.5f;
				}
				else if(tmp[j] > .5f){
					tmp[j] = .5f;
				}
			}
			//Abstand in Weltkoordinaten
			Vector3f d = tmp-tmp2;
			d[0] *= c1.la;
			d[1] *= c1.lb;
			d[2] *= c1.lc;
			f32 len = d.length();

			//Ecke zum Schnittpolygon hinzufügen
			if(fEqual(len, dif)){
				Vector3f p = c2m2w * Cuboid::CUBE_CORNERS()[i];
				//relative Geschwindigkeit dieser Ecke
				Vector3f vP = vRel + crossProduct(c2.omega, p-c2.x)
					- crossProduct(c1.omega, p-c1.x);
				if(signX * (vP*n) <= 0){//bewegen sich aufeinander zu
					polygon[polyIndex2++] = p;
				}
			}
			else if(len < dif){
				Vector3f p = c2m2w * Cuboid::CUBE_CORNERS()[i];
				//relative Geschwindigkeit dieser Ecke
				Vector3f vP = vRel + crossProduct(c2.omega, p-c2.x)
					- crossProduct(c1.omega, p-c1.x);
				if(signX * (vP*n) <= 0){//bewegen sich aufeinander zu
					dif = len;
					polyIndex2 = polyIndex+1;
					polygon[polyIndex] = p;
				}
			}
		}

		if(polyIndex2 == 0){
			//std::cout << "polygon..." << std::endl;
			return false;
		}

		Vector3f sum;
		for(int i = 0; i < polyIndex2; i++){
			sum += polygon[i];
			//cout << polygon[i][0] << "," << polygon[i][1] << "," << polygon[i][2] << endl;
		}
		//cout << endl;

		//TODO: Schwerpunk auf Quaderoberflächen projizieren
		pt = sum/static_cast<f32>(polyIndex2);

		Vector4f pq = c1w2m * Vector4f(pt,1);
		f32 minabs = INFINITY;
		int mini = 3;
		for(int i=0; i<3; i++){
			if(pq[i] < -.5f){
				pq[i] = -.5f;
			}
			else if(pq[i] > .5f){
				pq[i] = .5f;
			}
			else{
				if(abs(pq[i] - .5f) < minabs){
					minabs = abs(pq[i] - .5f);
					mini = i;
				}
				if(abs(pq[i] + .5f) < minabs){
					minabs = abs(pq[i] + .5f);
					mini = i;
				}
			}
		}
		if(mini < 3){
			if(pq[mini] < 0)
				pq[mini] = -.5f;
			else
				pq[mini] = .5f;
		}

		pt = c1m2w * pq;

		return true;
	}


	CUDA_CALLABLE_MEMBER bool operator () (const Cuboid& c1, const FixCuboid& c2, Vector3f& pt, Vector3f& n) const{
		Vector3f dist = c2.x - c1.x;
		Array<Vector3f, 3> axes1 = c1.getMainAxes();
		Array<Vector3f, 3> axes2 = c2.getMainAxes();

		Array<Vector3f, 3> normAxes1;
		for(int i=0; i<3; i++){
			normAxes1[i] = axes1[i].getNormalized();
		}
		f32 smallestLap = INFINITY;

		//testen, ob die Achsen von c1 separierend sind
		for(int i=0; i<3; i++){
			f32 projSum1 = axes1[i].length();

			f32 projSum2 = abs(axes2[0] * normAxes1[i])
						+ abs(axes2[1] * normAxes1[i])
						+ abs(axes2[2] * normAxes1[i]);		
		
			f32 lap = .5f*(projSum1+projSum2) - abs(dist * normAxes1[i]);
			if(lap < 0){
				return false; // separierende Achse axes1[i]
			}
			else{
				if(lap < smallestLap){
					smallestLap = lap;
					n = normAxes1[i];
				}
			}
		}

		Array<Vector3f, 3> normAxes2;
		for(int i=0; i<3; i++){
			normAxes2[i] = axes2[i].getNormalized();
		}

		//testen, ob die Achsen von c2 separierend sind
		for(int i=0; i<3; i++){
			f32 projSum2 = axes2[i].length();

			f32 projSum1 = abs(axes1[0] * normAxes2[i])
						+ abs(axes1[1] * normAxes2[i])
						+ abs(axes1[2] * normAxes2[i]);		
		
			f32 lap = .5f*(projSum1+projSum2) - abs(dist * normAxes2[i]);
			if(lap < 0){
				return false; // separierende Achse axes2[i]
			}
			else{
				if(lap < smallestLap){
					smallestLap = lap;
					n = normAxes2[i];
				}
			}
		}


		//testen, ob die Orthogonalen der Kantenpaare separierende Achsen sind
		for(int i=0; i<3; i++){
			for(int j=0; j<3; j++){
				Vector3f orth = crossProduct(axes1[i], axes2[j]).getNormalized();

				if(orth){
					f32 projSum1 = abs(axes1[0] * orth)
								+ abs(axes1[1] * orth)
								+ abs(axes1[2] * orth);

					f32 projSum2 = abs(axes2[0] * orth)
								+ abs(axes2[1] * orth)
								+ abs(axes2[2] * orth);

					f32 lap = .5f*(projSum1+projSum2) - abs(dist * orth);
					if(lap < 0){
						return false; // separierende Achse orth
					}
					else{
						if(lap < smallestLap){
							smallestLap = lap;
							n = orth;
						}
					}
				}
			}
		}


		//Berührpunkt finden
		Matrix4f c1m2w = c1.getModel2World();
		Matrix4f c2m2w = c2.getModel2World();
		Matrix4f c1w2m = c1.getWorld2Model();
		Matrix4f c2w2m = c2.getWorld2Model();
		Matrix4f m12 = c2w2m * c1m2w;
		Matrix4f m21 = c1w2m * c2m2w;
		Array<Vector3f, 8> polygon;
		int polyIndex = 0;

		f32 signX = c1.x*n;


		f32 dif = INFINITY;

		//Ecken von c1 dargestellt in Koordinatensystem von c2
		//und kleinsten Abstand suchen
		for(int i=0; i<8; i++){
			Vector4f tmp = m12*Cuboid::CUBE_CORNERS()[i];
			Vector4f tmp2 = tmp;
			for(int j=0; j<3; j++){
				if(tmp[j] < -.5f){
					tmp[j] = -.5f;
				}
				else if(tmp[j] > .5f){
					tmp[j] = .5f;
				}
			}
			//Abstand in Weltkoordinaten
			Vector3f d = tmp-tmp2;
			d[0] *= c2.la;
			d[1] *= c2.lb;
			d[2] *= c2.lc;
			f32 len = d.length();

			//Ecke zum Schnittpolygon hinzufügen
			if(fEqual(len, dif)){
				Vector3f p = c1m2w * Cuboid::CUBE_CORNERS()[i];
				//relative Geschwindigkeit dieser Ecke
				Vector3f vP = c1.v + crossProduct(c1.omega, p-c1.x);
				if(signX * (vP*n) <= 0){//bewegen sich aufeinander zu
					polygon[polyIndex++] = p;
				}
			}
			else if(len < dif){
				Vector3f p = c1m2w * Cuboid::CUBE_CORNERS()[i];
				//relative Geschwindigkeit dieser Ecke
				Vector3f vP = c1.v + crossProduct(c1.omega, p-c1.x);
				if(signX * (vP*n) <= 0){//bewegen sich aufeinander zu
					dif = len;
					polyIndex=1;
					polygon[0] = p;
				}
			}
		}


		dif = INFINITY;
		int polyIndex2 = polyIndex;

		signX = c2.x*n;

		//Ecken von c2 dargestellt in Koordinatensystem von c1
		//und kleinsten Abstand suchen
		for(int i=0; i<8; i++){
			Vector3f tmp = m21*Cuboid::CUBE_CORNERS()[i];
			Vector3f tmp2 = tmp;
			for(int j=0; j<3; j++){
				if(tmp[j] < -.5f){
					tmp[j] = -.5f;
				}
				else if(tmp[j] > .5f){
					tmp[j] = .5f;
				}
			}
			//Abstand in Weltkoordinaten
			Vector3f d = tmp-tmp2;
			d[0] *= c1.la;
			d[1] *= c1.lb;
			d[2] *= c1.lc;
			f32 len = d.length();

			//Ecke zum Schnittpolygon hinzufügen
			if(fEqual(len, dif)){
				Vector3f p = c2m2w * Cuboid::CUBE_CORNERS()[i];
				//relative Geschwindigkeit dieser Ecke
				Vector3f vP = c1.v + crossProduct(c1.omega, p-c1.x);
				if(signX * (vP*n) >= 0){//bewegen sich aufeinander zu
					polygon[polyIndex2++] = p;
				}
			}
			else if(len < dif){
				Vector3f p = c2m2w * Cuboid::CUBE_CORNERS()[i];
				//relative Geschwindigkeit dieser Ecke
				Vector3f vP = c1.v + crossProduct(c1.omega, p-c1.x);
				if(signX * (vP*n) >= 0){//bewegen sich aufeinander zu
					dif = len;
					polyIndex2 = polyIndex+1;
					polygon[polyIndex] = p;
				}
			}
		}

		if(polyIndex2 == 0){
			//std::cout << "polygon..." << std::endl;
			return false;
		}

		Vector3f sum;
		for(int i = 0; i < polyIndex2; i++){
			sum += polygon[i];
			//cout << polygon[i][0] << "," << polygon[i][1] << "," << polygon[i][2] << endl;
		}
		//cout << endl;

		//TODO: Schwerpunk auf Quaderoberflächen projizieren
		pt = sum/static_cast<f32>(polyIndex2);
		return true;
	}

	CUDA_CALLABLE_MEMBER bool operator () (const FixCuboid& fc, const Cuboid& c, Vector3f& pt, Vector3f& n) const{
		return operator () (c, fc, pt, n);	
	}


	CUDA_CALLABLE_MEMBER bool operator () (const Sphere& s, const FixCuboid& fc, Vector3f& pt, Vector3f& n) const{

		//Kugel in Koordinatensystem vom Quader darstellen
		Vector4f x = fc.getWorld2Model() * Vector4f(s.x, 1);
		
		//x zu dem Punkt von c machen, der s.x am nächsten ist (falls s.x außerhalb von c)
		for(int i=0; i<3; i++){
			if(x[i] < -.5f){
				x[i] = -.5f;
			}
			else if(x[i] > .5f){
				x[i] = .5f;
			}
		}

		//pt ist jetzt Punkt auf c mit kleinstem Abstand zu s.x
		pt = fc.getModel2World() * x;
		n = pt - s.x;
		if(n.length() > s.r){
			return false;
		}

		n = n.getNormalized();
		if(s.v*n >= 0) //wenn sie sich aufeinander zubewegen
			return true;
		return false;
	}

	CUDA_CALLABLE_MEMBER bool operator () (const FixCuboid& fc, const Sphere& s, Vector3f& pt, Vector3f& n) const{
		return operator () (s, fc, pt, n);
	}


	//CUDA_CALLABLE_MEMBER bool operator () (const Plane& p, const Particle& particle) const;
};




#endif