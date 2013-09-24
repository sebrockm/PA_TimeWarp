#include "CollisionDetector.h"



/**
Kollisionserkennung von zwei Kugeln. Beide Kugeln müssen bereits ihre
zukünftigen Positionen berechnet haben mit calculateNextsByTime(t).

@param s1 erste Kugel
@param s2 zweit Kugel
@return true bei Kollision im nächsten Zeitintervall, false sonst
*/
inline bool CollisionDetector::operator () (const Sphere& s1, const Sphere& s2, Vector3f& pt, Vector3f& n) const {

	n = s2.x - s1.x;	
	f32 r = s1.r + s2.r;
		
	//wenn der Abstand der nächsten Mittelpunkte kleiner ist als die beiden Radien,
	//dann hat eine Kollision stattgefunden
	f32 len = n.length();
	if(fLess(len, r)){
		//darauf achten, dass die Kugeln sich aufeinander zu bewegen
		n = n/len;
		Vector3f v = s2.v - s1.v;
		//v = v.getParallelPartToNormal(n);
		if(fLess(v*n, 0)){
			pt = .5f*(s1.x+s2.x + (s1.r-s2.r)*n);
			return true;
		}
	}

	return false;
}


inline bool CollisionDetector::operator () (const Sphere& s, const Plane& p, Vector3f& pt) const {
	f32 d = p.orientatedDistanceTo(s.x);
	if(abs(d) < s.r){
		//darauf achten, dass sich die Kugel auf die Ebene zu bewegt
		//Vector3f v = s.v.getParallelPartToNormal(p.n);
		if(fLess(d * (s.v*p.n), 0)){
			pt = s.x - d*p.n;
			return true;
		}
	}
		
	return false;
}


inline bool CollisionDetector::operator () (const Plane& p, const Sphere& s, Vector3f& pt) const {
	return operator () (s, p, pt);
}


inline bool CollisionDetector::operator () (const Cuboid& c, const Plane& p, Vector3f& pt) const {
	Array<Vector3f, 8> corners = c.getCorners();

	//Annahme, dass der Schwerpunkt des Quaders noch auf der "richtigen" Seite der Ebene liegt
	f32 sign = p.orientatedDistanceTo(c.x);
	if(sign == 0.f){//notfalls einen vorherigen Punkt wählen
		sign = p.orientatedDistanceTo(c.x - c.v);
	}
	Vector3f contactPt;
	f32 weight = 0.f;

	for(int i = 0; i < 8; i++){
		f32 tmp = p.orientatedDistanceTo(corners[i]);
		if(tmp*sign <= 0){ // unterschiedliche Vorzeichen => Quader schneidet Ebene
			// Ecke muss sich auf Ebene zubewegen
			Vector3f vCorner = c.v + crossProduct(c.omega, corners[i]-c.x);
			if(sign * (vCorner*p.n) < 0){
				weight += abs(tmp);
				contactPt += corners[i]*abs(tmp);
			}
		}
	}

	if(weight > 0){ //Kontakt
		pt = contactPt/weight;
		return true;
	}

	return false;
}


inline bool CollisionDetector::operator () (const Plane& p, const Cuboid& c, Vector3f& pt) const {
	return operator () (c, p, pt);
}


inline bool CollisionDetector::operator () (const Cuboid& c, const Sphere& s, Vector3f& pt, Vector3f& n) const {
	Vector3f v = c.v - s.v;
	n = c.x - s.x;
	//Kugel in Koordinatensystem vom Quader darstellen
	Vector4f x = c.getWorld2Model() * Vector4f(s.x[0], s.x[1], s.x[2], 1);

	//v zu dem Punkt von c machen, der s.x am nächsten ist (falls s.x außerhalb von c)
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
	if((s.x-pt).lengthSqr() > s.r*s.r){
		return false;
	}

	n = (pt - s.x).getNormalized();
	Vector3f vPt = c.v + crossProduct(c.omega, pt-c.x) - s.v;
	if(vPt*n < 0) //wenn sie sich aufeinander zubewegen
		return true;
	return false;
}


inline bool CollisionDetector::operator () (const Sphere& s, const Cuboid& c, Vector3f& pt, Vector3f& n) const {
	return operator () (c, s, n, pt);
}


inline bool CollisionDetector::operator () (const Cuboid& c1, const Cuboid& c2, Vector3f& pt, Vector3f& n) const {
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

			if(orth.lengthSqr() != 0){
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
	Matrix4f m12 = c2.getWorld2Model() * c1.getModel2World();
	Matrix4f m21 = c1.getWorld2Model() * c2.getModel2World();
	std::vector<Vector3f> polygon;
			
	Vector4f corners[] = {
		Vector4f(-.5, -.5, -.5, 1),
		Vector4f(.5, -.5, -.5, 1),
		Vector4f(.5, .5, -.5, 1),
		Vector4f(-.5, .5, -.5, 1),
		Vector4f(-.5, -.5, .5, 1),
		Vector4f(.5, -.5, .5, 1),
		Vector4f(.5, .5, .5, 1),
		Vector4f(-.5, .5, .5, 1)};

	f32 dif = INFINITY;
	int index = -1;//index für Ecke mit kleinstem Abstand

	//Ecken von c1 dargestellt in Koordinatensystem von c2
	//und kleinsten Abstand suchen
	for(int i=0; i<8 && dif>0; i++){
		Vector3f tmp = m12*corners[i];
		Vector3f tmp2 = tmp;
		for(int j=0; j<3; j++){
			if(tmp[j] < -.5f){
				tmp[j] = -.5f;
			}
			else if(tmp[j] > .5f){
				tmp[j] = .5f;
			}
		}
		f32 len = (tmp-tmp2).length();
		if(len < dif){
			dif = len;
			index = i;
		}
	}

	auto c1corners = c1.getCorners();
	Plane p1(n, c1corners[index]);//Stützebene
	f32 signX = p1.orientatedDistanceTo(c1.x);
	Vector3f vRel = c1.v - c2.v;
	for(auto i = c1corners.begin(); i != c1corners.end(); ++i){
		if(fEqual(p1.orientatedDistanceTo(*i), 0)){ //Ecken zum Schnittpolygon hinzufügen
			Vector3f vCorner = vRel + crossProduct(c1.omega, *i-c1.x) - crossProduct(c2.omega, *i-c2.x);
			if(signX * (vCorner*n) < 0){ // wenn Ecke sich auf anderen Quader zubewegt
				polygon.push_back(*i);
			}
		}
	}


	dif = INFINITY;
	index = -1;

	//Ecken von c2 dargestellt in Koordinatensystem von c1
	//und kleinsten Abstand suchen
	for(int i=0; i<8 && dif>0; i++){
		Vector3f tmp = m21*corners[i];
		Vector3f tmp2 = tmp;
		for(int j=0; j<3; j++){
			if(tmp[j] < -.5f){
				tmp[j] = -.5f;
			}
			else if(tmp[j] > .5f){
				tmp[j] = .5f;
			}
		}
		f32 len = (tmp-tmp2).length();
		if(len < dif){
			dif = len;
			index = i;
		}
	}

	auto c2corners = c2.getCorners();
	Plane p2(n, c2corners[index]);//Stützebene
	signX = p2.orientatedDistanceTo(c2.x);
	vRel = -vRel;
	for(auto i = c2corners.begin(); i != c2corners.end(); ++i){
		if(fEqual(p2.orientatedDistanceTo(*i), 0)){ //Ecken zum Schnittpolygon hinzufügen
			Vector3f vCorner = vRel + crossProduct(c2.omega, *i-c2.x) - crossProduct(c1.omega, *i-c1.x);
			if(signX * (vCorner*n) < 0){ // wenn Ecke sich auf Ebene zubewegt
				polygon.push_back(*i);
			}
		}
	}

	if(polygon.size() == 0){
		//std::cout << "polygon..." << std::endl;
		return false;
	}

	Plane finalPlane(n, .5f*(p1.d+p2.d), rubber);
	Vector3f sum;
	for(auto i = polygon.begin(); i != polygon.end(); ++i){
		sum += *i - finalPlane.n*finalPlane.orientatedDistanceTo(*i);
	}

	//TODO: Schwerpunk auf Quaderoberflächen projizieren
	pt = sum/static_cast<f32>(polygon.size());
	return true;
}



inline bool CollisionDetector::operator () (const Cuboid& c, const FixCuboid& fc, Vector3f& pt, Vector3f& n) const {
	Vector3f dist = fc.x - c.x;
	Array<Vector3f, 3> axes1 = c.getMainAxes();
	Array<Vector3f, 3> axes2 = fc.getMainAxes();

	Array<Vector3f, 3> normAxes1;
	for(int i=0; i<3; i++){
		normAxes1[i] = axes1[i].getNormalized();
	}
	f32 smallestLap = INFINITY;

	//testen, ob die Achsen von c separierend sind
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

	//testen, ob die Achsen von fc separierend sind
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
	Matrix4f mcfc = fc.getWorld2Model() * c.getModel2World();
	Matrix4f mfcc = c.getWorld2Model() * fc.getModel2World();
	std::vector<Vector3f> polygon;
			
	Vector4f corners[] = {
		Vector4f(-.5, -.5, -.5, 1),
		Vector4f(.5, -.5, -.5, 1),
		Vector4f(.5, .5, -.5, 1),
		Vector4f(-.5, .5, -.5, 1),
		Vector4f(-.5, -.5, .5, 1),
		Vector4f(.5, -.5, .5, 1),
		Vector4f(.5, .5, .5, 1),
		Vector4f(-.5, .5, .5, 1)};

	f32 dif = INFINITY;
	int index = -1;//index für Ecke mit kleinstem Abstand

	//Ecken von c dargestellt in Koordinatensystem von fc
	//und kleinsten Abstand suchen
	for(int i=0; i<8 && dif>0; i++){
		Vector3f tmp = mcfc*corners[i];
		Vector3f tmp2 = tmp;
		for(int j=0; j<3; j++){
			if(tmp[j] < -.5f){
				tmp[j] = -.5f;
			}
			else if(tmp[j] > .5f){
				tmp[j] = .5f;
			}
		}
		f32 len = (tmp-tmp2).length();
		if(len < dif){
			dif = len;
			index = i;
		}
	}

	auto ccorners = c.getCorners();
	Plane p1(n, ccorners[index]);//Stützebene
	f32 signX = p1.orientatedDistanceTo(c.x);
	for(auto i = ccorners.begin(); i != ccorners.end(); ++i){
		if(fEqual(p1.orientatedDistanceTo(*i), 0)){ //Ecken zum Schnittpolygon hinzufügen
			Vector3f vCorner = c.v + crossProduct(c.omega, *i-c.x);
			if(signX * (vCorner*n) < 0){ // wenn Ecke sich auf anderen Quader zubewegt
				polygon.push_back(*i);
			}
		}
	}


	dif = INFINITY;
	index = -1;
	Vector3f cNearestCorner;

	//Ecken von cf dargestellt in Koordinatensystem von c
	//und kleinsten Abstand suchen
	for(int i=0; i<8 && dif>0; i++){
		Vector3f tmp = mfcc*corners[i];
		Vector3f tmp2 = tmp;
		for(int j=0; j<3; j++){
			if(tmp[j] < -.5f){
				tmp[j] = -.5f;
			}
			else if(tmp[j] > .5f){
				tmp[j] = .5f;
			}
		}
		f32 len = (tmp-tmp2).length();
		if(len < dif){
			dif = len;
			index = i;
			cNearestCorner = fc.getModel2World() * corners[i];
		}
	}

	auto fccorners = fc.getCorners();
	Plane p2(n, fccorners[index]);//Stützebene
	for(auto i = fccorners.begin(); i != fccorners.end(); ++i){
		if(fEqual(p2.orientatedDistanceTo(*i), 0)){ //Ecken zum Schnittpolygon hinzufügen
			Vector3f vCorner = c.v + crossProduct(c.omega, cNearestCorner-c.x);
			if(signX * (vCorner*n) < 0){ // wenn Ecke sich auf anderen Quader zubewegt
				polygon.push_back(*i);
			}
		}
	}

	if(polygon.size() == 0){
		//std::cout << "polygon..." << std::endl;
		return false;
	}

	Plane finalPlane(n, .5f*(p1.d+p2.d), rubber);
	Vector3f sum;
	for(auto i = polygon.begin(); i != polygon.end(); ++i){
		sum += *i - finalPlane.n*finalPlane.orientatedDistanceTo(*i);
	}

	//TODO: Schwerpunk auf Quaderoberflächen projizieren
	pt = sum/static_cast<f32>(polygon.size());
	return true;
}

inline bool CollisionDetector::operator () (const FixCuboid& fc, const Cuboid& c, Vector3f& pt, Vector3f& n) const {
	return operator () (c, fc, pt, n);	
}


inline bool CollisionDetector::operator () (const Sphere& s, const FixCuboid& fc, Vector3f& pt, Vector3f& n) const{
	Vector3f v = - s.v;
	n = fc.x - s.x;
	//Kugel in Koordinatensystem vom Quader darstellen
	Vector4f x = fc.getWorld2Model() * Vector4f(s.x[0], s.x[1], s.x[2], 1);

	//v zu dem Punkt von c machen, der s.x am nächsten ist (falls s.x außerhalb von c)
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
	if((s.x-pt).lengthSqr() > s.r*s.r){
		return false;
	}

	n = (pt - s.x).getNormalized();
	if(v*n < 0) //wenn sie sich aufeinander zubewegen
		return true;
	return false;
}

inline bool CollisionDetector::operator () (const FixCuboid& fc, const Sphere& s, Vector3f& pt, Vector3f& n) const {
	return operator () (s, fc, pt, n);
}
