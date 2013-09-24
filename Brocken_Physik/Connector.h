#ifndef _CONNECTOR_H_
#define _CONNECTOR_H_

#include "Vector.h"


class Connector{
public:
	u32 id1, id2;
	Vector4f p1, p2;// Verbindungspunkt in Modelkoordinaten
	f32 k, l; //Federstärke, Ruhelänge

	void set(u32 id1, u32 id2, const Vector3f& p1, const Vector3f& p2, f32 k, f32 l = -1);
};



#endif