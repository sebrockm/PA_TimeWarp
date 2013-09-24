#ifndef _MATERIAL_H_
#define _MATERIAL_H_


#include "types.h"
#include "cuda_macro.h"


enum Material{
	steel,
	wood,
	glass,
	rubber,
	material_N
};

//Restitutionsskoeffizenten der Materialpaarungen
CUDA_CONST_VAR f32 resCoef[material_N][material_N] =  
									{{	0.75f,	0.6625f,0.845f,	0.9f	},
									{	0.6625f,0.575f,	0.7575f,0.8f	},
									{	0.845f,	0.7575f,0.94f,	0.84f	},
									{	0.9f,	0.8f,	0.84f,	0.75f	}};

//Haftreibungskoeffizienten
CUDA_CONST_VAR f32 staticFricCoef[material_N][material_N] = 
										{	{0.15f,	0.4f,	0.6f,	1.0f},
											{0.4f,	0.6f,	0.5f,	1.0f},
											{0.6f,	0.5f,	0.9f,	1.0f},
											{1.0f,	1.0f,	1.0f,	1.0f}};


#endif