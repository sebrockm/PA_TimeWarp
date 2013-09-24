#ifndef _OPENGL_H_
#define _OPENGL_H_

#define GLEW_STATIC
#include <gl\glew.h>
//#include <gl\glut.h>
#define FREEGLUT_STATIC
#include <gl\freeglut.h>

//#pragma comment(lib, "glew32s.lib")
//#pragma comment(lib, "freeglut.lib")


static const GLuint ATTRIB_POS = 0;
static const GLuint ATTRIB_NOR = 1;
static const GLuint ATTRIB_TEX_POS = 2;

static const GLuint RESTART_INDEX = 0xffffffff;




#endif