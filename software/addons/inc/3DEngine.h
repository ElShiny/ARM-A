#ifndef ENGINE_H_
#define ENGINE_H_

#include <stdint.h>

#define MESH_MAX_SIZE 31

typedef struct
{
	float x;
	float y;
	float z;
	float w;
}Vect3D;

typedef struct
{
	Vect3D v[3];
}Triangle;


typedef struct
{
	uint8_t size;
	Triangle t[MESH_MAX_SIZE];
}OBJMesh;


typedef struct
{
	float m[4][4];
}mat4x4;


//void MatrixMult(Vect3D *i, Vect3D *o, mat4x4 *m);
void Init_3D(void);
void Project_And_Draw(mat4x4 *matProj, OBJMesh Mesh);
OBJMesh Draw_Finger(mat4x4 *MatHand);


#endif /* ENGINE_H_ */
