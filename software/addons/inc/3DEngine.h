#ifndef ENGINE_H_
#define ENGINE_H_

#define MESH_SIZE 4

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
	Triangle t[MESH_SIZE];
}OBJMesh;


typedef struct
{
	float m[4][4];
}mat4x4;


void MatrixMult(Vect3D *i, Vect3D *o, mat4x4 *m);
void Init_3D(void);


#endif /* ENGINE_H_ */
