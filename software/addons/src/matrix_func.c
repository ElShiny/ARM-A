/*
 * matrix_func.c
 *
 *  Created on: 1 Oct 2022
 *      Author: Javidx9
 */

#include "3DEngine.h"
#include "matrix_func.h"
#include <math.h>








	Vect3D Matrix_MultiplyVector(mat4x4 *m, Vect3D *i)
	{
		Vect3D v = {0};
		v.x = i->x * m->m[0][0] + i->y * m->m[0][1] + i->z * m->m[0][2] + i->w * m->m[0][3];
		v.y = i->x * m->m[1][0] + i->y * m->m[1][1] + i->z * m->m[1][2] + i->w * m->m[1][3];
		v.z = i->x * m->m[2][0] + i->y * m->m[2][1] + i->z * m->m[2][2] + i->w * m->m[2][3];
		v.w = i->x * m->m[3][0] + i->y * m->m[3][1] + i->z * m->m[3][2] + i->w * m->m[3][3];

		if (v.w != 0.0f)
		{
			v.x /= v.w; v.y /= v.w; v.z /= v.w;
		}
		return v;
	}

	Triangle Matrix_MultiplyTriangle(mat4x4 *m, Triangle *i)
	{
		Triangle tri = {0};
		tri.v[0] = Matrix_MultiplyVector(m, &i->v[0]);
		tri.v[1] = Matrix_MultiplyVector(m, &i->v[1]);
		tri.v[2] = Matrix_MultiplyVector(m, &i->v[2]);
		return tri;
	}

	mat4x4 Matrix_MakeIdentity()
	{
		mat4x4 matrix = {0};
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	mat4x4 Matrix_MakeRotationX(float fAngleRad)
	{
		mat4x4 matrix = {0};

		float cos = cosf(fAngleRad);
		float sin = sinf(fAngleRad);

		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = cos;
		matrix.m[1][2] = -sin;
		matrix.m[2][1] = sin;
		matrix.m[2][2] = cos;
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	mat4x4 Matrix_MakeRotationY(float fAngleRad)
	{
		mat4x4 matrix = {0};

		float cos = cosf(fAngleRad);
		float sin = sinf(fAngleRad);

		matrix.m[0][0] = cos;
		matrix.m[0][2] = sin;
		matrix.m[2][0] = -sin;
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = cos;
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	mat4x4 Matrix_MakeRotationZ(float fAngleRad)
	{
		mat4x4 matrix = {0};

		float cos = cosf(fAngleRad);
		float sin = sinf(fAngleRad);

		matrix.m[0][0] = cos;
		matrix.m[0][1] = -sin;
		matrix.m[1][0] = sin;
		matrix.m[1][1] = cos;
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	mat4x4 Matrix_MakeRotationAll(float fXAngleRad, float fYAngleRad, float fZAngleRad)
	{
		float cosa, cosb, cosc, sina, sinb, sinc;

		cosa = cosf(fZAngleRad);
		cosb = cosf(fYAngleRad);
		cosc = cosf(fXAngleRad);
		sina = sinf(fZAngleRad);
		sinb = sinf(fYAngleRad);
		sinc = sinf(fXAngleRad);

		mat4x4 matrix;
		matrix.m[0][0] = cosb*cosc;
		matrix.m[0][1] = sina*sinb*cosc-cosa*sinc;
		matrix.m[0][2] = cosa*sinb*cosc+sina*sinc;
		matrix.m[1][0] = cosb*sinc;
		matrix.m[1][1] = sina*sinb*sinc+cosa*cosc;
		matrix.m[1][2] = cosa*sinb*sinc-sina*cosc;
		matrix.m[2][0] = -sinb;
		matrix.m[2][1] = sina*cosb;
		matrix.m[2][2] = cosa*cosb;
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	mat4x4 Matrix_MakeTranslation(float x, float y, float z)
	{
		mat4x4 matrix = {0};
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		matrix.m[0][3] = x;
		matrix.m[1][3] = y;
		matrix.m[2][3] = z;
		return matrix;
	}

	Triangle Vec_MakeTranslation(Triangle *triRotated, float x, float y, float z)
	{
		Triangle triTranslated;
		triTranslated.v[0].z = triRotated->v[0].z + z;
		triTranslated.v[1].z = triRotated->v[1].z + z;
		triTranslated.v[2].z = triRotated->v[2].z + z;
		triTranslated.v[0].y = triRotated->v[0].y + y;
		triTranslated.v[1].y = triRotated->v[1].y + y;
		triTranslated.v[2].y = triRotated->v[2].y + y;
		triTranslated.v[0].x = triRotated->v[0].x + x;
		triTranslated.v[1].x = triRotated->v[1].x + x;
		triTranslated.v[2].x = triRotated->v[2].x + x;
		return triTranslated;
	}

	mat4x4 Matrix_MakeProjection(float fFovDegrees, float fAspectRatio, float fNear, float fFar)
	{
		float fFovRad = 1.0 / tanf((fFovDegrees*0.5)/(180.0 * 3.14159));
		mat4x4 matrix = {0};
		matrix.m[0][0] = fAspectRatio * fFovRad;
		matrix.m[1][1] = fFovRad;
		matrix.m[2][2] = fFar / (fFar - fNear);
		matrix.m[2][3] = (-fFar * fNear) / (fFar - fNear);
		matrix.m[3][2] = 1.0f;
		matrix.m[3][3] = 0.0f;
		return matrix;
	}

	mat4x4 Matrix_MultiplyMatrix(mat4x4 *m1, mat4x4 *m2)
	{
		mat4x4 matrix;
		for (int c = 0; c < 4; c++)
			for (int r = 0; r < 4; r++)
				matrix.m[r][c] = m1->m[r][0] * m2->m[0][c] + m1->m[r][1] * m2->m[1][c] + m1->m[r][2] * m2->m[2][c] + m1->m[r][3] * m2->m[3][c];
		return matrix;
	}

//	mat4x4 Matrix_PointAt(Vect3D *pos, Vect3D *target, Vect3D *up)
//	{
//		// Calculate new forward direction
//		Vect3D newForward = Vector_Sub(target, pos);
//		newForward = Vector_Normalise(newForward);
//
//		// Calculate new Up direction
//		Vect3D a = Vector_Mul(newForward, Vector_DotProduct(up, newForward));
//		Vect3D newUp = Vector_Sub(up, a);
//		newUp = Vector_Normalise(newUp);
//
//		// New Right direction is easy, its just cross product
//		Vect3D newRight = Vector_CrossProduct(newUp, newForward);
//
//		// Construct Dimensioning and Translation Matrix
//		mat4x4 matrix;
//		matrix.m[0][0] = newRight.x;	matrix.m[0][1] = newRight.y;	matrix.m[0][2] = newRight.z;	matrix.m[0][3] = 0.0f;
//		matrix.m[1][0] = newUp.x;		matrix.m[1][1] = newUp.y;		matrix.m[1][2] = newUp.z;		matrix.m[1][3] = 0.0f;
//		matrix.m[2][0] = newForward.x;	matrix.m[2][1] = newForward.y;	matrix.m[2][2] = newForward.z;	matrix.m[2][3] = 0.0f;
//		matrix.m[3][0] = pos.x;			matrix.m[3][1] = pos.y;			matrix.m[3][2] = pos.z;			matrix.m[3][3] = 1.0f;
//		return matrix;
//
//	}

	mat4x4 Matrix_QuickInverse(mat4x4 *m) // Only for Rotation/Translation Matrices
	{
		mat4x4 matrix;
		matrix.m[0][0] = m->m[0][0]; matrix.m[0][1] = m->m[1][0]; matrix.m[0][2] = m->m[2][0]; matrix.m[0][3] = 0.0f;
		matrix.m[1][0] = m->m[0][1]; matrix.m[1][1] = m->m[1][1]; matrix.m[1][2] = m->m[2][1]; matrix.m[1][3] = 0.0f;
		matrix.m[2][0] = m->m[0][2]; matrix.m[2][1] = m->m[1][2]; matrix.m[2][2] = m->m[2][2]; matrix.m[2][3] = 0.0f;
		matrix.m[3][0] = -(m->m[3][0] * matrix.m[0][0] + m->m[3][1] * matrix.m[1][0] + m->m[3][2] * matrix.m[2][0]);
		matrix.m[3][1] = -(m->m[3][0] * matrix.m[0][1] + m->m[3][1] * matrix.m[1][1] + m->m[3][2] * matrix.m[2][1]);
		matrix.m[3][2] = -(m->m[3][0] * matrix.m[0][2] + m->m[3][1] * matrix.m[1][2] + m->m[3][2] * matrix.m[2][2]);
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	Vect3D Vector_Add(Vect3D *v1, Vect3D *v2)
	{
		return (Vect3D){ v1->x + v2->x, v1->y + v2->y, v1->z + v2->z };
	}

	Vect3D Vector_Sub(Vect3D *v1, Vect3D *v2)
	{
		return (Vect3D){ v1->x - v2->x, v1->y - v2->y, v1->z - v2->z };
	}

	Vect3D Vector_Mul(Vect3D *v1, float k)
	{
		return (Vect3D){ v1->x * k, v1->y * k, v1->z * k };
	}

	Vect3D Vector_Div(Vect3D *v1, float k)
	{
		return (Vect3D){ v1->x / k, v1->y / k, v1->z / k };
	}

	float Vector_DotProduct(Vect3D *v1, Vect3D *v2)
	{
		return v1->x*v2->x + v1->y*v2->y + v1->z * v2->z;
	}

	float Vector_Length(Vect3D *v)
	{
		return sqrtf(Vector_DotProduct(v, v));
	}

	Vect3D Vector_Normalise(Vect3D *v)
	{
		float l = Vector_Length(v);
		return (Vect3D){ v->x / l, v->y / l, v->z / l };
	}

	Vect3D Vector_CrossProduct(Vect3D *v1, Vect3D *v2)
	{
		Vect3D v;
		v.x = v1->y * v2->z - v1->z * v2->y;
		v.y = v1->z * v2->x - v1->x * v2->z;
		v.z = v1->x * v2->y - v1->y * v2->x;
		return v;
	}

	void Matrix_Print(mat4x4 *m) // Only for printing matrices
	{
		printf("###################################################\r\n");
		printf("0,0: %f, 0,1: %f, 0,2: %f, 0,3: %f\r\n", m->m[0][0], m->m[0][1], m->m[0][2], m->m[0][3]);
		printf("1,0: %f, 1,1: %f, 1,2: %f, 1,3: %f\r\n", m->m[1][0], m->m[1][1], m->m[1][2], m->m[1][3]);
		printf("2,0: %f, 2,1: %f, 2,2: %f, 2,3: %f\r\n", m->m[2][0], m->m[2][1], m->m[2][2], m->m[2][3]);
		printf("3,0: %f, 3,1: %f, 3,2: %f, 3,3: %f\r\n", m->m[3][0], m->m[3][1], m->m[3][2], m->m[3][3]);
		printf("###################################################\r\n");

	}
