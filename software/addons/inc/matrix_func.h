/*
 * matrix_func.h
 *
 *  Created on: Oct 9, 2022
 *      Author: Matej
 */

#ifndef INC_MATRIX_FUNC_H_
#define INC_MATRIX_FUNC_H_





Vect3D Matrix_MultiplyVector(mat4x4 *m, Vect3D *i);
Triangle Matrix_MultiplyTriangle(mat4x4 *m, Triangle *i);
Triangle Vec_MakeTranslation(Triangle *triRotated, float x, float y, float z);
mat4x4 Matrix_MakeIdentity();
mat4x4 Matrix_MakeRotationX(float fAngleRad);
mat4x4 Matrix_MakeRotationY(float fAngleRad);
mat4x4 Matrix_MakeRotationZ(float fAngleRad);
mat4x4 Matrix_MakeRotationAll(float fXAngleRad, float fYAngleRad, float fZAngleRad);
mat4x4 Matrix_MakeTranslation(float x, float y, float z);
mat4x4 Matrix_MakeProjection(float fFovDegrees, float fAspectRatio, float fNear, float fFar);
mat4x4 Matrix_MultiplyMatrix(mat4x4 *m1, mat4x4 *m2);
mat4x4 Matrix_QuickInverse(mat4x4 *m);
Vect3D Vector_Add(Vect3D *v1, Vect3D *v2);
Vect3D Vector_Sub(Vect3D *v1, Vect3D *v2);
Vect3D Vector_Mul(Vect3D *v1, float k);
Vect3D Vector_Div(Vect3D *v1, float k);
float Vector_DotProduct(Vect3D *v1, Vect3D *v2);
float Vector_Length(Vect3D *v);
Vect3D Vector_Normalise(Vect3D *v);
Vect3D Vector_CrossProduct(Vect3D *v1, Vect3D *v2);
void Matrix_Print(mat4x4 *m);


#endif /* INC_MATRIX_FUNC_H_ */
