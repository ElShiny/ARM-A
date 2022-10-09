/*
 * 3DEngine.c
 *
 *  Created on: 15. 9. 2022
 *      Author: ElShiny
 */



/* **************** MODULE DESCRIPTION *************************

Ta modul implementira sistemske funkcije za simulacijo 3d objektov







************************************************************* */


// ----------- Include other modules (for private) -------------

#include "3DEngine.h"
#include "lcd.h"
#include "SCI.h"
#include "lcd_ili9341.h"
#include "mcp3464.h"
#include "pca9685.h"
#include "joystick.h"
#include <math.h>


// ---------------------- Private definitions ------------------



mat4x4 matProj;
OBJMesh cubeMesh = {

		// SOUTH
		 0.0, 0.0, 0.0,    0.5, 1.0, 0.5,    1.0, 0.0, 0.0 ,
		 1.0, 0.0, 0.0,    0.5, 1.0, 0.5,    0.5, 0.0, 1.0 ,

		// EAST
		 0.0, 0.0, 0.0,    0.5, 1.0, 0.5,    0.5, 0.0, 1.0 ,
		 0.0, 0.0, 0.0,    0.5, 0.0, 1.0,    1.0, 0.0, 0.0 ,

//		// NORTH
//		 1.0, 0.0, 1.0,    1.0, 1.0, 1.0,    0.0, 1.0, 1.0 ,
//		 1.0, 0.0, 1.0,    0.0, 1.0, 1.0,    0.0, 0.0, 1.0 ,
//
//		// WEST
//		 0.0, 0.0, 1.0,    0.0, 1.0, 1.0,    0.0, 1.0, 0.0 ,
//		 0.0, 0.0, 1.0,    0.0, 1.0, 0.0,    0.0, 0.0, 0.0 ,
//
//		// TOP
//		 0.0, 1.0, 0.0,    0.0, 1.0, 1.0,    1.0, 1.0, 1.0 ,
//		 0.0, 1.0, 0.0,    1.0, 1.0, 1.0,    1.0, 1.0, 0.0 ,
//
//		// BOTTOM
//		 1.0, 0.0, 1.0,    0.0, 0.0, 1.0,    0.0, 0.0, 0.0 ,
//		 1.0, 0.0, 1.0,    0.0, 0.0, 0.0,    1.0, 0.0, 0.0

		};



// -------------- Public function implementations --------------

void Init_3D(void){

	OBJMesh outMesh = {0};

 	printf("lamo\r\n");
	printf("%d\r\n", cubeMesh.t[11].v[2].x);

	float fNear = 0.1;
	float fFar = 1000.0;
	float fFov = 90.0;
	float fAspectRatio = (float)ILI9341_HEIGHT / (float)ILI9341_WIDTH;
	float fFovRad = 1.0 / tanf((fFov*0.5)/(180.0 * 3.14159));


	matProj.m[0][0] = fAspectRatio * fFovRad;
	matProj.m[1][1] = fFovRad;
	matProj.m[2][2] = fFar / (fFar - fNear);
	matProj.m[3][2] = (-fFar * fNear) / (fFar - fNear);
	matProj.m[2][3] = 1.0f;
	matProj.m[3][3] = 0.0f;

	mat4x4 matRotZ = {0}, matRotX = {0};


	float fTheta = 1.0f;

	while(true){
	// Rotation Z
	fTheta = (JOY_get_axis_position(X) * M_PI)/50;

	matRotZ.m[0][0] = cosf(fTheta);
	matRotZ.m[0][1] = sinf(fTheta);
	matRotZ.m[1][0] = -sinf(fTheta);
	matRotZ.m[1][1] = cosf(fTheta);
	matRotZ.m[2][2] = 1;
	matRotZ.m[3][3] = 1;

	//LCD_ClearScreen();


	for(int i = 0; i < MESH_SIZE; i++){

		Triangle triProjected, triTranslated, triRotatedZ, triRotatedZX;


		MatrixMult(&cubeMesh.t[i].v[0], &triRotatedZ.v[0], &matRotZ);
		MatrixMult(&cubeMesh.t[i].v[1], &triRotatedZ.v[1], &matRotZ);
		MatrixMult(&cubeMesh.t[i].v[2], &triRotatedZ.v[2], &matRotZ);

		// Offset into the screen
		triTranslated = triRotatedZ;
		triTranslated.v[0].z = triRotatedZ.v[0].z + 3.0f;
		triTranslated.v[1].z = triRotatedZ.v[1].z + 3.0f;
		triTranslated.v[2].z = triRotatedZ.v[2].z + 3.0f;


		MatrixMult(&triTranslated.v[0], &triProjected.v[0], &matProj);
		MatrixMult(&triTranslated.v[1], &triProjected.v[1], &matProj);
		MatrixMult(&triTranslated.v[2], &triProjected.v[2], &matProj);

		//scaling the model
		triProjected.v[0].x += 5.0f; triProjected.v[0].y += 6.0f;
		triProjected.v[1].x += 5.0f; triProjected.v[1].y += 6.0f;
		triProjected.v[2].x += 5.0f; triProjected.v[2].y += 6.0f;
		triProjected.v[0].x *= 0.1f * (float)ILI9341_WIDTH;
		triProjected.v[0].y *= 0.1f * (float)ILI9341_HEIGHT;
		triProjected.v[1].x *= 0.1f * (float)ILI9341_WIDTH;
		triProjected.v[1].y *= 0.1f * (float)ILI9341_HEIGHT;
		triProjected.v[2].x *= 0.1f * (float)ILI9341_WIDTH;
		triProjected.v[2].y *= 0.1f * (float)ILI9341_HEIGHT;

		//draw triangles
		outMesh.t[i] = triProjected;



		LCD_Triangle(triProjected.v[0].x, triProjected.v[0].y,
				triProjected.v[1].x, triProjected.v[1].y,
				triProjected.v[2].x, triProjected.v[2].y, 0xffff);
		//HAL_Delay(50);
	}
	HAL_Delay(50);
	LCD_DrawMesh(outMesh, 0x0000);
	//HAL_Delay(10);
	//HAL_Delay(50);


	}

	printf("end\r\n");
}

void MatrixMult(Vect3D *i, Vect3D *o, mat4x4 *m){

	o->x = i->x * m->m[0][0] + i->y * m->m[1][0] + i->z * m->m[2][0] + m->m[3][0];
	o->y = i->x * m->m[0][1] + i->y * m->m[1][1] + i->z * m->m[2][1] + m->m[3][1];
	o->z = i->x * m->m[0][2] + i->y * m->m[1][2] + i->z * m->m[2][2] + m->m[3][2];
	float w = i->x * m->m[0][3] + i->y * m->m[1][3] + i->z * m->m[2][3] + m->m[3][3];

	if (w != 0.0f)
	{
		o->x /= w; o->y /= w; o->z /= w;
	}

}
