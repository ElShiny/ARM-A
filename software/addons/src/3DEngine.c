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
#include "matrix_func.h"
#include "mpu6050.h"
#include <math.h>


// ---------------------- Private definitions ------------------



mat4x4 matProj = {0};
OBJMesh DlanMesh = {

		31,

		//dlan
		 0.0, 0.0, 0.0, 1,    2.0, 0.0, 0.0, 1,    0.0, 3.0, 0.0, 1,
		 2.0, 0.0, 0.0, 1,    3.0, 3.0, 0.0, 1,    0.0, 3.0, 0.0, 1,
		-1.0, 1.5, 0.0, 1,    0.0, 3.0, 0.0, 1,    0.0, 0.0, 0.0, 1,

		 0.0, 0.0, -1.0, 1,    2.0, 0.0, -1.0, 1,    0.0, 3.0, -1.0, 1,
		 2.0, 0.0, -1.0, 1,    3.0, 3.0, -1.0, 1,    0.0, 3.0, -1.0, 1,
		-1.0, 1.5, -1.0, 1,    0.0, 3.0, -1.0, 1,    0.0, 0.0, -1.0, 1,

		 0.0, 0.0, -1.0, 1,    2.0, 0.0, 0.0, 1,    0.0, 0.0, 0.0, 1,
		 0.0, 0.0, -1.0, 1,    2.0, 0.0,-1.0, 1,    2.0, 0.0, 0.0, 1,

		 0.0, 3.0, -1.0, 1,    3.0, 3.0, 0.0, 1,    0.0, 3.0, 0.0, 1,
		 0.0, 3.0, -1.0, 1,    3.0, 3.0,-1.0, 1,    3.0, 3.0, 0.0, 1,

		 0.0, 0.0, 0.0, 1,    -1.0, 1.5,-1.0, 1,    -1.0, 1.5, 0.0, 1,
		 0.0, 0.0, 0.0, 1,    -1.0, 1.5,-1.0, 1,    0.0, 0.0, -1.0, 1,

		 3.0, 3.0, -1.0, 1,    2.0, 0.0,-1.0, 1,    2.0, 0.0, 0.0, 1,
		 3.0, 3.0, -1.0, 1,    2.0, 0.0,-1.0, 1,    3.0, 3.0,-1.0, 1,

		 -1.0, 1.5, 0.0, 1,    0.0, 3.0, -1.0, 1,    0.0, 3.0, 0.0, 1,
		 -1.0, 1.5, 0.0, 1,    0.0, 3.0, -1.0, 1,   -1.0, 1.5,-1.0, 1,

		// thumb
		-1.0, 1.5, 0.0, 1,   -0.7, 2.0, 0.0, 1,   -1.0, 2.5, -0.5, 1,
		-1.0, 1.5, -1.0, 1,   -0.7, 2.0, -1.0, 1,   -1.0, 2.5, -0.5, 1,
		-1.0, 1.5, 0.0, 1,   -0.7, 2.0, 0.0, 1,   -1.0, 2.5, -0.5, 1,

		// index finger
		 0.0, 3.0, 0.0, 1,   0.75, 3.0, 0.0, 1,   0.375, 4.0, -0.5, 1,
		 0.0, 3.0, 0.0, 1,   0.375,3.0,-1.0, 1,   0.375, 4.0, -0.5, 1,
		 0.75, 3.0, 0.0, 1,  0.375,3.0,-1.0, 1,   0.375, 4.0, -0.5, 1,

		 // middle finger
		 0.0+0.75, 3.0, 0.0, 1,   0.75+0.75, 3.0, 0.0, 1,   0.375+0.75, 4.0, -0.5, 1,
		 0.0+0.75, 3.0, 0.0, 1,   0.375+0.75,3.0,-1.0, 1,   0.375+0.75, 4.0, -0.5, 1,
		 0.75+0.75, 3.0, 0.0, 1,  0.375+0.75,3.0,-1.0, 1,   0.375+0.75, 4.0, -0.5, 1,


		 0.0+0.75*2, 3.0, 0.0, 1,   0.75+0.75*2, 3.0, 0.0, 1,   0.375+0.75*2, 4.0, -0.5, 1,
		 0.0+0.75*2, 3.0, 0.0, 1,   0.375+0.75*2,3.0,-1.0, 1,   0.375+0.75*2, 4.0, -0.5, 1,
		 0.75+0.75*2, 3.0, 0.0, 1,  0.375+0.75*2,3.0,-1.0, 1,   0.375+0.75*2, 4.0, -0.5, 1,


		 0.0+0.75*3, 3.0, 0.0, 1,   0.75+0.75*3, 3.0, 0.0, 1,   0.375+0.75*3, 4.0, -0.5, 1,
		 0.0+0.75*3, 3.0, 0.0, 1,   0.375+0.75*3,3.0,-1.0, 1,   0.375+0.75*3, 4.0, -0.5, 1,
		 0.75+0.75*3, 3.0, 0.0, 1,  0.375+0.75*3,3.0,-1.0, 1,   0.375+0.75*3, 4.0, -0.5, 1,

		};

OBJMesh boneMesh = {

		4,

		// SOUTH
		 0.0, 0.0, 0.5, 1,   -0.5, 0.0,-0.5, 1,    0.0, 1.0, 0.0 , 1,
		 0.0, 0.0, 0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 1.0, 0.0 , 1,
		-0.5, 0.0,-0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 1.0, 0.0 , 1,
		-0.5, 0.0,-0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 0.0, 0.5 , 1,

		};



// -------------- Public function implementations --------------

void Init_3D(void){

	OBJMesh tempMesh = {31};

 	printf("lamo\r\n");

	float fNear = 0.1;
	float fFar = 1000.0;
	float fFov = 90.0;
	float fAspectRatio = (float)ILI9341_HEIGHT / (float)ILI9341_WIDTH;
	//float fFovRad = 1.0 / tanf((fFov*0.5)/(180.0 * 3.14159));


	matProj = Matrix_MakeProjection(fFov, fAspectRatio, fNear, fFar);


	//LCD_ClearScreen();

	mat4x4 matRotZ = {0}, matTransl = {0}, matRotY = {0};


	while(true){

	analogRead();

	HAL_Delay(10);
	LCD_ClearScreen();

	matRotZ = Matrix_MakeRotationAll(hand1_mpu.yaw * 0.01745, hand1_mpu.pitch * 0.01745, hand1_mpu.roll * 0.01745);
	//matTransl = Matrix_MakeTranslation(1, 0, -0.5);


	for(int i = 0; i < DlanMesh.size; i++){

		Triangle triTranslated, triRotatedZ;

		triRotatedZ = Matrix_MultiplyTriangle(&matRotZ, &DlanMesh.t[i]);



		// Offset into the screen
		triTranslated = triRotatedZ;
		triTranslated.v[0].z = triRotatedZ.v[0].z + 8.0f;
		triTranslated.v[1].z = triRotatedZ.v[1].z + 8.0f;
		triTranslated.v[2].z = triRotatedZ.v[2].z + 8.0f;
		tempMesh.t[i] = triTranslated;


	}

	Project_And_Draw(&matProj, tempMesh);

	Project_And_Draw(&matProj, Draw_Finger(&matRotZ));



	}


	printf("end\r\n");
}

OBJMesh Draw_Finger(mat4x4 *MatHand){
	mat4x4 TransMat, RotMat;
	Triangle triTranslated, triRotated1, triRotated;
	OBJMesh tempMesh = {boneMesh.size};

	//TransMat = Matrix_MakeTranslation(-1, 2.5, -0.5);
	RotMat = Matrix_MakeRotationX(hand.position_raw[0]*0.001);
	//printf("pos1 adc: %d\r\n", hand.position_raw[0]);
//	TransMat1 = Matrix_MakeTranslation(0, 0, -8);

	for(int i = 0; i < boneMesh.size; i++){
	//triTranslated = Matrix_MultiplyTriangle(&TransMat, &boneMesh.t[i]);
	//triTranslated = Matrix_MultiplyTriangle(&TransMat1, &triTranslated);
		// Offset into the screen

		triRotated = Matrix_MultiplyTriangle(&RotMat, &boneMesh.t[i]);

		//triTranslated = Matrix_MultiplyTriangle(&TransMat, &triRotated);

		triRotated1 = Matrix_MultiplyTriangle(MatHand, &triRotated);

		//triRotated1 = Matrix_MultiplyTriangle(&RotMat, &boneMesh.t[i]);




		printf("trans1 x: %f, y: %f, z: %f, w: %f\r\n", triRotated.v[0].x, triRotated.v[0].y, triRotated.v[0].z, triRotated.v[0].w);
		tempMesh.t[i] = triRotated1;
	}

return tempMesh;

}

void Project_And_Draw(mat4x4 *matProj, OBJMesh Mesh){

	Triangle triProjected;

	OBJMesh outMesh = {Mesh.size};


	for(int i = 0; i < Mesh.size; i++){

	triProjected.v[0] = Matrix_MultiplyVector(matProj, &Mesh.t[i].v[0]);
	triProjected.v[1] = Matrix_MultiplyVector(matProj, &Mesh.t[i].v[1]);
	triProjected.v[2] = Matrix_MultiplyVector(matProj, &Mesh.t[i].v[2]);

	//scaling the model
	triProjected.v[0].x += 30.0f; triProjected.v[0].y += 30.0f;
	triProjected.v[1].x += 30.0f; triProjected.v[1].y += 30.0f;
	triProjected.v[2].x += 30.0f; triProjected.v[2].y += 30.0f;


	triProjected.v[0].x *= 0.01f * (float)ILI9341_WIDTH;
	triProjected.v[0].y *= 0.01f * (float)ILI9341_HEIGHT;
	triProjected.v[1].x *= 0.01f * (float)ILI9341_WIDTH;
	triProjected.v[1].y *= 0.01f * (float)ILI9341_HEIGHT;
	triProjected.v[2].x *= 0.01f * (float)ILI9341_WIDTH;
	triProjected.v[2].y *= 0.01f * (float)ILI9341_HEIGHT;

	//draw triangles
	outMesh.t[i] = triProjected;

	}

//	LCD_Triangle(triProjected.v[0].x, triProjected.v[0].y,
//			triProjected.v[1].x, triProjected.v[1].y,
//			triProjected.v[2].x, triProjected.v[2].y, 0xffff);
//	//HAL_Delay(50);
//	}

	LCD_DrawMesh(outMesh, 0xffff);


}
