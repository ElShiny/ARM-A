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

OBJMesh ThumbMesh = {

		4,

		// SOUTH
		 0.0, 0.0, 0.5, 1,   -0.5, 0.0,-0.5, 1,    0.0, 1.3, 0.0 , 1,
		 0.0, 0.0, 0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 1.3, 0.0 , 1,
		-0.5, 0.0,-0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 1.3, 0.0 , 1,
		-0.5, 0.0,-0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 0.0, 0.5 , 1,

		};

OBJMesh MiddleMesh = {

		4,

		// SOUTH
		 0.0, 0.0, 0.5, 1,   -0.5, 0.0,-0.5, 1,    0.0, 1.75, 0.0 , 1,
		 0.0, 0.0, 0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 1.75, 0.0 , 1,
		-0.5, 0.0,-0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 1.75, 0.0 , 1,
		-0.5, 0.0,-0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 0.0, 0.5 , 1,

		};

OBJMesh FingerMesh = {

		4,

		// SOUTH
		 0.0, 0.0, 0.5, 1,   -0.5, 0.0,-0.5, 1,    0.0, 1.5, 0.0 , 1,
		 0.0, 0.0, 0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 1.5, 0.0 , 1,
		-0.5, 0.0,-0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 1.5, 0.0 , 1,
		-0.5, 0.0,-0.5, 1,    0.5, 0.0,-0.5, 1,    0.0, 0.0, 0.5 , 1,

		};



// -------------- Public function implementations --------------

void Init_3D(void){

 	printf("lamo\r\n");

 	OBJMesh tempMesh = {31};
	float fNear = 0.1;
	float fFar = 1000.0;
	float fFov = 90.0;
	float fAspectRatio = (float)ILI9341_HEIGHT / (float)ILI9341_WIDTH;


	matProj = Matrix_MakeProjection(fFov, fAspectRatio, fNear, fFar);

	mat4x4 matRotZ = {0}, matTransl = {0}, matMult = {0};
	float theta = 0;


	while(true){

	analogRead();
	MCP3464_Radians();
	theta += 0.1;
									//yaw pitch roll
	matRotZ = Matrix_MakeRotationAll(-hand1_mpu.yaw * 0.01745, hand1_mpu.pitch * 0.01745 + M_PI, hand1_mpu.roll * 0.01745);
	//printf("pitch: %f, yaw: %f, roll: %f\r\n", hand1_mpu.pitch , hand1_mpu.yaw ,  hand1_mpu.roll );
	matTransl = Matrix_MakeTranslation(-1, 0, 0.5);
	matMult = Matrix_MultiplyMatrix(&matRotZ, &matTransl);


	for(int i = 0; i < DlanMesh.size; i++){

		Triangle triTranslated = {0};

		triTranslated = Matrix_MultiplyTriangle(&matMult, &DlanMesh.t[i]);
		//triTranslated = Matrix_MultiplyTriangle(&matMult, &triRotatedZ);
		tempMesh.t[i] = triTranslated;

	}

	Project_And_Draw(&matProj, tempMesh);

	Draw_Thumb(&matMult);
	Draw_Finger(&matMult, &MiddleMesh, (Vect3D){0.375+0.75, 4.0, -0.5, 1}, 1.75, 8, 7, 6);
	Draw_Finger(&matMult, &FingerMesh, (Vect3D){0.375, 4.0, -0.5, 1}, 1.5, 5, 4, 3);
	Draw_Finger(&matMult, &FingerMesh, (Vect3D){0.375+0.75*2, 4.0, -0.5, 1}, 1.5, 11, 9, 10);
	Draw_Finger(&matMult, &ThumbMesh, (Vect3D){0.375+0.75*3, 4.0, -0.5, 1}, 1.3, 14, 13, 12);



	HAL_Delay(30);
	LCD_ClearScreen();
	}



	printf("end\r\n");
}

void Draw_Thumb(mat4x4 *MatHand){
	mat4x4 TransMat, RotMat, RotMat2, matMult, matMult1, matMult2, matClen2;
	Triangle tri1, tri2;
	OBJMesh tempMesh1 = {ThumbMesh.size};
	OBJMesh tempMesh2 = {ThumbMesh.size};

	//printf("d\r\n");

	TransMat = Matrix_MakeTranslation(-1, 2.5, -0.5);
	matMult = Matrix_MultiplyMatrix(MatHand, &TransMat);
	RotMat = Matrix_MakeRotationX(hand.position_rad[1]);
	matMult1 = Matrix_MultiplyMatrix(&matMult, &RotMat);
	matClen2 = Matrix_MakeTranslation(0, 1.3, 0);
	matClen2 = Matrix_MultiplyMatrix(&matMult1, &matClen2);
	RotMat2 = Matrix_MakeRotationX(hand.position_rad[0]);
	matMult2 = Matrix_MultiplyMatrix(&matClen2, &RotMat2);



	for(int i = 0; i < ThumbMesh.size; i++){

		tri1 = Matrix_MultiplyTriangle(&matMult1, &ThumbMesh.t[i]);
		tri2 = Matrix_MultiplyTriangle(&matMult2, &ThumbMesh.t[i]);

		tempMesh1.t[i] = tri1;
		tempMesh2.t[i] = tri2;
	}

	Project_And_Draw(&matProj, tempMesh1);
	Project_And_Draw(&matProj, tempMesh2);
}


void Draw_Finger(mat4x4 *MatHand, OBJMesh *Mesh, Vect3D tipPos, float tip_length, uint8_t adc1, uint8_t adc2, uint8_t adc3){
	mat4x4 matTrans, RotMat, matMult, matMult1, matMult2, matMult3;
	Triangle tri1, tri2, tri3;
	OBJMesh tempMesh1 = {Mesh->size};
	OBJMesh tempMesh2 = {Mesh->size};
	OBJMesh tempMesh3 = {Mesh->size};

	//printf("d\r\n");

	matTrans = Matrix_MakeTranslation(tipPos.x, tipPos.y, tipPos.z);
	matMult = Matrix_MultiplyMatrix(MatHand, &matTrans);
	RotMat = Matrix_MakeRotationX(hand.position_rad[adc1]);
	matMult1 = Matrix_MultiplyMatrix(&matMult, &RotMat);

	matTrans = Matrix_MakeTranslation(0, tip_length, 0);
	matMult = Matrix_MultiplyMatrix(&matMult1, &matTrans);
	RotMat = Matrix_MakeRotationX(hand.position_rad[adc2]);
	matMult2 = Matrix_MultiplyMatrix(&matMult, &RotMat);

	matTrans = Matrix_MakeTranslation(0, tip_length, 0);
	matMult = Matrix_MultiplyMatrix(&matMult2, &matTrans);
	RotMat = Matrix_MakeRotationX(hand.position_rad[adc3]);
	matMult3 = Matrix_MultiplyMatrix(&matMult, &RotMat);



	for(int i = 0; i < ThumbMesh.size; i++){

		tri1 = Matrix_MultiplyTriangle(&matMult1, &Mesh->t[i]);
		tri2 = Matrix_MultiplyTriangle(&matMult2, &Mesh->t[i]);
		tri3 = Matrix_MultiplyTriangle(&matMult3, &Mesh->t[i]);

		tempMesh1.t[i] = tri1;
		tempMesh2.t[i] = tri2;
		tempMesh3.t[i] = tri3;
	}

	Project_And_Draw(&matProj, tempMesh1);
	Project_And_Draw(&matProj, tempMesh2);
	Project_And_Draw(&matProj, tempMesh3);
}

void Project_And_Draw(mat4x4 *matProj, OBJMesh Mesh){

	OBJMesh outMesh = {Mesh.size};

	for(int i = 0; i < Mesh.size; i++){
	Triangle triProjected = {0};

//	printf("d\r\n");

	//triProjected = Matrix_MultiplyTriangle(matProj, &Mesh.t[i]);
	//printf("mesh x: %f, y: %f, z: %f, w: %f\r\n", Mesh.t[i].v[0].x, Mesh.t[i].v[0].y, Mesh.t[i].v[0].z, Mesh.t[i].v[0].w);

	Mesh.t[i].v[0].z = Mesh.t[i].v[0].z + 20.0;
	Mesh.t[i].v[1].z = Mesh.t[i].v[1].z + 20.0;
	Mesh.t[i].v[2].z = Mesh.t[i].v[2].z + 20.0;

	triProjected.v[0] = Matrix_MultiplyVector(matProj, &Mesh.t[i].v[0]);
	triProjected.v[1] = Matrix_MultiplyVector(matProj, &Mesh.t[i].v[1]);
	triProjected.v[2] = Matrix_MultiplyVector(matProj, &Mesh.t[i].v[2]);

	//printf("trans0 x: %f, y: %f, z: %f, w: %f\r\n", triProjected.v[0].x, triProjected.v[0].y, triProjected.v[0].z, triProjected.v[0].w);


	triProjected.v[0].x *= (float)ILI9341_WIDTH_SCALE;
	triProjected.v[0].y *= (float)ILI9341_HEIGHT_SCALE;
	triProjected.v[1].x *= (float)ILI9341_WIDTH_SCALE;
	triProjected.v[1].y *= (float)ILI9341_HEIGHT_SCALE;
	triProjected.v[2].x *= (float)ILI9341_WIDTH_SCALE;
	triProjected.v[2].y *= (float)ILI9341_HEIGHT_SCALE;

	//printf("trans1 x: %f, y: %f, z: %f, w: %f\r\n", triProjected.v[0].x, triProjected.v[0].y, triProjected.v[0].z, triProjected.v[0].w);

	//scaling the model
	triProjected.v[0].x += ILI9341_WIDTH_2; triProjected.v[0].y += 0;
	triProjected.v[1].x += ILI9341_WIDTH_2; triProjected.v[1].y += 0;
	triProjected.v[2].x += ILI9341_WIDTH_2; triProjected.v[2].y += 0;


	//draw triangles
	outMesh.t[i] = triProjected;
	//printf("trans2 x: %f, y: %f, z: %f, w: %f\r\n", triProjected.v[0].x, triProjected.v[0].y, triProjected.v[0].z, triProjected.v[0].w);

	}

	LCD_DrawMesh(outMesh, 0xffff);

}
