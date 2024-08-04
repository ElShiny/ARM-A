/*
 * mcp3464.c
 *
 *  Created on: 17 Sep 2022
 *      Author: Matej
 */


/* **************** MODULE DESCRIPTION *************************

Ta modul implementira komunikacijo in obdelavo podatkov Zunanjega
AD pretvornika MCP3464. Modul je implementiran na podlagi pristopa
s pomočjo DMA enote (angl. DMA mode), kjer DMA enota poskrbi za
prenos rezultatov AD pretvornika v sistemske spremenljivke.

Modul omogoča:

	- kalibracijo osi "joysticka",

	- branje trenutne relativne pozicije "joystika" (izražene v
	 	 procentualnem deležu polnega odklona osi),

	- detekcijo pritiska tipke,

	- branje trenutnega stanja tipke,

	- demonstracijo delovanja "joysticka" (s pomočjo SCI in LED
		modulov)


************************************************************* */

#include "mcp3464.h"
#include "SCI.h"
#include "kbd.h"
#include <math.h>



SPI_HandleTypeDef 	MCP3464_SPI_PORT;
//MCP3464 hand;

#define CFG0 0X22
#define CFG1 0X18
#define CFG2 0X8B
#define CFG3 0X80
#define IRQ  0X07
#define MUX  0X0C


int direction[16] = {1,-1, 1, 1, 1, -1, 1, -1, -1, -1, 1, -1, 1, -1, -1};



static void MCP3464_ChipSelect(uint8_t cs)
{
	if(cs == 1){
		WRITE_REG(MCP3464_CS1_GPIO_Port->BRR, MCP3464_CS1_Pin);
	}
	if(cs == 2){
		WRITE_REG(MCP3464_CS2_GPIO_Port->BRR, MCP3464_CS2_Pin);
	}
	if(cs == 0){
		WRITE_REG(MCP3464_CS1_GPIO_Port->BRR, MCP3464_CS1_Pin);
		WRITE_REG(MCP3464_CS2_GPIO_Port->BRR, MCP3464_CS2_Pin);
	}

}



static void MCP3464_ChipsDeselect()
{
	WRITE_REG(MCP3464_CS1_GPIO_Port->BSRR, MCP3464_CS1_Pin);
	WRITE_REG(MCP3464_CS2_GPIO_Port->BSRR, MCP3464_CS2_Pin);
}

void MCP_init(){
										//	conf0		conf1		conf2		conf3		IRQ			MUX			SCAN
	static const uint8_t mcp_init_data[] = {0b11100000, 0b00001100, 0b10001011, 0b10000000, 0b00000010, 0b00001000, 0b10000000, 0, 0b11111111};
	//uint8_t irq = 0b00000111;

	fastCMD(0b01111000, 0);
	HAL_Delay(50);
	writeReg(0x1, (uint8_t*) mcp_init_data, 9, 0);

	//start a simple adc test
	analogRead();
	HAL_Delay(1000);
	if (hand.pointer != 16) {
		for (int i = 0; i < hand.pointer; i++) {
			printf("adc val: %d, pointer: %d\r\n", (int16_t) hand.position_raw[i], i);
		}
		printf("\r\n");
	}
	else{printf("ADC Initialisation finished\r\n");}

}

void analogRead(){

	fastCMD(0b01101100, 0);
	hand.pointer = 0;

	//uint8_t data = (ch << 4) | 0b1000;

	//writeReg(0x6, &data, 1, 1);
	fastCMD(0b01101000, 1);

}

void writeReg(uint8_t reg, uint8_t* data, uint8_t len, uint8_t cs){

	uint8_t reg_data =  0b01000010 | (reg << 2);

	MCP3464_ChipSelect(cs);
	HAL_SPI_Transmit(&MCP3464_SPI_PORT, &reg_data, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&MCP3464_SPI_PORT, data, len, HAL_MAX_DELAY);
	MCP3464_ChipsDeselect();
}

void readReg(uint8_t reg, uint8_t* data, uint8_t len, uint8_t cs){

	uint8_t reg_data =  0b01000001 | (reg << 2);

	MCP3464_ChipSelect(cs);
	HAL_SPI_TransmitReceive(&MCP3464_SPI_PORT, &reg_data, data, len, HAL_MAX_DELAY);
	MCP3464_ChipsDeselect();
	//printf("adc val: %d\r\n", (int16_t)hand.position_raw[hand.pointer]);
}


void fastCMD(uint8_t cmd, uint8_t cs){

	MCP3464_ChipSelect(cs);
	HAL_SPI_Transmit(&MCP3464_SPI_PORT, &cmd, 1, HAL_MAX_DELAY);
	MCP3464_ChipsDeselect();
}

void MCP3464_Calibrate(void){

	printf("############################################\r\n");
	printf("Lay the hand flat on a table and press OK button\r\n");

	while(KBD_get_pressed_key() != BTN_OK){}

	analogRead();
	HAL_Delay(300);

	if (hand.pointer == 16) {
		for (int i = 0; i < hand.pointer; i++) {
			//printf("adc val: %d, pointer: %d\r\n", (int16_t) hand.position_raw[i], i);
			hand.position_offset_streched[i] = hand.position_raw[i];
		}
		printf("\r\n");
	}

	printf("Make a fist and press OK button\r\n");
	while(KBD_get_pressed_key() != BTN_OK){}

	analogRead();
	HAL_Delay(300);

	if (hand.pointer == 16) {
		for (int i = 0; i < hand.pointer; i++) {
			//printf("adc val: %d, pointer: %d\r\n", (int16_t) hand.position_raw[i], i);
			hand.position_offset_squished[i] = hand.position_raw[i];
			hand.position_offset_squished[i] = M_PI / hand.position_offset_squished[i];
		}
		printf("\r\n");
	}

	printf("############################################\r\n");

}

void MCP3464_Radians(void){

	for (int i = 0; i < 16; i++) {
		//printf("adc val: %d, pointer: %d\r\n", (int16_t) hand.position_raw[i], i);
		hand.position_rad[i] = hand.position_call[i] * hand.position_offset_squished[i];
	}
}

void EXTIValue_callback() {

	uint8_t buffer[3] = {0};
	uint8_t n = 1;

	if(hand.pointer <= 7)n = 1;
	else n = 2;

	readReg(0x0, buffer, 3, n);

	if(!(buffer[0] & 0x04)){//check if this is the first reading of value

	hand.position_raw[hand.pointer] = buffer[1] << 8 | buffer[2];
	hand.position_call[hand.pointer] = (hand.position_raw[hand.pointer] - hand.position_offset_streched[hand.pointer])*direction[hand.pointer];
	//printf("adc val: %d, pointer: %d, mcpRead: %x\r\n", (int16_t)hand.position_raw[hand.pointer], hand.pointer, n);
	hand.pointer = hand.pointer + 1;

	if (hand.pointer == 8){fastCMD(0b01101000, 2);}
	}
}


