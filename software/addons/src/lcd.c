/*
 * Grafični gonilnik za platformo MiŠKo3.
 * Vsebuje:
 *   - lcd.{c,h}                Inicializacija in osnovno upravljanje zaslona
 *   - lcd_ili9341.{c,h}        Strojni vmesniki za inicializacijo in nastavljanje
 *                              krmilnika Ili9341 prek STMovega pomnilniškega vmesnika FMC
 *   - lcd_ili9341_registers.h  Seznam ukazov iz podatkovnega lista krmilnika Ili9341
 *
 *
 * (C) 2022 Nejc Bertoncelj <nbertoncelj afna student.uni-lj.si>
 *
 * Deli povzeti po vmesniku platforme MiŠKo2
 * (C) 2015 Pirc, Jankovec, Matič et al, Fakulteta za elektrotehniko
 */


#include "lcd.h"
#include "SCI.h"







// ------------------ Privatni prototipi funkcij ------------------------------








// ---------------- Osnovno upravljanje LCD zaslona preko ILI9341 vmesnika ----------------


/*!
 * @brief Nizkonivojska inicializacija zaslona
 * @internal
 */
static void LCD_IO_Init()
{
	LCD_RST_LOW();
	HAL_Delay(120);
	LCD_RST_HIGH();
	HAL_Delay(120);
}

/*!
 * @brief Strojno pospešen izris polnega pravokotnega polja
 * @param x x koordinata izhodišča
 * @param y y koordinata izhodišča
 * @param h višina polja
 * @param w širina polja
 * @param c podatek o barvi
 * @internal
 *
 * Funkcija izbere želeno območje, potem pa tolikokrat pošlje izbrano barvo,
 * kolikor slikovnih točk je potrebnih.
 */
void LCD_FillRect(uint32_t x, uint32_t y, uint32_t w, uint32_t h, uint16_t c)
{
	uint32_t max_count   = ILI9341_GetParam(LCD_AREA);     /* Št. vseh pikslov     */
	uint32_t pixel_count = ((w + 1) - x) * ((h + 1) - y);  /* Dejansko št. pikslov */

	if(pixel_count > max_count)
		pixel_count = max_count;

	// Izbor koordinat piksla
	ILI9341_SetDisplayWindow(x, y, w, h);

	ILI9341_SendRepeatedData(c, pixel_count);
	//while (pixel_count) {
	//	LCD_IO_SendData((uint16_t *)&c, LCD_IO_DATA_WRITE_CYCLES);
	//	pixel_count--;
	//}
}

//function copied from ugui library
void LCD_DrawLine(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint16_t c)
{
	   int16_t n, dx, dy, sgndx, sgndy, dxabs, dyabs, x, y, drawx, drawy;

	   dx = x2 - x1;
	   dy = y2 - y1;
	   dxabs = (dx>0)?dx:-dx;
	   dyabs = (dy>0)?dy:-dy;
	   sgndx = (dx>0)?1:-1;
	   sgndy = (dy>0)?1:-1;
	   x = dyabs >> 1;
	   y = dxabs >> 1;
	   drawx = x1;
	   drawy = y1;

	   if(!(drawx>ILI9341_WIDTH || drawy > ILI9341_HEIGHT)) UserPixelSetFunction(drawx, drawy, c);

	   if( dxabs >= dyabs )
	   {
	      for( n=0; n<dxabs; n++ )
	      {
	         y += dyabs;
	         if( y >= dxabs )
	         {
	            y -= dxabs;
	            drawy += sgndy;
	         }
	         drawx += sgndx;
	         if(!(drawx>ILI9341_WIDTH || drawy > ILI9341_HEIGHT)) UserPixelSetFunction(drawx, drawy, c);
	      }
	   }
	   else
	   {
	      for( n=0; n<dyabs; n++ )
	      {
	         x += dxabs;
	         if( x >= dyabs )
	         {
	            x -= dyabs;
	            drawx += sgndx;
	         }
	         drawy += sgndy;
	         if(!(drawx>ILI9341_WIDTH || drawy > ILI9341_HEIGHT)) UserPixelSetFunction(drawx, drawy, c);
	      }
	   }
}

void LCD_DrawFastHLine(uint32_t x, uint32_t y, uint32_t h, uint16_t c){

	ILI9341_SetDisplayWindow(x, y, h, 0);
	ILI9341_SendRepeatedData(c, h);
}

void LCD_Triangle(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t x3, uint32_t y3, uint16_t c){

	LCD_DrawLine(x1, y1, x2, y2, c);
	LCD_DrawLine(x2, y2, x3, y3, c);
	LCD_DrawLine(x3, y3, x1, y1, c);
}

void LCD_DrawMesh(OBJMesh mesh, uint16_t c){

	for(int i = 0; i< mesh.size; i++){

		LCD_Triangle(mesh.t[i].v[0].x, mesh.t[i].v[0].y,
				mesh.t[i].v[1].x, mesh.t[i].v[1].y,
				mesh.t[i].v[2].x, mesh.t[i].v[2].y, c);
	}
}

//draws 8 by 8 dot
void LCD_DrawDot(uint32_t x, uint32_t y, uint16_t c){
	uint8_t dot[8] = {0b00111100, 0b01111110, 0b11111111, 0b11111111, 0b11111111, 0b11111111, 0b01111110, 0b00111100};
	uint16_t tmp;

	ILI9341_SetDisplayWindow(x, y, 8, 8);
	for(int i = 0; i < 8; i++){
		for(int j = 0; j < 8; j++){
			tmp = (dot[i] & 1<<j)? c:0x0;
			ILI9341_SendData((LCD_IO_Data_t *)&tmp , 1);
		}
	}
}




/*!
 * @brief Glavna inicializacija zaslona
 *
 * Inicializacija nastavi privzet barvni prostor (RGB565), orientacijo zaslona, vklopi osvetlitev,
 * zapolni zaslon s črno barvo in nastavi bel tekst na črni podlagi.
 * @note Za inicializacijo posameznih strojnih enot mora poskrbeti uporabnik (npr. FMC)
 */
void LCD_Init()
{
	// Resetiraj V/I linijo zaslona
	LCD_IO_Init();

	// Inicializiraj krmilnik Ili9341 v barvnem prostoru RBG565 in brez zasuka slike
	ILI9341_Init(ILI9341_COLORSPACE_RBG565, ILI9341_MISKO_ROTATE_0);
	ILI9341_DisplayOn();

	LCD_ClearScreen();
	// Gornja funkcija se bo zaključila pred izbrisom celega zaslona, ker si FMC zapomni vse
	// ukaze in sporoči, da se je zaključil prenos, ne pa pošiljanje.
	// Brez zamika bo zaslon za trenutek utripnil belo. Prej:  HAL_Delay(25);
	ILI9341_WaitTransfer();

	// Inicializiramo še osvetlitev LCD zaslona.
	LCD_BKLT_init();
}





/*!
 * @brief Počisti zaslon (prebarvaj s črno barvo)
 */
void LCD_ClearScreen()
{
    LCD_FillRect(0, 0, ILI9341_GetParam(LCD_HEIGHT), ILI9341_GetParam(LCD_WIDTH), 0);
}


void UserPixelSetFunction(int16_t x, int16_t y, uint16_t c)
{
	ILI9341_SetDisplayWindow(x, y, 1, 1);
	ILI9341_SendData((LCD_IO_Data_t *)&c, 1);
}



// ------------------ Testne demo funkcije ----------------


/*!
 * @brief Preprosta demonstracija delovanja LCD zaslona s pomočjo funkcij
 * za neposredno delo z LCD krmilnikom ILI9341.
 */
void LCD_demo_simple()
{
	// Izberemo si barvo. Sami moramo premisliti "kodo", ki kodira barvo,
	// saj nimamo na voljo nekakšne pred-definirane barvne lestvice,
	// iz katere bi lahko izbrali eno od barv.
	uint16_t color = 0x0; 	// rdeča?

	// Nastavimo pozicijo in velikost okna, kjer bomo nato
	// izrisali obarvane piksle.
	ILI9341_SetDisplayWindow(0, 0, 320, 240);

	// In nato barvamo okno piksel za pikslom
	for (uint32_t i=0; i<320*240; i++)
	{
		// z uporabo preprostega ILI9341 ukaza.
		ILI9341_SendData(&color, 1);
	}

	LCD_DrawDot(100, 100, 0xfafa);
	LCD_Triangle(10, 10, 100, 100, 50, 200, 0xffff);
}





