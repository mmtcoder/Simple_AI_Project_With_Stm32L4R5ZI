/*
 * ImageProccess.c
 *
 *  Created on: Oct 25, 2021
 *      Author: H3RK3S
 */

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "sdCard.h"


extern uint8_t vsyncIT;


extern TIM_HandleTypeDef htim6; // 25 nano Second
extern TIM_HandleTypeDef htim7; // 100 nano Seconds


#define DELAYTWENTYFIVENANO(time)   \
	__HAL_TIM_SET_COUNTER(&htim6,0); \
	while( __HAL_TIM_GET_COUNTER(&htim6) < (time));

#define DELAYONEHUNDRENDNANO(time) \
	__HAL_TIM_SET_COUNTER(&htim7,0);\
	while( __HAL_TIM_GET_COUNTER(&htim7) < (time));




uint8_t  imageBuffer[120][320]  ;
uint8_t  copiedImage[IMAGE_HEIGHT/2][IMAGE_WIDTH];
uint8_t  compareCurrentImage[IMAGE_HEIGHT/2][IMAGE_WIDTH];

uint8_t horizImageBuffer[320];

uint32_t getScannedImageSize =(IMAGE_HEIGHT* IMAGE_WIDTH *2)/4;
uint8_t currentRowHorizantalData = 0;
uint16_t timerResult =0;
uint16_t timerOneHundredResult =0;
uint32_t imageBufSize = 38400;

	//External variables
extern  uint8_t cameraWriteOrRead ;
extern imageAIprocess_t   imageProcess ;


//In order to obtain 160 horizantel pixel for LCD you need to catch 320 pixel
//(8 bit = 1 pixel for Camera)
const uint16_t horizantelPixel = 320;
const uint8_t verticalPixel = 120;
//I converted according to TwentyFiveNano Delay. Pclk Period = 1333 micro second
const uint8_t pclkPeriodTime = 54;
const uint8_t halfPclkPeriod = 27;
//Hrefs distance between each other = 1659 mikro second
//I converted according to OnehundredNano Delay
const uint32_t hrefsDistance = 1659;


void GetFramesFromOvCam()
{

	  //PD0 = D0	    PD1 = D1		PD2 = D2
	 // PD3 = D3	    PD4 = D4		PD5 = D5
	 // PD6 = D6	    PD7 = D7




	if(vsyncIT == vSyncSecondState)
	{

		DELAYONEHUNDRENDNANO(8454);
		while(vsyncIT == vSyncHrefCycleState)
		{


			    while(!(GPIOF->IDR &(1UL << 8))){};
				for(uint16_t verticalCount =0; verticalCount < verticalPixel; verticalCount++)
				{


					/*
					 * When the program enters the "for loop" below for the first time, it delays about 500 nanoseconds.
					 * While in the loop, the time until the for loop comes to the end and starts again is 250 nanoSeconds
					 */

					__HAL_TIM_SET_COUNTER(&htim7,0);
					__HAL_TIM_SET_COUNTER(&htim6,0);

					for( uint16_t horizCount =0; horizCount < horizantelPixel ; horizCount++)
					{



						if(HAL_GPIO_ReadPin(GPIOF, Cam_PCLK_Pin))//Bu if yaklasik 225 veya 250 nano saniye

						{
							//Reading values and transfering dates to array(sequence two lines) take 275 - 325 nano second
							currentRowHorizantalData = (uint8_t) (GPIOD->IDR);//Reading camera values
							imageBuffer[verticalCount][horizCount] = currentRowHorizantalData; // Transfer 8 bit values to array
							timerResult = (halfPclkPeriod )- __HAL_TIM_GET_COUNTER(&htim6);
							DELAYTWENTYFIVENANO(timerResult);
						}
						else
						{
							currentRowHorizantalData =  (uint8_t)(GPIOD->IDR);//Reading camera values
							imageBuffer[verticalCount][horizCount] = currentRowHorizantalData; // Transfer 8 bit values to array
						    timerResult = (pclkPeriodTime )- __HAL_TIM_GET_COUNTER(&htim6);
							DELAYTWENTYFIVENANO(timerResult);
						}



						__HAL_TIM_SET_COUNTER(&htim6,0);

					}


					DELAYTWENTYFIVENANO(20000);

					while(!HAL_GPIO_ReadPin(GPIOF, Cam_HREF_Pin));
			}


				// Check If the user press the User button to take a photo(or releated EXTI is triggered from external device )
				if(imageProcess.PhotoTakeStatus == TOOKAPHOTO )
				{
					getScannedCentralImage((uint8_t*)copiedImage, (uint8_t *)imageBuffer);
					cnvrtRGBtoMonochrome((uint8_t*)copiedImage,getScannedImageSize);
					ILI9341_Draw_Rectangle(0, 130, 320, 110, BLACK);
					ILI9341_Draw_MultiText("Image to be scanned saved", 130, RED, BLACK);
					sendImageBufferToLed(&hspi1, (uint8_t*)copiedImage, 170, IMAGE_HEIGHT/2, IMAGE_WIDTH);
					HAL_Delay(2000);
					ILI9341_Draw_MultiText("Please show the object inside of the rectangle", 130, RED, BLACK);
					imageProcess.PhotoTakeStatus = SCANE_PHOTO;
				}
				else
				{
					if(imageProcess.PhotoTakeStatus == SCANE_PHOTO)
					{

						sendImageBufferToLed(&hspi1, (uint8_t*)imageBuffer, 0, IMAGE_HEIGHT, IMAGE_WIDTH * 2);
						ILI9341_Draw_Hollow_Rectangle_Coord(IMAGE_WIDTH/4, 30, 120, 90, ORANGE);

						getScannedCentralImage((uint8_t*)compareCurrentImage, cnvrtRGBtoMonochrome((uint8_t *) imageBuffer, imageBufSize));
						uint16_t totalBlackCompareImage =  countBlackDots((uint8_t *)compareCurrentImage, getScannedImageSize);
						uint16_t totalBlackCopiedImage = countBlackDots((uint8_t *)copiedImage, getScannedImageSize);

						if(totalBlackCompareImage > totalBlackCopiedImage -5 && totalBlackCompareImage < totalBlackCopiedImage +5 )
						{
							ILI9341_Draw_MultiText("Image matched with saved Image", 130, RED, BLACK);
							HAL_Delay(2000);
							ILI9341_Draw_MultiText("Please show the object inside of the rectangle", 130, RED, BLACK);
						}
					}
					//Idle condition
					else
					{
						sendImageBufferToLed(&hspi1,cnvrtRGBtoMonochrome((uint8_t*)imageBuffer, imageBufSize),0,IMAGE_HEIGHT,IMAGE_WIDTH*2);
						ILI9341_Draw_Hollow_Rectangle_Coord(IMAGE_WIDTH/4, 30, 120, 90, ORANGE);
					}


				}
		}
	}
}

/*
 * @brief: Convert RGB pixels to Monochrome in order to
 * make image comparison more easy
 * @param :image, To be converted image Rgb to Monochrome
 * @param : size, Size of the Image
 */
uint8_t * cnvrtRGBtoMonochrome(uint8_t * image, uint32_t size)
{
uint16_t tempBuffer =0;

for(uint16_t counter =0; counter < size; counter += 2)
{
	tempBuffer = (uint16_t)image[counter];
	tempBuffer |= (uint16_t)(image[counter + 1] << 8);

	//1084 value means R4,G5,B4 value must be 1 and rest of them will be 0
	//if this statement upper then 4, pixel gets white color.
				if((tempBuffer & 0x1084) > 4)
				{
					image[counter] = 0xFF;
					image[counter + 1] = 0xFF;
				}
				else
				{
					image[counter] = 0;
					image[counter + 1] = 0;
				}
}

return image;

}

/*
 * @brief : This function produce central image from full size image
 * @param : targetImage , Destination of the array
 * @param : sourceImage , An image which is applied to central algorithm.
 */
void getScannedCentralImage(uint8_t * targetImage ,uint8_t * sourceImage  )
{

	uint16_t tempBuf = 0;

for(uint8_t vertical =0; vertical < IMAGE_HEIGHT /2 ; vertical++)
{
	for(uint16_t horiz =0; horiz < IMAGE_WIDTH ; horiz++)
	{
		//6280 + (160 * vertical) equations for QQVGA format central calculation
		//I want to be more quick this calculation so I preferred to subscript values
		// If you want to make this central scanned proccess to any image Size, the required formula is below
		// b = Image height
		// c = Image width
		// y = verticalCounter
		// A(y) = start index point for source Image
		// A(y) = (2*c*b + 8*c*y -6*c)/4
		targetImage[tempBuf+ horiz] = sourceImage[(9360 +(320 * vertical)) + horiz];
	}
	tempBuf += IMAGE_WIDTH;
}


}

/*
 * @brief : This algorithm detects black pixels. It should be use after
 * Monochrome algorithm applied to Image_Array
 */
uint16_t countBlackDots(uint8_t * sourceImage,uint32_t imageSize)
{
	uint16_t result =0;

	for(uint32_t counter =0; counter < imageSize ; counter += 2)
	{
		if(sourceImage[counter] == 0x00)
		{
			result ++;
		}
	}

	return result;
}
