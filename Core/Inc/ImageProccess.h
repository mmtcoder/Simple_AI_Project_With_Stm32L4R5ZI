/*
 * ImageProccess.h
 *
 *  Created on: Oct 25, 2021
 *      Author: H3RK3S
 */

#ifndef INC_IMAGEPROCCESS_H_
#define INC_IMAGEPROCCESS_H_

#include "stdint.h"

#define RES_120x160 	1

#if RES_120x160

#define IMAGE_HEIGHT	120U
#define IMAGE_WIDTH	160U

#else

#endif

#define TOOKAPHOTO		2
#define NOTTAKEAPHOTO	0
#define SCANE_PHOTO		3

//I used this macros for testing rgb and monochrome mode with stm32 user Button
//I wrote needed codes into stm32l4xx_it.c class "EXTI9 ..." func
#define MONOCHROMEMODE	3
#define RGBMODE			0

 enum
{
	vSyncResetState =0,
	vSyncSecondState =1,
	vSyncHrefCycleState =2

}vSyncState_em;

typedef struct
{
	uint8_t ImageRgbOrMonochrome ;
	uint8_t PhotoTakeStatus;

}imageAIprocess_t;

void GetFramesFromOvCam();
void delayTwentyFiveNano(uint16_t time);
void delayOneHundredNano(uint16_t time);
void flipDisplay( );
void GetImageSendSPI();
void getScannedCentralImage(uint8_t * targetImage ,uint8_t * sourceImage  );

uint8_t * cnvrtRGBtoMonochrome(uint8_t * image, uint32_t size);
uint16_t countBlackDots(uint8_t * sourceImage,uint32_t imageSize);

#endif /* INC_IMAGEPROCCESS_H_ */
