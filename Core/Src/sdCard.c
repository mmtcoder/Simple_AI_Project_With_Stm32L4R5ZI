/*
 * sdCard.c
 *
 *  Created on: Dec 12, 2021
 *      Author: H3RK3S
 */

#include "sdCard.h"
#include "fatfs.h"
#include "fatfs_sd.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart3;


extern uint8_t cameraPhotoStatus;
extern  uint8_t cameraWriteOrRead;

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw,brw ;  // File read/write count


/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];  // to store strings..

int i=0;
uint8_t imageBufferGetted[38400];

int bufsize (char *buf)
{
	int i=0;
	while (*buf++ != '\0') i++;
	return i;
}

void clear_buffer (void)
{
	for (int i=0; i<BUFFER_SIZE; i++) buffer[i] = '\0';
}

void send_uart (char *string)
{
	uint8_t len = strlen (string);
	HAL_UART_Transmit(&huart3, (uint8_t *) string, len, HAL_MAX_DELAY);  // transmit in blocking mode
}

 void mountSDcard()
{
	 fresult = f_mount(&fs, "", 0);
	 if (fresult != FR_OK) send_uart ("ERROR!!! in mounting SD CARD...\n\n");
	  else send_uart("SD CARD mounted successfully...\n\n");

}



void writeBitMaptoFile(uint8_t * imageBuffer, uint32_t imageSize)
{
	if(cameraWriteOrRead == CAMERA_WRITE_ENABLE)
	{

		uint16_t tempValue =0;
		uint8_t  tempSmallBuffer[320];


		//For seperating spi proccess you need to set led_Cs pin
		 HAL_GPIO_WritePin(GPIOC, LED_CS_Pin, SET);
		 HAL_GPIO_WritePin(GPIOC, SPI_SD_NSS_Pin, RESET);



		   fresult = f_open(&fil, "image.bmp", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
			if(fresult == FR_OK)
			{

				if(fresult == FR_OK)
				{
				for(uint16_t verticCounter =0; verticCounter < 120 ; verticCounter++)
				{

				for(uint16_t horizCount =0; horizCount< 320 ; horizCount++)
				{
					tempSmallBuffer[horizCount] = imageBuffer[tempValue + horizCount];
				}

					f_write(&fil, tempSmallBuffer, 320, &brw);
					tempValue += 320;
				}

				send_uart("image is written successfully \n\n");
				cameraWriteOrRead = CAMERA_IDLE_STATE;
				}
			}
			else
			{


				send_uart("image process is failed \n\n");

			}

			f_close(&fil);
			HAL_GPIO_WritePin(GPIOC, SPI_SD_NSS_Pin, SET);


	}

}

uint8_t * readImageFromFile()
{
	if(cameraWriteOrRead == CAMERA_READ_ENABLE)
		{



			//For seperating spi proccess you need to set led_Cs pin
			 HAL_GPIO_WritePin(GPIOC, LED_CS_Pin, SET);


			   fresult = f_open(&fil, "image.bmp", FA_READ );
				if(fresult == FR_OK)
				{


					f_read(&fil, imageBufferGetted, f_size(&fil), &br);

					send_uart("image is readed successfully \n\n");

					f_close(&fil);
					HAL_GPIO_WritePin(GPIOC, SPI_SD_NSS_Pin, SET);

					cameraWriteOrRead = CAMERA_IDLE_STATE;

					return imageBufferGetted;

				}
				else
				{
				;
					send_uart("image process is failed \n\n");
					f_close(&fil);
					HAL_GPIO_WritePin(GPIOC, SPI_SD_NSS_Pin, SET);
					cameraWriteOrRead = CAMERA_IDLE_STATE;

					return 0;
				}
		}
	return 0;

}

