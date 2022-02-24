/*
 * ov7670Init.c
 *
 *  Created on: Oct 20, 2021
 *      Author: H3RK3S
 */


#include "ov7670Init.h"
//#include "stm32l4xx_hal.h"
#include "main.h"
//#include  "ILI9341_GFX.h"

extern I2C_HandleTypeDef hi2c2;
//extern SPI_HandleTypeDef hspi1;
//extern TIM_HandleTypeDef htim7;

//This settings includes RGB565 format,QQVGA format and color optimization settings.
//You can find "OV7670 Implementation " document as pdf by OmniVision Company

const uint8_t OV7670_reg[][2] = {
 {REG_COM7, 0x80},
 {REG_CLKRC, 0x83},//Old Clock value = 0x83
 {REG_COM11, 0x0A},
 {REG_TSLB, 0x04},
 {REG_TSLB, 0x04},
 {REG_COM7, 0x04},

 {REG_RGB444, 0x00},
 {REG_COM15, 0xD0},

 {REG_HSTART, 0x16},
 {REG_HSTOP, 0x04},
 {REG_HREF, 0x24},
 {REG_VSTART, 0x02},
 {REG_VSTOP, 0x7a},
 {REG_VREF, 0x0a},
 {REG_COM10, 0x02},
 {REG_COM3, 0x04},
 {REG_MVFP, 0x3f},
 // 3 consecutive lines of code are QQVGA format settings.
 {REG_COM14, 0x1a},
 {0x72, 0x22},
 {0x73, 0xf2},

 {0x4f, 0x80},
 {0x50, 0x80},
 {0x51, 0x00},
 {0x52, 0x22},
 {0x53, 0x5e},
 {0x54, 0x80},
 {0x56, 0x40},
 {0x58, 0x9e},
 {0x59, 0x88},
 {0x5a, 0x88},
 {0x5b, 0x44},
 {0x5c, 0x67},
 {0x5d, 0x49},
 {0x5e, 0x0e},
 {0x69, 0x00},
 {0x6a, 0x40},
 {0x6b, 0x0a},
 {0x6c, 0x0a},
 {0x6d, 0x55},
 {0x6e, 0x11},
 {0x6f, 0x9f},

 {0xb0, 0x84},

 //renk denemesi yapıyorum
 /*{REG_GAIN,0xFF},
 {REG_BLUE, 0xFF},*/

 /*{REG_AD_CHB, 0x2F},
 {REG_AD_CHR, 0x28},*/
/*
 {REG_STR_B, 0x0F},
 {REG_STR_G, 0x0F},
 {REG_STR_R, 0x0F},
*/
 {0xFF, 0xFF},

};

/*
const uint8_t needForHalfMicroDelay =0;
const uint16_t hrefPeriod= 9406; //Micro Second
const uint16_t pclkHighPixSynTime = 6; //Micro Second
const uint16_t pclkLowPixSynTime = 4;  //Micro Second
const uint16_t vsyncLowTime = 7052; //Micro Second

uint8_t horizArray[320];

uint8_t vsync =0;
//static uint8_t denemArray [6000000][320]; // eğer bilgileri main flash memory
// yani resim pixellerini kaydedersek maximum yaklasik bu kadar boyutta array
//kullanabiliyoruz. Burada kiyaslanacak objelerin resimlerini tutabiliriz.
*/



char ov7670_init(void)
{
	  ResetRegisterForOvCam();
	HAL_Delay(30);

	  uint8_t buffer[4];
	ReadOperationOVCam(REG_VER, buffer);
	  if ( buffer[0] != 0x73)
	  {
		  return 0;
	  }
	  else
	  {
		  ov7670_config();
	  }

	  return 1;
}

char ov7670_config()
{
	ResetRegisterForOvCam();
    HAL_Delay(50);

  for(int i = 0; OV7670_reg[i][0]!=0xFF; i++) {
    WriteOperationOVCam(OV7670_reg[i][0], OV7670_reg[i][1]);
   HAL_Delay(2);
  }
  return 0;
}


void ConfigurePWDNandRESETpins()
{
	//for pwdn configure as Reset state to PC3
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, RESET);

	//for RESET configure as Set state to PC2
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, SET);
}

void ResetRegisterForOvCam()
{
	uint8_t pData[] = {0x80};

	//You need to set of Bit[7] for 0x12 Register Address
	if( HAL_I2C_Mem_Write(&hi2c2, writeAddressSCCB, REG_COM7, I2C_MEMADD_SIZE_8BIT, pData, 1, 10) != HAL_OK)
	{

		return;
	}
}

void WriteOperationOVCam(uint16_t memADdress, uint8_t  pData)
{
	if( HAL_I2C_Mem_Write(&hi2c2, writeAddressSCCB, memADdress, I2C_MEMADD_SIZE_8BIT, &pData, 1, 10) != HAL_OK)
	{

		return;
	}
}

void ReadOperationOVCam(uint16_t memAddress, uint8_t* buffer)
{

	if(HAL_I2C_Master_Transmit(&hi2c2, writeAddressSCCB, (uint8_t*)&memAddress, 1, 20) != HAL_OK)
	{
		return ;
	}
   if(HAL_I2C_Master_Receive(&hi2c2, readAddressSCCB, buffer, 1, 20) != HAL_OK)
   {
	   return;
   }


}


/*
void delayUsecForTimSeven(uint16_t time)
{
	__HAL_TIM_SET_COUNTER(&htim7,0);
	while( __HAL_TIM_GET_COUNTER(&htim7) < time);
}
*/
