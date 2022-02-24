# Simple_AI_Project_With_Stm32L4R5ZI

First of all, I will continue to develop this project. 

Main goal of the project is detecting different shapes. If you want to get correct result, You must put the camera stable place and distance of the object with camera must be the same for matching between saved image and current image.Second thing that I made detection algorithm for one object into scanable area. It means object to be detected must dark colors( like black, grey, brown etc.), background color must be open colors(like white, open green, red, blue etc.) and another object must not be inside of the rectangle area( scanning area). Stable or good ambient lighting increases object detection at the forward rate.

Camera Frame Detection Algorithm made for 120 MHz and PWM generation produce 24 MHz to the Camera.If you have more high frequency mcu you must set all of them (or related bus) 120 MHz.
My TIMER6 provides 25 nanoSecond and TIMER7 provides 100 nanoSecond .Timer settings calculations is divided 120 MHz.

IF YOU HAVE DIFFERENT MCU FREQUENCY, YOU MUST SET CLOCK FREQUENCY 120 MHZ FOR MATCHING MY CAMERA FRAME DETECTION ALGORITHM.

 If you have low mcu frequency ( I Think minimum mcu freq must be bigger than 90 Mhz ) You need to increase Camera Clock Prescaler Register Value ( Adress is 0x11). You must observe  Pclk, Href and Vsync variables wiht Logic Analyzer or Oscilloscope so you can set timers variables.

Last thing that I didn't use sd card right now but I prepared required settings in seperate files called " sdCard.c  ..." so you can use Sd card.

USED MATERIALS...

* Nucleo L4R5ZI( It has max 120 Mhz freq)
* Ov7670 camera (OmniVision)
* ILI9341 RGB Led with TFT
* Logic Analyzer (For Detecting Pclk, Href and Vref Times )

Used PINS

PIN_D0 ... PIN_D7 = Cam_D0 .... Cam_D7 (Input Mode)
PC13 = Nucleo Blue Button (EXTI mode)
I2C pins may be different from mine
PF7 = Cam_PCLK (Input mode)
PF8 = Cam_HREF (Input Model)
PF9 = Cam_Vsyn_EXTI ( EXTI mode)
PC2 = Camera_RESET(Output Mode)
PC3 = Camera_PWDN (Output Mode)
PA1 = TIM2_CH2 (PWM Pin)
SPI  pins may be different from mine
PC5 = LED_Activate( Output mode)
PC6 = SPI_SD_NSS( Output Mode This pin is optional )
PC8= LED_CS (Output Mode)
PC9 = LED_DC (Output mode)
PC10 = LED_RESET( Output Mode)
