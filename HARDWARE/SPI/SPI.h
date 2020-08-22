#ifdef  _SPI_H
#define _SPI_H
#include "Includes.h"
//PB12 13 14 15 NSS SCK MISO MOSI
#define  ON_CS()    {GPIO_ResetBits(GPIOB,GPIO_Pin_12);}//delay_us(1);
#define  OFF_CS()   {GPIO_SetBits(GPIOB,GPIO_Pin_12);}//delay_us(1);
void SPI_init(int flag);
char SPI_SendReceive(char data);

#endif
