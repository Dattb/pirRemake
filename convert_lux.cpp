#include<stdio.h>
#include<math.h>

unsigned int lux_cb;
float real_lux;
unsigned int lux_val;
float lux_real;
unsigned int CalculateLux(unsigned int rsp_lux);
int main (void){
		while(1){
		printf("\n***************************************************************************");
		printf("\nlux I2C input= ");
		scanf("%d",&lux_cb);
		//lux_cb = CalculateLux(lux_cb);
		
		printf("\nLux sensor: %d",lux_cb);
		lux_real = 1.7339*lux_cb - 39.868;
		printf("\nLux real: %d",(unsigned int) lux_real);
	}
	return 0;
}

unsigned int CalculateLux(unsigned int rsp_lux)
{
	unsigned int lux_LSB = 0;
	unsigned char lux_MSB = 0;
	unsigned int lux_Value = 0;
	unsigned int pow = 1;
	unsigned char i;
	lux_LSB = rsp_lux & 0x0FFF;
	lux_MSB = ((rsp_lux>>12) & 0x0F);
	//Lux_Value = 0.01 * pow(2,Lux_MSB) * Lux_LSB; //don't use
	for(i=0;i<lux_MSB;i++){
		pow=pow*2;
	}
	lux_Value=0.01 * pow * lux_LSB;
	return lux_Value;
}
