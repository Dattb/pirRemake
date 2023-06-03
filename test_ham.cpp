#include<stdio.h>

int lux_cb;
float real_lux;
unsigned int lux_val;
int main (void){
		while(1){
		
		printf("\nlux cam bien = \n");
		scanf("%d",&lux_cb);
		real_lux  = 1.7339*lux_cb - 39.868;
		if(real_lux<0) real_lux = 0;
		lux_val  = (unsigned int)(real_lux);
		
		printf("\nLux float : %.2f",real_lux);
		printf("\nLux unsigned int : %d",lux_val);
	}
	return 0;
}
