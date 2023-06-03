/*
 * RD_flash.c
 *
 *  Created on: Apr 26, 2022
 *      Author: PC5
 */


#include "RD_flash.h"

//void rd_flash_read_data(unsigned long addr, u16 *out){
//	u8 buff_read[2] = {0};
//	flash_read_page(addr,2,buff_read);
//	*out = buff_read[1]<<8|buff_read[0];
//}
//
//
//
//unsigned char  flashReadJoinFlag(){
//	unsigned char flag = 0;
//	flash_read_page(0x7f000,1,&flag);
//	return flag;
//}
//
//
//void flashWriteJoinFlag(unsigned char *data){
//	flash_erase_sector(0x7f000);
//	flash_write_page(0x7f000,1,data);
//	flash_read_page(0x7f000,1,data);
//}
