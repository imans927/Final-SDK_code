/*
 * memory_write_online.c
 *
 *  Created on: 08.07.2019
 *      Author: ga85piw
 */


#include "pred_controller_header.h"

void memory_write_online(float *x_kk,float *y_ref_kk,float *u_kk,unsigned int* Mem_start)
{

	unsigned int *x_kk_mem_address=Mem_start;
	unsigned int *y_ref_kk_mem_address=Mem_start+4;
	unsigned int *u_kk_mem_address=Mem_start+4+2*Nh;




	for (int i=0;i<4;i++){

		//Xil_Out32(x_kk_mem_address+4*i,float_to_u32(x_kk[i]));
		x_kk_mem_address[i]=float_to_u32(x_kk[i]);
	}

	for (int i=0;i<2*Nh;i++){

		//Xil_Out32(x_kk_mem_address+4*i,float_to_u32(x_kk[i]));
		y_ref_kk_mem_address[i]=float_to_u32(y_ref_kk[i]);
	}


	for (int i=0;i<3*Nh;i++){

		//Xil_Out32(x_kk_mem_address+4*i,float_to_u32(x_kk[i]));
		u_kk_mem_address[i]=float_to_u32(u_kk[i]);
	}



}
