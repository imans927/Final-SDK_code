/*
 * memory_write.c
 *
 *  Created on: 28.06.2019
 *      Author: ga85piw
 */

#include "pred_controller_header.h"

void memory_write_offline(float *y_hat,float *r_hat,float *v_mul_h_inv, float *v_gen,float *h_hat_inv,unsigned int* Mem_start)
{


	unsigned int *y_hat_mem_address=Mem_start+4+2*Nh+3*Nh;
	unsigned int *r_hat_mem_address=Mem_start+4+2*Nh+3*Nh+6*Nh*Nh;
	unsigned int *v_mul_h_inv_mem_address=Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh;
	unsigned int *v_gen_mem_address=Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh;
	unsigned int *h_hat_inv_mem_address=Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh+9*Nh*Nh;





	for (int i=0;i<6*Nh*Nh;i++){

		//Xil_Out32(x_kk_mem_address+4*i,float_to_u32(x_kk[i]));
		y_hat_mem_address[i]=float_to_u32(y_hat[i]);
	}

	for (int i=0;i<8*Nh;i++){

		//Xil_Out32(x_kk_mem_address+4*i,float_to_u32(x_kk[i]));
		r_hat_mem_address[i]=float_to_u32(r_hat[i]);
	}

	for (int i=0;i<9*Nh*Nh;i++){

		//Xil_Out32(x_kk_mem_address+4*i,float_to_u32(x_kk[i]));
		v_mul_h_inv_mem_address[i]=float_to_u32(v_mul_h_inv[i]);
	}

	for (int i=0;i<9*Nh*Nh;i++){

		//Xil_Out32(x_kk_mem_address+4*i,float_to_u32(x_kk[i]));
		v_gen_mem_address[i]=float_to_u32(v_gen[i]);
	}

		for (int i=0;i<9*Nh*Nh;i++){

			//Xil_Out32(x_kk_mem_address+4*i,float_to_u32(x_kk[i]));
			h_hat_inv_mem_address[i]=float_to_u32(h_hat_inv[i]);


	}



}
