#include <stdio.h>
#include <string.h>
#include "pred_controller_header.h"


// Equivalent code of the one imlemented on FPGA, FPGA output can be verified from this code



void predictive_controller_sw(volatile float *X_KK_src,volatile float *Y_REF_KK_src,
							volatile float *U_KK_src,volatile float *Y_HAT_src,
							volatile float *R_HAT_src,volatile float *V_MUL_H_INV_src,
							volatile float *V_GEN_src,volatile float *H_HAT_INV_src,
							volatile int *out,volatile int *flag)


{


  int i;
  int copy_flag;
  float X_KK_a[4];
  float Y_Ref_KK_a[2*Nh];
  float U_KK_a[3*Nh];
  static float Y_Hat_a[6*Nh*Nh];
  static float R_Hat_a[8*Nh];
  static float V_Mul_H_Inv_a[9*Nh*Nh];
  static float V_Gen_a[9*Nh*Nh];
  static float V_Gen_a_cpy[9*Nh*Nh];
  static float H_Hat_Inv_a[9*Nh*Nh];



  float U_unc_kk[3*Nh];
  float U_unc_kk_cpy[3*Nh];
  float roh_educated;
  float roh_babay;
  float theta_kk[3*Nh];
  float U_babay[3*Nh];
  int U_opt[3*Nh+1];
  int iteration_count=0;
  int sph_dec_sol_status=0;




int qq=0;
copy_flag=*flag;

x_kk_cpy:for(int row=0;row<4;row++){
#pragma HLS PIPELINE
	  	  X_KK_a[row]=X_KK_src[row];
}
////


y_ref_kk_cpy:for(int row=0;row<2*Nh;row++){
#pragma HLS PIPELINE
	Y_Ref_KK_a[row]=Y_REF_KK_src[row];
 }

u_kk_cpy:for(int row=0;row<3*Nh;row++){
#pragma HLS PIPELINE
	U_KK_a[row]=U_KK_src[row];
 }

if (copy_flag==1){

y_hat_cpy:for(int row=0;row<6*Nh*Nh;row++){

#pragma HLS PIPELINE

	Y_Hat_a[row]=Y_HAT_src[row];
	  }

r_hat_cpy:for(int row=0;row<8*Nh;row++){

#pragma HLS PIPELINE
	R_Hat_a[row]=R_HAT_src[row];
	  }

VHinv_cpy:for(int row=0;row<9*Nh*Nh;row++){

#pragma HLS PIPELINE
	V_Mul_H_Inv_a[row]=V_MUL_H_INV_src[row];
	  }

Vgen_cpy:for(int row=0;row<9*Nh*Nh;row++){

#pragma HLS PIPELINE
	V_Gen_a[row]=V_GEN_src[row];
	  }


Hhat_inv_cpy:for(int row=0;row<9*Nh*Nh;row++){

#pragma HLS PIPELINE
	H_Hat_Inv_a[row]=H_HAT_INV_src[row];
	  }

}

  unconstrained(R_Hat_a,X_KK_a,Y_Hat_a,Y_Ref_KK_a,U_KK_a,V_Mul_H_Inv_a,U_unc_kk,theta_kk);


  U_Unc_kk_copy:for (int row=0;row<3*Nh;row++)
  {
	  	  U_unc_kk_cpy[row]=U_unc_kk[row];
  }

  V_gen_copy:for (int row=0;row<9*Nh*Nh;row++)
  {
	  	  V_Gen_a_cpy[row]=V_Gen_a[row];
  }




  roh_educated=guess_edu (U_KK_a,V_Gen_a_cpy,U_unc_kk_cpy);
  roh_babay=guess_babay(V_Gen_a,H_Hat_Inv_a,U_unc_kk,theta_kk,U_babay);



	float roh;

	if (roh_educated < roh_babay)
	{
		roh=roh_educated;
	}
	else
		roh=roh_babay;

	sph_dec(V_Gen_a,roh,U_unc_kk,U_opt,&iteration_count,&sph_dec_sol_status);


	if (sph_dec_sol_status == 0){ // if limit of 170 nodes is reached, U_babay is sent at output



			u_babay_copy_to_out:for(int qq=0;qq<3*Nh;qq++){

				U_opt[qq]=(int)U_babay[qq];

			}



		}

	U_opt[3*Nh]=iteration_count; // U_opt is of size , 3*Nh+1 , last value of U_opt contains iteration_count value of sphere decoder







 memcpy((int *)out,U_opt,3*(Nh+1)*sizeof(int));


}



