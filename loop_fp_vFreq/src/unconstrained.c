#include <stdio.h>
#include <string.h>
#include "pred_controller_header.h"


void unconstrained (float R_Hat_a[8*Nh],float X_KK_a[4],float Y_Hat_a[6*Nh*Nh],float Y_Ref_KK_a[2*Nh],float U_KK_a[3*Nh],float V_Mul_H_Inv_a[9*Nh*Nh],float unconstrained[3*Nh],float theta_kk[3*Nh])

{

	float temp[2*Nh];
	float accu[4]={0.0f,0.0f,0.0f,0.0f};
	float accu_value=0.0f;


	rHAt_Mul_xkk_row:for (int row=0 ; row < r_hat_row ; row++){  // row and col are defined according to left matrix
						temp[row]=0;
						rHAt_Mul_xkk_col:for (int col=0; col < r_hat_col; col++){
							accu[col]=R_Hat_sw(row,col)*X_KK_sw(col);
							}

							accu_temp:for(int i=0;i<4;i++)
							{
								accu_value+=accu[i];
								accu[i]=0.0f;
							}

							temp[row]=accu_value;
							accu_value=0.0f;

	}




	Theta_kk_row:for (int row=0 ; row < y_hat_col ; row++){   // row and col are defined according to left matrix
			theta_kk[row]=0;
		Theta_kk_col:for (int col=0 ; col < y_hat_row ; col++){
			theta_kk[row]+=Y_Hat_sw(col,row)*(temp[col]-Y_Ref_KK_sw(col));


		}



		if (row<3){
			theta_kk[row]=theta_kk[row]- labda_u*U_KK_sw(row);
		}

	}




	unconstrained_row:for (int row=0 ; row < 3*Nh ; row++){   // row and col are defined according to left matrix

		unconstrained[row]=0;
			unconstrained_col:for (int col=0 ; col < 3*Nh ; col=col+4){



				acc_part:for (int col_s=0;col_s<4;col_s++){

					accu[col_s]+=-V_Mul_H_Inv_sw(row,col+col_s)*theta_kk[col+col_s];
				}
			}

				accu_uncos:for(int i=0;i<4;i++)
				{
					accu_value+=accu[i];
					accu[i]=0.0f;
				}

				unconstrained[row]=accu_value;
				accu_value=0.0f;

		}






}

