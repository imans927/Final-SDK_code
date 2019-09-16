#include <stdio.h>
#include <string.h>
#include "pred_controller_header.h"

float guess_babay (float V_Gen_a[9*Nh*Nh],float H_Hat_Inv_a[9*Nh*Nh],float U_unc_kk[3*Nh],float theta_kk[3*Nh],float U_babay[3*Nh])
{

	float babay_rho=0.0f;


	float temp;

	U_babay_cal_row:for (int row=0; row < 3*Nh ; row++) // row and col defined according to left matrix
	{
		U_babay[row]=0;
		temp=0;

		U_babay_cal_col:for (int col=0; col < 3*Nh ; col++){

			U_babay[row]+= -1 * H_Hat_Inv_sw(row,col)*theta_kk[col];


		}


		if (U_babay[row]>=0.5)

				{ U_babay[row]=1;}

		else if (U_babay[row]<0.5 & U_babay[row]>=-0.5)

				{ U_babay[row]=0;}  // rounding of -1 , 0 , 1

		else if (U_babay[row]<-0.5)
				{ U_babay[row]=-1; }
	};



	float temp_value;



	roh_babay_row:for (int row=0; row < 3*Nh ; row++) // row and col defined according to left matrix
	{
		temp_value=0.0f;

		roh_babay_col:for (int col=0; col <= 3*Nh ; col++){

			if (col<=row){
				temp_value+=V_Gen_sw(row,col)*U_babay[col];
			}

		}

		temp_value=U_unc_kk[row]-temp_value;
		temp_value=temp_value*temp_value;
		babay_rho=babay_rho+temp_value;


	};


	return babay_rho;

	}

