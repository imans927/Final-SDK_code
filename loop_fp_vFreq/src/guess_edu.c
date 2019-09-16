#include <stdio.h>
#include <string.h>
#include "pred_controller_header.h"



float guess_edu (float U_KK_a[3*Nh],float V_Gen_a_cpy[9*Nh*Nh],float U_unc_kk[3*Nh])
{



	float educated_rho=0.0;

	float u_educated[3*Nh]={};


	u_educated_cal_1:for (int i=3*Nh-3; i<3*Nh; i++)
	{
		u_educated[i]=U_KK_sw(i);


	};

	u_educated_cal_2:for (int i=0; i < 3*Nh-3 ; i++)
	{

		u_educated[i]=U_KK_sw(i+3);


	};


	float temp_value;



	roh_educated_row:for (int row=0; row < 3*Nh ; row++) // row and col defined according to left matrix
	{
		temp_value=0.0f;

		roh_educated_col:for (int col=0; col < 3*Nh ; col++){

			if (col<=row){

			temp_value+=V_Gen_cpy_sw(row,col)*u_educated[col];

			}
		}


		temp_value=U_unc_kk[row]-temp_value;
		temp_value=temp_value*temp_value;
		educated_rho=educated_rho+temp_value;


	};




return educated_rho;

}


