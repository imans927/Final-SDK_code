#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pred_controller_header.h"

void sph_dec (float V_Gen_a[9*Nh*Nh],float roh, float U_unc[3*Nh], int U_opt[3*Nh], int *counter,int *sph_dec_sol_status){



	// Initialize variables
	int solution_found    = 0;  // termination of while loop
	int level              = 0;      // current level of tree-search
	float dist_array[Nh*3]   = {0.0f};
    // accumulated costs
	float dist_temp        = 0;      // temporary costs for current step
	float dist_matmul      = 0;



	int switch_point[Nh*3];
	int U[Nh*3]={0U};
	int sol[Nh*3];

	int count=0;



	int ii,jj,kk,ll; // loop counter
	int i=0;

	init:for (ll = 0; ll < 3*Nh; ++ll)
	{
		switch_point[ll] = -1;
	}



	//while (solution_found==false) // While loop changed to for loop
	sphdec: for (int i=0;i< 170; i++)
		{

		if(solution_found==0)
				{

			//printf ("\n inside sphere decode algo level: %i, switch_point: %i, %i, %i ", level, switch_point[0],switch_point[1],switch_point[2]);
			U[level] = switch_point[level];

			distance_calculation_fp(U,V_Gen_a,U_unc,level,dist_array,&dist_temp);

			decision: {
			// depth first search
			if (dist_temp <= roh + 1e-6) { //distance smaller than current radius?
				if (level == 3*Nh-1)
				{
					//Copy array U_opt = U;
					U_opt:for (jj = 0; jj < 3*Nh; ++jj)
					{
						U_opt[jj]= U[jj];

					}
					roh   = dist_temp;
					switch_point[3*Nh-1] +=1; // same as switch_point[level]
				}
				else
				{
					level += 1;
					dist_array[level] = dist_temp;
				}
			}	else switch_point[level] +=1;


				}

			back_tracking:{
			// Backtracking
			Backtracking:for (kk = 3*Nh-1; kk > 0; --kk)
			{
				if (switch_point[kk]>1)	{
					//ap_wait();
					switch_point[kk] = -1;
					level = kk-1;
					switch_point[level] += 1;
				}
			}

				}

			// Check if full tree is searched
			if (switch_point[0]>1) {
				solution_found = 1;
				*sph_dec_sol_status=1;
				//printf("\n value of i at solution found is %d",i);

			}

			count=count+1;

		}

	*counter=count;
		}

}



void distance_calculation_fp(int *U,float *V_Gen_a,float *U_unc, int level, float *dist_array,  float *dist_temp){

	float V_gen_fixed[3*Nh]={0,0,0,0,0,0,0,0,0,0,0,0};
	float dist_matmul_array[3*Nh] ={0,0,0,0,0,0,0,0,0,0,0,0};
	float distance[4]={0,0,0,0};
	float product_answer;
	float final_distance=0;


	buff_copy_loop:for (int i=0;i<3*Nh;i++)
	{
		V_gen_fixed[i]=(V_Gen_sw(level,i));

	}


	dist_matmul_loop:for (int ii = 0; ii<3*Nh; ii++)
			{

				dist_matmul_array[ii]=V_gen_fixed[ii]*U[ii];  // approach used for floating , make array then accumulated

				}




			dist_summition:for (int ii = 0; ii<3*Nh; ii+=4)
			{
				for (int q=0;q<4;q++){

				distance[q]+=dist_matmul_array[ii+q];

				}
			}


			float partial_sum1=distance[0]+distance[2];
			float partial_sum2=distance[1]+distance[3];
			product_answer=partial_sum1+partial_sum2;



			float dist_temp_temp=U_unc[level]-product_answer;


			float dist_temp_fp= dist_temp_temp*dist_temp_temp + dist_array[level];

			*dist_temp = dist_temp_fp ;





}
