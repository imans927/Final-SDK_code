/*
 * state_space_model.c
 *
 *  Created on: 08.07.2019
 *      Author: ga85piw
 */


#include "pred_controller_header.h"
#include "stdio.h"

void state_space_model(float *u_opt,double *x_kk_d)
{

    double A_ss[4][4]={

    		{0.999411495568444,1.00830960387419e-06,0.000224902693821756,0.029430909213924},
    		{-1.00830960387419e-06,0.999411495568444,-0.029430909213924,0.000224902693821756},
    		{6.85042872624856e-05,-2.6901914194539e-07,0.999939993871078,-0.00785266328386251},
    		{2.69019141945391e-07,6.85042872624856e-05,0.00785266328386251,0.999939993871078}
    };

    double B_mul_Pab[4][3]={
    	    {0.0198237047463068,-0.00991184660068532,-0.00991185814562149},
    	   {-6.66547200502782e-09,0.0171678352401599,-0.0171678285746878},
    	    {6.7927769461335e-07,-3.41178911449813e-07,-3.38098783163537e-07},
    	    {1.77831289513944e-09,5.87382583311719e-07,-5.89160896206859e-07},
    };

    double A_mul_x_kk_ss[4]={};
    double temp_ss;

    // coping x_kk into x_kk minus 1


    // Multilying A*xkk

    for (int i=0;i<4;i++){

    	A_mul_x_kk_ss[i]=0;

    	for (int j=0;j<4;j++){

    		A_mul_x_kk_ss[i]+=A_ss[i][j]*x_kk_d[j];
    	}
    }
	  // performing A_mul_x_kk+B_mul_Pab*Uopt

        for (int i=0;i<4;i++){

        	temp_ss=0;

        	for (int j=0;j<3;j++){

        		temp_ss+=B_mul_Pab[i][j]*u_opt[j];
        	}

        	x_kk_d[i]=temp_ss+A_mul_x_kk_ss[i];



    }


    }
