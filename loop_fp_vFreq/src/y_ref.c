/*
 * y_ref.c
 *
 *  Created on: 08.07.2019
 *      Author: ga85piw
 */

#include "pred_controller_header.h"
#include "stdio.h"
#include <math.h>

void y_ref(double *x_kk_minus_1, float *y_ref_kk, int T_ref){


	double phi_int;
	double phi;
	double delta_phi;
	double Phi_N[Nh];
	double p_dq_ab[2*Nh][2];
	// Lookup Table for Torque Ref for T_Ref value of 1 and 0.5
	double is_dq_ref_full[2]={0.388958805065727,0.927121420626787};
	double is_dq_ref_half[2]={0.397606703854977,0.453478319683166};


	double y_ref[2*Nh];
	double d_ref,q_ref;

	delta_phi=ts/we;

	phi_int=we*ts;

	phi=atan2(x_kk_minus_1[3],x_kk_minus_1[2]);


	phi=phi+phi_int;


	for (int i=0;i<Nh;i++){

		Phi_N[i]=phi+delta_phi*i;


	}

	for (int i=0;i<2*Nh;i++){

		if (i<Nh){

			p_dq_ab[i][0]=cos(Phi_N[i]);
			p_dq_ab[i][1]=-1*sin(Phi_N[i]);


		}

		if (i>=Nh){

			p_dq_ab[i][0]=sin(Phi_N[i-Nh]);
			p_dq_ab[i][1]=cos(Phi_N[i-Nh]);


		}

	}

//  Lookup Table for Torque Ref
	if(T_ref==1){ // for torque ref 1 pu

		d_ref=is_dq_ref_full[0];
		q_ref=is_dq_ref_full[1];
	}

	if(T_ref==0){ // for torque ref 0.5 pu

		d_ref=is_dq_ref_half[0];
		q_ref=is_dq_ref_half[1];
	}


	/// y_ref=p_dq_ab*is_dq_ref
//printf("\n");
	for (int i=0;i<2*Nh;i++){


		y_ref[i]=p_dq_ab[i][0]*d_ref+p_dq_ab[i][1]*q_ref;


	}

// re arranging
		for (int i=0;i<Nh;i++){

			y_ref_kk[i*N_out]=y_ref[i];
			y_ref_kk[(i*N_out)+1]=y_ref[i+Nh];
		}

		printf("\n");
			for (int i=0;i<2*Nh;i++){
				//printf ("   y_ref_kk[%d]=%.9f",i,y_ref_kk[i]);
				//printf (" %.9f,",y_ref_kk[i]);

			}



}


