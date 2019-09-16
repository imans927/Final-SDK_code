/*
 * double_to_float.c
 *
 *  Created on: 08.07.2019
 *      Author: ga85piw
 */

//Convertes the double value of x_kk to float, in order to avoid divergence error with MATLAB code
void double_to_float(double *x_kk_d,float *x_kk){

	for (int i=0; i<4;i++){

		x_kk[i]=x_kk_d[i];
	}

}
