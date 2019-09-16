/*
 * pred_controller_header.h
 *
 *  Created on: 27.06.2019
 *      Author: ga85piw
 */

#ifndef SRC_PRED_CONTROLLER_HEADER_H_
#define SRC_PRED_CONTROLLER_HEADER_H_



#define Nh 4
#define we 1
#define ts 0.007853981633
#define lambda 0.00136
#define N_out 2

void offline(double *x_kk,float *y_ref_kk,float *u_kk,float *y_hat,float *r_hat, float *v_mul_h_inv, float *v_gen,float *h_hat_inv);
void memory_write_offline(float *y_hat,float *r_hat,float *v_mul_h_inv, float *v_gen,float *h_hat_inv,unsigned int* Mem_start);
void memory_write_online(float *x_kk,float *y_ref_kk,float *u_kk,unsigned int* Mem_start);
void memory_read(float *u_opt,unsigned int* Mem_start);
void print_memory(unsigned int* Mem_start);
void y_ref(double *x_kk_minus_1, float *y_ref_kk, int T_ref);
void state_space_model(float *u_opt,double *x_kk_d);
//void print_memory(unsigned int* Mem_start,float *y_hat,float *r_hat);
unsigned int float_to_u32(float value);
float u32_to_float(unsigned int value);
int u32_to_int(unsigned int value);
void double_to_float(double *x_kk_d,float *x_kk);



#define y_hat_row 2*Nh
#define y_hat_col 3*Nh

#define r_hat_row 2*Nh
#define r_hat_col 4

#define v_mul_h_inv_row 3*Nh
#define v_mul_h_inv_col 3*Nh

#define v_gen_row 3*Nh
#define v_gen_col 3*Nh

#define h_hat_inv_row 3*Nh
#define h_hat_inv_col 3*Nh


#define Y_Hat(p,q)			(y_hat_mem_address[p*y_hat_col +q])
#define R_Hat(p,q)			(r_hat_mem_address[p*r_hat_col +q])
#define V_Mul_H_Inv(p,q)	(v_mul_h_inv_mem_address[p*v_mul_h_inv_col +q])
#define V_Gen(p,q)			(v_gen_mem_address[p*v_gen_col +q])
#define H_Hat_Inv(p,q)		(h_hat_inv_mem_address[p*h_hat_inv_col +q])


// Functions Initization for Arm

//#define labda_u 6.8e-3
#define labda_u lambda

void predictive_controller_sw(volatile float *X_KK_src,volatile float *Y_REF_KK_src,
							volatile float *U_KK_src,volatile float *Y_HAT_src,
							volatile float *R_HAT_src,volatile float *V_MUL_H_INV_src,
							volatile float *V_GEN_src,volatile float *H_HAT_INV_src,
							volatile int *out,volatile int *flag);

void unconstrained (float R_Hat_a[8*Nh],float X_KK_a[4],float Y_Hat_a[6*Nh*Nh],float Y_Ref_KK_a[2*Nh],float U_KK_a[3*Nh],float V_Mul_H_Inv_a[9*Nh*Nh],float unconstrained[3*Nh],float theta_kk[3*Nh]);
float guess_edu (float U_KK_a[3*Nh],float V_Gen_a[9*Nh*Nh],float U_unc_kk[3*Nh]);
float guess_babay (float V_Gen_a[9*Nh*Nh],float H_Hat_Inv_a[9*Nh*Nh],float U_unc_kk[3*Nh],float theta_kk[3*Nh],float U_babay[3*Nh]);
void sph_dec (float V_Gen_a[9*Nh*Nh],float roh, float U_unc[3*Nh], int U_opt[3*Nh], int *counter,int *sph_dec_sol_status);
void distance_calculation_fp(int *U,float *V_Gen_a,float *U_unc, int level, float *dist_array,  float *dist_temp);



#define X_KK_sw(p) 				(X_KK_a[p])
#define Y_Ref_KK_sw(p) 			(Y_Ref_KK_a[p])
#define U_KK_sw(p)				(U_KK_a[p])
#define Y_Hat_sw(p,q)			(Y_Hat_a[p*y_hat_col +q])
#define R_Hat_sw(p,q)			(R_Hat_a[p*r_hat_col +q])
#define V_Mul_H_Inv_sw(p,q)		(V_Mul_H_Inv_a[p*v_mul_h_inv_col +q])
#define V_Gen_sw(p,q)			(V_Gen_a[p*v_gen_col +q])
#define V_Gen_cpy_sw(p,q)		(V_Gen_a_cpy[p*v_gen_col +q])
#define H_Hat_Inv_sw(p,q)		(H_Hat_Inv_a[p*h_hat_inv_col +q])






#endif /* SRC_PRED_CONTROLLER_HEADER_H_ */
