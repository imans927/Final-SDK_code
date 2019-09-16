#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "pred_controller_header.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xpredictive_controller.h"
#include "xtime_l.h"
#include <math.h>


XPredictive_controller predictive_ctrl;
XPredictive_controller_Config *predictive_ctrl_ptr;



int main()
{
    init_platform();
    int status;
    float x_kk[4];
    double x_kk_d[4];
    float y_ref_kk[2*Nh];
    float u_kk[3*Nh];
    float y_hat[6*Nh*Nh];
    float r_hat[2*4*Nh];
    float v_mul_h_inv[9*Nh*Nh];
    float v_gen[9*Nh*Nh];
    float h_hat_inv[9*Nh*Nh];
    float u_opt[3*Nh+1]={};
    int u_out_sw[3*Nh]={0};
    int flag;
    float lambda_var=lambda;
    int T_ref=1;
    XTime tStart,tEnd;
    XTime tStart_sw,tEnd_sw;



    // look up the device config

    predictive_ctrl_ptr=XPredictive_controller_LookupConfig(XPAR_PREDICTIVE_CONTROLLER_0_DEVICE_ID);
    if(!predictive_ctrl_ptr){
    	xil_printf("ERROR: Lookup of acclerator configuration failed.\n\r");
    	return XST_FAILURE;
    }

    // Initialize the device

    status=XPredictive_controller_CfgInitialize(&predictive_ctrl,predictive_ctrl_ptr);
    if (status != XST_SUCCESS)
    {
    	xil_printf("Error could not initialize accelerator.\n\r");
    	exit(-1);

    }

    unsigned int* Mem_start=(unsigned int*)XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR;
    unsigned int *flag_mem_address=Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh+9*Nh*Nh+9*Nh*Nh;
    unsigned int *lambda_in_mem_address=Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh+9*Nh*Nh+9*Nh*Nh+1;
    flag_mem_address[0]=1;
    flag=1;






   offline(x_kk_d,y_ref_kk,u_kk,y_hat,r_hat,v_mul_h_inv,v_gen,h_hat_inv); // copies value from offline buffers into the memory





   printf("\n Nh is %d \n",Nh);

   // Sets the memeory offset for each port in m-axi data port

   XPredictive_controller_Set_X_KK_src(&predictive_ctrl,Mem_start);
   XPredictive_controller_Set_Y_REF_KK_src(&predictive_ctrl,Mem_start+4);
   XPredictive_controller_Set_U_KK_src(&predictive_ctrl,Mem_start+4+2*Nh);
   XPredictive_controller_Set_Y_HAT_src(&predictive_ctrl,Mem_start+4+2*Nh+3*Nh);
   XPredictive_controller_Set_R_HAT_src(&predictive_ctrl,Mem_start+4+2*Nh+3*Nh+6*Nh*Nh);
   XPredictive_controller_Set_V_MUL_H_INV_src(&predictive_ctrl,Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh);
   XPredictive_controller_Set_V_GEN_src(&predictive_ctrl,Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh);
   XPredictive_controller_Set_H_HAT_INV_src(&predictive_ctrl,Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh+9*Nh*Nh);
   XPredictive_controller_Set_flag(&predictive_ctrl,Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh+9*Nh*Nh+9*Nh*Nh);
   XPredictive_controller_Set_lamda_in(&predictive_ctrl,Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh+9*Nh*Nh+9*Nh*Nh+1);
   XPredictive_controller_Set_out_r(&predictive_ctrl,Mem_start+4+2*Nh+3*Nh+6*Nh*Nh+8*Nh+9*Nh*Nh+9*Nh*Nh+9*Nh*Nh+1+1);



   memory_write_offline(y_hat,r_hat,v_mul_h_inv,v_gen,h_hat_inv,Mem_start);
   lambda_in_mem_address[0]=float_to_u32(lambda_var); // loads value of lambda into the port

   printf("%d : %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f,\n",1,u_opt[0],u_opt[1],u_opt[2],u_opt[3],u_opt[4],u_opt[5],u_opt[6],
   u_opt[7],u_opt[8],u_opt[9],u_opt[10],u_opt[11]);
   printf("%d :  %.6f, %.6f, %.6f, %.6f",1,x_kk_d[0],x_kk_d[1],x_kk_d[2],x_kk_d[3]);

for (int loop=2;loop<=500;loop++){ // FPGA in Loop simulation

if (loop>2)
	{flag_mem_address[0]=0;
	flag=0;}

// If cnditions  to check trasnient Conditions,

// Lokkup table inside y_ref only have values for reference torque 1pu and torque 0.5pu
// if in the code T_ref=1 is selected , lookup table correspinding to torque 1pu is selected if T_ref=0 is selected , torque ref of 0.5 is selected
if (loop>=2 && loop<=1000){

	T_ref=1;

}

if (loop>1000 && loop<=2400){

	T_ref=1;

}

if (loop>2400 && loop<=3200){

	T_ref=1;

}

//

   y_ref(x_kk_d,y_ref_kk,T_ref);
   double_to_float(x_kk_d,x_kk);

for (int i=0;i<3*Nh;i++)
{
	u_kk[i]=u_opt[i];

}


memory_write_online(x_kk,y_ref_kk,u_kk,Mem_start);




//Start of predictive controller

XPredictive_controller_Start(&predictive_ctrl);
XTime_GetTime((XTime*)&tStart);
while(!XPredictive_controller_IsDone(&predictive_ctrl));
XTime_GetTime((XTime*)&tEnd);



XTime_GetTime((XTime*)&tStart_sw);
predictive_controller_sw(x_kk,y_ref_kk,u_kk,y_hat,r_hat,v_mul_h_inv,v_gen,h_hat_inv,u_out_sw,&flag);
XTime_GetTime((XTime*)&tEnd_sw);

memory_read(u_opt,Mem_start);
state_space_model(u_opt,x_kk_d);
double_to_float(x_kk_d,x_kk);

for (int qq=0;qq<3*Nh;qq++){ // to check both FPGA and Doftware code results match

	if (u_opt[qq]-u_out_sw[qq] > 0.0001f || u_opt[qq]-u_out_sw[qq] < -0.0001f)
	{

		printf("Error , difference in output");
		return 1;
	}

}


printf("\n");
printf("%d,%d, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f,\n",loop,0,u_opt[0],u_opt[1],u_opt[2],u_opt[3],u_opt[4],u_opt[5],u_opt[6],u_opt[7],u_opt[8],u_opt[9],u_opt[10],u_opt[11]);
printf ("%d,%d, %.2f\n",loop,1,1.0*(tEnd-tStart)/(COUNTS_PER_SECOND/1000000));
printf("%d,%d, %d,%d,%d, %d,%d,%d, %d,%d,%d, %d,%d,%d,\n",loop,2,u_out_sw[0],u_out_sw[1],u_out_sw[2],u_out_sw[3],u_out_sw[4],u_out_sw[5],u_out_sw[6],u_out_sw[7],u_out_sw[8],u_out_sw[9],u_out_sw[10],u_out_sw[11]);
printf ("%d,%d, %.2f\n",loop,3,1.0*(tEnd_sw-tStart_sw)/(COUNTS_PER_SECOND/1000000));
printf("%d,%d,  %.6f, %.6f, %.6f, %.6f,  %.6f, %.6f, %.6f, %.6f\n",loop,4,y_ref_kk[0],y_ref_kk[1],y_ref_kk[2],y_ref_kk[3],y_ref_kk[4],y_ref_kk[5],y_ref_kk[6],y_ref_kk[7]);
printf("%d,%d,  %.6f, %.6f, %.6f, %.6f\n",loop,5,x_kk[0],x_kk[1],x_kk[2],x_kk[3]);
printf("%d,%d,  %.3f",loop,6,u_opt[12]);


}

    cleanup_platform();
    return 0;
}



