#include "NUC131.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//Master main1 with BT cmd echo

//=========================================================
// Pin Mapping
//=========================================================
// Right
// Motor_01_IN3			PA12/PWM0_CH0
// Motor_01_IN4			PA1 /PWM0_CH5
// Motor_01_Enable		PA4
//=========================================================
// Left
// Motor_02_IN3			PA2 /PWM1_CH0
// Motor_02_IN4 		PA3 /PWM1_CH1
// Motor_02_Enable		PA5
//=========================================================
// Right
// Encoder_01_A			PC0
// Encoder_01_B			PC1
//=========================================================
// Left
// Encoder_02_A			PC2
// Encoder_02_B			PC3
//=========================================================
// BT_Control_UART_TX	PB0/UART0_RXD
// BT_Control_UART_RX	PB1/UART0_TXD
//=========================================================
// BT_Debug_UART_TX		PB4/UART1_RXD
// BT_Debug_UART_RX		PB5/UART1_TXD
//=========================================================
// CAN_RXD				PD6/CAN0_RXD
// CAN_TXD				PD7/CAN0_TXD
// CAN_Standby			PD14  // 0 = Normal Mode, 1 = Standby Mode
//=========================================================

//Extra motors
//=========================================================
// Right1
// Motor_01_IN3			PA14/PWM0_CH2
// Motor_01_IN4			PA15/PWM0_CH3
// Motor_01_Enable			//not assigned, share?
//=========================================================
// Left1
// Motor_02_IN3			PF4 /PWM1_CH4
// Motor_02_IN4 			PF5 /PWM1_CH5
// Motor_02_Enable			//not assigned, share?
//=========================================================
// Right1
// Encoder_01_A			//not assigned
// Encoder_01_B			//not assigned
//=========================================================
// Left1
// Encoder_02_A			//not assigned
// Encoder_02_B			//not assigned
//=========================================================


//=========================================================
// Robot Action Command
//=========================================================
int 			g_Robot_Action_CMD_01 = 0;
//=========================================================



//=========================================================
// UART
//=========================================================
unsigned int	g_UART_00_RX_Result_Byte 			= 0;
unsigned int  	g_UART_01_RX_Result_Byte 			= 0;
//=========================================================
uint8_t     	g_UART_00_RX_Result_Buffer[100];
uint8_t     	g_UART_00_RX_Result_Buffer_Count 	= 0;
uint8_t     	g_UART_00_RX_Result_Status 			= 0;
//=========================================================
uint8_t     	g_UART_01_RX_Result_Buffer[100];
uint8_t     	g_UART_01_RX_Result_Buffer_Count 	= 0;
uint8_t     	g_UART_01_RX_Result_Status 			= 0;
//=========================================================



//=========================================================
// Motor
//=========================================================
unsigned int  	g_Motor_Control_PWM_Frequency        = 0;
unsigned int  	g_Motor_Control_PWM_DutyCycle_R_01   = 0;
unsigned int  	g_Motor_Control_PWM_DutyCycle_L_01   = 0;
unsigned int  	g_Motor_Control_Direction_Value_R_01 = 0;
unsigned int  	g_Motor_Control_Direction_Value_L_01 = 0;
//=========================================================

//=========================================================
// xtra Motor
//=========================================================
unsigned int  	g_Motor1_Control_PWM_Frequency        = 0;
unsigned int  	g_Motor1_Control_PWM_DutyCycle_R_01   = 0;
unsigned int  	g_Motor1_Control_PWM_DutyCycle_L_01   = 0;
unsigned int  	g_Motor1_Control_Direction_Value_R_01 = 0;
unsigned int  	g_Motor1_Control_Direction_Value_L_01 = 0;
//=========================================================

//=========================================================
// PID //xtra seems needed
//=========================================================
float 			g_PID_Parameter_L_P 				= 0;
float 			g_PID_Parameter_L_I 				= 0;
float 			g_PID_Parameter_L_D 				= 0;
int 			g_PID_Parameter_L_S 				= 0;
//=========================================================
float 			g_PID_Parameter_R_P 				= 0;
float 			g_PID_Parameter_R_I 				= 0;
float 			g_PID_Parameter_R_D 				= 0;
int  			g_PID_Parameter_R_S 				= 0;
//=========================================================
int 			g_PID_Action_Command_R				= 0;
int 			g_PID_Error_Last_R					= 0;
int 			g_PID_Action_Command_L				= 0;
int 			g_PID_Error_Last_L					= 0;
int 			g_PID_Value_Input_Target_Speed_R	= 0;
int 			g_PID_Value_Input_Target_Speed_L	= 0;
int 			g_PID_Value_Input_Current_Speed_R	= 0;
int 			g_PID_Value_Input_Current_Speed_L	= 0;
//=========================================================
unsigned long 	g_Encoder_Resolution 				= 0;
unsigned long 	g_Wheel_Diameter 					= 0;
unsigned long 	g_Motor_Status_Speed_Count_R_01     = 0;
unsigned long 	g_Motor_Status_Speed_Count_L_01     = 0;
unsigned long 	g_Motor_Status_Speed_Value_R_01     = 0;
unsigned long 	g_Motor_Status_Speed_Value_L_01     = 0;
unsigned int  	g_Motor_Status_Direction_Value_R_01	= 0;
unsigned int  	g_Motor_Status_Direction_Value_L_01	= 0;
//=========================================================



//=========================================================
// Function
//=========================================================
void f_Byte_Converter(unsigned long t_Data_Input_01, char *t_Data_Output_01);
//=========================================================
void f_Motor_Control(unsigned int t_Motor, unsigned int t_Direction, unsigned int t_PWM_DutyCycle, unsigned int t_PWM_Frequency);
//=========================================================
void f_PID_Control_Init(float t_PID_Parameter_L_P, float t_PID_Parameter_L_I, float t_PID_Parameter_L_D, int t_PID_Parameter_L_S, float t_PID_Parameter_R_P, float t_PID_Parameter_R_I, float t_PID_Parameter_R_D, int t_PID_Parameter_R_S, int t_Encoder_Resolution, int t_Wheel_Diameter);
void f_PID_Control_Equation(int *t_PID_Action_Command_01, int *t_PID_Error_Last_01, int t_PID_Value_Input_Target_01, int t_PID_Value_Input_Current_01, float t_PID_Parameter_P, float t_PID_Parameter_I, float t_PID_Parameter_D);



//=========================================================
void f_SYS_Init(void);
void f_SYS_Exit(void);
void f_Startup_Init(void);
//=========================================================



void GPCDEF_IRQHandler(void)
{
	//==================================================
	// Encoder
	//==================================================
	// Right
	// Encoder_01_A		PC0
	// Encoder_01_B		PC1
	//==================================================
	// Left
	// Encoder_02_A		PC2
	// Encoder_02_B 	PC3
	//==================================================

    //==================================================
    // Right - Encoder_01_A		PC0
    if(GPIO_GET_INT_FLAG(PC, BIT0))
    {
     	GPIO_CLR_INT_FLAG(PC, BIT0);
      	g_Motor_Status_Speed_Count_R_01 = g_Motor_Status_Speed_Count_R_01 + 1;
    }
	//==================================================
    // Left  - Encoder_02_A		PC2
    else if(GPIO_GET_INT_FLAG(PC, BIT2))
    {
        GPIO_CLR_INT_FLAG(PC, BIT2);
        g_Motor_Status_Speed_Count_L_01 = g_Motor_Status_Speed_Count_L_01 +1;
    }
    //TODO: xtra motors
    /*
    // Right1 - not assigned
    if(GPIO_GET_INT_FLAG(PC, BIT0))
    {
    	TODO: change
     	GPIO_CLR_INT_FLAG(PC, BIT0);
      	g_Motor_Status_Speed_Count_R_01 = g_Motor_Status_Speed_Count_R_01 + 1;
    }
	//==================================================
    // Left1 - not assigned
    else if(GPIO_GET_INT_FLAG(PC, BIT2))
    {
    	TODO: Change
        GPIO_CLR_INT_FLAG(PC, BIT2);
        g_Motor_Status_Speed_Count_L_01 = g_Motor_Status_Speed_Count_L_01 +1;
    }


     */
    else
    {
        PC->ISRC = PC->ISRC;
        PD->ISRC = PD->ISRC;
        PE->ISRC = PE->ISRC;
        PF->ISRC = PF->ISRC;
    }
}

void TMR0_IRQHandler(void)
{
	//====================================================================================
	// PID Control Operation Loop
	//====================================================================================

	//====================================================================================
	// Right - PID Control Operation
	//====================================================================================
	unsigned long t_Motor_Status_Speed_Count_R_01 = g_Motor_Status_Speed_Count_R_01;

	g_PID_Value_Input_Target_Speed_R = g_PID_Parameter_R_S;
	f_PID_Control_Equation(&g_PID_Action_Command_R, &g_PID_Error_Last_R, abs(g_PID_Value_Input_Target_Speed_R), t_Motor_Status_Speed_Count_R_01, g_PID_Parameter_R_P, g_PID_Parameter_R_I, g_PID_Parameter_R_D);
	g_Motor_Control_PWM_DutyCycle_R_01 = g_PID_Action_Command_R;

	if(g_Robot_Action_CMD_01==0)
	{
		g_Motor_Control_PWM_DutyCycle_R_01   = 0;
		g_Motor_Control_Direction_Value_R_01 = 1;
		f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
	}
	else
	{
		f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
	}
	/*
	 */
	//====================================================================================
	// Left - PID Control Operation
	//====================================================================================
	unsigned long t_Motor_Status_Speed_Count_L_01 = g_Motor_Status_Speed_Count_L_01;

	g_PID_Value_Input_Target_Speed_L = g_PID_Parameter_L_S;
	f_PID_Control_Equation(&g_PID_Action_Command_L, &g_PID_Error_Last_L, abs(g_PID_Value_Input_Target_Speed_L), t_Motor_Status_Speed_Count_L_01, g_PID_Parameter_L_P, g_PID_Parameter_L_I, g_PID_Parameter_L_D);
	g_Motor_Control_PWM_DutyCycle_L_01 = g_PID_Action_Command_L;

	if(g_Robot_Action_CMD_01==0)
	{
		g_Motor_Control_PWM_DutyCycle_L_01   = 0;
		g_Motor_Control_Direction_Value_L_01 = 1;
		f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);
	}
	else
	{
		f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);
	}
	/*
	 */
	//====================================================================================


	//====================================================================================
	// Robot Status Monitoring
	//====================================================================================
	uint8_t t_UART_NewLine[2] 					= {0x0A, 0x0D};
	char 	m_Speed_Value_String_01[10] 		= {0,0,0,0,0,0,0,0,0,0};
	char 	m_Speed_Value_String_02[10] 		= {0,0,0,0,0,0,0,0,0,0};
	char 	m_Speed_Value_String_99[10] 		= {0,0,0,0,0,0,0,0,0,0};
	char 	m_Final_Speed_Value_String_01[20]	= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	char 	m_Final_Speed_Value_String_99[24]	= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	int 	m_CRC_Error_Check_01 				= 0;
	//====================================================================================
	f_Byte_Converter(t_Motor_Status_Speed_Count_L_01, (char*)m_Speed_Value_String_01);
	m_Speed_Value_String_01[0] 					= 'L';
	//====================================================================================
	f_Byte_Converter(t_Motor_Status_Speed_Count_R_01, (char*)m_Speed_Value_String_02);
	m_Speed_Value_String_02[0] 					= 'R';
	//====================================================================================
	m_Speed_Value_String_99[0] 					= m_Speed_Value_String_01[0];
	m_Speed_Value_String_99[1] 					= m_Speed_Value_String_01[6];
	m_Speed_Value_String_99[2] 					= m_Speed_Value_String_01[7];
	m_Speed_Value_String_99[3] 					= m_Speed_Value_String_01[8];
	m_Speed_Value_String_99[4] 					= m_Speed_Value_String_01[9];
	m_Speed_Value_String_99[5] 					= m_Speed_Value_String_02[0];
	m_Speed_Value_String_99[6] 					= m_Speed_Value_String_02[6];
	m_Speed_Value_String_99[7] 					= m_Speed_Value_String_02[7];
	m_Speed_Value_String_99[8] 					= m_Speed_Value_String_02[8];
	m_Speed_Value_String_99[9] 					= m_Speed_Value_String_02[9];
	//====================================================================================
	m_Final_Speed_Value_String_01[0] 			= (((m_Speed_Value_String_99[0]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[1] 			= (((m_Speed_Value_String_99[0]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[2] 			= (((m_Speed_Value_String_99[1]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[3] 			= (((m_Speed_Value_String_99[1]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[4] 			= (((m_Speed_Value_String_99[2]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[5] 			= (((m_Speed_Value_String_99[2]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[6] 			= (((m_Speed_Value_String_99[3]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[7] 			= (((m_Speed_Value_String_99[3]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[8] 			= (((m_Speed_Value_String_99[4]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[9] 			= (((m_Speed_Value_String_99[4]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[10] 			= (((m_Speed_Value_String_99[5]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[11] 			= (((m_Speed_Value_String_99[5]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[12] 			= (((m_Speed_Value_String_99[6]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[13] 			= (((m_Speed_Value_String_99[6]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[14] 			= (((m_Speed_Value_String_99[7]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[15] 			= (((m_Speed_Value_String_99[7]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[16] 			= (((m_Speed_Value_String_99[8]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[17] 			= (((m_Speed_Value_String_99[8]&0x0F)     )+0x30);
	m_Final_Speed_Value_String_01[18] 			= (((m_Speed_Value_String_99[9]&0xF0)/0x10)+0x30);
	m_Final_Speed_Value_String_01[19] 			= (((m_Speed_Value_String_99[9]&0x0F)     )+0x30);
	//====================================================================================
	for (int Temp_Count_01 = 0; Temp_Count_01 < 20; Temp_Count_01++)
	{
		m_CRC_Error_Check_01 = m_CRC_Error_Check_01 ^ m_Final_Speed_Value_String_01[Temp_Count_01];
	}
	//====================================================================================
	m_Final_Speed_Value_String_99[0]  			= 0x41;									// Start Byte 'A'
	m_Final_Speed_Value_String_99[1]  			= 20;									// Length
	m_Final_Speed_Value_String_99[2]  			= m_Final_Speed_Value_String_01[0];		// Data Byte 00
	m_Final_Speed_Value_String_99[3]  			= m_Final_Speed_Value_String_01[1];		// Data Byte 01
	m_Final_Speed_Value_String_99[4]  			= m_Final_Speed_Value_String_01[2];		// Data Byte 02
	m_Final_Speed_Value_String_99[5]  			= m_Final_Speed_Value_String_01[3];		// Data Byte 03
	m_Final_Speed_Value_String_99[6]  			= m_Final_Speed_Value_String_01[4]; 	// Data Byte 04
	m_Final_Speed_Value_String_99[7]  			= m_Final_Speed_Value_String_01[5];		// Data Byte 05
	m_Final_Speed_Value_String_99[8]  			= m_Final_Speed_Value_String_01[6];		// Data Byte 06
	m_Final_Speed_Value_String_99[9]  			= m_Final_Speed_Value_String_01[7];		// Data Byte 07
	m_Final_Speed_Value_String_99[10] 			= m_Final_Speed_Value_String_01[8];		// Data Byte 08
	m_Final_Speed_Value_String_99[11] 			= m_Final_Speed_Value_String_01[9];		// Data Byte 09
	m_Final_Speed_Value_String_99[12] 			= m_Final_Speed_Value_String_01[10]; 	// Data Byte 10
	m_Final_Speed_Value_String_99[13] 			= m_Final_Speed_Value_String_01[11];	// Data Byte 11
	m_Final_Speed_Value_String_99[14] 			= m_Final_Speed_Value_String_01[12];	// Data Byte 12
	m_Final_Speed_Value_String_99[15] 			= m_Final_Speed_Value_String_01[13];	// Data Byte 13
	m_Final_Speed_Value_String_99[16] 			= m_Final_Speed_Value_String_01[14];	// Data Byte 14
	m_Final_Speed_Value_String_99[17] 			= m_Final_Speed_Value_String_01[15];	// Data Byte 15
	m_Final_Speed_Value_String_99[18] 			= m_Final_Speed_Value_String_01[16]; 	// Data Byte 16
	m_Final_Speed_Value_String_99[19] 			= m_Final_Speed_Value_String_01[17];	// Data Byte 17
	m_Final_Speed_Value_String_99[20] 			= m_Final_Speed_Value_String_01[18];	// Data Byte 18
	m_Final_Speed_Value_String_99[21] 			= m_Final_Speed_Value_String_01[19];	// Data Byte 19
	m_Final_Speed_Value_String_99[22] 			= m_CRC_Error_Check_01;					// CRC Error Check
	m_Final_Speed_Value_String_99[23] 			= 0x42;									// Stop Byte 'B'
	//====================================================================================
	// Send Robot Status by Bluetooth Debug
	UART_Write(UART1, m_Final_Speed_Value_String_99, sizeof(m_Final_Speed_Value_String_99));
	//====================================================================================
	// Reset Motor Speed Count Record
	g_Motor_Status_Speed_Count_L_01 			= 0;
	g_Motor_Status_Speed_Count_R_01 			= 0;
	//====================================================================================

    TIMER_ClearIntFlag(TIMER0);
}

void UART02_IRQHandler(void)
{
    uint32_t     u32IntSts= UART0->ISR;
    uint8_t      t_UART_00_RX_Result_Buffer[1] = {0};
    unsigned int t_UART_00_RX_Result_Byte      = 0;

    if(u32IntSts & UART_IS_RX_READY(UART0))
    {
       	UART_Read(UART0, t_UART_00_RX_Result_Buffer, sizeof(t_UART_00_RX_Result_Buffer));
       	t_UART_00_RX_Result_Byte = (unsigned int)t_UART_00_RX_Result_Buffer[0];
       	g_UART_00_RX_Result_Byte = t_UART_00_RX_Result_Byte;
       	UART_Write(UART0, t_UART_00_RX_Result_Buffer, sizeof(t_UART_00_RX_Result_Buffer));
       	//the following is to echo cmd
       	UART_Write(UART4, t_UART_00_RX_Result_Buffer, sizeof(t_UART_00_RX_Result_Buffer));


       	if(g_UART_00_RX_Result_Byte==0x41)
       	{
       		g_UART_00_RX_Result_Status 			= 0;
       		g_UART_00_RX_Result_Buffer_Count 	= 0;
       		g_UART_00_RX_Result_Buffer[0] 		= g_UART_00_RX_Result_Byte;
       	}
       	else if(g_UART_00_RX_Result_Byte==0x42)
       	{
       		if(g_UART_00_RX_Result_Status==1)
       		{
       			g_UART_00_RX_Result_Status = 2;
       			g_UART_00_RX_Result_Buffer_Count++;
       			g_UART_00_RX_Result_Buffer[g_UART_00_RX_Result_Buffer_Count] = g_UART_00_RX_Result_Byte;
       			UART_Write(UART1, g_UART_00_RX_Result_Buffer, sizeof(g_UART_00_RX_Result_Buffer));
       		}
       	}
       	else
       	{
       		if((g_UART_00_RX_Result_Status==0)||(g_UART_00_RX_Result_Status==1))
       		{
       			g_UART_00_RX_Result_Status = 1;
       			g_UART_00_RX_Result_Buffer_Count++;
       			g_UART_00_RX_Result_Buffer[g_UART_00_RX_Result_Buffer_Count] = g_UART_00_RX_Result_Byte;
       		}
       	}
    }
}

void UART1_IRQHandler(void)
{
    uint32_t     u32IntSts= UART1->ISR;
    uint8_t      t_UART_01_RX_Result_Buffer[1] = {0};
    unsigned int t_UART_01_RX_Result_Byte      = 0;

    if(u32IntSts & UART_IS_RX_READY(UART1))
    {
       	UART_Read(UART1, t_UART_01_RX_Result_Buffer, sizeof(t_UART_01_RX_Result_Buffer));
       	t_UART_01_RX_Result_Byte = (unsigned int)t_UART_01_RX_Result_Buffer[0];
       	g_UART_01_RX_Result_Byte = t_UART_01_RX_Result_Byte;

       	if(g_UART_01_RX_Result_Byte==0x41)
       	{
       		g_UART_01_RX_Result_Status 			= 0;
       		g_UART_01_RX_Result_Buffer_Count 	= 0;
       	    g_UART_01_RX_Result_Buffer[0] 		= g_UART_01_RX_Result_Byte;
       	}
       	else if(g_UART_01_RX_Result_Byte==0x42)
       	{
       		if(g_UART_01_RX_Result_Status==1)
       		{
           		g_UART_01_RX_Result_Status = 2;
           		g_UART_01_RX_Result_Buffer_Count++;
           		g_UART_01_RX_Result_Buffer[g_UART_01_RX_Result_Buffer_Count] = g_UART_01_RX_Result_Byte;

           		uint8_t Temp_Data_Setting_01_List[10];
           		Temp_Data_Setting_01_List[0] 				= (g_UART_01_RX_Result_Buffer[2] -0x30)*0x10+(g_UART_01_RX_Result_Buffer[3] -0x30);
           		Temp_Data_Setting_01_List[1] 				= (g_UART_01_RX_Result_Buffer[4] -0x30)*0x10+(g_UART_01_RX_Result_Buffer[5] -0x30);
           		Temp_Data_Setting_01_List[2] 				= (g_UART_01_RX_Result_Buffer[6] -0x30)*0x10+(g_UART_01_RX_Result_Buffer[7] -0x30);
           		Temp_Data_Setting_01_List[3] 				= (g_UART_01_RX_Result_Buffer[8] -0x30)*0x10+(g_UART_01_RX_Result_Buffer[9] -0x30);
           		Temp_Data_Setting_01_List[4] 				= (g_UART_01_RX_Result_Buffer[10]-0x30)*0x10+(g_UART_01_RX_Result_Buffer[11]-0x30);
           		Temp_Data_Setting_01_List[5] 				= (g_UART_01_RX_Result_Buffer[12]-0x30)*0x10+(g_UART_01_RX_Result_Buffer[13]-0x30);
           		Temp_Data_Setting_01_List[6] 				= (g_UART_01_RX_Result_Buffer[14]-0x30)*0x10+(g_UART_01_RX_Result_Buffer[15]-0x30);
           		Temp_Data_Setting_01_List[7] 				= (g_UART_01_RX_Result_Buffer[16]-0x30)*0x10+(g_UART_01_RX_Result_Buffer[17]-0x30);
           		Temp_Data_Setting_01_List[8] 				= (g_UART_01_RX_Result_Buffer[18]-0x30)*0x10+(g_UART_01_RX_Result_Buffer[19]-0x30);
           		Temp_Data_Setting_01_List[9] 				= (g_UART_01_RX_Result_Buffer[20]-0x30)*0x10+(g_UART_01_RX_Result_Buffer[21]-0x30);

           		char m_Final_Setting_Value_String_01[20]    = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
           		char m_Final_Setting_Value_String_99[24] 	= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

           		m_Final_Setting_Value_String_01[0] 			= (((Temp_Data_Setting_01_List[0]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[1] 			= (((Temp_Data_Setting_01_List[0]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[2] 			= (((Temp_Data_Setting_01_List[1]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[3] 			= (((Temp_Data_Setting_01_List[1]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[4] 			= (((Temp_Data_Setting_01_List[2]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[5] 			= (((Temp_Data_Setting_01_List[2]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[6] 			= (((Temp_Data_Setting_01_List[3]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[7] 			= (((Temp_Data_Setting_01_List[3]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[8] 			= (((Temp_Data_Setting_01_List[4]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[9] 			= (((Temp_Data_Setting_01_List[4]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[10] 		= (((Temp_Data_Setting_01_List[5]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[11] 		= (((Temp_Data_Setting_01_List[5]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[12] 		= (((Temp_Data_Setting_01_List[6]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[13] 		= (((Temp_Data_Setting_01_List[6]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[14] 		= (((Temp_Data_Setting_01_List[7]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[15] 		= (((Temp_Data_Setting_01_List[7]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[16] 		= (((Temp_Data_Setting_01_List[8]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[17] 		= (((Temp_Data_Setting_01_List[8]&0x0F)     )+0x30);
           		m_Final_Setting_Value_String_01[18] 		= (((Temp_Data_Setting_01_List[9]&0xF0)/0x10)+0x30);
           		m_Final_Setting_Value_String_01[19] 		= (((Temp_Data_Setting_01_List[9]&0x0F)     )+0x30);

           		int m_CRC_Error_Check_01 = 0;
           		for (int Temp_Count_01 = 0; Temp_Count_01 < 20; Temp_Count_01++)
           		{
           			m_CRC_Error_Check_01 = m_CRC_Error_Check_01 ^ m_Final_Setting_Value_String_01[Temp_Count_01];
           		}

           		m_Final_Setting_Value_String_99[0] 			= 0x41;
           		m_Final_Setting_Value_String_99[1] 			= 20;
           		m_Final_Setting_Value_String_99[2] 			= m_Final_Setting_Value_String_01[0];
           		m_Final_Setting_Value_String_99[3] 			= m_Final_Setting_Value_String_01[1];
           		m_Final_Setting_Value_String_99[4] 			= m_Final_Setting_Value_String_01[2];
           		m_Final_Setting_Value_String_99[5] 			= m_Final_Setting_Value_String_01[3];
           		m_Final_Setting_Value_String_99[6] 			= m_Final_Setting_Value_String_01[4];
           		m_Final_Setting_Value_String_99[7] 			= m_Final_Setting_Value_String_01[5];
           		m_Final_Setting_Value_String_99[8] 			= m_Final_Setting_Value_String_01[6];
           		m_Final_Setting_Value_String_99[9] 			= m_Final_Setting_Value_String_01[7];
           		m_Final_Setting_Value_String_99[10] 		= m_Final_Setting_Value_String_01[8];
           		m_Final_Setting_Value_String_99[11] 		= m_Final_Setting_Value_String_01[9];
           		m_Final_Setting_Value_String_99[12] 		= m_Final_Setting_Value_String_01[10];
           		m_Final_Setting_Value_String_99[13] 		= m_Final_Setting_Value_String_01[11];
           		m_Final_Setting_Value_String_99[14] 		= m_Final_Setting_Value_String_01[12];
           		m_Final_Setting_Value_String_99[15] 		= m_Final_Setting_Value_String_01[13];
           		m_Final_Setting_Value_String_99[16] 		= m_Final_Setting_Value_String_01[14];
           		m_Final_Setting_Value_String_99[17] 		= m_Final_Setting_Value_String_01[15];
           		m_Final_Setting_Value_String_99[18] 		= m_Final_Setting_Value_String_01[16];
           		m_Final_Setting_Value_String_99[19] 		= m_Final_Setting_Value_String_01[17];
           		m_Final_Setting_Value_String_99[20] 		= m_Final_Setting_Value_String_01[18];
           		m_Final_Setting_Value_String_99[21] 		= m_Final_Setting_Value_String_01[19];
           		m_Final_Setting_Value_String_99[22] 		= m_CRC_Error_Check_01;
           		m_Final_Setting_Value_String_99[23] 		= 0x42;

           		if (g_UART_01_RX_Result_Buffer[22]==m_CRC_Error_Check_01)
           		{
           			char Temp_CMD_RL_01   			= Temp_Data_Setting_01_List[0];
           			char Temp_CMD_Type_01 			= Temp_Data_Setting_01_List[1];

           			Temp_Data_Setting_01_List[0] 	= '0';
           			Temp_Data_Setting_01_List[1] 	= '0';

           			if(Temp_CMD_RL_01=='L')
           			{
           				if(Temp_CMD_Type_01=='P')
           				{
           					float Temp_CMD_Data_01 	= (float)atof(Temp_Data_Setting_01_List);
           					g_PID_Parameter_L_P    	= Temp_CMD_Data_01 - 0.00052;
           				}
           				else if(Temp_CMD_Type_01=='I')
           				{
           					float Temp_CMD_Data_01 	= (float)atof(Temp_Data_Setting_01_List);
           					g_PID_Parameter_L_I    	= Temp_CMD_Data_01 - 0.00052;
           				}
           				else if(Temp_CMD_Type_01=='D')
           				{
           					float Temp_CMD_Data_01 	= (float)atof(Temp_Data_Setting_01_List);
           					g_PID_Parameter_L_D    	= Temp_CMD_Data_01 - 0.00052;
           				}
           				else if(Temp_CMD_Type_01=='S')
           				{
           					int Temp_CMD_Data_01  	= (int)atoi(Temp_Data_Setting_01_List);
           					g_PID_Parameter_L_S   	= Temp_CMD_Data_01;
           				}
           			}
           			else if(Temp_CMD_RL_01=='R')
           			{
           				if(Temp_CMD_Type_01=='P')
           				{
           					float Temp_CMD_Data_01  = (float)atof(Temp_Data_Setting_01_List);
           					g_PID_Parameter_R_P  	= Temp_CMD_Data_01 - 0.00052;
           				}
           				else if(Temp_CMD_Type_01=='I')
           				{
           					float Temp_CMD_Data_01  = (float)atof(Temp_Data_Setting_01_List);
           					g_PID_Parameter_R_I 	= Temp_CMD_Data_01 - 0.00052;
           				}
           				else if(Temp_CMD_Type_01=='D')
           				{
           					float Temp_CMD_Data_01 	= (float)atof(Temp_Data_Setting_01_List);
           					g_PID_Parameter_R_D 	= Temp_CMD_Data_01 - 0.00052;
           				}
           				else if(Temp_CMD_Type_01=='S')
           				{
           					int Temp_CMD_Data_01 	= (int)atoi(Temp_Data_Setting_01_List);
           					g_PID_Parameter_R_S 	= Temp_CMD_Data_01;
           				}
           			}

           			UART_Write(UART1, m_Final_Setting_Value_String_99, sizeof(m_Final_Setting_Value_String_99));
           		}
       		}
       	}
       	else
       	{
       		if((g_UART_01_RX_Result_Status==0)||(g_UART_01_RX_Result_Status==1))
       		{
       			g_UART_01_RX_Result_Status = 1;
       			g_UART_01_RX_Result_Buffer_Count++;
       			g_UART_01_RX_Result_Buffer[g_UART_01_RX_Result_Buffer_Count] = g_UART_01_RX_Result_Byte;
       		}
       	}
    }
}

void f_Byte_Converter(unsigned long t_Data_Input_01, char *t_Data_Output_01)
{
    unsigned long t_Motor_Status_Speed_Count_L_01 = t_Data_Input_01;

    unsigned int t_Motor_Status_Speed_Count_L_String[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	unsigned int t_Motor_Status_Speed_Count_R_String[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	unsigned long t_Value = t_Motor_Status_Speed_Count_L_01;
	unsigned int t_D_10 = (int)(t_Value)/1000000000;
	unsigned int t_D_09 = (int)(t_Value-(t_D_10*1000000000))/100000000;
	unsigned int t_D_08 = (int)(t_Value-(t_D_10*1000000000+t_D_09*100000000))/10000000;
	unsigned int t_D_07 = (int)(t_Value-(t_D_10*1000000000+t_D_09*100000000+t_D_08*10000000))/1000000;
	unsigned int t_D_06 = (int)(t_Value-(t_D_10*1000000000+t_D_09*100000000+t_D_08*10000000+t_D_07*1000000))/100000;
	unsigned int t_D_05 = (int)(t_Value-(t_D_10*1000000000+t_D_09*100000000+t_D_08*10000000+t_D_07*1000000+t_D_06*100000))/10000;
	unsigned int t_D_04 = (int)(t_Value-(t_D_10*1000000000+t_D_09*100000000+t_D_08*10000000+t_D_07*1000000+t_D_06*100000+t_D_05*10000))/1000;
	unsigned int t_D_03 = (int)(t_Value-(t_D_10*1000000000+t_D_09*100000000+t_D_08*10000000+t_D_07*1000000+t_D_06*100000+t_D_05*10000+t_D_04*1000))/100;
	unsigned int t_D_02 = (int)(t_Value-(t_D_10*1000000000+t_D_09*100000000+t_D_08*10000000+t_D_07*1000000+t_D_06*100000+t_D_05*10000+t_D_04*1000+t_D_03*100))/10;
	unsigned int t_D_01 = (int)(t_Value-(t_D_10*1000000000+t_D_09*100000000+t_D_08*10000000+t_D_07*1000000+t_D_06*100000+t_D_05*10000+t_D_04*1000+t_D_03*100+t_D_02*10))/1;

    t_D_10 = t_D_10 + 0x30;
    t_D_09 = t_D_09 + 0x30;
    t_D_08 = t_D_08 + 0x30;
    t_D_07 = t_D_07 + 0x30;
    t_D_06 = t_D_06 + 0x30;
    t_D_05 = t_D_05 + 0x30;
    t_D_04 = t_D_04 + 0x30;
    t_D_03 = t_D_03 + 0x30;
    t_D_02 = t_D_02 + 0x30;
    t_D_01 = t_D_01 + 0x30;

    t_Data_Output_01[0] = t_D_10;
    t_Data_Output_01[1] = t_D_09;
    t_Data_Output_01[2] = t_D_08;
    t_Data_Output_01[3] = t_D_07;
    t_Data_Output_01[4] = t_D_06;
    t_Data_Output_01[5] = t_D_05;
    t_Data_Output_01[6] = t_D_04;
    t_Data_Output_01[7] = t_D_03;
    t_Data_Output_01[8] = t_D_02;
    t_Data_Output_01[9] = t_D_01;
}

void f_Motor_Control(unsigned int t_Motor, unsigned int t_Direction, unsigned int t_PWM_DutyCycle, unsigned int t_PWM_Frequency)
{
	//===================================================
	// t_Motor 			: 1=Right,   2=Left
	// t_Direction 		: 1=Forward, 2=Backward
	// t_PWM_DutyCycle 	: 0-100

	if (t_Motor == 1)
	{
		if (t_Direction == 1)
		{
			PA1 = 0;
			if(0<t_PWM_DutyCycle)
			{
				t_PWM_DutyCycle = 100-t_PWM_DutyCycle;
			}
		}
		else if (t_Direction == 2)
		{
			PA1 = 1;
			int abc = 999;
		}

		if(((0<t_PWM_DutyCycle)&&(t_PWM_DutyCycle<=100))&&((100<=t_PWM_Frequency)&&(t_PWM_Frequency<=5000)))
		{
			PA4 = 1;
			t_PWM_DutyCycle = 100 - t_PWM_DutyCycle;
			PWM_ConfigOutputChannel(PWM0, 0, t_PWM_Frequency, t_PWM_DutyCycle);
			PWM_EnableOutput(PWM0, PWM_CH_0_MASK);
			PWM_Start(PWM0, PWM_CH_0_MASK);
		}
		else if((t_PWM_DutyCycle==0)&&((100<=t_PWM_Frequency)&&(t_PWM_Frequency<=5000)))
		{
			PA4 = 0;
			PWM_ConfigOutputChannel(PWM0, 0, t_PWM_Frequency, t_PWM_DutyCycle);
			PWM_EnableOutput(PWM0, PWM_CH_0_MASK);
			PWM_Start(PWM0, PWM_CH_0_MASK);
		}
		else
		{
			t_PWM_Frequency = 100;
			t_PWM_DutyCycle = 0;
			PA4 = 0;
			PWM_ConfigOutputChannel(PWM0, 0, t_PWM_Frequency, t_PWM_DutyCycle);
			PWM_EnableOutput(PWM0, PWM_CH_0_MASK);
			PWM_Start(PWM0, PWM_CH_0_MASK);
		}
	}
	else if (t_Motor == 2)
	{
		if (t_Direction == 1)
		{
			PA3 = 1;
		}
		else if (t_Direction == 2)
		{
			PA3 = 0;
			if(0<t_PWM_DutyCycle)
			{
				t_PWM_DutyCycle = 100-t_PWM_DutyCycle;
			}
		}

		if(((0<t_PWM_DutyCycle)&&(t_PWM_DutyCycle<=100))&&((100<=t_PWM_Frequency)&&(t_PWM_Frequency<=5000)))
		{
			PA5 = 1;
			t_PWM_DutyCycle = 100 - t_PWM_DutyCycle;
			PWM_ConfigOutputChannel(PWM1, 0, t_PWM_Frequency, t_PWM_DutyCycle);
			PWM_EnableOutput(PWM1, PWM_CH_0_MASK);
			PWM_Start(PWM1, PWM_CH_0_MASK);
		}
		else if((t_PWM_DutyCycle==0)&&((100<=t_PWM_Frequency)&&(t_PWM_Frequency<=5000)))
		{
			PA5 = 0;
			PWM_ConfigOutputChannel(PWM1, 0, t_PWM_Frequency, t_PWM_DutyCycle);
			PWM_EnableOutput(PWM1, PWM_CH_0_MASK);
			PWM_Start(PWM1, PWM_CH_0_MASK);
		}
		else
		{
			t_PWM_Frequency = 100;
			t_PWM_DutyCycle = 0;
			PA5 = 0;
			PWM_ConfigOutputChannel(PWM1, 0, t_PWM_Frequency, t_PWM_DutyCycle);
			PWM_EnableOutput(PWM1, PWM_CH_0_MASK);
			PWM_Start(PWM1, PWM_CH_0_MASK);
		}
	}
}

void f_PID_Control_Init(float t_PID_Parameter_L_P, float t_PID_Parameter_L_I, float t_PID_Parameter_L_D, int t_PID_Parameter_L_S, float t_PID_Parameter_R_P, float t_PID_Parameter_R_I, float t_PID_Parameter_R_D, int t_PID_Parameter_R_S, int t_Encoder_Resolution, int t_Wheel_Diameter)
{
	g_PID_Parameter_L_P 		= t_PID_Parameter_L_P;
	g_PID_Parameter_L_I 		= t_PID_Parameter_L_I;
	g_PID_Parameter_L_D 		= t_PID_Parameter_L_D;
	g_PID_Parameter_L_S 		= t_PID_Parameter_L_S;
	g_PID_Parameter_R_P 		= t_PID_Parameter_R_P;
	g_PID_Parameter_R_I 		= t_PID_Parameter_R_I;
	g_PID_Parameter_R_D 		= t_PID_Parameter_R_D;
	g_PID_Parameter_R_S 		= t_PID_Parameter_R_S;
	g_Encoder_Resolution 		= t_Encoder_Resolution;
	g_Wheel_Diameter 			= t_Wheel_Diameter;
}

void f_PID_Control_Equation(int *t_PID_Action_Command_01, int *t_PID_Error_Last_01, int t_PID_Value_Input_Target_01, int t_PID_Value_Input_Current_01, float t_PID_Parameter_P, float t_PID_Parameter_I, float t_PID_Parameter_D)
{
	int 	m_PID_Error_01 		= abs(t_PID_Value_Input_Target_01) - abs(t_PID_Value_Input_Current_01);
	float	m_PID_Term_01  		= (t_PID_Parameter_P * m_PID_Error_01) + (t_PID_Parameter_D * (m_PID_Error_01 - *t_PID_Error_Last_01));
	*t_PID_Error_Last_01 		= m_PID_Error_01;
	*t_PID_Action_Command_01 	= *t_PID_Action_Command_01 + (int)(m_PID_Term_01);

	if (*t_PID_Action_Command_01 < 0)
	{
		*t_PID_Action_Command_01 = 0;
	}
	else if (*t_PID_Action_Command_01 > 99)
	{
		*t_PID_Action_Command_01 = 99;
	}
}

void f_SYS_Init(void)
{
	//================================================================================
	// Init Clock
	//================================================================================

    //==================================================
    // Init System Clock
    //==================================================
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);
    CLK_SetCoreClock(50000000);
    //==================================================
    // Init Module Clock - TIMER0
    //==================================================
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0_S_HCLK, NULL);
    CLK_EnableModuleClock(TMR0_MODULE);
    SYS_ResetModule(TMR0_RST);
    //==================================================
    // Init Module Clock - PWM0
    //==================================================
    CLK_SetModuleClock(PWM0_MODULE, CLK_CLKSEL3_PWM0_S_PLL, 0);
    CLK_EnableModuleClock(PWM0_MODULE);
    SYS_ResetModule(PWM0_RST);
    //==================================================
    // Init Module Clock - PWM1
    //==================================================
    CLK_SetModuleClock(PWM1_MODULE, CLK_CLKSEL3_PWM1_S_PLL, 0);
    CLK_EnableModuleClock(PWM1_MODULE);
    SYS_ResetModule(PWM1_RST);
    //==================================================
    // Init Module Clock - UART0
    //==================================================
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_EnableModuleClock(UART0_MODULE);
    SYS_ResetModule(UART0_RST);
    //==================================================
    // Init Module Clock - UART1
    //==================================================
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_EnableModuleClock(UART1_MODULE);
    SYS_ResetModule(UART1_RST);

    //==================================================
    // !!!! Init Module Clock - UART4
    //==================================================
    CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    CLK_EnableModuleClock(UART4_MODULE);
    SYS_ResetModule(UART4_RST);

    //==================================================
    // Init Module Clock - CAN0
    //==================================================
    CLK_EnableModuleClock(CAN0_MODULE);
    SYS_ResetModule(CAN0_RST);


    //================================================================================
    // Init Multi Function Pin
    //================================================================================

    //==================================================
    // Init Multi Function Pin - PWM0 //Right //Motor_01_IN3 //PA12/PWM0_CH0
    //==================================================
    SYS->GPA_MFP  &= ~(SYS_GPA_MFP_PA12_Msk);
    SYS->GPA_MFP  |=   SYS_GPA_MFP_PA12_PWM0_CH0;
    SYS->ALT_MFP4 &= ~(SYS_ALT_MFP4_PA12_Msk);
    SYS->ALT_MFP4 |=   SYS_ALT_MFP4_PA12_PWM0_CH0;
    //==================================================
    // Init Multi Function Pin - PWM1 //Left //Motor_02_IN3 //PA2/PWM1_CH0
    //==================================================
    SYS->GPA_MFP  &= ~(SYS_GPA_MFP_PA2_Msk);
    SYS->GPA_MFP  |=   SYS_GPA_MFP_PA2_PWM1_CH0;
    SYS->ALT_MFP3 &= ~(SYS_ALT_MFP3_PA2_Msk);
    SYS->ALT_MFP3 |=   SYS_ALT_MFP3_PA2_PWM1_CH0;
    //==================================================
    // Init Multi Function Pin - UART0
    //==================================================
    SYS->GPB_MFP  &= ~(SYS_GPB_MFP_PB0_Msk       | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP  |=   SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
    //==================================================
    // Init Multi Function Pin - UART1
    //==================================================
    SYS->GPB_MFP  &= ~(SYS_GPB_MFP_PB4_Msk       | SYS_GPB_MFP_PB5_Msk);
    SYS->GPB_MFP  |=   SYS_GPB_MFP_PB4_UART1_RXD | SYS_GPB_MFP_PB5_UART1_TXD;
    //==================================================
    // Init Multi Function Pin - CAN0
    //==================================================
    SYS->GPD_MFP  &= ~(SYS_GPD_MFP_PD6_Msk       | SYS_GPD_MFP_PD7_Msk);
    SYS->GPD_MFP  =    SYS_GPD_MFP_PD6_CAN0_RXD  | SYS_GPD_MFP_PD7_CAN0_TXD;
}

void f_SYS_Exit(void)
{
	TIMER_Close(TIMER0);
	PWM_Stop(PWM0, PWM_CH_0_MASK);
	PWM_Stop(PWM1, PWM_CH_0_MASK);
	UART_Close(UART0);
	UART_Close(UART1);
	UART_Close(UART4);
	CAN_Close(CAN0);
}

void f_Startup_Init(void)
{
	//==================================================
	// I/O Configuration
	//==================================================
	// Motor
	//==================================================
	// Right (1)
	// PWM    --> PA12 --> PWM0_CH0
	// DIR    --> PA1
	// Enable --> PA4
	// Left  (2)
	// PWM    --> PA2  --> PWM1_CH0
	// DIR    --> PA3
	// Enable --> PA5
	//==================================================
	// Encoder
	//==================================================
	// Right (1)
	// Encoder_01_A - PC0
	// Encoder_01_B - PC1
	// Left  (2)
    // Encoder_02_A - PC2
	// Encoder_02_B - PC3
	//==================================================

	GPIO_SetMode(PC, BIT2,  GPIO_PMD_INPUT); 	// R_A Input
	GPIO_SetMode(PC, BIT3,  GPIO_PMD_INPUT);	// R_B Input
	GPIO_SetMode(PC, BIT0,  GPIO_PMD_INPUT); 	// L_A Input
	GPIO_SetMode(PC, BIT1, 	GPIO_PMD_INPUT);	// L_B Input

	GPIO_EnableInt(PC, 0,   GPIO_INT_RISING);	// R_A Interrupt
	GPIO_EnableInt(PC, 2,   GPIO_INT_RISING);	// L_A Interrupt
	NVIC_EnableIRQ(GPCDEF_IRQn);

	// Motor_01_IN3  PA12/PWM0_CH0 // Right
	GPIO_SetMode(PA, BIT1,  GPIO_PMD_OUTPUT); 	// R_Direction
	GPIO_SetMode(PA, BIT4,  GPIO_PMD_OUTPUT); 	// R_Enable

	// Motor_02_IN3  PA2/PWM1_CH0 // Left
	GPIO_SetMode(PA, BIT3,  GPIO_PMD_OUTPUT); 	// L_Direction
	GPIO_SetMode(PA, BIT5,  GPIO_PMD_OUTPUT); 	// L_Enable

	g_Motor_Control_PWM_Frequency        = 100;

	g_Motor_Control_PWM_DutyCycle_R_01   = 0;
	g_Motor_Control_Direction_Value_R_01 = 1;
	f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
	g_Motor_Control_PWM_DutyCycle_L_01   = 0;
	g_Motor_Control_Direction_Value_L_01 = 1;
	f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);

	// CAN Standby 	PD14 //0=Normal, 1=Standby
	GPIO_SetMode(PD, BIT14,  GPIO_PMD_OUTPUT); 	// CAN Standby
	PD14 = 0;

	// Init (Timer0) // PID Control Operation Loop
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 5);
	TIMER_EnableInt(TIMER0);
	NVIC_EnableIRQ(TMR0_IRQn);
	TIMER_Start(TIMER0);

	// Init (UART0) // BT Control
	UART_Open(UART0, 9600);
	UART_EnableInt(UART0, (UART_IER_RDA_IEN_Msk));
	NVIC_EnableIRQ(UART02_IRQn);

	// Init (UART1) // BT Debug
	UART_Open(UART1, 9600);
	UART_EnableInt(UART1, (UART_IER_RDA_IEN_Msk));
	NVIC_EnableIRQ(UART1_IRQn);

	// !!!! Init (UART4) //CMD echo
	UART_Open(UART4, 9600);
	UART_EnableInt(UART4, (UART_IER_RDA_IEN_Msk));
	NVIC_EnableIRQ(UART4_IRQn);
}

int main(void)
{
	unsigned int m_User_Action_CMD_00;

	SYS_UnlockReg();
	f_SYS_Init();
	SYS_LockReg();

	f_Startup_Init();

	m_User_Action_CMD_00 == '0';
	g_Robot_Action_CMD_01=0;
	//f_PID_Control_Init(1, 0, 0.01, 70, 1, 0, 0.01, 70, 888, 999);
	f_PID_Control_Init(0.5, 0, 0.05, 70, 0.5, 0, 0.05, 70, 888, 999);

    while(1)
    {
    	m_User_Action_CMD_00 = g_UART_00_RX_Result_Byte;

    	if(m_User_Action_CMD_00 == '0') // stop
    	{
    		m_User_Action_CMD_00  = 'z';
    		g_Robot_Action_CMD_01 = 0;

    		g_PID_Parameter_R_S = 0;
    		g_PID_Parameter_L_S = 0;
    		g_Motor_Control_Direction_Value_R_01 = 1;
    		g_Motor_Control_Direction_Value_L_01 = 1;

    		//g_Motor_Control_PWM_DutyCycle_R_01   = 0;
    		//g_Motor_Control_PWM_DutyCycle_L_01   = 0;
    		//f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
    		//f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);
    	}
    	if (m_User_Action_CMD_00 == 'f') // forward
    	{
    		m_User_Action_CMD_00  = 'z';
    		g_Robot_Action_CMD_01 = 1;

    		g_PID_Parameter_R_S = 50;
    		g_PID_Parameter_L_S = 50;
    		g_Motor_Control_Direction_Value_R_01 = 1;
    		g_Motor_Control_Direction_Value_L_01 = 1;

    		//g_Motor_Control_PWM_DutyCycle_R_01   = 90;
    		//g_Motor_Control_PWM_DutyCycle_L_01   = 90;
    		//f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
    		//f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);
    	}
    	if (m_User_Action_CMD_00 == 'b') // backward
    	{
    		m_User_Action_CMD_00  = 'z';
    		g_Robot_Action_CMD_01 = 2;

    		g_PID_Parameter_R_S = 50;
    		g_PID_Parameter_L_S = 50;
    		g_Motor_Control_Direction_Value_R_01 = 2;
    		g_Motor_Control_Direction_Value_L_01 = 2;

    		//g_Motor_Control_PWM_DutyCycle_R_01   = 40;
    		//g_Motor_Control_PWM_DutyCycle_L_01   = 40;
    		//f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
    		//f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);
    	}
    	if (m_User_Action_CMD_00 == 'c') // clockwise
    	{
    		m_User_Action_CMD_00  = 'z';
    		g_Robot_Action_CMD_01 = 3;

    		g_PID_Parameter_R_S = 20;
    		g_PID_Parameter_L_S = 20;
    		g_Motor_Control_Direction_Value_R_01 = 2;
    		g_Motor_Control_Direction_Value_L_01 = 1;

    		//g_Motor_Control_PWM_DutyCycle_R_01   = 60;
    		//g_Motor_Control_PWM_DutyCycle_L_01   = 80;
    		//f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
    		//f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);
    	}
    	if (m_User_Action_CMD_00 == 'a') // anticlockwise
    	{
    		m_User_Action_CMD_00  = 'z';
    		g_Robot_Action_CMD_01 = 4;

    		g_PID_Parameter_R_S = 20;
    		g_PID_Parameter_L_S = 20;
    	    g_Motor_Control_Direction_Value_R_01 = 1;
    	    g_Motor_Control_Direction_Value_L_01 = 2;

    	    //g_Motor_Control_PWM_DutyCycle_R_01   = 80;
    		//g_Motor_Control_PWM_DutyCycle_L_01   = 60;
    		//f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
    	    //f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);
    	}
    	/*else if (m_User_Action_CMD_00 == 'o') // center rotate
    	{
    		m_User_Action_CMD_00  = 'z';
    		g_Robot_Action_CMD_01 = 5;

    		g_PID_Parameter_R_S = 30;
    		g_PID_Parameter_L_S = 30;
    		g_Motor_Control_Direction_Value_R_01 = 1;
    		g_Motor_Control_Direction_Value_L_01 = 2;

    		//g_Motor_Control_PWM_DutyCycle_R_01   = 70;
    		//g_Motor_Control_PWM_DutyCycle_L_01   = 70;
    		//f_Motor_Control(1, g_Motor_Control_Direction_Value_R_01, g_Motor_Control_PWM_DutyCycle_R_01, g_Motor_Control_PWM_Frequency);
    	    //f_Motor_Control(2, g_Motor_Control_Direction_Value_L_01, g_Motor_Control_PWM_DutyCycle_L_01, g_Motor_Control_PWM_Frequency);
    	}*/

    }

    f_SYS_Exit();

    return 0;
}
