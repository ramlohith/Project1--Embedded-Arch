/*----------------------------------------------------------------------------
 *----------------------------------------------------------------------------*/
#include <MKL25Z4.H>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "gpio_defs.h"
#include "LEDs.h"
#include "i2c.h"
#include "mma8451.h"
#include "delay.h"
#include "RTCS.h"
#define FLASH_DELAY 2
#define ACC_SENSITIVITY 90
#define TASK_MOTION_SENSOR_FREQ_HZ (50) 

int mode,returnval;
void Init_Debug_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;

	PORTB->PCR[DBG_0] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_1] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_2] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_3] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_4] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_5] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_6] |= PORT_PCR_MUX(1);          
	PORTB->PCR[DBG_7] |= PORT_PCR_MUX(1);          

	PTB->PDDR |= MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);
	PTB->PCOR = MASK(DBG_0) | MASK(DBG_1) | MASK(DBG_2) | MASK(DBG_3) | MASK(DBG_4) | MASK(DBG_5) | MASK(DBG_6) | MASK(DBG_7);

}


void Init_Config_Signals(void) {
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

	PORTE->PCR[CONFIG1_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG1_POS);

	PORTE->PCR[CONFIG2_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG2_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG2_POS);

	PORTE->PCR[CONFIG3_POS] &= ~PORT_PCR_MUX_MASK;          
	PORTE->PCR[CONFIG3_POS] |= PORT_PCR_MUX(1);          
	PTE->PDDR &= ~MASK(CONFIG3_POS);
	
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_PE(1);
	PORTE->PCR[CONFIG1_POS] |= PORT_PCR_PS(1);
}
void Task_I2C_Server()
{
	enum States{StIDL,StRDev,StRReg,StWR,StWait,StRST,StRec,StStop,StRdComplete,StWRPOST,StWRReg};
	static enum States next_state_server;
	static int lock_detect = 0;
	static enum States next_state_post_wait;
	static uint8_t num_bytes_written;
	static uint8_t dummy, num_bytes_read=0, is_last_read=0;
	
	switch(next_state_server)
	{
		case StIDL:
			SET_BIT(DEBUG_I2C_CODE);
			if (g_I2C_Msg.Command == READ)
			{next_state_server = StRDev;}
			else if (g_I2C_Msg.Command == WRITE)
			{next_state_server = StWR;}
			CLEAR_BIT(DEBUG_I2C_CODE);
		break;
		case StRDev:
		if (g_I2C_Msg.Command == READ)
			{
				SET_BIT(DEBUG_I2C_CODE);
				g_I2C_Msg.Status = READING;
				I2C_TRAN;		//	set to transmit mode
				SET_BIT(DEBUG_MSG_ON_BUS);								
				I2C_M_START;											//	send start		
				I2C0->D = g_I2C_Msg.Dev_adx;								//	send dev address (write)		
				next_state_server = StWait;
				next_state_post_wait = StRReg;	
				CLEAR_BIT(DEBUG_I2C_CODE);
			}
			break;
		case StWait:		
				SET_BIT(DEBUG_I2C_BUSY_WAIT);
				lock_detect = 0;
				if( (( I2C0->S & I2C_S_IICIF_MASK) == 0) ) {
					lock_detect++;
				next_state_server = StWait;
				}
				else{
					lock_detect = 0;
				I2C0->S |= I2C_S_IICIF_MASK;
				next_state_server = next_state_post_wait;
				}
				CLEAR_BIT(DEBUG_I2C_BUSY_WAIT);				//	wait for completion								
		break;
		case StRReg:
				SET_BIT(DEBUG_I2C_CODE);
				I2C0->D = g_I2C_Msg.Reg_adx;
				next_state_server = StWait;//	send register address									
				next_state_post_wait = StRST;
				CLEAR_BIT(DEBUG_I2C_CODE);
		break;
		case StRST:
				SET_BIT(DEBUG_I2C_CODE);
				I2C_M_RSTART;											//	repeated start									
				I2C0->D = g_I2C_Msg.Dev_adx | 0x01 ;				//	send dev address (read)						
					next_state_server = StWait;
					next_state_post_wait = StRec;
				CLEAR_BIT(DEBUG_I2C_CODE);
		break;							
		case StRec:
				SET_BIT(DEBUG_I2C_CODE);																	
				if (num_bytes_read < g_I2C_Msg.Data_count) {
					I2C_REC;  									//	set to receive mode	
					is_last_read = (num_bytes_read == g_I2C_Msg.Data_count-1)? 1: 0;
					if (is_last_read){
					NACK;													// tell HW to send NACK after read							
					} else {
					ACK;													// tell HW to send ACK after read								
					}

				dummy = I2C0->D;								//	dummy read
				next_state_server = StWait;	
				next_state_post_wait = StStop;				
				}
				else{next_state_server = StRdComplete; num_bytes_read = 0;}
				CLEAR_BIT(DEBUG_I2C_CODE);
		break;
		case StStop:
				SET_BIT(DEBUG_I2C_CODE);
				if (is_last_read){
					I2C_M_STOP;	
					CLEAR_BIT(DEBUG_MSG_ON_BUS);			//	send stop										
				}
				I2C_TRAN;
				g_I2C_Msg.Data[num_bytes_read++] = I2C0->D;
				next_state_server = StRec; 
				next_state_post_wait = StStop;
				CLEAR_BIT(DEBUG_I2C_CODE);
		break; 
		case StRdComplete:
				SET_BIT(DEBUG_I2C_CODE);
				g_I2C_Msg.Status = READ_COMPLETE;
				g_I2C_Msg.Command = NONE;
				next_state_server = StIDL;
				CLEAR_BIT(DEBUG_I2C_CODE);
		break;
		case StWR:
			if (g_I2C_Msg.Command == WRITE)
			{
			SET_BIT(DEBUG_I2C_CODE);
			SET_BIT(DEBUG_WRITE_BIT);
			g_I2C_Msg.Status = WRITING;
			I2C_TRAN;				
			SET_BIT(DEBUG_MSG_ON_BUS);	
			I2C_M_START;			//	send start										
			I2C0->D = g_I2C_Msg.Dev_adx;								//	send dev address (write)	
			next_state_server = StWait;
			next_state_post_wait = StWRReg; 
			}					
			CLEAR_BIT(DEBUG_I2C_CODE);
		break;
		case StWRReg:
			if(g_I2C_Msg.Status == WRITING)
			{
			SET_BIT(DEBUG_I2C_CODE);
			I2C0->D = g_I2C_Msg.Reg_adx;								//	send register address									
			next_state_server = StWait;
			next_state_post_wait = StWRPOST;
			CLEAR_BIT(DEBUG_I2C_CODE);
			}
			break;
		case StWRPOST:
			if(g_I2C_Msg.Status == WRITING)
			{
			SET_BIT(DEBUG_I2C_CODE);
			if (num_bytes_written < g_I2C_Msg.Data_count) {
				I2C0->D = g_I2C_Msg.Data[num_bytes_written++]; //	write data										
				next_state_server = StWait;
				next_state_post_wait = StWRPOST;//	wait for completion								
			}
			else{
				I2C_M_STOP;												//		send stop										
				CLEAR_BIT(DEBUG_MSG_ON_BUS);
				g_I2C_Msg.Status = WRITE_COMPLETE;
				g_I2C_Msg.Command = NONE;
				CLEAR_BIT(DEBUG_I2C_CODE);
				next_state_server = StIDL;
				}
			CLEAR_BIT(DEBUG_WRITE_BIT);
			CLEAR_BIT(DEBUG_I2C_CODE);
			}
		break;
		default: next_state_server = StIDL;
			 g_I2C_Msg.Status = IDLE;
			 g_I2C_Msg.Command = NONE;
		break;
}
}
	
void Task_Motion_Sensor(void) {
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	uint8_t data[1];
	static uint8_t dataxyz[6];
	int i;
	int16_t temp[3];
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
  static	uint8_t rf, gf, bf;
	static int mma_faulty = 0;
	static enum {INITMMARD,INITMMAERR,INITMMAWR,XYZRD,SETXYZ,DISPLAY} next_state = INITMMARD;

	switch(next_state)
	{
		case INITMMARD:
			if((g_I2C_Msg.Status == IDLE) & (g_I2C_Msg.Command == NONE))
			{
				SET_BIT(DEBUG_ACC);
				Control_RGB_LEDs(1, 1, 0);
			SET_BIT(DEBUG_TASK_MOTION_SENSOR);
			g_I2C_Msg.Command = READ;
			g_I2C_Msg.Dev_adx = MMA_ADDR;
			g_I2C_Msg.Reg_adx = REG_WHOAMI;
			g_I2C_Msg.Data_count = 1;
			next_state = INITMMAERR;
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
			}
		break;
		case INITMMAERR:
			if((g_I2C_Msg.Status == IDLE) & (g_I2C_Msg.Command == NONE))
			{
			SET_BIT(DEBUG_TASK_MOTION_SENSOR);
	  	if (g_I2C_Msg.Data[0] != WHOAMI)	{
					mma_faulty = 1; 
					Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
		while (1)																/* not able to initialize mma */
			;															// error code after initial read bit in init mma
			}
		else
				next_state = INITMMAWR;
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
			}
			
		break;
		case INITMMAWR:
	//set active mode, 14 bit samples, 2g full scale, low noise and 800 Hz ODR 
		if((g_I2C_Msg.Status == IDLE)  & (g_I2C_Msg.Command == NONE))
		{
		
			SET_BIT(DEBUG_TASK_MOTION_SENSOR);
			g_I2C_Msg.Dev_adx = MMA_ADDR;
			g_I2C_Msg.Reg_adx = REG_CTRL1;
			g_I2C_Msg.Data[0] = 0x05;
			g_I2C_Msg.Data_count = 1;
			g_I2C_Msg.Command = WRITE;
			next_state = XYZRD;
			Control_RGB_LEDs(0, 0, 0);
			CLEAR_BIT(DEBUG_ACC);
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); 
		}
		break;
		case XYZRD:
			if((g_I2C_Msg.Status == IDLE) & (g_I2C_Msg.Command == NONE))
			{
			SET_BIT(DEBUG_TASK_MOTION_SENSOR); 
			g_I2C_Msg.Dev_adx = MMA_ADDR;
			g_I2C_Msg.Reg_adx = REG_XHI;
			g_I2C_Msg.Data_count = 6;
			g_I2C_Msg.Command = READ;
			next_state = SETXYZ;
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR); 
			}
			break;
		case SETXYZ:
			if((g_I2C_Msg.Status == IDLE) & (g_I2C_Msg.Command == NONE))
			{
			SET_BIT(DEBUG_TASK_MOTION_SENSOR);
			for ( i=0; i<3; i++ ) {
			temp[i] = (int16_t) ((g_I2C_Msg.Data[2*i]<<8) | g_I2C_Msg.Data[2*i+1]);
			}
	// Align for 14 bits
			acc_X = temp[0]/4;
			acc_Y = temp[1]/4;
			acc_Z = temp[2]/4;

			rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
			gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
			bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;
			
			prev_acc_X = acc_X;
			prev_acc_Y = acc_Y;
			prev_acc_Z = acc_Z;
			
			next_state = DISPLAY;
			g_I2C_Msg.Status = IDLE;
			g_I2C_Msg.Command = NONE;
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
		//	returnval = RTCS_Set_Task_Period(g_I2C_Msg.Client,TICKS(TASK_MOTION_SENSOR_FREQ_HZ) * 2 , 0);
		}
		break;	
		case DISPLAY:
			if((g_I2C_Msg.Status == IDLE) & (g_I2C_Msg.Command == NONE))
			{
			SET_BIT(DEBUG_TASK_MOTION_SENSOR);
			Control_RGB_LEDs(rf, gf, bf);
			//	Control_RGB_LEDs(1,0,0);
			Delay(FLASH_DELAY);
			//	returnval = RTCS_Set_Task_Period(g_I2C_Msg.Client,TICKS(TASK_MOTION_SENSOR_FREQ_HZ) * 4 , 0);
			Control_RGB_LEDs(0, 0, 0);							
			Delay(FLASH_DELAY*2);		
			next_state = XYZRD;
			CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
			//returnval = RTCS_Set_Task_Period(g_I2C_Msg.Client,TICKS(TASK_MOTION_SENSOR_FREQ_HZ) , 0);				
			}
			break;
		default: next_state = XYZRD;
		break;
	}
}
void Task_Motion_Sensor_base(void) {
	static int16_t prev_acc_X=0, prev_acc_Y=0, prev_acc_Z=0;
	int16_t acc_X=0, acc_Y=0, acc_Z=0;
	uint8_t rf, gf, bf;
	SET_BIT(DEBUG_TASK_MOTION_SENSOR);
	read_full_xyz(&acc_X, &acc_Y, &acc_Z);

	rf = abs(prev_acc_X - acc_X) > ACC_SENSITIVITY ? 1 : 0;
	gf = abs(prev_acc_Y - acc_Y) > ACC_SENSITIVITY ? 1 : 0;
	bf = abs(prev_acc_Z - acc_Z) > ACC_SENSITIVITY ? 1 : 0;

	Control_RGB_LEDs(rf, gf, bf);
	Delay(FLASH_DELAY);
	Control_RGB_LEDs(0, 0, 0);							
	Delay(FLASH_DELAY*2);		

	prev_acc_X = acc_X;
	prev_acc_Y = acc_Y;
	prev_acc_Z = acc_Z;
	CLEAR_BIT(DEBUG_TASK_MOTION_SENSOR);
}
	/*----------------------------------------------------------------------------
  MAIN function
 *----------------------------------------------------------------------------*/
int main (void) {
	Init_RGB_LEDs();
	Init_Debug_Signals();
	Init_Config_Signals();
	i2c_init();																/* init i2c	*/
	Delay(200);
		if(PTE->PDIR & MASK(CONFIG1_POS))
			mode = 1;
		else
			mode = 0;
	if(mode == 1)
	{
	Control_RGB_LEDs(1, 1, 0);								/* yellow: starting up */
	if (!init_mma()) {												/* init mma peripheral */
		Control_RGB_LEDs(1, 0, 0);							/* Light red error LED */
  	while (1)																/* not able to initialize mma */
		;
	}  
	Control_RGB_LEDs(0, 0, 0);						
	RTCS_Init(SCHED_FREQ_HZ);
	RTCS_Add_Task(Task_Motion_Sensor_base, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	//RTCS_Add_Task(Task_I2C_Server, 1, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	RTCS_Run_Scheduler();
	//RTCS_Run_Scheduler_EventTrigger();
}
else
{
	RTCS_Init(SCHED_FREQ_HZ);
	RTCS_Add_Task(Task_Motion_Sensor, 0, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	RTCS_Add_Task(Task_I2C_Server, 1, TICKS(TASK_MOTION_SENSOR_FREQ_HZ)); // Run periodically
	RTCS_Run_Scheduler_EventTrigger();
//	RTCS_Run_Scheduler_FSM();
}
}
