/* ###################################################################
**     Filename    : main.c
**     Project     : P2-1210764-1010545
**     Processor   : MC9S08QE128CLK
**     Version     : Driver 01.12
**     Compiler    : CodeWarrior HCS08 C Compiler
**     Date/Time   : 2016-09-28, 14:30, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.12
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "AS1.h"
#include "TI1.h"
#include "CI2C1.h"
#include "Bit1.h"
#include "Cap1.h"
#include "PWM1.h"
/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "string.h"
// Nuestros modulos
#include "I2C.h"
#include "kalman.h"

/* User includes (#include below this line is not maintained by Processor Expert) */

// velocidades angulares leidas
static uint8_t w_x1 = 0x01, w_x2 = 0x02, w_y1 = 0x03, w_y2 = 0x04, w_z1 = 0x05, w_z2 = 0x06;
// Aceleraciones lineales leidas
static uint8_t a_x1 = 0x01, a_x2 = 0x02, a_y1 = 0x03, a_y2 = 0x04, a_z1 = 0x05, a_z2 = 0x06;
int8_t * b;

// variables de Roll y Pitch dadas
static int16_t a_x = 0x0000, a_y = 0x0000, a_z = 0x0000, w_x = 0x0000, w_y = 0x0000, w_z = 0x0000;
static float roll = 0.0, pitch = 0.0, gyr_r = 0.0, gyr_p = 0.0;
static float ax = 0.0, ay = 0.0, az = 0.0;
static float wx = 0.0, wy = 0.0, wz = 0.0;
static float den = 0.0;
static float aux = 0.0;				

// Matrices para el filtro de kalman
static float Qr[2][2] = {0.1919,5.7436,5.7436,3025.559};		// 
static float Qp[2][2] = {0.0577,2.0515,2.0515,285.071};			//  -> Matrices de covarianzas de ruido de planta
static float Rr[2][2] = {0.0,0.0,0.0,0.0013};				//
static float Rp[2][2] = {0.0,-0.0001,-0.0001,0.2733};		    //  -> Matrices de covarianzas de ruido de medicion
static float Pr[2][2] = {0,0,0,0};							//  
static float Pp[2][2] = {0,0,0,0};							//  -> Matrices de covarianzas de error de estimacion
static float xr[2] = {0,0}; 								//
static float xp[2] = {0,0};								    //  -> Vectores de estados estimados

// Configuracion del sensor por ultrasonido --> HC-SR04
unsigned int TRIGGER = 10; //pulso de 10us para iniciar sensor
unsigned int time = 0;
unsigned int distance = 0;

//Variables de la Máquina de estados
unsigned char estado = 0;
unsigned char j = 0;

// Configuracion WiFi
unsigned char socket[] = "AT+CIPSTART=\"UDP\",\"192.168.4.2\",1024,1024,0\r\n";
extern unsigned char i;
unsigned char enviar_2[] = "AT+CIPSEND=2\r\n";
unsigned char enviar_4[] = "AT+CIPSEND=4\r\n";
unsigned char EOL[] = "\r\n"; //End of Line --> Necesario luego de cada comando

// Otras variables auxiliares
word snd=0;
word size;

void Kalman_Filter(float data1,float data2,float *xplus,float pplus[2][2],float Q[2][2],float R[2][2]){
	//inicializa las matrices
	float Z[2];
	float pminus[2][2];
	float xminus[2];
	float K[2][2];
	float determinante = 1.0;
	
	//Lee los datos de referencia
	Z[0] = data1;
	Z[1] = data2;

	// 1.Pk(-)=phi*Pk-1(+)*phi'+Q;
	// estima la covarianza
	pminus[0][0] = (float)(Q[0][0] + pplus[0][0] + dt*(pplus[1][0] + pplus[0][1] + dt*pplus[1][1])); 
	pminus[0][1] = (float)(Q[0][1] + pplus[0][1] + dt*pplus[1][1]);
	pminus[1][0] = (float)(Q[1][0] + pplus[1][0] + dt*pplus[1][1]);
	pminus[1][1] = Q[1][1] + pplus[1][1];
	
	// 2.Kk= Pk(-)Hk^t[HkPk(-)Hk^t+Rk]^-1
	// calcula ganancia de kalman
	determinante = ((pminus[0][0] + R[0][0])*(pminus[1][1] + R[1][1]) - (pminus[0][1] + R[0][1])*(pminus[1][0] + R[1][0]));
	K[0][0] = (pminus[0][0]*(pminus[1][1] + R[1][1]) - pminus[0][1]*(pminus[1][0] + R[1][0]))/determinante;
	K[0][1] = (pminus[0][1]*R[0][0] - pminus[0][0]*R[0][1])/determinante;
	K[1][0] = (pminus[1][0]*R[1][1] - pminus[1][1]*R[1][0])/determinante;
	K[1][1] = (pminus[1][1]*(R[0][0] + pminus[0][0]) - pminus[1][0]*(R[0][1] + pminus[0][1]))/determinante;

	// 3.Pk(+)= (1-Kk*H)Pk(-)
	// actualiza covarianza
	pplus[0][0] = pminus[0][0]*(1 - K[0][0]) - K[0][1]*pminus[1][0];
	pplus[0][1] = pminus[0][1]*(1 - K[0][0]) - K[0][1]*pminus[1][1];
	pplus[1][0] = pminus[1][0]*(1 - K[1][1]) - K[1][0]*pminus[0][0];
	pplus[1][1] = pminus[1][1]*(1 - K[1][1]) - K[1][0]*pminus[0][1];
	
	// 4.Xk(-)= phi*Xk-1(+)
	// actualiza medida
	xminus[0] = (float)(xplus[0] + dt*xplus[1]);
	xminus[1] = xplus[1];
	
	// 5.Xk(+)=Xk(-)+Kk*(Zk-Hk*Xk(-))
	// obtiene valor estimado
	xplus[0] = xminus[0] - K[0][0]*(xminus[0] - Z[0]) - K[0][1]*(xminus[1] - Z[1]);
	xplus[1] = xminus[1] - K[1][0]*(xminus[0] - Z[0]) - K[1][1]*(xminus[1] - Z[1]);
};

void main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example:*/ 
  MPU_INIT();
  size = strlen(socket); //obtiene tamaño del bloque a enviar (en Bytes)
  (void)AS1_SendBlock(&socket,size,&snd); // Enviar configuracion de conexion al ESP8266
  while(i==0){/*waiting for transmission*/} // esperando que se vacie el Buffer de salida
  Cpu_Delay100US(5); // Espera de 500us para transmision WiFi
  snd = 0;
  i=0;
  Adjust_Noise_Matrix(Rr,Qr,20.0,20.0); // Estimacion de Matrices de Ruido
  Adjust_Noise_Matrix(Rp,Qp,20.0,20.0);
  for(;;) {

	  	/*
	  	 *	CICLO DE LA MAQUINA DE ESTADOS
	  	 *  (0)	DO NOTHING
	  	 *  (1) LEE ULTRASONIDO
	  	 *	(2) LEE ACELEROMETRO
	  	 *	(3) LEER GYROSCOPO
	  	 *	(4) TRADUCE A SISTEMA MÉTRICO
	  	 *	(5) ENVIAR A ESP8266
	  	 */
		  
		  switch(estado){
		  	  case ESTADO0:
		  		j=0;  
		  		// Hello World !
		  		break;
		  	  case LEER_ULTRASONIDO: 
		  		(void)PWM1_SetDutyUS(TRIGGER); // Trigger de 10us		  		  
		  		(void)PWM1_Enable(); // Enviar Trigger a traves de PWM
		  		(void)Cap1_Reset(); // Reinicia valor del contador
		  		(void)Cap1_Enable(); // Activa contador
		  		estado = LEER_ACEL; // Se ejecutan nuevos procesos mientras se espera por señal de 'echo'
		  		break; 
		  	  case LEER_ACEL:
		  		 a_x1 = I2C_READ(MPU9250_ACCL_XOUT_H); // Leer Acel_X_MSB
		  		 a_x2 = I2C_READ(MPU9250_ACCL_XOUT_L); // Leer Acel_X_LSB
  
		  		 a_y1  = I2C_READ(MPU9250_ACCL_YOUT_H); // Leer Acel_Y_MSB
		  		 a_y2 = I2C_READ(MPU9250_ACCL_YOUT_L); // Leer Acel_Y_LSB
  
		  		 a_z1  = I2C_READ(MPU9250_ACCL_ZOUT_H); // Leer Acel_Z_MSB
		  		 a_z2 = I2C_READ(MPU9250_ACCL_ZOUT_L); // Leer Acel_Z_LSB	  		  
		  		 estado = LEER_W;
		  		break;
		  	  case LEER_W:
		  		//lee Wx
		  		w_x1  = I2C_READ(MPU9250_GYRO_XOUT_H);
		  	  	w_x2 = I2C_READ(MPU9250_GYRO_XOUT_L);
		  	  	//lee Wy	  
		  	  	w_y1  = I2C_READ(MPU9250_GYRO_YOUT_H);
		  	  	w_y2 = I2C_READ(MPU9250_GYRO_YOUT_L);
		  	  	//lee Wz	  
		  	  	w_z1  = I2C_READ(MPU9250_GYRO_ZOUT_H);
		  	  	w_z2 = I2C_READ(MPU9250_GYRO_ZOUT_L);
		  	  	
		  		estado = MET_SYSTEM;
		  		break;
		  	  case MET_SYSTEM:
		  	    // Concatenacion y ajustes de offset
		  		a_x = concat(a_x1, a_x2) - 1336 + 600 + 1276;
		  		a_y = concat(a_y1, a_y2) + 468 - 1098;
		  		a_z = concat(a_z1, a_z2) - 4000 + 100;
		  		
		  		w_x = concat(w_x1, w_x2) + 120 - 300;
		  		w_y = concat(w_y1, w_y2) - 40 + 300 - 290;
		  		w_z = concat(w_z1, w_z2) - 20;
		  		
		  		// conversion a float y ajuste de rango
		  		ax = (float)(a_x)/16384;
		  		ay = (float)(a_y)/16384;
		  		az = (float)(a_z)/16384;

		  		wx = (float) (w_x)*125/8192;
		  		wy = (float) (w_y)*125/8192;
		  		wz = (float) (w_z)*125/8192;
		  		
		  		// Rangos especificos
		  		if(az <= 0.01){
		  			if(ay < 0){
		  				roll = -PI/2;
		  			} 
		 			else{
		  				roll = PI/2;
		  			}
		  		} 
		  		else{
		  			roll = ATAN(ay/az) + 0.05 - 0.0586; // Calcular Roll y ajuste de offset
		  		}
		  		
		  		pitch = ASIN(-ax) + 0.018 - 0.0609; // Calcular Pitch y ajuste de offset
		  		den = COS(pitch);
		  		
		  		// Tasa de Roll y Tasa de Pitch	
		  		gyr_p = (1/den)*(az*wy - ay*wz); // dp/dt
		  		gyr_p -= -0.01; // Correccion de offset
		  			
		  		den = (1-ax*ax);
		  		
		  		gyr_r = wx - (ax/den)*(ay*wy + az*wz); // dr/dt
		  		gyr_r -= -0.07 + 0.1; // Correccion de offset
		  		
		  		Kalman_Filter(roll,gyr_r,xr,Pr,Qr,Rr); // Estimacion de estados a traves de Filtro de Kalman
		  		Kalman_Filter(pitch,gyr_p,xp,Pp,Qp,Rp);
		  		
		  		// Asignar estimacion de estados a nuevas variables
		  		roll  = xr[0];
		  		gyr_r = xr[1];
		  		pitch = xp[0];
		  		gyr_p = xp[1];
		  		
		  		estado = ENVIAR_ESP8266;
		  		break;
		  	  case ENVIAR_ESP8266:
			  	size = strlen(enviar_2); // Calcula tamaño (En Bytes) del bloque a enviar
		  		(void)AS1_SendBlock(&enviar_2,size,&snd); // AT+CIPSEND=2
		  		while(i==0){/*waiting for transmission*/}
		  		Cpu_Delay100US(3);
		  		snd = 0;
		  		i=0;
		  		(void)AS1_SendBlock(&distance,2,&snd); // enviar lectura del sensor por ultrasonido
		  		(void)AS1_SendBlock(&EOL,4,&snd);
		  		while(i==0){/*waiting for transmission*/}
		  		Cpu_Delay100US(10);
		  		snd = 0;
		  		i=0;		  		
			  	
			  	b = (byte *) &roll; // enviar roll
			  	size = strlen(enviar_4);
			  	(void)AS1_SendBlock(&enviar_4,size,&snd); // AT+CIPSEND=4
			  	while(i==0){/*waiting for transmission*/}
			  	Cpu_Delay100US(3);
			  	snd = 0;
			  	i=0;		  		
			  	(void)AS1_SendChar(b[0]); //Se envian los 4 Bytes de forma individual
			  	(void)AS1_SendChar(b[1]);
			  	(void)AS1_SendChar(b[2]);
			  	(void)AS1_SendChar(b[3]);
			  	(void)AS1_SendBlock(&EOL,4,&snd);
			  	while(i==0){/*waiting for transmission*/}
			  	Cpu_Delay100US(10);
			  	snd = 0;
			  	i=0;
			  	
			  	b = (byte *) &pitch; // enviar pitch
			  	size = strlen(enviar_4);
			  	(void)AS1_SendBlock(&enviar_4,size,&snd); // AT+CIPSEND=4
			  	while(i==0){/*waiting for transmission*/}
			  	Cpu_Delay100US(3);
			  	snd = 0;
			  	i=0;		  		
			  	(void)AS1_SendChar(b[0]);
			  	(void)AS1_SendChar(b[1]);
			  	(void)AS1_SendChar(b[2]);
			  	(void)AS1_SendChar(b[3]);
			  	(void)AS1_SendBlock(&EOL,4,&snd);
			  	while(i==0){/*waiting for transmission*/}
			  	Cpu_Delay100US(5);
			  	snd = 0;
			  	i=0;
		  		estado = ESTADO0; // Vuelve al inicio de la maquina de estados
		  	  	break; // El sistema trabaja en torno a los 30Hz (adquiere, procesa y transmite {distance, Roll, Pitch} cada 30ms)
		  	  default :
		  		break;
		  }
  }  
  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale HCS08 series of microcontrollers.
**
** ###################################################################
*/
