/*
 * I2C.c
 *
 *  Created on: Oct 14, 2016
 *      Author: user
 */


#include "Cpu.h"
#include "I2C.h"

extern unsigned int j;
unsigned char i = 0;

void MPU_INIT(void){
	// Rutina para inicializar el MPU9250
	I2C_SELECT_SLAVE(MPU9250_SLAVE_ADDRESS);
	for(j = 0; j < 20000; j++){
		
	}
	(void)I2C_WRITE_REGISTER(MPU9250_PWR_MGMT_1,0x00);  // Evitar el sleep mode
	(void)I2C_WRITE_REGISTER(MPU9250_SMPRT_DIV,0x00);   // Dejamos la tasa de muestreo en 8kHz la cual será truncada por el DLPF
	(void)I2C_WRITE_REGISTER(MPU9250_CONFIG,0x04);	    // Digital Low Pass Filter --> 20Hz
	(void)I2C_WRITE_REGISTER(MPU9250_GYRO_CONFIG,0x08); // +/- 500 dps, no self test
	(void)I2C_WRITE_REGISTER(MPU9250_ACCEL_CONFIG,0x00); // +/- 2 g, no self test
	(void)I2C_WRITE_REGISTER(MPU9250_INT_PIN_CFG,0x02); // Saltar el I2C auxiliar para acceder directamente el acelerómetro interno
}

byte I2C_READ(byte addressReg){
	// Rutina para recibir un byte del dispositivo esclavo
	byte data = 0;
	byte error = 0;
	I2C_SELECT_SLAVE(MPU9250_SLAVE_ADDRESS);
	(void)CI2C1_SendChar(addressReg); // enviar direccion que se desea leer
	while(i==0){/*waiting for transmission*/}
	i=0;
	(void)CI2C1_RecvChar(&data); // recibir valor almacenado en addressReg
	while(i==0){/*waiting for transmission*/}
	i=0;
	return data;
}

byte I2C_WRITE_REGISTER(byte addressReg,byte data){
	// Rutina para mandar dos datos, la dirección de registro y el contenido del mismo.
	word snd=0;
	word size=2;
	byte error;
	byte frame[2];
	frame[0]=addressReg;
	frame[1]=data;
	error=CI2C1_SendBlock(frame,size,&snd);
	while(i==0){/*waiting for transmission*/}
	i=0;
	return error; // Retornar código de error definido por processor expert
}

void I2C_SELECT_SLAVE(byte slaveAddress){
	// Rutina para seleccionar satisfactoriamente un nuevo esclavo I2C.
	byte error = 0;
	do{
		error = CI2C1_SelectSlave(slaveAddress);
	} while(error != ERR_OK);
}
