/*
 * I2C.h
 *
 *  Created on: Oct 14, 2016
 *      Author: user
 */

#ifndef I2C_H_
#define I2C_H_

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "CI2C1.h"

//direcciones del MPU6050 y MPU9250    
#define MPU9250_SLAVE_ADDRESS		0x68
#define MPU9250_GYRO_XOUT_H         0x43     
#define MPU9250_GYRO_XOUT_L         0x44     
#define MPU9250_GYRO_YOUT_H         0x45     
#define MPU9250_GYRO_YOUT_L         0x46     
#define MPU9250_GYRO_ZOUT_H         0x47     
#define MPU9250_GYRO_ZOUT_L         0x48
#define MPU9250_ACCL_XOUT_H         0x3B     
#define MPU9250_ACCL_XOUT_L         0x3C     
#define MPU9250_ACCL_YOUT_H         0x3D     
#define MPU9250_ACCL_YOUT_L         0x3E     
#define MPU9250_ACCL_ZOUT_H         0x3F     
#define MPU9250_ACCL_ZOUT_L         0x40
#define MPU9250_USER_CTRL           0x6A
#define MPU9250_PWR_MGMT_1          0x6B
#define MPU9250_PWR_MGMT_2          0x6C
#define MPU9250_GYRO_CONFIG			0x1B
#define MPU9250_ACCEL_CONFIG        0x1C  
#define MPU9250_SMPRT_DIV			0x19
#define MPU9250_CONFIG 				0x1A
#define MPU9250_INT_PIN_CFG	    	0x37
#define MPU6050_WHO_AM_I			0x75

//Encabezados de funciones
void MPU_INIT(void);

byte I2C_READ(byte addressReg);
byte I2C_WRITE_REGISTER(byte addressReg,byte data);
void I2C_SELECT_SLAVE(byte slaveAddress);

#endif /* I2C_H_ */
