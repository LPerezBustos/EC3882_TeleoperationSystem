#ifndef KALMAN_H_
#define KALMAN_H_

#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"

#define dt 		0.035
#define PI  	3.141592
#define ROLL	0
#define PITCH	1

//Aproximaciones polinomicas de funciones trigonometricas
float ATAN(float y);
float ATAN2(float num,float den);
float ASIN(float y);
float COS(float y);

//Estimacion de estado
int16_t concat(int8_t a_1, int8_t a_2);
void Adjust_Noise_Matrix(float R[][2],float Q[][2],float factorQ,float factorR);
#endif /* KALMAN_H_ */
