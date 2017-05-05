#include "Cpu.h"
#include "kalman.h"

//variables leidas en terminos de RPY y tasas de Euler
extern volatile float roll, pitch, gyr_r, gyr_p;
	
//variables en sistema metrico
extern volatile float ax, ay, az;
extern volatile float wx, wy, wz;
float aux;

//lecturas concatenadas
extern volatile int16_t a_x, a_y, a_z, w_x, w_y, w_z;

//Matrices para filtro de kalman
extern volatile float Qr[2][2] = {9.47,0.17,0.17,25.5};				// 
extern volatile float Qp[2][2] = {11.12,1.57,1.57,13.3};			//  -> Matrices de covarianzas de ruido de planta
extern volatile float Rr[2][2] = {0.014,0.0,0.0,0.02};				//
extern volatile float Rp[2][2] = {0.014,0.0,0.0,0.02};				//  -> Matrices de covarianzas de ruido de medición
extern volatile float Pr[2][2] = {0,0,0,0};							//  
extern volatile float Pp[2][2] = {0,0,0,0};							//  -> Matrices de covarianzas de error de estimación
extern volatile float xr[2] = {0,0}; 								//
extern volatile float xp[2] = {0,0};								//  -> Vectores de estados estimados

float ATAN(float y){
 	//calcula la arcotangente de la variable y.
 	//Rango: [-PI/2,PI/2] 
	float x = y;
	float angle = 0;
    if(x > 1.4 || x < -1.4){
        angle = ATAN(1/x);
        if(x>0){
            angle = (float)(PI/2 - angle);
        }
        else{
            angle = (float)(-PI/2 - angle);
        }
    }
    else{
        if (x >= 0.5){
            angle = (float)(0.015 + x*(1.031 + x*(-0.261)));
        }
        else if (x <= -0.5){
        	angle = (float)(-0.015 + x*(1.031 + x*(0.261)));
        }
        else{
        angle = x*x;
        angle = (float)(x*(1.0 + angle*(-0.333 + angle*(0.2 + angle*(-0.143)))));
        }
    }
    return angle;
}

float ATAN2(float num,float den){
	//calcula la arcotangente 2 de num/den en radianes. 
	//Rango: [-PI,PI]    
	float angle = 0;
    if(den == 0){
    	if(num > 0){
    		return PI/2;
    	}
    	else if(num < 0){
    		return -PI/2;
    	}
    	else{
    		return 0;
    	}
    }
    else if(num > 0.0 && den < 0.0){
			angle = ATAN(num/den) + PI;
		}
	else if(num < 0.0 && den < 0.0){
			angle = ATAN(num/den) - PI;
		}
	else{
			angle = ATAN(num/den);
		}
	return angle;
}

float ASIN(float y){
	//calcua el arcoseno de y. 
	//Rango: [-PI/2,PI/2] 
	float x = y;
    float angle = 0;
	if(x < -1.0){
		x = -1.0;
	}
	else if(x > 1.0){
		x = 1.0;
	}
    if (x >= 0.75){
    	angle = -21.49 + x*(78.79 + x*(-93.97 + x*38.14));
	}
	else if (x <= -0.75){	    
	    angle = 21.49 + x*(78.79 + x*(+93.97 + x*38.14));
	}
	else{
	    angle = x*x;
	    angle = x*(1 + angle*(0.17 + angle*(0.075 + angle*(0.045 + angle*(0.039)))));
	}
	return angle;
}

float COS(float y){
	//Calcula el coseno de y (en radianes). 
	//Rango[0,1]. 
	//Dominio: [-PI/2,PI/2] 
	float x = y;
    double angle = x*x;
    angle = 1 + angle*(-0.5 + angle*(0.042 + angle*(-0.0014)));
    return (float)angle;
}

void Adjust_Noise_Matrix(float R[][2],float Q[][2],float factorQ,float factorR){
	// Función para ajustar arbitrariamente las matrices de covarianza de ruido
	Q[0][0]/=factorQ;
	Q[0][1]/=factorQ;
	Q[1][0]/=factorQ;
	Q[1][1]/=factorQ;
	R[0][0]*=factorR;
	R[0][1]*=factorR;
	R[1][0]*=factorR;
	R[1][1]*=factorR;
}

int16_t concat(int8_t a_1, int8_t a_2){
	//concatena las aceleraciones lineales
	int16_t a = (int16_t) ((a_1 << 8) & 0xFF00) | (a_2 & 0x00FF);
	return a;
}
