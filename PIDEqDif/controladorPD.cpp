#include <iostream>
#include <iostream>
#include "defines.h"
#include <string>
#include <fstream>
#include <chrono>

////// MUDEI AQUI PARA AUMENTAR A RESOLUCAO JA QUE NAO ESTAVA FUNCIONANDO COM 256 //////
#define SHIFT (8)


class PDControlPF{
    private:
        float Kp;
        float Kd;

        float k1;
        float k2;
        float k3;

        float dt;
        int16_t errorOld;
        int16_t o_1;
        int16_t e_1;
        int16_t e_2;
        int16_t y_0;
        int16_t error;

        float KdConst;
        
    public:
    PDControlPF(float Kp, float Kd, float dt){
        this->Kp = Kp;
        this->Kd = Kd;
        this->dt = dt;
        this->o_1 = 0;
        this->e_1 = 0;
        this->e_2 = 0;
        this->error = 0;
        this->y_0 = 0;
        this->KdConst = Kd/dt;
        
    
        this->k1 = (Kp + (KdConst)); // i_k1 = (f_kp + f_ki * f_T + f_kd / f_T) * SHIFT;
        this->k2 = -(Kp + (2*(KdConst))); // i_k2 = -((f_kp + 2 * f_kd / f_T) * SHIFT);
        this->k3 = (KdConst); //i_k3 = (f_kd / f_T) * SHIFT;

        
    }
    
    
    int controlEqDif(int setPoint, int input){
        int16_t output;
        e_2 = e_1;
        e_1 = error;
        error = setPoint - input;
        
        y_0 = (Kp*(error - e_1)*SHIFT) + (KdConst*(error - 2*e_1 + e_2)*SHIFT);
        
////// MUDEI AQUI PARA AUMENTAR A RESOLUCAO JA QUE NAO ESTAVA FUNCIONANDO COM 256 //////
        y_0 = y_0>>3;
        y_0 = y_0 + o_1;

        output = y_0;
        o_1 = output;
        return output;
    }
    
    
    int controlConstEqDif(int16_t setPoint, int16_t input){
        e_2 = e_1;
        e_1 = error;
        error = setPoint - input;
        //2 x 4 = 8
        //2,0 * 4,0 = 8,0
        //20 * 40 = 800
        y_0 = ((k1*error) +(k2*e_1) + (k3*e_2));
        //y_0 = y_0 >> 8; // shift = 2**8
        y_0 += o_1;

        if (y_0>1023){
            y_0 = 1023;
        }
        /*
        if (y_0<0){
            y_0 = 0;
        }
        */
        
        o_1 = y_0;
        return y_0;
    }   
};

int main()
{
    PDControlPF control(0.5, 0.005, 0.1);
    int16_t setPoint = 100;
    int16_t input = 0;
    int16_t output = 0;
    std::ofstream myfile;
    myfile.open ("example.txt");
    for (int i=0; i<1000; i++){
        output = control.controlEqDif(setPoint, input);
        myfile << output << ",";
        input = output;
    }
    myfile.close();


    return 0;
}