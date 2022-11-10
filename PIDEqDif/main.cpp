#include <iostream>
#include "defines.h"
#include <string>
#include <fstream>

#define SHIFT (256)

using namespace std;

int16_t constrain(int16_t e, int16_t min, int16_t max)
{
  if(e < min)return min;
  if(e > max)return max;
  return e;
}

class F_PIControl{
    private:
        float Kp;
        float Ki;
        float dt;
        float errorOld;
        float errorSum;
        float Iout;
        float Pout;
        float o_1;
        float e_1;
        float error;
        
    public:
    F_PIControl(float Kp, float Ki, float dt){
        this->Kp = Kp;
        this->Ki = Ki;
        this->dt = dt;
        this->errorOld = 0;
        this->errorSum = 0;
        this->Iout = 0;
        this->Pout = 0;
        this->o_1 = 0;
        this->e_1 = 0;
        this->error = 0;
    }
    float control(float setPoint, float input){
        float output;

        error = setPoint - input;

        Pout = Kp * error;

        float integral = error * dt;

        Iout += Ki * integral;

        output = Pout + Iout;
        
        return output;
    }
    float controlEqDif(float setPoint, float input){
        float output;
        e_1 = error;
        error = setPoint - input;

        Pout = Kp*(error - e_1);

        Iout = dt * Ki * error;

        output = o_1 + Pout + Iout;
        o_1 = output;
        return output;
    }
};

int multPF(float a, float b){
    int16_t d = (a*b)*SHIFT;
    d = d>>8;
    return d;
}

class PIControlPF{
private:
        float Kp;
        float Ki;

        int16_t k1;
        int16_t k2;
        int16_t k3;

        float dt;
        int errorOld;
        int16_t o_1;
        float e_1;
        int16_t y_0;
        float error;
        
    public:
    PIControlPF(float Kp, float Ki, float dt){
        this->Kp = Kp;
        this->Ki = Ki;
        this->dt = dt;
        this->o_1 = 0;
        this->e_1 = 0;
        this->error = 0;
        this->y_0 = 0;

        this->k1 = (Kp + (Ki*dt)) * SHIFT; // i_k1 = (f_kp + f_ki * f_T + f_kd / f_T) * SHIFT;
        this->k2 = -(Kp*SHIFT); // i_k2 = -((f_kp + 2 * f_kd / f_T) * SHIFT);
        this->k3 = 0; //i_k3 = (f_kd / f_T) * SHIFT;

    }

    int controlEqDif(float setPoint, float input){
        int output;
        float temp;
        e_1 = error;
        error = setPoint - input;


        //temp = (Kp*(error - e_1)) + (dt * Ki * error);//(k1 * error + k2 * e_1 + k3 * o_1) >> 10;
        //y_0 = (multPF(Kp, (error - e_1)) + multPF(dt, multPF(Ki, error)));
        y_0 = (Kp*(error - e_1)*SHIFT) + (dt * Ki * error *SHIFT);
        y_0 = y_0>>8;
        y_0 = y_0 + o_1;

        output = y_0;
        o_1 = output;
        return output;
    }
    /*
    int controlEqDif(float setPoint, float input){
        o_1 = y_0;
        e_1 = error;
        error = setPoint - input;

        y_0 = ((int32_t) k1*error + (int32_t) k2*e_1);
        y_0 = y_0 >> 8; // shift = 2**8
        y_0 += o_1;

        return y_0;
    }
    */
    
};

int main()
{
    //Create a PI controller
    F_PIControl PI(0.5, 7, 0.1);
    PIControlPF PIPF(0.5, 7, 0.1);
    
    int size = 200;

    //Create a file to save the data
    ofstream myfile;
    myfile.open ("PIControl.txt");

    //Create a file to save the data
    ofstream myfilePF;
    myfilePF.open ("PIControlPF.txt");

    float output = 0;
    float input = 0;

    float* deg = new float[size];

    for(int i = 0; i < size; i++){
        if (i < 25){
            deg[i] = 0;
        }else{
            deg[i] = 10;
        }
    }

    //Simulate a step response for the PI controller
    for(int i = 0; i < size; i++)
    {
        output = PI.controlEqDif(deg[i], input);
        myfile << output << ",";
        input = output;
    }


    int outputPF = 0;
    int inputPF = 0;

    int* degI = new int[size];

    for(int i = 0; i < size; i++){
        if (i < 25){
            degI[i] = 0;
        }else{
            degI[i] = 10;
        }
    }
    
    for(int i = 0; i < size; i++)
    {
        outputPF = PIPF.controlEqDif(degI[i], inputPF);
        myfilePF << outputPF << ",";
        inputPF = outputPF;
    }

    return 0;
}