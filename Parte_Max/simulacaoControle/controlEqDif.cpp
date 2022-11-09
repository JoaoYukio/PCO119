//https://github.com/rmaalmeida/PID_pic16f1619.X/blob/master/main.c
//https://embarcados.com.br/controle-pid-com-pic16f1619/

#include <iostream>
#include "globals.h"
#include "defines.h"
#include <string>
#include <fstream>


using namespace std;

int16_t constrain(int16_t e, int16_t min, int16_t max)
{
  if(e < min)return min;
  if(e > max)return max;
  return e;
}

float constrain(float e, int16_t min, int16_t max)
{
  if(e < min)return min;
  if(e > max)return max;
  return e;
}

// Usando a equacao de diferenca
// yn = yn-1 + kp(e(n) - e(n-1))+ T*kie(n) + kd/T(e(n) - 2e(n-1) + e(n-2))

class StabilityPDfloat{
    private:
        float Kp;
        float Kd;
        float setPointOld;
        float PID_errorOld, PID_errorOld1;
        float dt;
        float output;
        float error;
        float out_1;
    public:
    StabilityPDfloat(float Kp, float Kd, float dt){
        this->Kp = Kp;
        this->Kd = Kd;
        this->dt = dt;
        this->output = 0;
        this->out_1 = 0;
        this->error = 0;
        this->setPointOld = 0;
        this->PID_errorOld = 0;
    }
    float getControl(float input, float setPoint){
        error = setPoint - input;
        out_1 = output;
        
        // Kd is implemented in two parts
        //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
        //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
        //float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
        //output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / dt;
        //  output = Kp * error + (Kd * (setPoint - input - (setPointOld - PID_errorOld) ))) / DT;
        //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
        //PID_errorOld2 = PID_errorOld;
        output = out_1 + Kp*(error - PID_errorOld) + (Kd/dt)*(error - 2*PID_errorOld + PID_errorOld1);

        PID_errorOld1 = PID_errorOld;
        PID_errorOld = error;


        return output;
    }
};

class SpeedPIFloat{
    private:
        float Ki;
        float Kp;
        float dt;
        //float PID_errorSum;
        float e_1;
        float out_1;
        float error;
    public:
    SpeedPIFloat(float Kp, float Ki, float dt){
        this->Kp = Kp;
        this->Ki = Ki;
        this->dt = dt;
        this->e_1 = 0;
        this->out_1 = 0;
        this->error = 0;
    }
    float getControl(float input, float setPoint){
        float error = setPoint - input;
        PID_errorSum += error * dt;
        return Kp * error + Ki * PID_errorSum;
    }
    float getControlDif(float input, float setPoint){
        error = setPoint - input;
        out_1 = out_1 + Kp*(error - e_1) + Ki*error*dt;
        e_1 = error;
        return out_1;
    }
};


int main(){
    StabilityPDfloat control(0.3, 0.01, 0.1);

    SpeedPIFloat control2(0.5, 0.2, 0.1);


    float* out = new float[1000];
    float* in = new float[1000];
    float* set = new float[1000];
    
    for(int i = 0; i < 1000; i++){
        in[i] = 0;
        set[i] = 0;
    }
    /*
    for(int i = 0; i < 100; i++){
        if(i > 10){
            set[i] = 1;
        }
        out[i] = control.getControl(in[i], set[i]);
        in[i+1] = out[i];
    }
    */
    // Get the output of the control2 using the diference equation
    for(int i = 0; i < 1000; i++){
        if(i > 10){
            set[i] = 1;
        }
        out[i] = control2.getControlDif(in[i], set[i]);
        in[i+1] = out[i];
    }  

    ofstream myfile;
    myfile.open ("eqDifPI.txt");
    for(int i = 0; i < 1000; i++){
        myfile << out[i] << ",";
    }
    myfile.close();
    return 0;
}