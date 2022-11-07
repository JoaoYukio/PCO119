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
        float PID_errorOld, PID_errorOld2;
        float dt;
        float output;
    public:
    StabilityPDfloat(float Kp, float Kd, float dt){
        this->Kp = Kp;
        this->Kd = Kd;
        this->dt = dt;
        this->output = 0;
        this->setPointOld = 0;
        this->PID_errorOld = 0;
    }
    float getControl(float input, float setPoint){
        float error;

        error = setPoint - input;

        // Kd is implemented in two parts
        //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
        //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
        //float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
        //output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / dt;
        //  output = Kp * error + (Kd * (setPoint - input - (setPointOld - PID_errorOld) ))) / DT;
        //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
        //PID_errorOld2 = PID_errorOld;
        output += Kp*(error - PID_errorOld) + Kd*(error - 2*PID_errorOld + PID_errorOld2)/dt;
        PID_errorOld2 = PID_errorOld;
        PID_errorOld = error;//input  // error for Kd is only the input component
        //setPointOld = setPoint;
        return (output);
    }
    float* setOutputObserver(int simTime){
        float* output = (float*)malloc(sizeof(float)*simTime);
        float input = 0;
        float setPoint = 0;
        float control = 0;
        for(int i = 0; i < simTime; i++){
            if(i < 10){
                setPoint = 0;
            }else{
                setPoint = 1;
            }
            control = this->getControl(input, setPoint);
            input = input + control;
            output[i] = input;
        }
        return output;
    }
};

int main(){
    StabilityPDfloat control(0.3, 0.01, 0.1);
    float* output = control.setOutputObserver(1000);
    ofstream myfile;
    myfile.open ("eqDifPD.txt");
    for(int i = 0; i < 1000; i++){
        myfile << output[i] << ",";
    }
    myfile.close();
    return 0;
}