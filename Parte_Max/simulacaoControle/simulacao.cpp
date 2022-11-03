#include <iostream>
#include "globals.h"
#include "defines.h"
#include <string>
#include <fstream>

/*
    Estrutura da simulação
*/

#define int16_t int16_t

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

class degrau{
    public:
    int size;
    int* deg;
    degrau(int size, int delay)
    {
        if(size < delay){
            printf("Tamanho menor do que o delay!\n");
        }
        this->size = size;
        deg = (int*)malloc(sizeof(int)*size);
        for(int i = 0; i < size; i++)
        {
            if(i<delay)
            {
                deg[i]=0;
            }else{
                deg[i] = 1;
            }
        }
    }
};

class stabilityPDControl{
    private:
        float Kp;
        float Kd;
        float setPointOld;
        float PID_errorOld;
        float dt;
    public:
    stabilityPDControl(float Kp, float Kd, float dt){
        this->Kp = Kp;
        this->Kd = Kd;
        this->dt = dt;
    }
    float getKp(){
        return Kp;
    }
    float getKd(){
        return Kd;
    }
    float getDt(){
        return dt;
    }
    void setKp(float Kp){
        this->Kp = Kp;
    }
    void setKd(float Kd){
        this->Kd = Kd;
    }
    void setDt(float dt){
        this->dt = dt;
    }
    float getControl(float input, float setPoint){
        float error = setPoint - input;

        float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8);

        float control = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / dt;//Kp*error + Kd*(error/dt);

        PID_errorOld = input;  // error for Kd is only the input component
        setPointOld = setPoint;

        control = constrain(control, -255, 255);
        return control;
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
            control = getControl(input, setPoint);
            input = input + control;
            output[i] = input;
        }
        return output;
    }
    float* setOutputObserver(int simTime, float* setPoint, float* input){
        float* output = (float*)malloc(sizeof(float)*simTime);
        float control = 0;
        for(int i = 0; i < simTime; i++){
            control = getControl(input[i], setPoint[i]);
            input[i+1] = input[i] + control;
            output[i] = input[i];
        }
        return output;
    }
};

class speedPIControl{
    private:
        float Ki;
        float Kp;
        float dt;
        float PID_errorSum;
    public:
    speedPIControl(float Kp, float Ki, float dt){
        this->Kp = Kp;
        this->Ki = Ki;
        this->dt = dt;
    }
    float getKp(){
        return Kp;
    }
    float getKi(){
        return Ki;
    }
    float getDt(){
        return dt;
    }
    void setKp(float Kp){
        this->Kp = Kp;
    }
    void setKi(float Ki){
        this->Ki = Ki;
    }
    void setDt(float dt){
        this->dt = dt;
    }
    float getControl(float input, float setPoint){
        float error = setPoint - input;
        PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);;
        float control = Kp * error + Ki * PID_errorSum * dt;// DT is in miliseconds
        control = constrain(control, -255, 255);
        return control;
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
            control = getControl(input, setPoint);
            input = input + control;
            output[i] = input;
        }
        return output;
    }
};

// Series control function
void seriesControl(stabilityPDControl pd1, speedPIControl pi)
{
    float setPoint = 0; 
    float input = 0;
    float inputPI = 0;
    float* outputPI = (float*)malloc(sizeof(float)*100);
    float* outputPD = (float*)malloc(sizeof(float)*100);

    for(int i = 0; i < 100 - 1; i++)
    {
        if(i < 10){
            setPoint = 0;
        }else{
            setPoint = 1;
        }
        outputPD[i] = pd1.getControl(input, setPoint);
        outputPI[i] = pi.getControl(setPoint,outputPD[i]);
        
        input = input + outputPI[i];
        inputPI = outputPD[i] - outputPI[i];
    }

    printf("Saida controlador PD\n");
    for(int i = 0; i < 100; i++)
    {
        //printf("%f,%f", outputPD[i], outputPI[i]);
        
        printf("%f,", outputPD[i]);
        printf("\n");
    }

    printf("Saida controlador PI\n");
    for(int i = 0; i < 100; i++)
    {
        //printf("%f,%f", outputPD[i], outputPI[i]);
        
        printf("%f,", outputPI[i]);
        printf("\n");
    }

    free(outputPI);
    free(outputPD);

}

int main()
{
    stabilityPDControl control(0.2, 0.02, 0.1);
    /*float* output = control.setOutputObserver(100);
    for(int i = 0; i < 100; i++){
        printf("%f,\n", output[i]);
    }*/

    speedPIControl control2(0.2, 0.05, 0.1);
    
    /*float* output = control2.setOutputObserver(1000);
    for(int i = 0; i < 1000; i++){
        printf("%f,\n", output[i]);
    }
    
    //Write the output to a file
    ofstream myfile;
    myfile.open ("output.txt");
    for(int i = 0; i < 1000; i++){
        myfile << output[i]<< ',';
    }
    myfile.close();
    */

   seriesControl(control, control2);
    return 0;
}