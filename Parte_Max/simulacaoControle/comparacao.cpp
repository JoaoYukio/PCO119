#include <iostream>
//#include "globals.h"
#include "defines.h"
#include <string>
#include <fstream>

#define SHIFT (256)

// Create constrain
int16_t constrain(int16_t e, int16_t min, int16_t max)
{
  if(e < min)return min;
  if(e > max)return max;
  return e;
}

//Create a PI controller
class PIControl{
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
    PIControl(float Kp, float Ki, float dt){
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


class PDControl{
    private:
        float Kp;
        float Kd;
        float dt;
        float errorOld;
        float errorSum;
        float Dout;
        float Pout;
        float o_1;
        float e_1;
        float error;
        float e_2;
        float setPointOld;
        float PID_errorOld;
    public:
    PDControl(float Kp, float Kd, float dt){
        this->Kp = Kp;
        this->Kd = Kd;
        this->dt = dt;
        this->errorOld = 0;
        this->errorSum = 0;
        this->Dout = 0;
        this->Pout = 0;
        this->o_1 = 0;
        this->e_1 = 0;
        this->error = 0;
        this->e_2 = 0;
        this->setPointOld = 0;
        this->PID_errorOld = 0;
    }

    float control(float setPoint, float input){
        float output;

        errorOld = error;
        error = setPoint - input;

        Pout = Kp * error;

        float derivative = (error - errorOld) / dt;

        Dout = Kd * derivative;

        output = Pout + Dout;
        
        return output;
    }
    float controlEqDif(float setPoint, float input){
        float output;
        e_2 = e_1;
        e_1 = error;
        error = setPoint - input;

        Pout = Kp*(error - e_1);

        Dout = (1/dt) * Kd * (error - 2*e_1 + e_2);

        output = o_1 + Pout + Dout;
        o_1 = output;
        return output;
    }
    float getControl(float setPoint, float input){
        error = setPoint - input;

        float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8);

        float control = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / dt;//Kp*error + Kd*(error/dt);

        PID_errorOld = input;  // error for Kd is only the input component
        setPointOld = setPoint;

        control = constrain(control, -255, 255);
        return control;
    }
};

class PIControlPF{
private:
        int16_t Kp;
        int16_t Ki;
        int16_t k1;
        int16_t k2;
        int16_t k3;

        float dt;
        float errorOld;
        float errorSum;
        float Iout;
        float Pout;
        float o_1;
        float e_1;
        float error;
        
    public:
    PIControlPF(int16_t Kp, int16_t Ki, float dt){
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

        this->k1 = (Kp + (Ki*dt)) * SHIFT; // i_k1 = (f_kp + f_ki * f_T + f_kd / f_T) * SHIFT;
        this->k2 = -(Kp*SHIFT); // i_k2 = -((f_kp + 2 * f_kd / f_T) * SHIFT);
        this->k3 = 0; //i_k3 = (f_kd / f_T) * SHIFT;

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

using namespace std;

int main()
{
    // Create a PI controller
    PIControl pi(0.5, 0.2, 0.1);

    PDControl pd(0.3, 0.01, 0.1);
    // Create a step
    int size = 5000;
    int delay = 10;
    int* deg = (int*)malloc(sizeof(int)*size);
    for(int i = 0; i < size; i++)
    {
        if(i<delay)
        {
            deg[i]=0;
        }else{
            deg[i] = 1;
        }
    }
    
    // Create a file to save the data
    ofstream myfile;

    float input = 0;
    float output = 0;
    /*
    
    myfile.open ("example.txt");
    // Simulate the system
    

    for(int i = 0; i < size; i++)
    {
        output = pi.control(deg[i], input);
        myfile << output << ",";
        input = output;
    }

    myfile.close();
    */
    

    // Simulate the system with the equation of difference
    myfile.open ("seriesEx.txt");
    // Simulate the system
    /*
    for(int i = 0; i < size; i++)
    {
        output = pd.getControl(deg[i], input);
        myfile << output << ",";
        input = output;
    }
    */
    // Simulate a PI controller in cascade with a PD controller
    float outputPI = 0;
    for(int i = 0; i < size; i++)
    {
        outputPI = pi.control(deg[i], input);
        output = pd.control(outputPI, input);
        myfile << output << ",";
        input = output;
    }

    

    myfile.close();

    return 0;
}
