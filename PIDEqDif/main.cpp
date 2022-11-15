#include <iostream>
#include "defines.h"
#include <string>
#include <fstream>
#include <chrono>

#define SHIFT (256)

using namespace std;

int16_t constrain(int16_t e, int16_t min, int16_t max)
{
  if(e < min)return min;
  if(e > max)return max;
  return e;
}

class PIControl{
    private:
    int16_t error;
    float output;
    float Kp;
    float Ki;
    float DT;
    int16_t PID_errorSum;
    //int16_t PID_errorOld;
    //int16_t setPointOld;
    public:
    PIControl(float Kp, float Ki, float DT)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->DT = DT;
        PID_errorSum = 0;
        //PID_errorOld = 0;
        //setPointOld = 0;
    }
    float speedPIControl(int16_t input, int16_t setPoint){
        error = setPoint - input;
        PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
        PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);
        output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
        return (output);
    }
};

class F_PDControl{
    private:
        float Kp;
        float Kd;
        float dt;
        float errorOld;
        float errorSum;
        float Pout;
        float Dout;
        float o_1;
        float e_1;
        float e_2;
        float error;
        float Kd_setPoint;
        float setPointOld;
        float PID_errorOld;
        public:
        F_PDControl(float Kp, float Kd, float dt){
            this->Kp = Kp;
            this->Kd = Kd;
            this->dt = dt;
            this->errorOld = 0;
            this->errorSum = 0;
            this->Pout = 0;
            this->Dout = 0;
            this->o_1 = 0;
            this->e_1 = 0;
            this->e_2 = 0;
            this->error = 0;
            this->Kd_setPoint = 0;
            this->setPointOld = 0;
            this->PID_errorOld = 0;
        }
        float eqDifPD(float setPoint, float input)
        {   
            error = setPoint - input;
            float output = o_1 + Kp*(error - e_1) + (Kd/dt)*(error - 2*e_1 + e_2);
            
            /*
            if(output > 255)output = 255;
            if(output < 0)output = 0;
            */
            e_2 = e_1;
            e_1 = error;
            o_1 = output;
            return output;
        }
        float control(float setPoint, float input)
        {
            //Implements a PD controller
            error = setPoint - input;
            float derivative = (error - e_1) / dt;
            Pout = Kp * error;
            Dout = Kd * derivative;
            float output = Pout + Dout;

            e_1 = error;
            return output;

        }
        float stabilityPDControl(float setPoint, float input)
        {
            error = setPoint - input;
            Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
            float output = Kp * error +(Kd*Kd_setPoint - Kd *(input -PID_errorOld)) / dt;
            PID_errorOld = input;
            setPointOld = setPoint;
            return (output);
        }
        
};

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

// PRECISEI MUDAR O SHIFT AQUI, PARA AUMENTAR A RESOLUCAO//
#define SHIFT2 (8)

class PDControlPF{
    private:
        float Kp;
        float Kd;

        int16_t k1;
        int16_t k2;
        int16_t k3;

        float dt;
        int16_t errorOld;
        int16_t o_1;
        int16_t e_1;
        int16_t e_2;
        int16_t y_0;
        int16_t error;
        
    public:
    PDControlPF(float Kp, float Kd, float dt){
        this->Kp = Kp;
        this->Kd = Kd;
        this->dt = dt;
        this->o_1 = 0;
        this->e_1 = 0;
        this->error = 0;
        this->y_0 = 0;
        this->e_2 = 0;

        this->k1 = (Kp + (Kd/dt)) * SHIFT2; // i_k1 = (f_kp + f_ki * f_T + f_kd / f_T) * SHIFT;
        this->k2 = -((Kp + (2*(Kd/dt)))*SHIFT2); // i_k2 = -((f_kp + 2 * f_kd / f_T) * SHIFT);
        this->k3 = (Kd/dt) * SHIFT2; //i_k3 = (f_kd / f_T) * SHIFT;

    }    

    void printKd(){
        cout << "Kd: " << Kd << endl;
    }

    void printKp(){
        cout << "Kp: " << Kp << endl;
    }

    void printK1(){
        cout << "K1: " << k1 << endl;
    }

    void printK2(){
        cout << "K2: " << k2 << endl;
    }

    void printK3(){
        cout << "K3: " << k3 << endl;
    }
    
    int controlEqDif(int16_t setPoint, int16_t input){
        error = setPoint - input;
        //2 x 4 = 8
        //2,0 * 4,0 = 8,0
        //20 * 40 = 800
        y_0 = ((k1*error) + (k2*e_1) + (k3*e_2));
        // NOVO SHIFT, PARA TER UMA RESOLUCAO MELHOR
        y_0 = y_0 >> 3; // shift2 = 2**12
        y_0 += o_1;
        
        
        if (y_0>1023){
            y_0 = 1023;
        }
        
        if (y_0<-10){
            y_0 = -10;
        }
        e_2 = e_1;
        e_1 = error;
        o_1 = y_0;
        return y_0;
    }   
};

class PIControlPF{
    private:
        float Kp;
        float Ki;

        int16_t k1;
        int16_t k2;
        int16_t k3;

        float dt;
        int16_t errorOld;
        int16_t o_1;
        int16_t e_1;
        int16_t y_0;
        int16_t error;
        
    public:
    PIControlPF(float Kp, float Ki, float dt){
        this->Kp = Kp;
        this->Ki = Ki;
        this->dt = dt;
        this->o_1 = 0;
        this->e_1 = 0;
        this->error = 0;
        this->y_0 = 0;
//fast inverse square root
        this->k1 = (Kp + (Ki*dt)) * SHIFT; // i_k1 = (f_kp + f_ki * f_T + f_kd / f_T) * SHIFT;
        this->k2 = -(Kp*SHIFT); // i_k2 = -((f_kp + 2 * f_kd / f_T) * SHIFT);
        this->k3 = 0; //i_k3 = (f_kd / f_T) * SHIFT;

    }
    
    /*
    int controlEqDif(float setPoint, float input){
        int output;
        e_1 = error;
        error = setPoint - input;
        
        y_0 = (Kp*(error - e_1)*SHIFT) + (dt * Ki * error *SHIFT);
        y_0 = y_0>>8;
        y_0 = y_0 + o_1;

        output = y_0;
        o_1 = output;
        return output;
    }
    */
    
    
    int controlEqDif(int16_t setPoint, int16_t input){
        o_1 = y_0;
        e_1 = error;
        error = setPoint - input;
        //2 x 4 = 8
        //2,0 * 4,0 = 8,0
        //20 * 40 = 800
        y_0 = (k1*error + k2*e_1);
        y_0 = y_0 >> 8; // shift = 2**8
        y_0 += o_1;

        if (y_0>1023){
            y_0 = 1023;
        }
        if (y_0<0){
            y_0 = 0;
        }
        return y_0;
    }   
};

int main()
{
    //Create a PI controller
    PIControl myPI(0.5, 7, 0.1);
    F_PIControl PI(0.5, 7, 0.1);
    PIControlPF PIPF(0.5, 7, 0.1);
    
    int size = 200;

    //Create a file to save the data
    /*
    ofstream myfile;
    myfile.open ("PIControl.txt");

    //Create a file to save the data
    ofstream myfilePF;
    myfilePF.open ("PIControlPF.txt");

    ofstream myfileF;
    myfileF.open ("PIControlF.txt");

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
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;
    //Simulate a step response for the PI controller

    begin = std::chrono::steady_clock::now();

    for(int i = 0; i < size; i++)
    {
        output = PI.controlEqDif(deg[i], input);
        myfile << output << ",";
        input = output;
    }
    end = std::chrono::steady_clock::now();

    cout << "Time difference using float = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    begin = std::chrono::steady_clock::now();
    for(int i = 0; i < size; i++)
    {
        output = myPI.speedPIControl(input, deg[i]);
        myfileF << output << ",";
        input = output;
    }
    end = std::chrono::steady_clock::now();
    cout << "Time difference using original control = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;
    */
    int outputPF = 0;
    int inputPF = 0;
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;

    int* degI = new int[size];

    ofstream myfilePF;
    myfilePF.open ("PIControlPF.txt");

    for(int i = 0; i < size; i++){
        if (i < 25){
            degI[i] = 0;
        }else{
            degI[i] = 1000;
        }
    }
    
    begin = std::chrono::steady_clock::now();
    for(int i = 0; i < size; i++)
    {
        outputPF = PIPF.controlEqDif(degI[i], inputPF);
        myfilePF << outputPF << ",";
        inputPF = outputPF;
    }
    end = std::chrono::steady_clock::now();
    cout << "Time difference using fixed point = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    
    F_PDControl PD(0.5, 0.005, 0.1);
    float output = 0;
    
    float* deg = new float[size];

    ofstream myfileF;
    myfileF.open ("PDControlF.txt");

    for(int i = 0; i < size; i++){
        if (i < 25){
            deg[i] = 0;
        }else{
            deg[i] = 1000;
        }
    }
    //std::chrono::steady_clock::time_point begin;
    //std::chrono::steady_clock::time_point end;

    begin = std::chrono::steady_clock::now();
    for(int i = 0; i < size; i++)
    {
        output = PD.stabilityPDControl(deg[i], output);
        myfileF << output << ",";
    }
    end = std::chrono::steady_clock::now();

    cout << "Time difference using float = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    F_PDControl control2(0.5, 0.005, 0.1);

    ofstream myfile2;
    myfile2.open ("PDControlImp.txt");    

    begin = std::chrono::steady_clock::now();
    for(int i = 0; i < size; i++)
    {
        output = control2.eqDifPD(deg[i], output);
        myfile2 << output << ",";
    }
    end = std::chrono::steady_clock::now();
    
    cout << "Time difference using eq. dif = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    ofstream myfile3;
    myfile3.open("PDControlConta.txt");

    begin = std::chrono::steady_clock::now();
    for(int i = 0; i < size; i++)
    {
        output = control2.control(deg[i], output);
        myfile3 << output << ",";
    }
    end = std::chrono::steady_clock::now();

    cout << "Time difference using conta = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    
    ofstream myfile4;
    myfile4.open("PDControlPF.txt");

    PDControlPF control3(0.5, 0.005, 0.1);
    
    /*
    control3.printK1();
    control3.printK2();
    control3.printK3();

    control3.printKd();
    control3.printKp();
    */
    
    int outInt = 0;
    int inputInt = 0;

    
    begin = std::chrono::steady_clock::now();
    for(int i = 0; i < size; i++)
    {
        outInt = control3.controlEqDif(degI[i], inputInt);
        myfile4 << outInt << ",";
        inputInt = outInt;
    }
    end = std::chrono::steady_clock::now();
    cout << "Time difference using fixed point = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << "[µs]" << std::endl;

    return 0;
}