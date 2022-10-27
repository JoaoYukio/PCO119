#include <iostream>
#include "control.h"
#include "globals.h"
#include "defines.h"

/*
    Estrutura da simulação
*/

int degrau[100];
using namespace std;

int main()
{
    for(int i = 0; i <100; i++)
    {
        if(i<15){
            degrau[i] = 0;
        }else{
            degrau[i] = 1;
        }
    }

    float dt = 1/100;
    for(int i = 0; i <100; i++){
        //Simulacao
        actual_robot_speed = (speed_M1 + speed_M2) / 2; // Positive: forward
    }
    
    return 0;
}