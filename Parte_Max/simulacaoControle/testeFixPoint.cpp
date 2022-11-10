#include <iostream>

using namespace std;

#define SHIFT (256)

int main()
{
    const float a = 10.0;
    const float b = 2.5;

    //Create a d variable and assing the value of a * b shifted 8 bits to the left
    int16_t d = (a*b)*SHIFT;
    d = d>>8;

    cout << d << endl;
}