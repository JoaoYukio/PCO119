#ifndef CONTROL_H_
#define CONTROL_H_

#define int16_t __INT16_TYPE__

float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd);
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki);
float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM);




#endif /* CONTROL_H_ */