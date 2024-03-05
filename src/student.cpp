#include "student.h"
     
/**
 * @brief Инициализировать систему управления
 * @details Вызывется один раз перед вызовом
 * функции \p controllerTick
 */
void controllerInit (Overlord &over)
{
    over.setSlider (SliderEnum::prog1, -12000, 12000);
    over.setSlider (SliderEnum::prog2, -10000, 10000);
}
   float step(uint32_t time, uint32_t period, float min_out, float max_out)
{
  if(time/period % 2)
  {
    return min_out;
  }
  return max_out;
}
/**
 * @brief Выполнить одну итерацию системы управления
 * @details Вызывается раз в 5мс
 */
void controllerTick (Overlord &over)
{
//     float setPoint = -over.getSetpoint ();
//     float carX = -over.getCarX ();
//     float carVel = -over.getCarVel ();
//     float motorAngle = over.getMotorTheta (); 
//     float motorVel = over.getMotorVel();

    float setPoint = -over.getSetpoint();
    // float carX = -over.getCarX ();
    // float carVel = -over.getCarVel ();
    // float motorAngle = over.getMotorTheta (); 
    float motorVel = over.getMotorVel();

    float w0 = over.getSlider(SliderEnum::prog1) * 1.0 / 1000;

    static float I = 0;
    static constexpr float Ki = 6.8;
    static constexpr float Kp = 1;
    // static constexpr float Kk = 0.68;
    static constexpr float Ts = 0.03;

    float e = w0 - motorVel;
    float eKp = e * Kp;
    float u = eKp + I;
    float uMax = 12;
    float eKi = eKp * Ki;

    if(u == constrain(u, -uMax, uMax) ||
        I * eKi < 0)
    {
    float dI = over.getTs() * Ki * e;
    // float dI = eKi * Ts;
    float I = I + dI;
        // float dI = over.getTs() * Ki * e;
        // I = I + dI;
        // u = e * Kk * (1 + I)
    }

    u = constrain(u, -12, 12);


    // float phi0 = over.getSlider(SliderEnum::prog1) * 1.0 / 1000;
    // float K2 = 5.0;
    // float e2 = phi0 - motorAngle;
    // float w0 = e2 * K2;
    

    // float e = w0 - motorVel;

    // static float I = 0;
    // static constexpr float Ki = 22.187708;
    // static constexpr float Kp = 4.17314815;
    // static constexpr float Ts = 0.006;

    // float eKp = e * Kp;
    // float u = I + eKp;
    // float umax = 12;
    // float eKi = eKp * Ki;
    // if(u == constrain(u, umax, -umax) ||
    //     I * eKi < 0)
    // {     
    //     float dI = Ts * Ki;
    //     I = I + dI;
    //  }     
    //  u = constrain(u, -umax, umax);

    // float e = w0 - w;
    // float u = I;
    // float eKi = e * Ki;
    // float dI = eKi * Ts;
    // float I = I + dI;

     over.setMotorU(u);

    Serial.print(w0);
    Serial.print(' ');
    Serial.print(u);
    Serial.print(' ');
    Serial.println(motorVel);
}
 
 