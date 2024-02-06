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

/**
 * @brief Выполнить одну итерацию системы управления
 * @details Вызывается раз в 5мс
 */
void controllerTick (Overlord &over)
{
    float setPoint = -over.getSetpoint ();
    float carX = -over.getCarX ();
    float carVel = -over.getCarVel ();
    float motorAngle = over.getMotorTheta ();
    float motorVel = over.getMotorVel ();

    float w0 = over.getSlider(SliderEnum::prog1) * 1.0 / 1000;
    float e = w0 - motorVel;

    static float I = 0;
    static constexpr float Ki = 6.8;
    static constexpr float Kp = 1;
    static constexpr float Ts = 0.006;

    //float u = e * Kp;
    // float u = I;
    // float eKi = e * Ki;
    // float dI = eKi * Ts;
    // I = I + dI; 

    float eKp = e * Kp;
    float u = I + eKp;
    float umax = 12;
    float eKi = eKp * Ki;
    if(u == constrain(u, umax, -umax) ||
        I * eKi < 0)
    {
        float dI = Ts * Ki;
        I = I + dI;
        
    }       

    u = constrain(u, -umax, umax);

    Serial.print(w0);
    Serial.print(' ');
    Serial.println(motorVel);

    over.setMotorU (u);
}
 