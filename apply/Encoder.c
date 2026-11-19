#include "Encoder.h"

void EncoderInit(Encoder_t *Encoder)
{
    Encoder->EncoderCount = 0;
    Encoder->t_int = 0;
    Encoder->v_int = 0;

    Encoder->V = 0.0f;
    Encoder->X = 0.0f;
    Encoder->pos = 0.0f;
    Encoder->vel = 0.0f;
}

void EncodersInit(Motors_t *Motor)
{
    static Encoder_t encoderLeftFront;
    static Encoder_t encoderRightFront;

    Motor->EncoderLeft = &encoderLeftFront;
    Motor->EncoderRight = &encoderRightFront;

    EncoderInit(Motor->EncoderLeft);
    EncoderInit(Motor->EncoderRight);

    DL_GPIO_clearInterruptStatus(Encoder1_PORT, Encoder1_A1_PIN);
    DL_GPIO_clearInterruptStatus(Encoder2_PORT, Encoder2_A2_PIN);
    NVIC_ClearPendingIRQ(Encoder1_INT_IRQN);
    NVIC_ClearPendingIRQ(Encoder2_INT_IRQN);
    NVIC_EnableIRQ(Encoder1_INT_IRQN);
    NVIC_EnableIRQ(Encoder2_INT_IRQN);
}

void EncoderDataUpdate(Motors_t *Motor)
{
    uint32_t primask;
    int32_t left_cnt;
    int32_t right_cnt;
    float temp;

    if ((Motor == NULL) ||
        (Motor->EncoderLeft == NULL) ||
        (Motor->EncoderRight == NULL)) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    left_cnt = Motor->EncoderLeft->EncoderCount;
    right_cnt = Motor->EncoderRight->EncoderCount;
    Motor->EncoderLeft->EncoderCount = 0;
    Motor->EncoderRight->EncoderCount = 0;
    __set_PRIMASK(primask);

    Motor->EncoderLeft->v_int = left_cnt;
    Motor->EncoderRight->v_int = right_cnt;

    temp = (EncoderReadingFrequency / EncoderLines) * 2.0f * PI;

    Motor->EncoderLeft->vel =
        ((float)Motor->EncoderLeft->v_int * temp) * ENCODER_VEL_LPF_ALPHA +
        Motor->EncoderLeft->vel * (1.0f - ENCODER_VEL_LPF_ALPHA);
    Motor->EncoderRight->vel =
        ((float)Motor->EncoderRight->v_int * temp) * ENCODER_VEL_LPF_ALPHA +
        Motor->EncoderRight->vel * (1.0f - ENCODER_VEL_LPF_ALPHA);

    Motor->EncoderLeft->V = Motor->EncoderLeft->vel * TireRadius;
    Motor->EncoderRight->V = Motor->EncoderRight->vel * TireRadius;

    Motor->EncoderLeft->X += Motor->EncoderLeft->V / EncoderReadingFrequency;
    Motor->EncoderRight->X += Motor->EncoderRight->V / EncoderReadingFrequency;
}

float EncoderTotalLengthGet(Motors_t *Motors)
{
    return (Motors->EncoderLeft->X + Motors->EncoderRight->X) * 0.5f;
}

void EncoderLengthClear(Motors_t *Motors)
{
    Motors->EncoderLeft->X = 0.0f;
    Motors->EncoderRight->X = 0.0f;
}
