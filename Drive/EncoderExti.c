#include "EncoderExti.h"

extern Car_t car;

void GROUP1_IRQHandler(void)
{
    if ((car.Motors == NULL) ||
        (car.Motors->EncoderLeft == NULL) ||
        (car.Motors->EncoderRight == NULL)) {
        return;
    }

    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1))
    {
        case Encoder1_INT_IIDX:
            if (DL_GPIO_getEnabledInterruptStatus(Encoder1_PORT, Encoder1_A1_PIN) != 0U) {
                DL_GPIO_clearInterruptStatus(Encoder1_PORT, Encoder1_A1_PIN);
                /* Read quadrature B1 phase on A1 edge to determine direction. */
                if (DL_GPIO_readPins(Encoder1_PORT, Encoder1_B1_PIN) != 0U) {
                    car.Motors->EncoderLeft->EncoderCount--;
                } else {
                    car.Motors->EncoderLeft->EncoderCount++;
                }
            }
            break;

        case Encoder2_INT_IIDX:
            if (DL_GPIO_getEnabledInterruptStatus(Encoder2_PORT, Encoder2_A2_PIN) != 0U) {
                DL_GPIO_clearInterruptStatus(Encoder2_PORT, Encoder2_A2_PIN);
                /* Read quadrature B2 phase on A2 edge to determine direction. */
                if (DL_GPIO_readPins(Encoder2_PORT, Encoder2_B2_PIN) != 0U) {
                    car.Motors->EncoderRight->EncoderCount--;
                } else {
                    car.Motors->EncoderRight->EncoderCount++;
                }
            }
            break;

        default:
            break;
    }
}
