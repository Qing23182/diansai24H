#include "Key.h"

extern Car_t car;

#define KEY_PRESSED     1U
#define KEY_UNPRESSED   0U

#define KEY_TIME_DOUBLE 200U
#define KEY_TIME_LONG   2000U
#define KEY_TIME_REPEAT 100U

volatile uint8_t Key_Flag[KEY_COUNT];

uint8_t read_pin(uint8_t n)
{
    if (n == 1U) {
        if (DL_GPIO_readPins(Key_PORT, Key_PIN_3_PIN) == 0U) {
            return KEY_PRESSED;
        }
    } else if (n == 2U) {
        if (DL_GPIO_readPins(Key_PORT, Key_PIN_4_PIN) == 0U) {
            return KEY_PRESSED;
        }
    } else if (n == 3U) {
        if (DL_GPIO_readPins(Key_PORT, Key_PIN_5_PIN) == 0U) {
            return KEY_PRESSED;
        }
    }

    return KEY_UNPRESSED;
}

uint8_t Key_check(uint8_t n, uint8_t flag)
{
    uint8_t hit = 0U;
    uint32_t primask;

    if (n >= KEY_COUNT) {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if ((Key_Flag[n] & flag) != 0U) {
        if (flag != KEY_FLAG_HOLD) {
            Key_Flag[n] &= (uint8_t)(~flag);
        }
        hit = 1U;
    }
    __set_PRIMASK(primask);
    return hit;
}

void Key_Tick(void)
{
    static uint8_t Count, i;
    static uint8_t CurrState[KEY_COUNT], PrevState[KEY_COUNT];
    static uint8_t S[KEY_COUNT];
    static uint16_t Time[KEY_COUNT];

    for (i = 0U; i < KEY_COUNT; i++) {
        if (Time[i] > 0U) {
            Time[i]--;
        }
    }

    Count++;
    if (Count >= 20U) {
        Count = 0U;
        for (i = 0U; i < KEY_COUNT; i++) {
            PrevState[i] = CurrState[i];
            CurrState[i] = read_pin((uint8_t)(i + 1U));

            if (CurrState[i] == KEY_PRESSED) {
                Key_Flag[i] |= KEY_FLAG_HOLD;
            } else {
                Key_Flag[i] &= (uint8_t)(~KEY_FLAG_HOLD);
            }
            if ((CurrState[i] == KEY_PRESSED) && (PrevState[i] == KEY_UNPRESSED)) {
                Key_Flag[i] |= KEY_FLAG_DOWN;
            }
            if ((CurrState[i] == KEY_UNPRESSED) && (PrevState[i] == KEY_PRESSED)) {
                Key_Flag[i] |= KEY_FLAG_UP;
            }

            if (S[i] == 0U) {
                if (CurrState[i] == KEY_PRESSED) {
                    Time[i] = KEY_TIME_LONG;
                    S[i] = 1U;
                }
            } else if (S[i] == 1U) {
                if (CurrState[i] == KEY_UNPRESSED) {
                    Time[i] = KEY_TIME_DOUBLE;
                    S[i] = 2U;
                } else if (Time[i] == 0U) {
                    Time[i] = KEY_TIME_REPEAT;
                    Key_Flag[i] |= KEY_FLAG_LONG;
                    S[i] = 4U;
                }
            } else if (S[i] == 2U) {
                if (CurrState[i] == KEY_PRESSED) {
                    Key_Flag[i] |= KEY_FLAG_DOUBLE;
                    S[i] = 3U;
                } else if (Time[i] == 0U) {
                    Key_Flag[i] |= KEY_FLAG_SINGLE;
                    S[i] = 0U;
                }
            } else if (S[i] == 3U) {
                if (CurrState[i] == KEY_UNPRESSED) {
                    S[i] = 0U;
                }
            } else if (S[i] == 4U) {
                if (CurrState[i] == KEY_UNPRESSED) {
                    S[i] = 0U;
                } else if (Time[i] == 0U) {
                    Time[i] = KEY_TIME_REPEAT;
                    Key_Flag[i] |= KEY_FLAG_REPEAT;
                    S[i] = 4U;
                }
            }
        }
    }
}

void Key_Task_Handle(void)
{
    if (car.task == NULL) {
        return;
    }

    /* Any key press while running or counting down triggers immediate E-stop. */
    if (car.task->Carstartflag != 0) {
        if (Key_check(0U, KEY_FLAG_DOWN) ||
            Key_check(1U, KEY_FLAG_DOWN) ||
            Key_check(2U, KEY_FLAG_DOWN)) {
            car.task->Carstartflag = 0;
            car.task->StartDelayTicks = 0;
            car.task->TaskState = 0;
            MotorStop(car.Motors);
            BuzzerBee(car.buzzer);
            LCD_ShowStatusLine("E-STOP!", LCD_RED);
        }
        return;
    }

    /* Long-press K1 while idle to run 2-step grayscale calibration. */
    if (Key_check(0U, KEY_FLAG_LONG)) {
        MotorStop(car.Motors);
        GraySensor_RunCalibration(car.GraySensor_t, car.buzzer);
        return;
    }

    /* K1 single = Task 1, K2 single = Task 2, K3 single = Task 3, K3 double = Task 4. */
    if (Key_check(0U, KEY_FLAG_SINGLE)) {
        car.task->TaskFlag = 1;
        car.task->TaskState = 0;
        car.task->StartDelayTicks = TASK_START_DELAY_TICKS;
        car.task->Carstartflag = 1;
        BuzzerBee(car.buzzer);
        LCD_ShowStatusLine("T1 ARMING...", LCD_YELLOW);
    } else if (Key_check(1U, KEY_FLAG_SINGLE)) {
        car.task->TaskFlag = 2;
        car.task->TaskState = 0;
        car.task->StartDelayTicks = TASK_START_DELAY_TICKS;
        car.task->Carstartflag = 1;
        BuzzerBee(car.buzzer);
        LCD_ShowStatusLine("T2 ARMING...", LCD_YELLOW);
    } else if (Key_check(2U, KEY_FLAG_SINGLE)) {
        car.task->TaskFlag = 3;
        car.task->TaskState = 0;
        car.task->StartDelayTicks = TASK_START_DELAY_TICKS;
        car.task->Carstartflag = 1;
        BuzzerBee(car.buzzer);
        LCD_ShowStatusLine("T3 ARMING...", LCD_YELLOW);
    } else if (Key_check(2U, KEY_FLAG_DOUBLE)) {
        car.task->TaskFlag = 4;
        car.task->TaskState = 0;
        car.task->StartDelayTicks = TASK_START_DELAY_TICKS;
        car.task->Carstartflag = 1;
        BuzzerBee(car.buzzer);
        LCD_ShowStatusLine("T4 ARMING...", LCD_YELLOW);
    }
}
