#include "headfile.h"

Car_t car;
extern volatile uint8 imuflag;
extern volatile uint8 controlflag;
extern volatile uint8 vofaflag;

static uint32_t lcd_last_ms;
static uint32_t gray_last_ms;
static uint32_t imu_update_last_us;
static uint32_t euler_vofa_last_us;
static uint8_t imu_status = ICM42688_NO_SENSOR;

void duty_100hz(void)
{
    static uint32_t last_tick;
    static uint8_t initialized;
    uint32_t now_tick = micros();
    float dt;

    if (imu_status != ICM42688_NO_ERROR) {
        return;
    }

    if (initialized == 0U) {
        last_tick = now_tick;
        initialized = 1U;
        return;
    }

    dt = (float)(now_tick - last_tick) * 0.000001f;
    last_tick = now_tick;
    IMU_Attitude_SetStaticBiasEnable((car.task == NULL) || (car.task->Carstartflag == 0));
    IMU_Attitude_Update(dt);
}

void duty_50hz(void)
{
    BuzzerDataUpdate(car.buzzer);
    EncoderDataUpdate(car.Motors);

    if ((car.task != NULL) && (car.task->Carstartflag != 0) && (car.task->StartDelayTicks > 0)) {
        MotorStop(car.Motors);
        car.task->StartDelayTicks--;
        if ((car.task->StartDelayTicks == (TASK_START_DELAY_TICKS / 2)) ||
            (car.task->StartDelayTicks == 0)) {
            BuzzerBee(car.buzzer);
        }
        return;
    }

    if (car.task->TaskFlag == 1) {
        Task1(car);
    } else if (car.task->TaskFlag == 2) {
        Task2(car);
    } else if (car.task->TaskFlag == 3) {
        Task3(car);
    } else if (car.task->TaskFlag == 4) {
        Task4(car);
    }
}

int main(void)
{
    SYSCFG_DL_init();
    DL_GPIO_setPins(CS_PORT, CS_PIN_6_PIN);
    uart0_init();
    MotorInit(&car.Motors);
    Vofa_Init(car.Motors);
    GraySensorInit(&car.GraySensor_t);
    EncodersInit(car.Motors);
    BeepInit(&car.buzzer);
    imu_status = ICM42688_Init();
    Task_Init(&car.task);
    if (imu_status == ICM42688_NO_ERROR) {
        IMU_Attitude_Init();
    }
    car.IMU = &IMU_Attitude;

    LCD_Init();
    LCD_Clear(LCD_BLACK);
    LCD_DebugInit();
    LCD_ShowBootStatus(imu_status);
    if (imu_status != ICM42688_NO_ERROR) {
        BuzzerBee(car.buzzer);
    }

    imu_update_last_us = micros();
    euler_vofa_last_us = imu_update_last_us;
    gray_last_ms = imu_update_last_us / 1000U;
    lcd_last_ms = gray_last_ms;

#if (APP_BENCH_DEBUG_MODE == 0)
    Time_Init();
#endif

    while (1) {
#if (APP_BENCH_DEBUG_MODE != 0)
        uint32_t now_us = micros();
        uint32_t now_ms = now_us / 1000U;

        if ((imu_status == ICM42688_NO_ERROR) &&
            ((uint32_t)(now_us - imu_update_last_us) >= IMU_UPDATE_PERIOD_US)) {
            float dt = (float)(now_us - imu_update_last_us) * 0.000001f;
            imu_update_last_us = now_us;
            IMU_Attitude_SetStaticBiasEnable(1U);
            IMU_Attitude_Update(dt);
        }

        if ((imu_status == ICM42688_NO_ERROR) &&
            ((uint32_t)(now_us - euler_vofa_last_us) >= EULER_VOFA_PERIOD_US)) {
            euler_vofa_last_us = now_us;
            Vofa_SendEulerFireWater(&IMU_Attitude);
        }

        if ((uint32_t)(now_ms - gray_last_ms) >= GRAY_UPDATE_PERIOD_MS) {
            gray_last_ms = now_ms;
            GraySensorDataUpdate(car.GraySensor_t);
        }

        if ((uint32_t)(now_ms - lcd_last_ms) >= 50U) {
            lcd_last_ms = now_ms;
            LCD_DebugUpdate(car);
        }
#else
        if (imuflag) {
            imuflag = 0;
            duty_100hz();
        }

        if (controlflag) {
            controlflag = 0;
            duty_50hz();
        }

        if (vofaflag) {
            vofaflag = 0;
            Vofa_SendJustFloat();
        }

        Key_Task_Handle();
        Vofa_Task();

        if ((car.task->Carstartflag == 0) && ((millis() - lcd_last_ms) >= 50U)) {
            lcd_last_ms = millis();
            GraySensorDataUpdate(car.GraySensor_t);
            LCD_DebugUpdate(car);
        }
#endif
    }
}
