#include "ti_msp_dl_config.h"

#include "car_task.h"
#include "icm42688.h"
#include "imu_task.h"
#include "line_task.h"
#include "motor_inf_task.h"
#include "power_led.h"
#include "tb6612.h"
#include "ts_time.h"
#include "tjc_protocol.h"
#include "tjc_uart.h"
#include "vofa.h"

uint32_t main_deltat;
uint32_t main_deltat_last;

int main(void)
{
    SYSCFG_DL_init();

    /*
     * GPIOA and GPIOB share Group 1. Keep the unconnected IMU source disabled
     * until ICM42688_Init() succeeds, otherwise enabling the encoder IRQ would
     * also expose a floating PB4 input.
     */
    DL_GPIO_disableInterrupt(
        GPIO_IMU_INT1_PORT,
        GPIO_IMU_INT1_IMU_INT1_PA_PIN);
    DL_GPIO_clearInterruptStatus(
        GPIO_IMU_INT1_PORT,
        GPIO_IMU_INT1_IMU_INT1_PA_PIN);

    Power_Led_Init();
    TS_Time_Init();
    TJC_UART_Init();
    VOFA_Init();
    TB6612_Init();

    //IMU_Task_Init();
    Line_Task_Init();
    Motor_Inf_Task_Init();
    Car_Task_Init();
    TJC_Protocol_Init();

    while (1)
    {
        TJC_Protocol_Process();

//        IMU_Task_Run();
        Line_Task_Run();
        Car_Task_Run();

        Motor_Inf_Task_Run();

        VOFA_ProcessCommand();
        VOFA_SendConfiguredChannels();
        main_deltat = TS_Time_GetDelta_us(&main_deltat_last) * 0.001f;
    }
}

void GROUP1_IRQHandler(void)
{
    uint32_t iidx;

    for (;;)
    {
        iidx = DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1);

        switch (iidx)
        {
            case DL_INTERRUPT_GROUP1_IIDX_GPIOA:
                Motor_Inf_LeftEncoderGPIO_IRQHandler();
                break;

            case GPIO_IMU_INT1_INT_IIDX:
                ICM42688_GPIO_IRQHandler();
                break;

            default:
                return;
        }
    }
}
