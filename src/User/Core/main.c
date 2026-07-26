#include "ti_msp_dl_config.h"

#include "car_task.h"
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
    Power_Led_Init();
    TS_Time_Init();
    TJC_UART_Init();
    VOFA_Init();
    TB6612_Init();

    IMU_Task_Init();
    Line_Task_Init();
    Motor_Inf_Task_Init();
    Car_Task_Init();
    TJC_Protocol_Init();

    while (1)
    {
        TJC_Protocol_Process();

        IMU_Task_Run();
        Line_Task_Run();
        Car_Task_Run();
        Motor_Inf_Task_Run();

        VOFA_ProcessCommand();
        VOFA_SendConfiguredChannels();
        main_deltat = TS_Time_GetDelta_us(&main_deltat_last) * 0.001f;
    }
}
