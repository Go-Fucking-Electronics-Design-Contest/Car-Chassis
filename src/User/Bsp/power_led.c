#include "power_led.h"
#include "ti_msp_dl_config.h"

void Power_Led_Init(void)
{
    Power_Led_Set(1u);
}

void Power_Led_Set(uint8_t on)
{
    if (on)
    {
        DL_GPIO_setPins(GPIO_RGB_LED_PORT, GPIO_RGB_LED_GREEN_PB27_PIN);
    }
    else
    {
        DL_GPIO_clearPins(GPIO_RGB_LED_PORT, GPIO_RGB_LED_GREEN_PB27_PIN);
    }
}
