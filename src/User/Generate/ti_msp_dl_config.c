/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerG_backupConfig gTIMEG6_MotorBackup;
DL_TimerG_backupConfig gQEI_ENCODER_RIGHTBackup;
DL_TimerG_backupConfig gCAPTURE_ENCODER_LEFTBackup;
DL_SPI_backupConfig gSPI_1Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_TIMEG6_Motor_init();
    SYSCFG_DL_QEI_ENCODER_RIGHT_init();
    SYSCFG_DL_CAPTURE_ENCODER_LEFT_init();
    SYSCFG_DL_TIMER_ADC_GRAY_init();
    SYSCFG_DL_TIMERG12_TS_init();
    SYSCFG_DL_UART_1_init();
    SYSCFG_DL_UART_TJC_init();
    SYSCFG_DL_SPI_1_init();
    SYSCFG_DL_ADC12_0_init();
    SYSCFG_DL_DMA_init();
    /* Ensure backup structures have no valid state */
	gTIMEG6_MotorBackup.backupRdy 	= false;
	gQEI_ENCODER_RIGHTBackup.backupRdy 	= false;
	gCAPTURE_ENCODER_LEFTBackup.backupRdy 	= false;


	gSPI_1Backup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_saveConfiguration(TIMEG6_Motor_INST, &gTIMEG6_MotorBackup);
	retStatus &= DL_TimerG_saveConfiguration(QEI_ENCODER_RIGHT_INST, &gQEI_ENCODER_RIGHTBackup);
	retStatus &= DL_TimerG_saveConfiguration(CAPTURE_ENCODER_LEFT_INST, &gCAPTURE_ENCODER_LEFTBackup);
	retStatus &= DL_SPI_saveConfiguration(SPI_1_INST, &gSPI_1Backup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerG_restoreConfiguration(TIMEG6_Motor_INST, &gTIMEG6_MotorBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(QEI_ENCODER_RIGHT_INST, &gQEI_ENCODER_RIGHTBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(CAPTURE_ENCODER_LEFT_INST, &gCAPTURE_ENCODER_LEFTBackup, false);
	retStatus &= DL_SPI_restoreConfiguration(SPI_1_INST, &gSPI_1Backup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(TIMEG6_Motor_INST);
    DL_TimerG_reset(QEI_ENCODER_RIGHT_INST);
    DL_TimerG_reset(CAPTURE_ENCODER_LEFT_INST);
    DL_TimerG_reset(TIMER_ADC_GRAY_INST);
    DL_TimerG_reset(TIMERG12_TS_INST);
    DL_UART_Main_reset(UART_1_INST);
    DL_UART_Main_reset(UART_TJC_INST);
    DL_SPI_reset(SPI_1_INST);
    DL_ADC12_reset(ADC12_0_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(TIMEG6_Motor_INST);
    DL_TimerG_enablePower(QEI_ENCODER_RIGHT_INST);
    DL_TimerG_enablePower(CAPTURE_ENCODER_LEFT_INST);
    DL_TimerG_enablePower(TIMER_ADC_GRAY_INST);
    DL_TimerG_enablePower(TIMERG12_TS_INST);
    DL_UART_Main_enablePower(UART_1_INST);
    DL_UART_Main_enablePower(UART_TJC_INST);
    DL_SPI_enablePower(SPI_1_INST);
    DL_ADC12_enablePower(ADC12_0_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_TIMEG6_Motor_C0_IOMUX,GPIO_TIMEG6_Motor_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TIMEG6_Motor_C0_PORT, GPIO_TIMEG6_Motor_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_TIMEG6_Motor_C1_IOMUX,GPIO_TIMEG6_Motor_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_TIMEG6_Motor_C1_PORT, GPIO_TIMEG6_Motor_C1_PIN);

    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_ENCODER_RIGHT_PHA_IOMUX,GPIO_QEI_ENCODER_RIGHT_PHA_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_QEI_ENCODER_RIGHT_PHB_IOMUX,GPIO_QEI_ENCODER_RIGHT_PHB_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunction(GPIO_CAPTURE_ENCODER_LEFT_C0_IOMUX,GPIO_CAPTURE_ENCODER_LEFT_C0_IOMUX_FUNC);
    DL_GPIO_initPeripheralInputFunction(GPIO_CAPTURE_ENCODER_LEFT_C1_IOMUX,GPIO_CAPTURE_ENCODER_LEFT_C1_IOMUX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_1_IOMUX_TX, GPIO_UART_1_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_1_IOMUX_RX, GPIO_UART_1_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_TJC_IOMUX_TX, GPIO_UART_TJC_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_TJC_IOMUX_RX, GPIO_UART_TJC_IOMUX_RX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_1_IOMUX_SCLK, GPIO_SPI_1_IOMUX_SCLK_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_1_IOMUX_PICO, GPIO_SPI_1_IOMUX_PICO_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_SPI_1_IOMUX_POCI, GPIO_SPI_1_IOMUX_POCI_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_SPI_1_IOMUX_CS0, GPIO_SPI_1_IOMUX_CS0_FUNC);

    DL_GPIO_initDigitalOutputFeatures(GPIO_CS_SPI_CS_PB5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);

    DL_GPIO_initDigitalInputFeatures(GPIO_IMU_INT1_IMU_INT1_PA_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_LED0_LED0_PA0_IOMUX);

    DL_GPIO_initDigitalInputFeatures(GPIO_KEY_KEY_PB21_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_RGB_LED_GREEN_PB27_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRAY_ADDR_ADDR0_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRAY_ADDR_ADDR1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_GRAY_ADDR_ADDR2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_TB6612_DIR_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_TB6612_DIR_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_TB6612_DIR_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(GPIO_TB6612_DIR_BIN2_IOMUX);

    DL_GPIO_clearPins(GPIOA, GPIO_GRAY_ADDR_ADDR1_PIN |
		GPIO_GRAY_ADDR_ADDR2_PIN |
		GPIO_TB6612_DIR_AIN2_PIN |
		GPIO_TB6612_DIR_AIN1_PIN);
    DL_GPIO_setPins(GPIOA, GPIO_LED0_LED0_PA0_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_LED0_LED0_PA0_PIN |
		GPIO_GRAY_ADDR_ADDR1_PIN |
		GPIO_GRAY_ADDR_ADDR2_PIN |
		GPIO_TB6612_DIR_AIN2_PIN |
		GPIO_TB6612_DIR_AIN1_PIN);
    DL_GPIO_clearPins(GPIOB, GPIO_RGB_LED_GREEN_PB27_PIN |
		GPIO_GRAY_ADDR_ADDR0_PIN |
		GPIO_TB6612_DIR_BIN1_PIN |
		GPIO_TB6612_DIR_BIN2_PIN);
    DL_GPIO_setPins(GPIOB, GPIO_CS_SPI_CS_PB5_PIN);
    DL_GPIO_enableOutput(GPIOB, GPIO_CS_SPI_CS_PB5_PIN |
		GPIO_RGB_LED_GREEN_PB27_PIN |
		GPIO_GRAY_ADDR_ADDR0_PIN |
		GPIO_TB6612_DIR_BIN1_PIN |
		GPIO_TB6612_DIR_BIN2_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_4_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOB, GPIO_IMU_INT1_IMU_INT1_PA_PIN);
    DL_GPIO_enableInterrupt(GPIOB, GPIO_IMU_INT1_IMU_INT1_PA_PIN);

}


SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
    /* Set default configuration */
    DL_SYSCTL_disableHFXT();
    DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);
    DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
    /* DMA Group Priority */
    NVIC_SetPriority(DMA_INT_IRQn, 1);

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMEG6_MotorClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gTIMEG6_MotorConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN,
    .period = 3200,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMEG6_Motor_init(void) {

    DL_TimerG_setClockConfig(
        TIMEG6_Motor_INST, (DL_TimerG_ClockConfig *) &gTIMEG6_MotorClockConfig);

    DL_TimerG_initPWMMode(
        TIMEG6_Motor_INST, (DL_TimerG_PWMConfig *) &gTIMEG6_MotorConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(TIMEG6_Motor_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(TIMEG6_Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(TIMEG6_Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(TIMEG6_Motor_INST, 3200, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(TIMEG6_Motor_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(TIMEG6_Motor_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(TIMEG6_Motor_INST, 3200, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(TIMEG6_Motor_INST);


    
    DL_TimerG_setCCPDirection(TIMEG6_Motor_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}


static const DL_TimerG_ClockConfig gQEI_ENCODER_RIGHTClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};


SYSCONFIG_WEAK void SYSCFG_DL_QEI_ENCODER_RIGHT_init(void) {

    DL_TimerG_setClockConfig(
        QEI_ENCODER_RIGHT_INST, (DL_TimerG_ClockConfig *) &gQEI_ENCODER_RIGHTClockConfig);

    DL_TimerG_configQEI(QEI_ENCODER_RIGHT_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_0_INDEX);
    DL_TimerG_configQEI(QEI_ENCODER_RIGHT_INST, DL_TIMER_QEI_MODE_2_INPUT,
        DL_TIMER_CC_INPUT_INV_NOINVERT, DL_TIMER_CC_1_INDEX);
    DL_TimerG_setLoadValue(QEI_ENCODER_RIGHT_INST, 65535);
    DL_TimerG_enableInterrupt(QEI_ENCODER_RIGHT_INST , DL_TIMER_EVENT_DC_EVENT);

    DL_TimerG_enableClock(QEI_ENCODER_RIGHT_INST);
}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gCAPTURE_ENCODER_LEFTClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CAPTURE_ENCODER_LEFT_INST_LOAD_VALUE = (2.048 ms * 32000000 Hz) - 1
 */

SYSCONFIG_WEAK void SYSCFG_DL_CAPTURE_ENCODER_LEFT_init(void) {

    DL_TimerG_setClockConfig(CAPTURE_ENCODER_LEFT_INST,
        (DL_TimerG_ClockConfig *) &gCAPTURE_ENCODER_LEFTClockConfig);

    DL_TimerG_setLoadValue(CAPTURE_ENCODER_LEFT_INST,65535);

    DL_TimerG_setCounterMode(CAPTURE_ENCODER_LEFT_INST,DL_TIMER_COUNT_MODE_UP);

    DL_TimerG_setCounterRepeatMode(CAPTURE_ENCODER_LEFT_INST,DL_TIMER_REPEAT_MODE_ENABLED);

    DL_TimerG_setCounterValueAfterEnable(CAPTURE_ENCODER_LEFT_INST,DL_TIMER_COUNT_AFTER_EN_LOAD_VAL);

    DL_TimerG_setCaptureCompareCtl(CAPTURE_ENCODER_LEFT_INST,
    DL_TIMER_CC_MODE_CAPTURE, (DL_TIMER_CC_ZCOND_NONE | DL_TIMER_CC_ACOND_TIMCLK | DL_TIMER_CC_CCOND_TRIG_EDGE),
    DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareInput(CAPTURE_ENCODER_LEFT_INST,
        DL_TIMER_CC_INPUT_INV_NOINVERT,DL_TIMER_CC_IN_SEL_CCPX, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareCtl(CAPTURE_ENCODER_LEFT_INST,
    DL_TIMER_CC_MODE_CAPTURE, (DL_TIMER_CC_ZCOND_NONE | DL_TIMER_CC_ACOND_TIMCLK | DL_TIMER_CC_CCOND_TRIG_EDGE),
    DL_TIMER_CC_1_INDEX);

    DL_TimerG_setCaptureCompareInput(CAPTURE_ENCODER_LEFT_INST,
        DL_TIMER_CC_INPUT_INV_NOINVERT,DL_TIMER_CC_IN_SEL_CCPX, DL_TIMER_CC_1_INDEX);


    DL_TimerG_setCounterControl(CAPTURE_ENCODER_LEFT_INST,
        DL_TIMER_CZC_CCCTL0_ZCOND,
        DL_TIMER_CAC_CCCTL0_ACOND,
        DL_TIMER_CLC_CCCTL0_LCOND
    );

    DL_TimerG_startCounter(CAPTURE_ENCODER_LEFT_INST);

    DL_TimerG_enableInterrupt(CAPTURE_ENCODER_LEFT_INST , DL_TIMERG_INTERRUPT_CC0_UP_EVENT |
		DL_TIMERG_INTERRUPT_CC1_UP_EVENT);

    DL_TimerG_enableClock(CAPTURE_ENCODER_LEFT_INST);

}


/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_ADC_GRAYClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_ADC_GRAY_INST_LOAD_VALUE = (20 us * 32000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_ADC_GRAYTimerConfig = {
    .period     = TIMER_ADC_GRAY_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_ADC_GRAY_init(void) {

    DL_TimerG_setClockConfig(TIMER_ADC_GRAY_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_ADC_GRAYClockConfig);

    DL_TimerG_initTimerMode(TIMER_ADC_GRAY_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_ADC_GRAYTimerConfig);
    DL_TimerG_enableClock(TIMER_ADC_GRAY_INST);


    DL_TimerG_enableEvent(TIMER_ADC_GRAY_INST, DL_TIMERG_EVENT_ROUTE_1, (DL_TIMERG_EVENT_ZERO_EVENT));

    DL_TimerG_setPublisherChanID(TIMER_ADC_GRAY_INST, DL_TIMERG_PUBLISHER_INDEX_0, TIMER_ADC_GRAY_INST_PUB_0_CH);



}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   4000000 Hz = 4000000 Hz / (8 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMERG12_TSClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMERG12_TS_INST_LOAD_VALUE = (1073.741824 s * 4000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMERG12_TSTimerConfig = {
    .period     = TIMERG12_TS_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMERG12_TS_init(void) {

    DL_TimerG_setClockConfig(TIMERG12_TS_INST,
        (DL_TimerG_ClockConfig *) &gTIMERG12_TSClockConfig);

    DL_TimerG_initTimerMode(TIMERG12_TS_INST,
        (DL_TimerG_TimerConfig *) &gTIMERG12_TSTimerConfig);
    DL_TimerG_enableClock(TIMERG12_TS_INST);





}



static const DL_UART_Main_ClockConfig gUART_1ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_1Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_1_init(void)
{
    DL_UART_Main_setClockConfig(UART_1_INST, (DL_UART_Main_ClockConfig *) &gUART_1ClockConfig);

    DL_UART_Main_init(UART_1_INST, (DL_UART_Main_Config *) &gUART_1Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 2000000
     *  Actual baud rate: 2000000
     */
    DL_UART_Main_setOversampling(UART_1_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_1_INST, UART_1_IBRD_32_MHZ_2000000_BAUD, UART_1_FBRD_32_MHZ_2000000_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_1_INST,
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_RX |
                                 DL_UART_MAIN_INTERRUPT_DMA_DONE_TX);

    /* Configure DMA Receive Event */
    DL_UART_Main_enableDMAReceiveEvent(UART_1_INST, DL_UART_DMA_INTERRUPT_RX);
    /* Configure DMA Transmit Event */
    DL_UART_Main_enableDMATransmitEvent(UART_1_INST);
    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(UART_1_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_1_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(UART_1_INST, DL_UART_TX_FIFO_LEVEL_ONE_ENTRY);

    DL_UART_Main_enable(UART_1_INST);
}

static const DL_UART_Main_ClockConfig gUART_TJCClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_TJCConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_TJC_init(void)
{
    DL_UART_Main_setClockConfig(UART_TJC_INST, (DL_UART_Main_ClockConfig *) &gUART_TJCClockConfig);

    DL_UART_Main_init(UART_TJC_INST, (DL_UART_Main_Config *) &gUART_TJCConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115211.52
     */
    DL_UART_Main_setOversampling(UART_TJC_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_TJC_INST, UART_TJC_IBRD_32_MHZ_115200_BAUD, UART_TJC_FBRD_32_MHZ_115200_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(UART_TJC_INST,
                                 DL_UART_MAIN_INTERRUPT_FRAMING_ERROR |
                                 DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
                                 DL_UART_MAIN_INTERRUPT_RX);

    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(UART_TJC_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_TJC_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
    DL_UART_Main_setTXFIFOThreshold(UART_TJC_INST, DL_UART_TX_FIFO_LEVEL_ONE_ENTRY);

    DL_UART_Main_enable(UART_TJC_INST);
}

static const DL_SPI_Config gSPI_1_config = {
    .mode        = DL_SPI_MODE_CONTROLLER,
    .frameFormat = DL_SPI_FRAME_FORMAT_MOTO4_POL0_PHA0,
    .parity      = DL_SPI_PARITY_NONE,
    .dataSize    = DL_SPI_DATA_SIZE_8,
    .bitOrder    = DL_SPI_BIT_ORDER_MSB_FIRST,
    .chipSelectPin = DL_SPI_CHIP_SELECT_0,
};

static const DL_SPI_ClockConfig gSPI_1_clockConfig = {
    .clockSel    = DL_SPI_CLOCK_BUSCLK,
    .divideRatio = DL_SPI_CLOCK_DIVIDE_RATIO_1
};

SYSCONFIG_WEAK void SYSCFG_DL_SPI_1_init(void) {
    DL_SPI_setClockConfig(SPI_1_INST, (DL_SPI_ClockConfig *) &gSPI_1_clockConfig);

    DL_SPI_init(SPI_1_INST, (DL_SPI_Config *) &gSPI_1_config);

    /* Configure Controller mode */
    /*
     * Set the bit rate clock divider to generate the serial output clock
     *     outputBitRate = (spiInputClock) / ((1 + SCR) * 2)
     *     1000000 = (32000000)/((1 + 15) * 2)
     */
    DL_SPI_setBitRateSerialClockDivider(SPI_1_INST, 15);

    /* Enable SPI TX interrupt as a trigger for DMA */
    DL_SPI_enableDMATransmitEvent(SPI_1_INST);

    /* Enable SPI RX interrupt as a trigger for DMA */
    DL_SPI_enableDMAReceiveEvent(SPI_1_INST, DL_SPI_DMA_INTERRUPT_RX);
    /* Set RX and TX FIFO threshold levels */
    DL_SPI_setFIFOThreshold(SPI_1_INST, DL_SPI_RX_FIFO_LEVEL_ONE_FRAME, DL_SPI_TX_FIFO_LEVEL_ONE_FRAME);

    /* Enable module */
    DL_SPI_enable(SPI_1_INST);
}

/* ADC12_0 Initialization */
static const DL_ADC12_ClockConfig gADC12_0ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_ULPCLK,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_0_init(void)
{
    DL_ADC12_setClockConfig(ADC12_0_INST, (DL_ADC12_ClockConfig *) &gADC12_0ClockConfig);
    DL_ADC12_initSingleSample(ADC12_0_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_EVENT,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_ADC_CH0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(ADC12_0_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC12_0_INST,8);
    DL_ADC12_enableDMA(ADC12_0_INST);
    DL_ADC12_setDMASamplesCnt(ADC12_0_INST,1);
    DL_ADC12_enableDMATrigger(ADC12_0_INST,(DL_ADC12_DMA_MEM0_RESULT_LOADED));
    DL_ADC12_setSubscriberChanID(ADC12_0_INST,ADC12_0_INST_SUB_CH);
    DL_ADC12_enableConversions(ADC12_0_INST);
}

static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode   = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_HALF_WORD,
    .srcWidth       = DL_DMA_WIDTH_HALF_WORD,
    .trigger        = ADC12_0_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH0_init(void)
{
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL0);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 40);
    DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID , (DL_DMA_Config *) &gDMA_CH0Config);
}
static const DL_DMA_Config gDMA_CH4Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_UNCHANGED,
    .srcIncrement   = DL_DMA_ADDR_INCREMENT,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = SPI_1_INST_DMA_TRIGGER_0,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH4_init(void)
{
    DL_DMA_setTransferSize(DMA, DMA_CH4_CHAN_ID, 15);
    DL_DMA_initChannel(DMA, DMA_CH4_CHAN_ID , (DL_DMA_Config *) &gDMA_CH4Config);
}
static const DL_DMA_Config gDMA_CH3Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = SPI_1_INST_DMA_TRIGGER_1,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH3_init(void)
{
    DL_DMA_clearInterruptStatus(DMA, DL_DMA_INTERRUPT_CHANNEL3);
    DL_DMA_enableInterrupt(DMA, DL_DMA_INTERRUPT_CHANNEL3);
    DL_DMA_setTransferSize(DMA, DMA_CH3_CHAN_ID, 15);
    DL_DMA_initChannel(DMA, DMA_CH3_CHAN_ID , (DL_DMA_Config *) &gDMA_CH3Config);
}
static const DL_DMA_Config gDMA_CH1Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_UNCHANGED,
    .srcIncrement   = DL_DMA_ADDR_INCREMENT,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_1_INST_DMA_TRIGGER_0,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH1_init(void)
{
    DL_DMA_initChannel(DMA, DMA_CH1_CHAN_ID , (DL_DMA_Config *) &gDMA_CH1Config);
}
static const DL_DMA_Config gDMA_CH2Config = {
    .transferMode   = DL_DMA_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_BYTE,
    .srcWidth       = DL_DMA_WIDTH_BYTE,
    .trigger        = UART_1_INST_DMA_TRIGGER_1,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH2_init(void)
{
    DL_DMA_initChannel(DMA, DMA_CH2_CHAN_ID , (DL_DMA_Config *) &gDMA_CH2Config);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void){
    SYSCFG_DL_DMA_CH0_init();
    SYSCFG_DL_DMA_CH4_init();
    SYSCFG_DL_DMA_CH3_init();
    SYSCFG_DL_DMA_CH1_init();
    SYSCFG_DL_DMA_CH2_init();
}


