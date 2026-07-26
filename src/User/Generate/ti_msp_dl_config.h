/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
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
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)


#define CPUCLK_FREQ                                                     32000000



/* Defines for TIMEG6_Motor */
#define TIMEG6_Motor_INST                                                  TIMG6
#define TIMEG6_Motor_INST_IRQHandler                            TIMG6_IRQHandler
#define TIMEG6_Motor_INST_INT_IRQN                              (TIMG6_INT_IRQn)
#define TIMEG6_Motor_INST_CLK_FREQ                                      32000000
/* GPIO defines for channel 0 */
#define GPIO_TIMEG6_Motor_C0_PORT                                          GPIOB
#define GPIO_TIMEG6_Motor_C0_PIN                                   DL_GPIO_PIN_2
#define GPIO_TIMEG6_Motor_C0_IOMUX                               (IOMUX_PINCM15)
#define GPIO_TIMEG6_Motor_C0_IOMUX_FUNC              IOMUX_PINCM15_PF_TIMG6_CCP0
#define GPIO_TIMEG6_Motor_C0_IDX                             DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_TIMEG6_Motor_C1_PORT                                          GPIOB
#define GPIO_TIMEG6_Motor_C1_PIN                                   DL_GPIO_PIN_3
#define GPIO_TIMEG6_Motor_C1_IOMUX                               (IOMUX_PINCM16)
#define GPIO_TIMEG6_Motor_C1_IOMUX_FUNC              IOMUX_PINCM16_PF_TIMG6_CCP1
#define GPIO_TIMEG6_Motor_C1_IDX                             DL_TIMER_CC_1_INDEX




/* Defines for QEI_ENCODER_RIGHT */
#define QEI_ENCODER_RIGHT_INST                                             TIMG8
#define QEI_ENCODER_RIGHT_INST_IRQHandler                        TIMG8_IRQHandler
#define QEI_ENCODER_RIGHT_INST_INT_IRQN                         (TIMG8_INT_IRQn)
/* Pin configuration defines for QEI_ENCODER_RIGHT PHA Pin */
#define GPIO_QEI_ENCODER_RIGHT_PHA_PORT                                    GPIOA
#define GPIO_QEI_ENCODER_RIGHT_PHA_PIN                            DL_GPIO_PIN_29
#define GPIO_QEI_ENCODER_RIGHT_PHA_IOMUX                          (IOMUX_PINCM4)
#define GPIO_QEI_ENCODER_RIGHT_PHA_IOMUX_FUNC              IOMUX_PINCM4_PF_TIMG8_CCP0
/* Pin configuration defines for QEI_ENCODER_RIGHT PHB Pin */
#define GPIO_QEI_ENCODER_RIGHT_PHB_PORT                                    GPIOA
#define GPIO_QEI_ENCODER_RIGHT_PHB_PIN                            DL_GPIO_PIN_30
#define GPIO_QEI_ENCODER_RIGHT_PHB_IOMUX                          (IOMUX_PINCM5)
#define GPIO_QEI_ENCODER_RIGHT_PHB_IOMUX_FUNC              IOMUX_PINCM5_PF_TIMG8_CCP1


/* Defines for CAPTURE_ENCODER_LEFT */
#define CAPTURE_ENCODER_LEFT_INST                                        (TIMG7)
#define CAPTURE_ENCODER_LEFT_INST_IRQHandler                        TIMG7_IRQHandler
#define CAPTURE_ENCODER_LEFT_INST_INT_IRQN                        (TIMG7_INT_IRQn)
#define CAPTURE_ENCODER_LEFT_INST_LOAD_VALUE                                (65535U)
/* GPIO defines for channel 0 */
#define GPIO_CAPTURE_ENCODER_LEFT_C0_PORT                                   GPIOA
#define GPIO_CAPTURE_ENCODER_LEFT_C0_PIN                          DL_GPIO_PIN_17
#define GPIO_CAPTURE_ENCODER_LEFT_C0_IOMUX                         (IOMUX_PINCM39)
#define GPIO_CAPTURE_ENCODER_LEFT_C0_IOMUX_FUNC             IOMUX_PINCM39_PF_TIMG7_CCP0
/* GPIO defines for channel 1 */
#define GPIO_CAPTURE_ENCODER_LEFT_C1_PORT                                   GPIOA
#define GPIO_CAPTURE_ENCODER_LEFT_C1_PIN                          DL_GPIO_PIN_18
#define GPIO_CAPTURE_ENCODER_LEFT_C1_IOMUX                         (IOMUX_PINCM40)
#define GPIO_CAPTURE_ENCODER_LEFT_C1_IOMUX_FUNC             IOMUX_PINCM40_PF_TIMG7_CCP1





/* Defines for TIMER_ADC_GRAY */
#define TIMER_ADC_GRAY_INST                                              (TIMG0)
#define TIMER_ADC_GRAY_INST_IRQHandler                          TIMG0_IRQHandler
#define TIMER_ADC_GRAY_INST_INT_IRQN                            (TIMG0_INT_IRQn)
#define TIMER_ADC_GRAY_INST_LOAD_VALUE                                    (639U)
#define TIMER_ADC_GRAY_INST_PUB_0_CH                                         (1)
/* Defines for TIMERG12_TS */
#define TIMERG12_TS_INST                                                (TIMG12)
#define TIMERG12_TS_INST_IRQHandler                            TIMG12_IRQHandler
#define TIMERG12_TS_INST_INT_IRQN                              (TIMG12_INT_IRQn)
#define TIMERG12_TS_INST_LOAD_VALUE                                (4294967295U)



/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOA
#define GPIO_UART_1_TX_PORT                                                GPIOA
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_9
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_8
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM20)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM19)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM20_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM19_PF_UART1_TX
#define UART_1_BAUD_RATE                                               (2000000)
#define UART_1_IBRD_32_MHZ_2000000_BAUD                                      (1)
#define UART_1_FBRD_32_MHZ_2000000_BAUD                                      (0)
/* Defines for UART_TJC */
#define UART_TJC_INST                                                      UART2
#define UART_TJC_INST_FREQUENCY                                         32000000
#define UART_TJC_INST_IRQHandler                                UART2_IRQHandler
#define UART_TJC_INST_INT_IRQN                                    UART2_INT_IRQn
#define GPIO_UART_TJC_RX_PORT                                              GPIOA
#define GPIO_UART_TJC_TX_PORT                                              GPIOA
#define GPIO_UART_TJC_RX_PIN                                      DL_GPIO_PIN_22
#define GPIO_UART_TJC_TX_PIN                                      DL_GPIO_PIN_21
#define GPIO_UART_TJC_IOMUX_RX                                   (IOMUX_PINCM47)
#define GPIO_UART_TJC_IOMUX_TX                                   (IOMUX_PINCM46)
#define GPIO_UART_TJC_IOMUX_RX_FUNC                    IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_TJC_IOMUX_TX_FUNC                    IOMUX_PINCM46_PF_UART2_TX
#define UART_TJC_BAUD_RATE                                              (115200)
#define UART_TJC_IBRD_32_MHZ_115200_BAUD                                    (17)
#define UART_TJC_FBRD_32_MHZ_115200_BAUD                                    (23)




/* Defines for SPI_1 */
#define SPI_1_INST                                                         SPI1
#define SPI_1_INST_IRQHandler                                   SPI1_IRQHandler
#define SPI_1_INST_INT_IRQN                                       SPI1_INT_IRQn
#define GPIO_SPI_1_PICO_PORT                                              GPIOB
#define GPIO_SPI_1_PICO_PIN                                       DL_GPIO_PIN_8
#define GPIO_SPI_1_IOMUX_PICO                                   (IOMUX_PINCM25)
#define GPIO_SPI_1_IOMUX_PICO_FUNC                   IOMUX_PINCM25_PF_SPI1_PICO
#define GPIO_SPI_1_POCI_PORT                                              GPIOB
#define GPIO_SPI_1_POCI_PIN                                       DL_GPIO_PIN_7
#define GPIO_SPI_1_IOMUX_POCI                                   (IOMUX_PINCM24)
#define GPIO_SPI_1_IOMUX_POCI_FUNC                   IOMUX_PINCM24_PF_SPI1_POCI
/* GPIO configuration for SPI_1 */
#define GPIO_SPI_1_SCLK_PORT                                              GPIOB
#define GPIO_SPI_1_SCLK_PIN                                       DL_GPIO_PIN_9
#define GPIO_SPI_1_IOMUX_SCLK                                   (IOMUX_PINCM26)
#define GPIO_SPI_1_IOMUX_SCLK_FUNC                   IOMUX_PINCM26_PF_SPI1_SCLK
#define GPIO_SPI_1_CS0_PORT                                               GPIOB
#define GPIO_SPI_1_CS0_PIN                                        DL_GPIO_PIN_6
#define GPIO_SPI_1_IOMUX_CS0                                    (IOMUX_PINCM23)
#define GPIO_SPI_1_IOMUX_CS0_FUNC                     IOMUX_PINCM23_PF_SPI1_CS0



/* Defines for ADC12_0 */
#define ADC12_0_INST                                                        ADC0
#define ADC12_0_INST_IRQHandler                                  ADC0_IRQHandler
#define ADC12_0_INST_INT_IRQN                                    (ADC0_INT_IRQn)
#define ADC12_0_ADCMEM_ADC_CH0                                DL_ADC12_MEM_IDX_0
#define ADC12_0_ADCMEM_ADC_CH0_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_0_ADCMEM_ADC_CH0_REF_VOLTAGE_V                                     3.3
#define ADC12_0_INST_SUB_CH                                                  (1)
#define GPIO_ADC12_0_C0_PORT                                               GPIOA
#define GPIO_ADC12_0_C0_PIN                                       DL_GPIO_PIN_27



/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (0)
#define ADC12_0_INST_DMA_TRIGGER                      (DMA_ADC0_EVT_GEN_BD_TRIG)
/* Defines for DMA_CH4 */
#define DMA_CH4_CHAN_ID                                                      (4)
#define SPI_1_INST_DMA_TRIGGER_0                              (DMA_SPI1_TX_TRIG)
/* Defines for DMA_CH3 */
#define DMA_CH3_CHAN_ID                                                      (3)
#define SPI_1_INST_DMA_TRIGGER_1                              (DMA_SPI1_RX_TRIG)
/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (1)
#define UART_1_INST_DMA_TRIGGER_0                            (DMA_UART1_TX_TRIG)
/* Defines for DMA_CH2 */
#define DMA_CH2_CHAN_ID                                                      (2)
#define UART_1_INST_DMA_TRIGGER_1                            (DMA_UART1_RX_TRIG)


/* Port definition for Pin Group GPIO_CS */
#define GPIO_CS_PORT                                                     (GPIOB)

/* Defines for SPI_CS_PB5: GPIOB.5 with pinCMx 18 on package pin 53 */
#define GPIO_CS_SPI_CS_PB5_PIN                                   (DL_GPIO_PIN_5)
#define GPIO_CS_SPI_CS_PB5_IOMUX                                 (IOMUX_PINCM18)
/* Port definition for Pin Group GPIO_IMU_INT1 */
#define GPIO_IMU_INT1_PORT                                               (GPIOB)

/* Defines for IMU_INT1_PA: GPIOB.4 with pinCMx 17 on package pin 52 */
// pins affected by this interrupt request:["IMU_INT1_PA"]
#define GPIO_IMU_INT1_INT_IRQN                                  (GPIOB_INT_IRQn)
#define GPIO_IMU_INT1_INT_IIDX                  (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_IMU_INT1_IMU_INT1_PA_IIDX                       (DL_GPIO_IIDX_DIO4)
#define GPIO_IMU_INT1_IMU_INT1_PA_PIN                            (DL_GPIO_PIN_4)
#define GPIO_IMU_INT1_IMU_INT1_PA_IOMUX                          (IOMUX_PINCM17)
/* Port definition for Pin Group GPIO_LED0 */
#define GPIO_LED0_PORT                                                   (GPIOA)

/* Defines for LED0_PA0: GPIOA.0 with pinCMx 1 on package pin 33 */
#define GPIO_LED0_LED0_PA0_PIN                                   (DL_GPIO_PIN_0)
#define GPIO_LED0_LED0_PA0_IOMUX                                  (IOMUX_PINCM1)
/* Port definition for Pin Group GPIO_KEY */
#define GPIO_KEY_PORT                                                    (GPIOB)

/* Defines for KEY_PB21: GPIOB.21 with pinCMx 49 on package pin 20 */
#define GPIO_KEY_KEY_PB21_PIN                                   (DL_GPIO_PIN_21)
#define GPIO_KEY_KEY_PB21_IOMUX                                  (IOMUX_PINCM49)
/* Port definition for Pin Group GPIO_RGB_LED */
#define GPIO_RGB_LED_PORT                                                (GPIOB)

/* Defines for GREEN_PB27: GPIOB.27 with pinCMx 58 on package pin 29 */
#define GPIO_RGB_LED_GREEN_PB27_PIN                             (DL_GPIO_PIN_27)
#define GPIO_RGB_LED_GREEN_PB27_IOMUX                            (IOMUX_PINCM58)
/* Defines for ADDR0: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_GRAY_ADDR_ADDR0_PORT                                        (GPIOB)
#define GPIO_GRAY_ADDR_ADDR0_PIN                                 (DL_GPIO_PIN_1)
#define GPIO_GRAY_ADDR_ADDR0_IOMUX                               (IOMUX_PINCM13)
/* Defines for ADDR1: GPIOA.28 with pinCMx 3 on package pin 35 */
#define GPIO_GRAY_ADDR_ADDR1_PORT                                        (GPIOA)
#define GPIO_GRAY_ADDR_ADDR1_PIN                                (DL_GPIO_PIN_28)
#define GPIO_GRAY_ADDR_ADDR1_IOMUX                                (IOMUX_PINCM3)
/* Defines for ADDR2: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_GRAY_ADDR_ADDR2_PORT                                        (GPIOA)
#define GPIO_GRAY_ADDR_ADDR2_PIN                                (DL_GPIO_PIN_31)
#define GPIO_GRAY_ADDR_ADDR2_IOMUX                                (IOMUX_PINCM6)
/* Defines for AIN2: GPIOA.11 with pinCMx 22 on package pin 57 */
#define GPIO_TB6612_DIR_AIN2_PORT                                        (GPIOA)
#define GPIO_TB6612_DIR_AIN2_PIN                                (DL_GPIO_PIN_11)
#define GPIO_TB6612_DIR_AIN2_IOMUX                               (IOMUX_PINCM22)
/* Defines for AIN1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_TB6612_DIR_AIN1_PORT                                        (GPIOA)
#define GPIO_TB6612_DIR_AIN1_PIN                                (DL_GPIO_PIN_12)
#define GPIO_TB6612_DIR_AIN1_IOMUX                               (IOMUX_PINCM34)
/* Defines for BIN1: GPIOB.16 with pinCMx 33 on package pin 4 */
#define GPIO_TB6612_DIR_BIN1_PORT                                        (GPIOB)
#define GPIO_TB6612_DIR_BIN1_PIN                                (DL_GPIO_PIN_16)
#define GPIO_TB6612_DIR_BIN1_IOMUX                               (IOMUX_PINCM33)
/* Defines for BIN2: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_TB6612_DIR_BIN2_PORT                                        (GPIOB)
#define GPIO_TB6612_DIR_BIN2_PIN                                 (DL_GPIO_PIN_0)
#define GPIO_TB6612_DIR_BIN2_IOMUX                               (IOMUX_PINCM12)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_TIMEG6_Motor_init(void);
void SYSCFG_DL_QEI_ENCODER_RIGHT_init(void);
void SYSCFG_DL_CAPTURE_ENCODER_LEFT_init(void);
void SYSCFG_DL_TIMER_ADC_GRAY_init(void);
void SYSCFG_DL_TIMERG12_TS_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_TJC_init(void);
void SYSCFG_DL_SPI_1_init(void);
void SYSCFG_DL_ADC12_0_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
