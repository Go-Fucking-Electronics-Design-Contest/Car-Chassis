#include "tjc_uart.h"

#include "ti_msp_dl_config.h"
#include <string.h>


static volatile uint8_t tjc_rx_buffer[TJC_UART_RX_BUFFER_SIZE];
static volatile uint16_t tjc_rx_head;
static volatile uint16_t tjc_rx_tail;

static volatile uint8_t tjc_tx_buffer[TJC_UART_TX_BUFFER_SIZE];
static volatile uint16_t tjc_tx_head;
static volatile uint16_t tjc_tx_tail;

volatile uint32_t tjc_uart_rx_byte_count;
volatile uint32_t tjc_uart_rx_overflow_count;
volatile uint32_t tjc_uart_tx_byte_count;
volatile uint32_t tjc_uart_tx_overflow_count;
volatile uint32_t tjc_uart_error_count;

static uint16_t TJC_RingNext(uint16_t index, uint16_t size);
static uint16_t TJC_TxFreeUnsafe(void);
static uint8_t TJC_UART_SendRawCommand(const uint8_t *cmd, uint16_t len);
static void TJC_UART_StartTx(void);
static void TJC_UART_PushRxByte(uint8_t data);

void TJC_UART_Init(void)
{
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();

    tjc_rx_head = 0u;
    tjc_rx_tail = 0u;
    tjc_tx_head = 0u;
    tjc_tx_tail = 0u;

    tjc_uart_rx_byte_count = 0u;
    tjc_uart_rx_overflow_count = 0u;
    tjc_uart_tx_byte_count = 0u;
    tjc_uart_tx_overflow_count = 0u;
    tjc_uart_error_count = 0u;

    __set_PRIMASK(primask);

    DL_UART_Main_disableInterrupt(UART_TJC_INST, DL_UART_MAIN_INTERRUPT_TX);
    DL_UART_Main_clearInterruptStatus(UART_TJC_INST,
        DL_UART_MAIN_INTERRUPT_RX |
        DL_UART_MAIN_INTERRUPT_TX |
        DL_UART_MAIN_INTERRUPT_OVERRUN_ERROR |
        DL_UART_MAIN_INTERRUPT_FRAMING_ERROR);

    NVIC_ClearPendingIRQ(UART_TJC_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_TJC_INST_INT_IRQN);
}

uint8_t TJC_UART_SendCommand(const char *cmd)
{
    size_t len;

    if (cmd == 0)
    {
        return 0u;
    }

    len = strlen(cmd);
    if (len > (TJC_UART_TX_BUFFER_SIZE - 4u))
    {
        tjc_uart_tx_overflow_count++;
        return 0u;
    }

    return TJC_UART_SendRawCommand((const uint8_t *)cmd, (uint16_t)len);
}

static uint8_t TJC_UART_SendRawCommand(const uint8_t *cmd, uint16_t len)
{
    static const uint8_t end_code[3] = {0xFFu, 0xFFu, 0xFFu};
    uint32_t primask;
    uint16_t i;

    if ((cmd == 0) || (len == 0u))
    {
        return 0u;
    }

    if (len > (TJC_UART_TX_BUFFER_SIZE - 4u))
    {
        tjc_uart_tx_overflow_count++;
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (TJC_TxFreeUnsafe() < (uint16_t)(len + sizeof(end_code)))
    {
        tjc_uart_tx_overflow_count++;
        __set_PRIMASK(primask);
        return 0u;
    }

    for (i = 0u; i < len; i++)
    {
        tjc_tx_buffer[tjc_tx_head] = cmd[i];
        tjc_tx_head = TJC_RingNext(tjc_tx_head, TJC_UART_TX_BUFFER_SIZE);
    }

    for (i = 0u; i < (uint16_t)sizeof(end_code); i++)
    {
        tjc_tx_buffer[tjc_tx_head] = end_code[i];
        tjc_tx_head = TJC_RingNext(tjc_tx_head, TJC_UART_TX_BUFFER_SIZE);
    }

    __set_PRIMASK(primask);
    TJC_UART_StartTx();

    return 1u;
}

uint8_t TJC_UART_ReadByte(uint8_t *data)
{
    uint32_t primask;

    if (data == 0)
    {
        return 0u;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    if (tjc_rx_head == tjc_rx_tail)
    {
        __set_PRIMASK(primask);
        return 0u;
    }

    *data = tjc_rx_buffer[tjc_rx_tail];
    tjc_rx_tail = TJC_RingNext(tjc_rx_tail, TJC_UART_RX_BUFFER_SIZE);

    __set_PRIMASK(primask);

    return 1u;
}

static uint16_t TJC_RingNext(uint16_t index, uint16_t size)
{
    index++;
    if (index >= size)
    {
        index = 0u;
    }

    return index;
}

static uint16_t TJC_TxFreeUnsafe(void)
{
    uint16_t used;

    if (tjc_tx_head >= tjc_tx_tail)
    {
        used = (uint16_t)(tjc_tx_head - tjc_tx_tail);
    }
    else
    {
        used = (uint16_t)(TJC_UART_TX_BUFFER_SIZE - tjc_tx_tail + tjc_tx_head);
    }

    return (uint16_t)(TJC_UART_TX_BUFFER_SIZE - used - 1u);
}

static void TJC_UART_StartTx(void)
{
    DL_UART_Main_enableInterrupt(UART_TJC_INST, DL_UART_MAIN_INTERRUPT_TX);
}

static void TJC_UART_PushRxByte(uint8_t data)
{
    uint16_t next_head;

    next_head = TJC_RingNext(tjc_rx_head, TJC_UART_RX_BUFFER_SIZE);
    if (next_head == tjc_rx_tail)
    {
        tjc_uart_rx_overflow_count++;
        tjc_rx_tail = tjc_rx_head;
    }

    tjc_rx_buffer[tjc_rx_head] = data;
    tjc_rx_head = next_head;
    tjc_uart_rx_byte_count++;
}

void UART_TJC_INST_IRQHandler(void)
{
    DL_UART_IIDX interrupt_status;

    do
    {
        interrupt_status = DL_UART_Main_getPendingInterrupt(UART_TJC_INST);

        switch (interrupt_status)
        {
            case DL_UART_MAIN_IIDX_RX:
            {
                while (!DL_UART_Main_isRXFIFOEmpty(UART_TJC_INST))
                {
                    TJC_UART_PushRxByte(DL_UART_Main_receiveData(UART_TJC_INST));
                }
                break;
            }

            case DL_UART_MAIN_IIDX_TX:
            {
                while ((!DL_UART_Main_isTXFIFOFull(UART_TJC_INST)) &&
                       (tjc_tx_tail != tjc_tx_head))
                {
                    DL_UART_Main_transmitData(UART_TJC_INST,
                        tjc_tx_buffer[tjc_tx_tail]);
                    tjc_tx_tail = TJC_RingNext(tjc_tx_tail, TJC_UART_TX_BUFFER_SIZE);
                    tjc_uart_tx_byte_count++;
                }

                if (tjc_tx_tail == tjc_tx_head)
                {
                    DL_UART_Main_disableInterrupt(UART_TJC_INST,
                        DL_UART_MAIN_INTERRUPT_TX);
                }
                break;
            }

            case DL_UART_MAIN_IIDX_OVERRUN_ERROR:
            case DL_UART_MAIN_IIDX_FRAMING_ERROR:
            {
                while (!DL_UART_Main_isRXFIFOEmpty(UART_TJC_INST))
                {
                    (void)DL_UART_Main_receiveData(UART_TJC_INST);
                }
                tjc_rx_tail = tjc_rx_head;
                tjc_uart_error_count++;
                break;
            }

            default:
                break;
        }
    }
    while (interrupt_status != DL_UART_MAIN_IIDX_NO_INTERRUPT);
}
