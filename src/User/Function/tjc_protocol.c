#include "tjc_protocol.h"


#include "line_task.h"
#include "tjc_uart.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define TJC_PROTOCOL_FORMAT_BUFFER_SIZE    (128u)

/*
 * 协议解析运行在主循环中，不运行在 UART 中断中。
 * UART ISR 只把字节放入环形缓冲区，本模块再逐字节组帧、转换数值并执行命令。
 * 这样可以避免 strtof()/strtoul() 和业务函数增加中断执行时间。
 */
TJC_Control_t tjc_control;
volatile TJC_ParseState_t tjc_parse_state;
volatile uint8_t tjc_last_command;
volatile float tjc_last_value;
volatile uint32_t tjc_valid_frame_count;
volatile uint32_t tjc_invalid_frame_count;
volatile uint32_t tjc_unknown_command_count;

static uint8_t tjc_command;
static uint8_t tjc_data_length;
static uint8_t tjc_data_index;
/* 只保存参数帧的 ASCII DATA，并额外留 1 字节存放字符串结束符。 */
static char tjc_data[TJC_PROTOCOL_MAX_DATA_LENGTH + 1u];
/* 记录 UART 错误计数快照，用于在丢字节后放弃残留的半帧。 */
static uint32_t tjc_last_uart_error_count;
static uint32_t tjc_last_uart_overflow_count;

/* 判断 CMD 是否属于 A5 CMD 格式的无参数命令。 */
static uint8_t TJC_IsSimpleCommand(uint8_t command);
/* 判断 CMD 是否属于 A5 CMD SUB 格式的模式命令。 */
static uint8_t TJC_IsModeCommand(uint8_t command);
/* 判断 CMD 是否属于 A5 CMD LEN DATA 0D 0A 格式的参数命令。 */
static uint8_t TJC_IsParameterCommand(uint8_t command);
/*
 * 每次处理一个字节。tjc_parse_state 保存当前帧接收到哪一步，
 * 因此能够处理一帧被拆成多次接收，以及多帧连续到达的情况。
 */
static void TJC_ParseByte(uint8_t data);
/* 清除当前半帧，回到等待帧头 A5 的状态。 */
static void TJC_ResetParser(void);
/* 执行无参数命令；未接入业务模块的动作通过 requested 标志通知上层。 */
static void TJC_DispatchSimpleCommand(uint8_t command);
/* 校验模式 SUB，并保存合法的底盘或云台模式。 */
static void TJC_DispatchModeCommand(uint8_t command, uint8_t sub_command);
/* 将 ASCII DATA 转成整数或浮点数，校验范围后映射到对应业务参数。 */
static void TJC_DispatchParameterCommand(uint8_t command);
/* 完整解析浮点字符串，拒绝尾随字符、NaN 和无穷大。 */
static uint8_t TJC_ParseFloat(float *value);
/* 完整解析十进制无符号整数，并检查 uint16_t 上限。 */
static uint8_t TJC_ParseUint16(uint16_t *value);
/*
 * 错误恢复：清除旧帧；如果当前错误字节是 A5，则直接把它作为下一帧帧头。
 */
static void TJC_Resync(uint8_t data);
/* 按命令检查数值范围，例如 ADC 黑阈值必须位于 0~4095。 */
static uint8_t TJC_IsParameterValueValid(uint8_t command,
                                        float value,
                                        uint16_t integer_value);


/*
 * 初始化协议状态、业务参数和调试计数。
 * 必须在 Line_Task_Init() 之后调用，因为这里会读取 line_pid 的初始参数。
 */
uint8_t TJC_SetText(const char *obj, const char *text)
{
    char buffer[TJC_PROTOCOL_FORMAT_BUFFER_SIZE];
    int length;

    if ((obj == 0) || (text == 0))
    {
        return 0u;
    }

    length = snprintf(buffer, sizeof(buffer), "%s.txt=\"%s\"", obj, text);
    if ((length < 0) || ((size_t)length >= sizeof(buffer)))
    {
        return 0u;
    }

    return TJC_UART_SendCommand(buffer);
}

uint8_t TJC_SetVal(const char *obj, int32_t value)
{
    char buffer[TJC_PROTOCOL_FORMAT_BUFFER_SIZE];
    int length;

    if (obj == 0)
    {
        return 0u;
    }

    length = snprintf(buffer, sizeof(buffer), "%s.val=%ld", obj, (long)value);
    if ((length < 0) || ((size_t)length >= sizeof(buffer)))
    {
        return 0u;
    }

    return TJC_UART_SendCommand(buffer);
}
void TJC_Protocol_Init(void)
{
    tjc_control.run_enabled = 0u;
    tjc_control.reset_requested = 0u;
    tjc_control.chassis_calib_requested = 0u;
    tjc_control.chassis_reset_requested = 0u;
    tjc_control.chassis_clear_error_requested = 0u;
    tjc_control.gimbal_calib_requested = 0u;
    tjc_control.gimbal_reset_requested = 0u;
    tjc_control.chassis_mode = TJC_CHASSIS_MODE_TRACE;
    tjc_control.gimbal_mode = TJC_GIMBAL_MODE_IDLE;
    tjc_control.task_id = 0u;
    tjc_control.lap_count = TJC_TASK_N_MIN;
    tjc_control.completed_laps = 0u;
    tjc_control.gimbal_task_running = 0u;
    tjc_control.task_finished = 0u;
    tjc_control.aim_power_enabled = 0u;
    tjc_control.aim_request = 0u;
    tjc_control.laser_enabled = 0u;
    tjc_control.chassis_speed = 0.0f;
    tjc_control.chassis_kp = line_pid.kp;
    tjc_control.chassis_ki = line_pid.ki;
    tjc_control.chassis_kd = line_pid.kd;
    tjc_control.black_threshold = 0u;
    tjc_control.yaw_angle_kp = 0.0f;
    tjc_control.yaw_angle_ki = 0.0f;
    tjc_control.yaw_angle_kd = 0.0f;
    tjc_control.pitch_angle_kp = 0.0f;
    tjc_control.pitch_angle_ki = 0.0f;
    tjc_control.pitch_angle_kd = 0.0f;
    tjc_control.yaw_speed_kp = 0.0f;
    tjc_control.yaw_speed_ki = 0.0f;
    tjc_control.yaw_speed_kd = 0.0f;
    tjc_control.pitch_speed_kp = 0.0f;
    tjc_control.pitch_speed_ki = 0.0f;
    tjc_control.pitch_speed_kd = 0.0f;

    tjc_last_command = 0u;
    tjc_last_value = 0.0f;
    tjc_valid_frame_count = 0u;
    tjc_invalid_frame_count = 0u;
    tjc_unknown_command_count = 0u;
    tjc_last_uart_error_count = tjc_uart_error_count;
    tjc_last_uart_overflow_count = tjc_uart_rx_overflow_count;
    TJC_ResetParser();
}

/*
 * 主循环周期调用。
 * 检查 UART 是否发生错误，然后取出 RX 环形缓冲区中的全部字节并推进状态机。
 */
void TJC_Protocol_Process(void)
{
    uint8_t data;

    /* UART 层丢过字节时，当前协议半帧不再可信，必须重新找 A5。 */
    if ((tjc_last_uart_error_count != tjc_uart_error_count) ||
        (tjc_last_uart_overflow_count != tjc_uart_rx_overflow_count))
    {
        tjc_last_uart_error_count = tjc_uart_error_count;
        tjc_last_uart_overflow_count = tjc_uart_rx_overflow_count;
        TJC_ResetParser();
    }

    /* 一次处理完当前已进入 RX 环形缓冲区的所有字节。 */
    while (TJC_UART_ReadByte(&data))
    {
        TJC_ParseByte(data);
    }
}


/* 判断 CMD 是否属于 A5 CMD 格式的无参数命令。 */
static uint8_t TJC_IsSimpleCommand(uint8_t command)
{
    return (uint8_t)(
        (command == TJC_CMD_START) ||
        (command == TJC_CMD_STOP) ||
        (command == TJC_CMD_RESET) ||
        (command == TJC_CMD_CHASSIS_CALIB) ||
        (command == TJC_CMD_CHASSIS_RESET) ||
        (command == TJC_CMD_CHASSIS_CLEAR_ERR) ||
        (command == TJC_CMD_GIMBAL_CALIB) ||
        (command == TJC_CMD_GIMBAL_RESET));
}

/* 判断 CMD 是否属于 A5 CMD SUB 格式的模式命令。 */
static uint8_t TJC_IsModeCommand(uint8_t command)
{
    return (uint8_t)((command == TJC_CMD_CHASSIS_MODE) ||
                     (command == TJC_CMD_GIMBAL_MODE));
}

/* 判断 CMD 是否属于 A5 CMD LEN DATA 0D 0A 格式的参数命令。 */
static uint8_t TJC_IsParameterCommand(uint8_t command)
{
    return (uint8_t)(
        (command == TJC_CMD_TASK_ID) ||
        (command == TJC_CMD_TASK_N) ||
        ((command >= TJC_CMD_CHASSIS_SPEED) &&
         (command <= TJC_CMD_BLACK_THRESHOLD)) ||
        ((command >= TJC_CMD_YAW_ANGLE_KP) &&
         (command <= TJC_CMD_PITCH_SPEED_KD)));
}

/*
 * 每次处理一个字节。tjc_parse_state 保存当前帧接收到哪一步，
 * 因此能够处理一帧被拆成多次接收，以及多帧连续到达的情况。
 */
static void TJC_ParseByte(uint8_t data)
{
    switch (tjc_parse_state)
    {
            /* 帧外字节全部忽略，只有 A5 能启动一条新帧。 */
        case TJC_PARSE_WAIT_HEADER:
            if (data == TJC_FRAME_HEADER)
            {
                tjc_parse_state = TJC_PARSE_WAIT_COMMAND;
            }
            break;

            /* 根据 CMD 判断这是简单帧、模式帧还是参数帧。 */
        case TJC_PARSE_WAIT_COMMAND:
            if (data == TJC_FRAME_HEADER)
            {
                break;
            }

            tjc_command = data;
            if (TJC_IsSimpleCommand(data))
            {
                TJC_DispatchSimpleCommand(data);
                TJC_ResetParser();
            }
            else if (TJC_IsModeCommand(data))
            {
                tjc_parse_state = TJC_PARSE_WAIT_SUB_COMMAND;
            }
            else if (TJC_IsParameterCommand(data))
            {
                tjc_parse_state = TJC_PARSE_WAIT_LENGTH;
            }
            else
            {
                tjc_unknown_command_count++;
                TJC_ResetParser();
            }
            break;

            /* SUB 到达后模式帧完整，立即校验并分发。 */
        case TJC_PARSE_WAIT_SUB_COMMAND:
            if (data == TJC_FRAME_HEADER)
            {
                tjc_invalid_frame_count++;
                tjc_parse_state = TJC_PARSE_WAIT_COMMAND;
                break;
            }
            TJC_DispatchModeCommand(tjc_command, data);
            TJC_ResetParser();
            break;

            /* LEN 只表示 ASCII DATA 长度，不包含 0D 0A。 */
        case TJC_PARSE_WAIT_LENGTH:
            if (data == TJC_FRAME_HEADER)
            {
                tjc_invalid_frame_count++;
                tjc_parse_state = TJC_PARSE_WAIT_COMMAND;
                break;
            }

            tjc_data_length = data;
            tjc_data_index = 0u;
            if (tjc_data_length > TJC_PROTOCOL_MAX_DATA_LENGTH)
            {
                tjc_invalid_frame_count++;
                tjc_parse_state = TJC_PARSE_DISCARD_DATA;
            }
            else if (tjc_data_length == 0u)
            {
                tjc_parse_state = TJC_PARSE_WAIT_CR;
            }
            else
            {
                tjc_parse_state = TJC_PARSE_WAIT_DATA;
            }
            break;

            /* 按 LEN 收集 DATA，收满后补字符串结束符 '\0'。 */
        case TJC_PARSE_WAIT_DATA:
            if (data == TJC_FRAME_HEADER)
            {
                tjc_invalid_frame_count++;
                tjc_parse_state = TJC_PARSE_WAIT_COMMAND;
                break;
            }

            tjc_data[tjc_data_index++] = (char)data;
            if (tjc_data_index >= tjc_data_length)
            {
                tjc_data[tjc_data_index] = '\0';
                tjc_parse_state = TJC_PARSE_WAIT_CR;
            }
            break;

            /* 参数 DATA 后必须是回车 0D，否则当前帧无效。 */
        case TJC_PARSE_WAIT_CR:
            if (data == '\r')
            {
                tjc_parse_state = TJC_PARSE_WAIT_LF;
            }
            else
            {
                tjc_invalid_frame_count++;
                TJC_Resync(data);
            }
            break;

            /* 只有继续收到换行 0A，才执行参数命令。 */
        case TJC_PARSE_WAIT_LF:
            if (data == '\n')
            {
                TJC_DispatchParameterCommand(tjc_command);
            }
            else
            {
                tjc_invalid_frame_count++;
                TJC_Resync(data);
                break;
            }
            TJC_ResetParser();
            break;

            /* LEN 超过缓冲区时只消费字节，不向 tjc_data 写入。 */
        case TJC_PARSE_DISCARD_DATA:
            if (data == TJC_FRAME_HEADER)
            {
                tjc_parse_state = TJC_PARSE_WAIT_COMMAND;
                break;
            }
            tjc_data_index++;
            if (tjc_data_index >= tjc_data_length)
            {
                tjc_parse_state = TJC_PARSE_DISCARD_CR;
            }
            break;

            /* 超长帧 DATA 消费完后继续等待其 0D。 */
        case TJC_PARSE_DISCARD_CR:
            if (data == '\r')
            {
                tjc_parse_state = TJC_PARSE_DISCARD_LF;
            }
            else
            {
                TJC_Resync(data);
            }
            break;

            /* 消费超长帧末尾 0A，然后重新等待下一帧。 */
        case TJC_PARSE_DISCARD_LF:
            if (data == '\n')
            {
                TJC_ResetParser();
            }
            else
            {
                TJC_Resync(data);
            }
            break;

        default:
            TJC_ResetParser();
            break;
    }
}

/* 清除当前半帧，回到等待帧头 A5 的状态。 */
static void TJC_ResetParser(void)
{
    tjc_parse_state = TJC_PARSE_WAIT_HEADER;
    tjc_command = 0u;
    tjc_data_length = 0u;
    tjc_data_index = 0u;
    tjc_data[0] = '\0';
}

/* 执行无参数命令；未接入业务模块的动作通过 requested 标志通知上层。 */
static void TJC_DispatchSimpleCommand(uint8_t command)
{
    tjc_last_command = command;
    tjc_valid_frame_count++;

    /* 将解析结果写入对应业务变量；有现成接口的命令会立即执行。 */
    switch (command)
    {
        case TJC_CMD_START:
            /* 协议层只记录运行请求，具体任务由 Task 层决定。 */
            tjc_control.run_enabled = 1u;
            tjc_control.task_finished = 0u;
            break;
        case TJC_CMD_STOP:
            tjc_control.run_enabled = 0u;
            break;
        case TJC_CMD_RESET:
            tjc_control.reset_requested = 1u;
            break;
        case TJC_CMD_CHASSIS_CALIB:
            tjc_control.chassis_calib_requested = 1u;
            break;
        case TJC_CMD_CHASSIS_RESET:
            tjc_control.chassis_reset_requested = 1u;
            break;
        case TJC_CMD_CHASSIS_CLEAR_ERR:
            tjc_control.chassis_clear_error_requested = 1u;
            break;
        case TJC_CMD_GIMBAL_CALIB:
            tjc_control.gimbal_calib_requested = 1u;
            break;
        case TJC_CMD_GIMBAL_RESET:
            tjc_control.gimbal_reset_requested = 1u;
            break;
        default:
            break;
    }
}

/* 校验模式 SUB，并保存合法的底盘或云台模式。 */
static void TJC_DispatchModeCommand(uint8_t command, uint8_t sub_command)
{
    uint8_t valid = 0u;

    if ((command == TJC_CMD_CHASSIS_MODE) &&
        ((sub_command == TJC_CHASSIS_MODE_TRACE) ||
         (sub_command == TJC_CHASSIS_MODE_LINE)))
    {
        tjc_control.chassis_mode = sub_command;
        valid = 1u;
    }
    else if ((command == TJC_CMD_GIMBAL_MODE) &&
             ((sub_command == TJC_GIMBAL_MODE_IDLE) ||
              (sub_command == TJC_GIMBAL_MODE_AUTO)))
    {
        tjc_control.gimbal_mode = sub_command;
        valid = 1u;
    }

    if (valid)
    {
        tjc_last_command = command;
        tjc_last_value = (float)sub_command;
        tjc_valid_frame_count++;
    }
    else
    {
        tjc_invalid_frame_count++;
    }
}

/* 将 ASCII DATA 转成整数或浮点数，校验范围后映射到对应业务参数。 */
static void TJC_DispatchParameterCommand(uint8_t command)
{
    float value;
    uint16_t integer_value = 0u;

    /* 任务号、圈数、黑阈值按整数解析，速度和 PID 按浮点数解析。 */
    if ((command == TJC_CMD_TASK_ID) ||
        (command == TJC_CMD_TASK_N) ||
        (command == TJC_CMD_BLACK_THRESHOLD))
    {
        if (!TJC_ParseUint16(&integer_value))
        {
            tjc_invalid_frame_count++;
            return;
        }
        value = (float)integer_value;
    }
    else if (!TJC_ParseFloat(&value))
    {
        tjc_invalid_frame_count++;
        return;
    }

    if (!TJC_IsParameterValueValid(command, value, integer_value))
    {
        tjc_invalid_frame_count++;
        return;
    }

    /* 将解析结果写入对应业务变量；有现成接口的命令会立即执行。 */
    switch (command)
    {
        case TJC_CMD_TASK_ID:
            tjc_control.task_id = integer_value;
            tjc_control.completed_laps = 0u;
            tjc_control.task_finished = 0u;
            if (tjc_control.task_id == 0u)
            {
                /* 任务 0 表示空闲，Task 层会据此停止底盘。 */
                tjc_control.run_enabled = 0u;
            }
            break;
        case TJC_CMD_TASK_N: tjc_control.lap_count = integer_value; break;
        case TJC_CMD_CHASSIS_SPEED: tjc_control.chassis_speed = value; break;
        case TJC_CMD_CHASSIS_KP:
            tjc_control.chassis_kp = value;
            /* 在线调参后清积分，避免旧积分在新参数下造成输出突跳。 */
            line_pid.kp = value;
            PID_ClearIntegral(&line_pid);
            break;
        case TJC_CMD_CHASSIS_KI:
            tjc_control.chassis_ki = value;
            line_pid.ki = value;
            PID_ClearIntegral(&line_pid);
            break;
        case TJC_CMD_CHASSIS_KD:
            tjc_control.chassis_kd = value;
            line_pid.kd = value;
            PID_ClearIntegral(&line_pid);
            break;
        case TJC_CMD_BLACK_THRESHOLD:
            tjc_control.black_threshold = integer_value;
            break;
        case TJC_CMD_YAW_ANGLE_KP: tjc_control.yaw_angle_kp = value; break;
        case TJC_CMD_YAW_ANGLE_KI: tjc_control.yaw_angle_ki = value; break;
        case TJC_CMD_YAW_ANGLE_KD: tjc_control.yaw_angle_kd = value; break;
        case TJC_CMD_PITCH_ANGLE_KP: tjc_control.pitch_angle_kp = value; break;
        case TJC_CMD_PITCH_ANGLE_KI: tjc_control.pitch_angle_ki = value; break;
        case TJC_CMD_PITCH_ANGLE_KD: tjc_control.pitch_angle_kd = value; break;
        case TJC_CMD_YAW_SPEED_KP: tjc_control.yaw_speed_kp = value; break;
        case TJC_CMD_YAW_SPEED_KI: tjc_control.yaw_speed_ki = value; break;
        case TJC_CMD_YAW_SPEED_KD: tjc_control.yaw_speed_kd = value; break;
        case TJC_CMD_PITCH_SPEED_KP: tjc_control.pitch_speed_kp = value; break;
        case TJC_CMD_PITCH_SPEED_KI: tjc_control.pitch_speed_ki = value; break;
        case TJC_CMD_PITCH_SPEED_KD: tjc_control.pitch_speed_kd = value; break;
        default:
            tjc_unknown_command_count++;
            return;
    }

    tjc_last_command = command;
    tjc_last_value = value;
    tjc_valid_frame_count++;
}

/* 完整解析浮点字符串，拒绝尾随字符、NaN 和无穷大。 */
static uint8_t TJC_ParseFloat(float *value)
{
    char *end_ptr;
    float parsed_value = strtof(tjc_data, &end_ptr);

    if ((end_ptr == tjc_data) || (*end_ptr != '\0') ||
        !isfinite(parsed_value))
    {
        return 0u;
    }

    *value = parsed_value;
    return 1u;
}

/* 完整解析十进制无符号整数，并检查 uint16_t 上限。 */
static uint8_t TJC_ParseUint16(uint16_t *value)
{
    char *end_ptr;
    unsigned long parsed_value = strtoul(tjc_data, &end_ptr, 10);

    if ((end_ptr == tjc_data) || (*end_ptr != '\0') ||
        (parsed_value > 65535ul))
    {
        return 0u;
    }

    *value = (uint16_t)parsed_value;
    return 1u;
}

/*
 * 错误恢复：清除旧帧；如果当前错误字节是 A5，则直接把它作为下一帧帧头。
 */
static void TJC_Resync(uint8_t data)
{
    TJC_ResetParser();
    if (data == TJC_FRAME_HEADER)
    {
        tjc_parse_state = TJC_PARSE_WAIT_COMMAND;
    }
}

/* 按命令检查数值范围，例如 ADC 黑阈值必须位于 0~4095。 */
static uint8_t TJC_IsParameterValueValid(uint8_t command,
                                        float value,
                                        uint16_t integer_value)
{
    switch (command)
    {
        case TJC_CMD_TASK_ID:
            return (uint8_t)((integer_value >= TJC_TASK_ID_MIN) &&
                             (integer_value <= TJC_TASK_ID_MAX));
        case TJC_CMD_TASK_N:
            return (uint8_t)((integer_value >= TJC_TASK_N_MIN) &&
                             (integer_value <= TJC_TASK_N_MAX));
        case TJC_CMD_CHASSIS_SPEED:
            return (uint8_t)((value >= TJC_CHASSIS_SPEED_MIN) &&
                             (value <= TJC_CHASSIS_SPEED_MAX));
        case TJC_CMD_CHASSIS_KP:
        case TJC_CMD_CHASSIS_KI:
        case TJC_CMD_CHASSIS_KD:
            return (uint8_t)((value >= TJC_CHASSIS_PID_MIN) &&
                             (value <= TJC_CHASSIS_PID_MAX));
        case TJC_CMD_BLACK_THRESHOLD:
            return (uint8_t)((integer_value >= TJC_BLACK_THRESHOLD_MIN) &&
                             (integer_value <= TJC_BLACK_THRESHOLD_MAX));
        case TJC_CMD_YAW_ANGLE_KP:
        case TJC_CMD_YAW_ANGLE_KI:
        case TJC_CMD_YAW_ANGLE_KD:
        case TJC_CMD_PITCH_ANGLE_KP:
        case TJC_CMD_PITCH_ANGLE_KI:
        case TJC_CMD_PITCH_ANGLE_KD:
        case TJC_CMD_YAW_SPEED_KP:
        case TJC_CMD_YAW_SPEED_KI:
        case TJC_CMD_YAW_SPEED_KD:
        case TJC_CMD_PITCH_SPEED_KP:
        case TJC_CMD_PITCH_SPEED_KI:
        case TJC_CMD_PITCH_SPEED_KD:
            return (uint8_t)((value >= TJC_GIMBAL_PID_MIN) &&
                             (value <= TJC_GIMBAL_PID_MAX));
        default:
            return 0u;
    }
}
