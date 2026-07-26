#ifndef TJC_PROTOCOL_H
#define TJC_PROTOCOL_H

#include <stdint.h>

/*
 * 串口屏发送给 MCU 的帧格式：
 * 1. 无参数命令：A5 CMD
 * 2. 模式命令：A5 CMD SUB
 * 3. 参数命令：A5 CMD LEN ASCII_DATA 0D 0A
 *
 * 例如底盘 Kp=0.2：A5 64 03 30 2E 32 0D 0A。
 */
#define TJC_FRAME_HEADER                 (0xA5u)
#define TJC_PROTOCOL_MAX_DATA_LENGTH     (32u)

/*
 * 各参数的合法范围集中定义在这里，修改协议限制时不需要改解析代码。
 * TASK_ID 和 TASK_N 当前由 uint16_t 保存，因此最大值不能超过 65535。
 */
#define TJC_TASK_ID_MIN                  (0u)
#define TJC_TASK_ID_MAX                  (4u)
#define TJC_TASK_N_MIN                   (1u)
#define TJC_TASK_N_MAX                   (5u)
#define TJC_CHASSIS_SPEED_MIN            (0.0f)
#define TJC_CHASSIS_SPEED_MAX            (1000.0f)
#define TJC_CHASSIS_PID_MIN              (0.0f)
#define TJC_CHASSIS_PID_MAX              (1000.0f)
#define TJC_BLACK_THRESHOLD_MIN          (0u)
#define TJC_BLACK_THRESHOLD_MAX          (4095u)
#define TJC_GIMBAL_PID_MIN               (-1000.0f)
#define TJC_GIMBAL_PID_MAX               (1000.0f)


#define TJC_CMD_START                    (0x01u)
#define TJC_CMD_STOP                     (0x02u)
#define TJC_CMD_RESET                    (0x03u)
#define TJC_CMD_TASK_ID                  (0x50u)
#define TJC_CMD_TASK_N                   (0x51u)
#define TJC_CMD_CHASSIS_CALIB            (0x60u)
#define TJC_CMD_CHASSIS_RESET            (0x61u)
#define TJC_CMD_CHASSIS_CLEAR_ERR        (0x62u)
#define TJC_CMD_CHASSIS_SPEED            (0x63u)
#define TJC_CMD_CHASSIS_KP               (0x64u)
#define TJC_CMD_CHASSIS_KI               (0x65u)
#define TJC_CMD_CHASSIS_KD               (0x66u)
#define TJC_CMD_BLACK_THRESHOLD          (0x67u)
#define TJC_CMD_CHASSIS_MODE             (0x6Bu)
#define TJC_CMD_GIMBAL_CALIB             (0x70u)
#define TJC_CMD_GIMBAL_RESET             (0x71u)
#define TJC_CMD_GIMBAL_MODE              (0x80u)
#define TJC_CMD_YAW_ANGLE_KP             (0x81u)
#define TJC_CMD_YAW_ANGLE_KI             (0x82u)
#define TJC_CMD_YAW_ANGLE_KD             (0x83u)
#define TJC_CMD_PITCH_ANGLE_KP           (0x84u)
#define TJC_CMD_PITCH_ANGLE_KI           (0x85u)
#define TJC_CMD_PITCH_ANGLE_KD           (0x86u)
#define TJC_CMD_YAW_SPEED_KP             (0x87u)
#define TJC_CMD_YAW_SPEED_KI             (0x88u)
#define TJC_CMD_YAW_SPEED_KD             (0x89u)
#define TJC_CMD_PITCH_SPEED_KP           (0x8Au)
#define TJC_CMD_PITCH_SPEED_KI           (0x8Bu)
#define TJC_CMD_PITCH_SPEED_KD           (0x8Cu)

#define TJC_CHASSIS_MODE_TRACE           (0x01u)
#define TJC_CHASSIS_MODE_LINE            (0x02u)
#define TJC_GIMBAL_MODE_IDLE             (0x01u)
#define TJC_GIMBAL_MODE_AUTO             (0x02u)

typedef enum
{
    TJC_PARSE_WAIT_HEADER = 0,  /* 帧外：等待 A5。 */
    TJC_PARSE_WAIT_COMMAND,     /* 已收到 A5，等待 CMD。 */
    TJC_PARSE_WAIT_SUB_COMMAND, /* 模式命令等待 SUB。 */
    TJC_PARSE_WAIT_LENGTH,      /* 参数命令等待 LEN。 */
    TJC_PARSE_WAIT_DATA,        /* 按 LEN 接收 ASCII DATA。 */
    TJC_PARSE_WAIT_CR,          /* DATA 完成，等待 0D。 */
    TJC_PARSE_WAIT_LF,          /* 已收到 0D，等待 0A。 */
    TJC_PARSE_DISCARD_DATA,     /* LEN 超限，丢弃 DATA。 */
    TJC_PARSE_DISCARD_CR,       /* 超长帧等待并丢弃 0D。 */
    TJC_PARSE_DISCARD_LF        /* 超长帧等待并丢弃 0A。 */
} TJC_ParseState_t;

/*
 * 已解析的控制数据。
 * requested 字段表示一次性动作请求，上层处理后应清零。
 * PID 和模式参数保存最近一次有效指令的值。
 */
typedef struct
{
    volatile uint8_t run_enabled;
    volatile uint8_t reset_requested;
    volatile uint8_t chassis_calib_requested;
    volatile uint8_t chassis_reset_requested;
    volatile uint8_t chassis_clear_error_requested;
    volatile uint8_t gimbal_calib_requested;
    volatile uint8_t gimbal_reset_requested;
    volatile uint8_t chassis_mode;
    volatile uint8_t gimbal_mode;
    volatile uint16_t task_id;
    volatile uint16_t lap_count;
    volatile uint16_t completed_laps;
    volatile uint8_t gimbal_task_running;
    volatile uint8_t task_finished;
    volatile uint8_t aim_power_enabled;
    volatile uint8_t aim_request;
    volatile uint8_t laser_enabled;
    volatile float chassis_speed;
    volatile float chassis_kp;
    volatile float chassis_ki;
    volatile float chassis_kd;
    volatile uint16_t black_threshold;
    volatile float yaw_angle_kp;
    volatile float yaw_angle_ki;
    volatile float yaw_angle_kd;
    volatile float pitch_angle_kp;
    volatile float pitch_angle_ki;
    volatile float pitch_angle_kd;
    volatile float yaw_speed_kp;
    volatile float yaw_speed_ki;
    volatile float yaw_speed_kd;
    volatile float pitch_speed_kp;
    volatile float pitch_speed_ki;
    volatile float pitch_speed_kd;
} TJC_Control_t;

/* 初始化解析状态机、控制数据和调试计数。 */
void TJC_Protocol_Init(void);
/* 主循环调用：读取 UART 缓冲区并解析所有待处理字节。 */
void TJC_Protocol_Process(void);

/* 修改串口屏文本控件和数值控件，obj 可包含页面前缀。 */
uint8_t TJC_SetText(const char *obj, const char *text);
uint8_t TJC_SetVal(const char *obj, int32_t value);

/* 以下变量可加入 CCS Watch，观察收帧和解析结果。 */
extern TJC_Control_t tjc_control;
extern volatile TJC_ParseState_t tjc_parse_state;
extern volatile uint8_t tjc_last_command;
extern volatile float tjc_last_value;
extern volatile uint32_t tjc_valid_frame_count;
extern volatile uint32_t tjc_invalid_frame_count;
extern volatile uint32_t tjc_unknown_command_count;

#endif
