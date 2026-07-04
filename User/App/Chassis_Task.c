#include "Chassis_Task.h"

#include "can.h"

#include <math.h>
#include <stddef.h>

extern DBUS_Typedef WHW_V_DBUS;

#define SWERVE_MODULE_NUM                4U
#define SWERVE_PI                        3.14159265358979323846f
#define SWERVE_TWO_PI                    (2.0f * SWERVE_PI)
#define SWERVE_HALF_PI                   (0.5f * SWERVE_PI)

#define GM6020_ENCODER_RESOLUTION        8192.0f

#define CHASSIS_LENGTH_M                 0.400f
#define CHASSIS_WIDTH_M                  0.400f
#define CHASSIS_WHEEL_RADIUS_M           0.076f

#define CHASSIS_DRIVE_GEAR_RATIO         19.0f  //原来�?9.0f
#define CHASSIS_STEER_GEAR_RATIO         1.0f

#define CHASSIS_MAX_VX_MPS               1.0f
#define CHASSIS_MAX_VY_MPS               1.0f
#define CHASSIS_MAX_WZ_RADPS             2.0f
#define CHASSIS_MAX_WHEEL_SPEED_MPS      1.5f

#define CHASSIS_TRANSLATION_CMD_MAX      660.0f
#define CHASSIS_ROTATION_CMD_MAX         660.0f
#define CHASSIS_INPUT_DEADBAND           10.0f

#define SWERVE_STOP_EPSILON_MPS          0.001f
#define SWERVE_ENABLE_SWITCH_VALUE       3U

#define SWERVE_STEER_POSITION_LIMIT_RPM  350.0f
#define SWERVE_STEER_CURRENT_LIMIT       16000.0f
#define SWERVE_STEER_INTEGRAL_LIMIT      4000.0f
#define SWERVE_STEER_START_CURRENT       5000.0f
#define SWERVE_STEER_START_SPEED_RPM     8.0f
#define SWERVE_STEER_START_ERROR_RAD     0.08f

#define SWERVE_DRIVE_CURRENT_LIMIT       16000.0f
#define SWERVE_DRIVE_INTEGRAL_LIMIT      4000.0f

typedef enum
{
    SWERVE_FL = 0,
    SWERVE_FR = 1,
    SWERVE_RL = 2,
    SWERVE_RR = 3
} Swerve_Module_ID_e;

typedef struct
{
    DJI_MOTOR_Typedef *drive;
    DJI_MOTOR_Typedef *steer;
} Swerve_Motor_Map_T;

typedef struct
{
    float steer_target_count[SWERVE_MODULE_NUM];
    float steer_error_rad[SWERVE_MODULE_NUM];
    float drive_target_rpm[SWERVE_MODULE_NUM];
} Swerve_Target_Watch_T;

typedef struct
{
    float module_vx[SWERVE_MODULE_NUM];
    float module_vy[SWERVE_MODULE_NUM];
    float module_speed[SWERVE_MODULE_NUM];
} Swerve_Runtime_T;

static const float module_x[SWERVE_MODULE_NUM] =
{
     CHASSIS_LENGTH_M * 0.5f,
     CHASSIS_LENGTH_M * 0.5f,
    -CHASSIS_LENGTH_M * 0.5f,
    -CHASSIS_LENGTH_M * 0.5f
};

static const float module_y[SWERVE_MODULE_NUM] =
{
     CHASSIS_WIDTH_M * 0.5f,
    -CHASSIS_WIDTH_M * 0.5f,
     CHASSIS_WIDTH_M * 0.5f,
    -CHASSIS_WIDTH_M * 0.5f
};

volatile float swerve_chassis_vx = 0.0f;
volatile float swerve_chassis_vy = 0.0f;
volatile float swerve_chassis_wz = 0.0f;

volatile uint8_t swerve_zero_ready[SWERVE_MODULE_NUM] = {0U};
volatile uint8_t swerve_steer_online[SWERVE_MODULE_NUM] = {0U};
volatile uint8_t swerve_drive_online[SWERVE_MODULE_NUM] = {0U};

volatile float swerve_steer_direction[SWERVE_MODULE_NUM] =
{
    1.0f, 1.0f, 1.0f, 1.0f
};

volatile float swerve_steer_output_sign[SWERVE_MODULE_NUM] =
{
    1.0f, 1.0f, 1.0f, 1.0f
};

volatile float swerve_drive_direction[SWERVE_MODULE_NUM] =
{
    1.0f, -1.0f, 1.0f, -1.0f    //原来是1.0f, 1.0f, -1.0f, -1.0f,这个是6月28日最新的
};
//在这里尝试提交一下
volatile float swerve_steer_zero_correction[SWERVE_MODULE_NUM] =
{
    0.0f,  /* FL */
    0.0f,  /* FR */
    0.0f,  /* RL */
    0.0f   /* RR */
};

volatile float meiyouyongdetijiao[3]={
    1.0f,
    2.0f,
    3.0f
    
};







volatile Swerve_Target_Watch_T Swerve_Target = {0};
volatile int16_t swerve_steer_current_cmd[SWERVE_MODULE_NUM] = {0};
volatile int16_t swerve_drive_current_cmd[SWERVE_MODULE_NUM] = {0};

volatile float swerve_zero_count[SWERVE_MODULE_NUM] = {0.0f};
volatile float swerve_last_target_count[SWERVE_MODULE_NUM] = {0.0f};



//新写的。。。。。。。。。。。。。。。。。。。。。。。�?
/*
 * 四个舵轮机械朝正前方时，GM6020 �?Angle_now�? * 顺序：FL、FR、RL、RR�? *
 * 下面数字先只是占位，必须换成实测值�? */
volatile int32_t swerve_steer_zero_single_count[SWERVE_MODULE_NUM] =
{
    5564,  /* FL：物理左前，反馈0x206 */
    1509,  /* FR：物理右前，反馈0x205 */   //原来5324, 1460, 1101, 5467
    1323,  /* RL：物理左后，反馈0x207 */
    5401   /* RR：物理右后，反馈0x208 */
};
//新写的。。。。。。。。。。。。。。。。。。。。。。。�?



















static Swerve_Runtime_T swerve_runtime = {0};

static void Chassis_BuildMotorMap(MOTOR_Typdef *MOTOR,
                                  Swerve_Motor_Map_T map[SWERVE_MODULE_NUM]);
static void Chassis_ResetController(DJI_MOTOR_Typedef *motor);
static void Chassis_UpdateOnline(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM]);
static void Chassis_CaptureSteerZero(
    const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM]);
static void Chassis_ReadCommand(const CONTAL_Typedef *CONTAL);
static void Chassis_SolveTargets(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM]);
static void Chassis_RunSteerControllers(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM]);
static void Chassis_RunDriveControllers(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM]);
static void Chassis_StopAllMotors(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM]);
static void Chassis_SendMotorCurrents(void);
static float Chassis_ApplyDeadband(float value, float deadband);
static float Chassis_LimitFloat(float value, float min_value, float max_value);
static int16_t Chassis_LimitCurrent(float current, float limit);
static float Chassis_WrapToPi(float angle_rad);

static float Chassis_GetNearestZeroCount(
    float current_count,
    float zero_single_count);

uint8_t MOTOR_PID_Chassis_INIT(MOTOR_Typdef *MOTOR)
{
    Swerve_Motor_Map_T map[SWERVE_MODULE_NUM];
    float drive_feedforward[3] =
    {
        1.0f,
        0.0f,
        0.0f
    };
    float drive_speed_pid[3] =
    {
        2.0f,
        0.0f,
        0.0f
    };
    float steer_position_pid[3] =
    {
        1.5f,//原来 1.0f
        0.0f,
        0.0f
    };
    float steer_speed_pid[3] =
    {
        25.0f,//原来 18.0f
        0.03f,//原来 0.03f
        0.0f
    };

    if (MOTOR == NULL)
    {
        return RUI_DF_ERROR;
    }

    Chassis_BuildMotorMap(MOTOR, map);

    for (uint8_t i = 0U; i < SWERVE_MODULE_NUM; i++)
    {
        Feedforward_Init(&map[i].drive->PID_F,
                         SWERVE_DRIVE_CURRENT_LIMIT,
                         drive_feedforward,
                         0.5f,
                         0,
                         0);

        PID_Init(&map[i].drive->PID_S,
                 SWERVE_DRIVE_CURRENT_LIMIT,
                 SWERVE_DRIVE_INTEGRAL_LIMIT,
                 drive_speed_pid,
                 1000.0f,
                 1000.0f,
                 0.0f,
                 0.0f,
                 0,
                 Integral_Limit);

        PID_Init(&map[i].steer->PID_P,
                 SWERVE_STEER_POSITION_LIMIT_RPM,
                 0.0f,
                 steer_position_pid,
                 1000.0f,
                 1000.0f,
                 0.0f,
                 0.0f,
                 0,
                 NONE);

        PID_Init(&map[i].steer->PID_S,
                 SWERVE_STEER_CURRENT_LIMIT,
                 SWERVE_STEER_INTEGRAL_LIMIT,
                 steer_speed_pid,
                 1000.0f,
                 1000.0f,
                 0.0f,
                 0.0f,
                 0,
                 Integral_Limit);

        map[i].drive->PID_INIT = RUI_DF_READY;
        map[i].steer->PID_INIT = RUI_DF_READY;
    }

    return RUI_DF_READY;
}

uint8_t Chassis_AIM_INIT(RUI_ROOT_STATUS_Typedef *Root, MOTOR_Typdef *MOTOR)
{
    (void)Root;
    (void)MOTOR;

    return RUI_DF_READY;
}

uint8_t chassis_task(CONTAL_Typedef *CONTAL,
                     RUI_ROOT_STATUS_Typedef *Root,
                     User_Data_T *User_data,
                     model_t *model,
                     CAP_RXDATA *CAP_GET,
                     MOTOR_Typdef *MOTOR)
{
    static uint8_t pid_initialized = RUI_DF_ERROR;
    Swerve_Motor_Map_T map[SWERVE_MODULE_NUM];

    (void)User_data;
    (void)model;
    (void)CAP_GET;

    if (CONTAL == NULL || Root == NULL || MOTOR == NULL)
    {
        return RUI_DF_ERROR;
    }

    Chassis_BuildMotorMap(MOTOR, map);

    if (pid_initialized != RUI_DF_READY)
    {
        pid_initialized = MOTOR_PID_Chassis_INIT(MOTOR);
        if (pid_initialized != RUI_DF_READY)
        {
            Chassis_StopAllMotors(map);
            return RUI_DF_ERROR;
        }
    }

    Chassis_UpdateOnline(map);
    Chassis_CaptureSteerZero(map);

    if ((Root->RM_DBUS == RUI_DF_OFFLINE) ||
        (WHW_V_DBUS.Remote.S1_u8 != SWERVE_ENABLE_SWITCH_VALUE))
    {
        Chassis_StopAllMotors(map);
        return RUI_DF_READY;
    }

    Chassis_ReadCommand(CONTAL);
    Chassis_SolveTargets(map);
    Chassis_RunSteerControllers(map);
    Chassis_RunDriveControllers(map);
    Chassis_SendMotorCurrents();

    return RUI_DF_READY;
}

static void Chassis_BuildMotorMap(MOTOR_Typdef *MOTOR,
                                  Swerve_Motor_Map_T map[SWERVE_MODULE_NUM])
{
    map[SWERVE_FL].drive = &MOTOR->DJI_3508_Chassis_1;
    map[SWERVE_FR].drive = &MOTOR->DJI_3508_Chassis_2;
    map[SWERVE_RL].drive = &MOTOR->DJI_3508_Chassis_3;
    map[SWERVE_RR].drive = &MOTOR->DJI_3508_Chassis_4;


    
    map[SWERVE_FL].steer = &MOTOR->DJI_6020_Chassis_FL;
    map[SWERVE_FR].steer = &MOTOR->DJI_6020_Chassis_FR;
    map[SWERVE_RL].steer = &MOTOR->DJI_6020_Chassis_RL;
    map[SWERVE_RR].steer = &MOTOR->DJI_6020_Chassis_RR;
}

static void Chassis_ResetController(DJI_MOTOR_Typedef *motor)
{
    if (motor == NULL)
    {
        return;
    }

    motor->PID_F.Output = 0.0f;
    motor->PID_P.Iout = 0.0f;
    motor->PID_P.ITerm = 0.0f;
    motor->PID_P.Output = 0.0f;
    motor->PID_S.Iout = 0.0f;
    motor->PID_S.ITerm = 0.0f;
    motor->PID_S.Output = 0.0f;
}

static void Chassis_UpdateOnline(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM])
{
    for (uint8_t i = 0U; i < SWERVE_MODULE_NUM; i++)
    {
        swerve_steer_online[i] =
            (map[i].steer->DATA.ONLINE_JUDGE_TIME != 0U) ? 1U : 0U;
        swerve_drive_online[i] =
            (map[i].drive->DATA.ONLINE_JUDGE_TIME != 0U) ? 1U : 0U;
    }
}

static void Chassis_CaptureSteerZero(
    const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM])
{
    for (uint8_t i = 0U; i < SWERVE_MODULE_NUM; i++)
    {
        if ((swerve_steer_online[i] != 0U) &&
            (swerve_zero_ready[i] == 0U))
        {
            float current_count =
                (float)map[i].steer->DATA.Angle_Infinite;

            /* Expand the fixed mechanical zero to the nearest turn. */
            swerve_zero_count[i] =
                Chassis_GetNearestZeroCount(
                    current_count,
                    (float)swerve_steer_zero_single_count[i]);

            /* Hold the current position until a motion command arrives. */
            swerve_last_target_count[i] = current_count;
            Swerve_Target.steer_target_count[i] = current_count;
            Swerve_Target.steer_error_rad[i] = 0.0f;

            swerve_zero_ready[i] = 1U;
        }
    }
}

static void Chassis_ReadCommand(const CONTAL_Typedef *CONTAL)
{
    float raw_vx;
    float raw_vy;
    float raw_wz;

    if (CONTAL == NULL)
    {
        swerve_chassis_vx = 0.0f;
        swerve_chassis_vy = 0.0f;
        swerve_chassis_wz = 0.0f;
        return;
    }

    /* 标定模式：固定要求四个舵轮朝正前方 */





    raw_vx = Chassis_ApplyDeadband(CONTAL->BOTTOM.VY,
                                   CHASSIS_INPUT_DEADBAND);
    raw_vy = Chassis_ApplyDeadband(-CONTAL->BOTTOM.VX,
                                   CHASSIS_INPUT_DEADBAND);
    raw_wz = Chassis_ApplyDeadband(CONTAL->BOTTOM.VW,
                                   CHASSIS_INPUT_DEADBAND);

    swerve_chassis_vx =
        Chassis_LimitFloat(raw_vx / CHASSIS_TRANSLATION_CMD_MAX,
                           -1.0f,
                            1.0f) * CHASSIS_MAX_VX_MPS;
    swerve_chassis_vy =
        Chassis_LimitFloat(raw_vy / CHASSIS_TRANSLATION_CMD_MAX,
                           -1.0f,
                            1.0f) * CHASSIS_MAX_VY_MPS;
    swerve_chassis_wz =
        Chassis_LimitFloat(raw_wz / CHASSIS_ROTATION_CMD_MAX,
                           -1.0f,
                            1.0f) * CHASSIS_MAX_WZ_RADPS;
}

static void Chassis_SolveTargets(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM])
{
    float max_speed = 0.0f;
    float speed_scale = 1.0f;

    for (uint8_t i = 0U; i < SWERVE_MODULE_NUM; i++)
    {
        swerve_runtime.module_vx[i] =
            swerve_chassis_vx -
            swerve_chassis_wz * module_y[i];
        swerve_runtime.module_vy[i] =
            swerve_chassis_vy +
            swerve_chassis_wz * module_x[i];
        swerve_runtime.module_speed[i] =
            sqrtf(swerve_runtime.module_vx[i] *
                  swerve_runtime.module_vx[i] +
                  swerve_runtime.module_vy[i] *
                  swerve_runtime.module_vy[i]);

        if (swerve_runtime.module_speed[i] > max_speed)
        {
            max_speed = swerve_runtime.module_speed[i];
        }
    }

    if (max_speed > CHASSIS_MAX_WHEEL_SPEED_MPS)
    {
        speed_scale = CHASSIS_MAX_WHEEL_SPEED_MPS / max_speed;
    }

    for (uint8_t i = 0U; i < SWERVE_MODULE_NUM; i++)
    {
        float drive_speed = 0.0f;
        float current_count;
        float current_angle;
        float target_count;
        float target_error_rad = 0.0f;

        swerve_runtime.module_speed[i] *= speed_scale;
        target_count = swerve_last_target_count[i];

        if ((swerve_steer_online[i] != 0U) &&
            (swerve_zero_ready[i] != 0U))
        {
            current_count =
                (float)map[i].steer->DATA.Angle_Infinite;

            if (swerve_runtime.module_speed[i] >=
                SWERVE_STOP_EPSILON_MPS)
            {
                float raw_target_angle;
                float angle_error;

                current_angle =
                    (current_count - swerve_zero_count[i]) *
                    SWERVE_TWO_PI /
                    (GM6020_ENCODER_RESOLUTION *
                     CHASSIS_STEER_GEAR_RATIO) *
                    swerve_steer_direction[i];

                raw_target_angle =
                    atan2f(swerve_runtime.module_vy[i],
                           swerve_runtime.module_vx[i]);

                angle_error =
                    Chassis_WrapToPi(raw_target_angle -
                                     current_angle);

                drive_speed = swerve_runtime.module_speed[i];
                        //90°优化下面
                        
                if (angle_error > SWERVE_HALF_PI)
                {
                    angle_error -= SWERVE_PI;
                    drive_speed = -drive_speed;
                }
                else if (angle_error < -SWERVE_HALF_PI)
                {
                    angle_error += SWERVE_PI;
                    drive_speed = -drive_speed;
                }



                drive_speed *= cosf(angle_error);

                target_count =
                    current_count +
                    angle_error *
                    GM6020_ENCODER_RESOLUTION *
                    CHASSIS_STEER_GEAR_RATIO /
                    SWERVE_TWO_PI *
                    swerve_steer_direction[i];

                swerve_last_target_count[i] = target_count;
                target_error_rad = angle_error;
            }
            else
            {
                target_error_rad =
                    (target_count - current_count) *
                    SWERVE_TWO_PI /
                    (GM6020_ENCODER_RESOLUTION *
                     CHASSIS_STEER_GEAR_RATIO) *
                    swerve_steer_direction[i];
                target_error_rad =
                    Chassis_WrapToPi(target_error_rad);
            }
        }

        if ((swerve_steer_online[i] == 0U) ||
            (swerve_zero_ready[i] == 0U))
        {
            drive_speed = 0.0f;
            target_error_rad = 0.0f;
        }

        Swerve_Target.steer_target_count[i] = target_count;
        Swerve_Target.steer_error_rad[i] = target_error_rad;
        Swerve_Target.drive_target_rpm[i] =
            drive_speed *
            60.0f *
            CHASSIS_DRIVE_GEAR_RATIO /
            (SWERVE_TWO_PI * CHASSIS_WHEEL_RADIUS_M) *
            swerve_drive_direction[i];
    }
}

static void Chassis_RunSteerControllers(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM])
{
    for (uint8_t i = 0U; i < SWERVE_MODULE_NUM; i++)
    {
        float current_count;
        float output_current;

        if ((swerve_steer_online[i] == 0U) ||
            (swerve_zero_ready[i] == 0U))
        {
            swerve_steer_current_cmd[i] = 0;
            Chassis_ResetController(map[i].steer);
            continue;
        }

        current_count =
            (float)map[i].steer->DATA.Angle_Infinite;

        map[i].steer->DATA.Aim =
            Swerve_Target.steer_target_count[i];

        PID_Calculate(&map[i].steer->PID_P,
                      current_count,
                      map[i].steer->DATA.Aim);
        PID_Calculate(&map[i].steer->PID_S,
                      (float)map[i].steer->DATA.Speed_now,
                      map[i].steer->PID_P.Output);

        output_current =
            map[i].steer->PID_S.Output *
            swerve_steer_output_sign[i];

        if ((fabsf(Swerve_Target.steer_error_rad[i]) >
             SWERVE_STEER_START_ERROR_RAD) &&
            (fabsf((float)map[i].steer->DATA.Speed_now) <
             SWERVE_STEER_START_SPEED_RPM) &&
            (fabsf(output_current) <
             SWERVE_STEER_START_CURRENT))
        {
            float sign_source = output_current;

            if (fabsf(sign_source) < 1.0f)
            {
                sign_source =
                    (map[i].steer->DATA.Aim - current_count) *
                    swerve_steer_output_sign[i];
            }

            output_current =
                (sign_source >= 0.0f) ?
                SWERVE_STEER_START_CURRENT :
                -SWERVE_STEER_START_CURRENT;
        }

        swerve_steer_current_cmd[i] =
            Chassis_LimitCurrent(output_current,
                                 SWERVE_STEER_CURRENT_LIMIT);
    }
}

static void Chassis_RunDriveControllers(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM])
{
    for (uint8_t i = 0U; i < SWERVE_MODULE_NUM; i++)
    {
        float target_rpm = Swerve_Target.drive_target_rpm[i];
        float output_current;

        if ((swerve_steer_online[i] == 0U) ||
            (swerve_zero_ready[i] == 0U))
        {
            target_rpm = 0.0f;
        }

        map[i].drive->DATA.Aim = target_rpm;
        Swerve_Target.drive_target_rpm[i] = target_rpm;

        if (swerve_drive_online[i] == 0U)
        {
            swerve_drive_current_cmd[i] = 0;
            Chassis_ResetController(map[i].drive);
            continue;
        }

        Feedforward_Calculate(&map[i].drive->PID_F,
                              map[i].drive->DATA.Aim);
        PID_Calculate(&map[i].drive->PID_S,
                      (float)map[i].drive->DATA.Speed_now,
                      map[i].drive->DATA.Aim);

        output_current =
            map[i].drive->PID_F.Output +
            map[i].drive->PID_S.Output;

        swerve_drive_current_cmd[i] =
            Chassis_LimitCurrent(output_current,
                                 SWERVE_DRIVE_CURRENT_LIMIT);
    }
}

static void Chassis_StopAllMotors(const Swerve_Motor_Map_T map[SWERVE_MODULE_NUM])
{
    swerve_chassis_vx = 0.0f;
    swerve_chassis_vy = 0.0f;
    swerve_chassis_wz = 0.0f;

    for (uint8_t i = 0U; i < SWERVE_MODULE_NUM; i++)
    {
        Swerve_Target.drive_target_rpm[i] = 0.0f;
        Swerve_Target.steer_error_rad[i] = 0.0f;
        swerve_drive_current_cmd[i] = 0;
        swerve_steer_current_cmd[i] = 0;
        map[i].drive->DATA.Aim = 0.0f;
        map[i].steer->DATA.Aim =
            Swerve_Target.steer_target_count[i];
        Chassis_ResetController(map[i].drive);
        Chassis_ResetController(map[i].steer);
    }

    Chassis_SendMotorCurrents();
}

static void Chassis_SendMotorCurrents(void)
{
    DJI_Current_Ctrl(&hcan1,
                     0x200,
                     swerve_drive_current_cmd[SWERVE_FL],
                     swerve_drive_current_cmd[SWERVE_FR],//原来是swerve_drive_current_cmd[SWERVE_FR]
                     swerve_drive_current_cmd[SWERVE_RL],
                     swerve_drive_current_cmd[SWERVE_RR]);

    DJI_Current_Ctrl(&hcan2,
                     0x1FE,
                     swerve_steer_current_cmd[SWERVE_FR],
                     swerve_steer_current_cmd[SWERVE_FL],
                     swerve_steer_current_cmd[SWERVE_RL],
                     swerve_steer_current_cmd[SWERVE_RR]);
}

static float Chassis_ApplyDeadband(float value, float deadband)
{
    if (fabsf(value) < deadband)
    {
        return 0.0f;
    }

    return value;
}

static float Chassis_LimitFloat(float value, float min_value, float max_value)
{
    if (value > max_value)
    {
        return max_value;
    }

    if (value < min_value)
    {
        return min_value;
    }

    return value;
}

static int16_t Chassis_LimitCurrent(float current, float limit)
{
    current = Chassis_LimitFloat(current, -limit, limit);
    return (int16_t)current;
}

static float Chassis_WrapToPi(float angle_rad)
{
    while (angle_rad > SWERVE_PI)
    {
        angle_rad -= SWERVE_TWO_PI;
    }

    while (angle_rad < -SWERVE_PI)
    {
        angle_rad += SWERVE_TWO_PI;
    }

    return angle_rad;
}



//  ..............        下面是新写的函数..................................
static float Chassis_GetNearestZeroCount(
    float current_count,
    float zero_single_count)
{
    float zero_count = zero_single_count;

    /*
     * �?�?191范围内的机械零点�?     * 展开到最接近当前Angle_Infinite的那一圈�?     */
    while ((zero_count - current_count) > 4096.0f)
    {
        zero_count -= GM6020_ENCODER_RESOLUTION;
    }

    while ((zero_count - current_count) < -4096.0f)
    {
        zero_count += GM6020_ENCODER_RESOLUTION;
    }

    return zero_count;
}

//上面是新写的函数..................................。。。。。。。。。。。。。。�?










