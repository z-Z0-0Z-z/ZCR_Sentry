#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"
#include "pid.h"
#include "my_can.h"


#define ANY_MOTOR_GROUP_SIZE                 4u
#define MOTOR_CHASSIS_NUM                    4u
#define MOTOR_CHASSIS_3508_GROUP_TX_ID       0x200u
#define MOTOR_CHASSIS_3508_RX_ID_BASE        0x200u
#define MOTOR_GIMBAL_6020_RX_ID              0x1FEu
#define MOTOR_3508_CURRENT_MAX               16384
#define MOTOR_6020_VOLTAGE_MAX               25000
/* ============================================================
 *  Motor entity: full state of one motor
 *  (IDs + PID params + PID state + target values + feedback)
 * ============================================================ */
typedef struct {
    uint8_t can_id;             /* RX frame ID 1~4: matched on RX, the actually received ID is stored back */
    uint32_t tx_id;              /* TX group frame ID 0x200: used when sending one frame for all 4 motors */
    PID_Speed_t      pid;        /* velocity-loop PID params (kp/ki/kd/output limit/increment limit) */
    PID_Speed_State_t spd;       /* velocity-loop PID state (error history, iterative output) */
    uint16_t real_angle;         /* mechanical angle feedback 0~8191 (rotor, maps to 0~360 deg) */
    float target_rpm;            /* target rpm (written by upper layer) */
    float real_rpm;              /* actual rpm (CAN feedback) */
    float target_current;        /* velocity-loop output -> current command mA (-16384~+16384) */
    float real_current;          /* actual torque current mA (CAN feedback) */
    uint8_t temp;                /* motor temperature degC (CAN feedback) */
} Motor_t;

void Motor_Init(void);

void Motor_SetRpms(uint32_t tx_id, float rpm1, float rpm2, float rpm3, float rpm4, float dt);

void Motor_SendGroup(Motor_t *motors);

void Motor_ReceiveCanRx(const My_Can_rx *rx);

#endif