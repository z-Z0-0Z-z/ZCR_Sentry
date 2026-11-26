#include "motor.h"
#include "my_can.h"

/* ============================================================
 *  Global motor static table: fixed size at compile time (currently
 *  the 4 chassis wheel motors), index 0~3 maps to ESC dial ID 1~4.
 *  No dynamic allocation.
 * ============================================================ */
static Motor_t motor_chassis_3508_table[MOTOR_CHASSIS_NUM]=
{
    {.can_id = 1u, .tx_id = MOTOR_CHASSIS_3508_GROUP_TX_ID},
    {.can_id = 2u, .tx_id = MOTOR_CHASSIS_3508_GROUP_TX_ID},
    {.can_id = 3u, .tx_id = MOTOR_CHASSIS_3508_GROUP_TX_ID},
    {.can_id = 4u, .tx_id = MOTOR_CHASSIS_3508_GROUP_TX_ID},
};

void Motor_Init(void)
{
    uint8_t i;   /* motor index loop variable */
    for (i = 0; i < MOTOR_CHASSIS_NUM; i++) {
        /* Default velocity-loop PID parameters (tune later via the handle) */
        motor_chassis_3508_table[i].pid.kp       = 1.0f;
        motor_chassis_3508_table[i].pid.ki       = 0.0f;
        motor_chassis_3508_table[i].pid.kd       = 0.0f;                    /* D term off by default, only P + I */
        motor_chassis_3508_table[i].pid.out_max  = (float)MOTOR_3508_CURRENT_MAX; /* current command limit +-16384 */
        motor_chassis_3508_table[i].pid.inc_max  = 1000.0f;                 /* increment limit per frame */

        /* Clear velocity-loop state and target/feedback values */
        motor_chassis_3508_table[i].target_rpm     = 0.0f;
        motor_chassis_3508_table[i].real_rpm       = 0.0f;
        motor_chassis_3508_table[i].target_current = 0.0f;
        motor_chassis_3508_table[i].real_current   = 0.0f;
        motor_chassis_3508_table[i].real_angle     = 0u;
        motor_chassis_3508_table[i].temp           = 0u;
        PID_Speed_Init(&motor_chassis_3508_table[i].spd);
    }
    MyCanSetRxCallback(Motor_ReceiveCanRx);
}

void Motor_SetRpms(uint32_t tx_id, float rpm1, float rpm2, float rpm3, float rpm4, float dt)
{
    if(tx_id==MOTOR_CHASSIS_3508_GROUP_TX_ID)
    {
        float rpms[4] = {rpm1, rpm2, rpm3, rpm4};
        for(uint8_t i=0; i<4; i++)
        {
            motor_chassis_3508_table[i].target_rpm = rpms[i];
            motor_chassis_3508_table[i].target_current = 
            PID_Speed_Calc(&motor_chassis_3508_table[i].pid, &motor_chassis_3508_table[i].spd, motor_chassis_3508_table[i].target_rpm, motor_chassis_3508_table[i].real_rpm, dt);
        }
    }
    Motor_SendGroup(motor_chassis_3508_table);
}

void Motor_SendGroup(Motor_t *motors)
{
    uint8_t i;                 /* motor index loop variable (0~3 maps to ESC ID 1~4) */
    uint8_t tx[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    /* Pack 4 current commands into one frame (slot 0~3 = ESC 1~4, 2 bytes each,
     * big-endian, high byte first) */
    for (i = 0; i < ANY_MOTOR_GROUP_SIZE; i++) {
        int16_t cur = (int16_t)motors[i].target_current;   /* current command from this motor's loop */

        /* Protocol boundary clamp (+-16384), prevent out-of-range torque command */
        if (cur >  MOTOR_3508_CURRENT_MAX) cur =  MOTOR_3508_CURRENT_MAX;
        if (cur < -MOTOR_3508_CURRENT_MAX) cur = -MOTOR_3508_CURRENT_MAX;

        tx[i * 2]     = (uint8_t)((uint16_t)cur >> 8);   /* high byte of the current command */
        tx[i * 2 + 1] = (uint8_t)(cur & 0xFFu);          /* low byte of the current command */
    }
    /* TX frame ID comes from the motor structure (0x200, one frame for ESC 1~4) */
    MyCanSend(motors[0].tx_id, tx);
}

void Motor_ReceiveCanRx(const My_Can_rx *rx)
{
    if(MOTOR_CHASSIS_3508_RX_ID_BASE+1 <= rx->rx_header.StdId && rx->rx_header.StdId <= MOTOR_CHASSIS_3508_RX_ID_BASE+4) {
        
        uint8_t motor_can_id = rx->rx_header.StdId - MOTOR_CHASSIS_3508_RX_ID_BASE;
        Motor_t *m = &motor_chassis_3508_table[motor_can_id - 1];
        m->real_angle   = (uint16_t)(((uint16_t)rx->can_rx[0] << 8) | (uint16_t)rx->can_rx[1]);
        m->real_rpm     = (float)(int16_t)(((uint16_t)rx->can_rx[2] << 8) | (uint16_t)rx->can_rx[3]);
        m->real_current = (float)(int16_t)(((uint16_t)rx->can_rx[4] << 8) | (uint16_t)rx->can_rx[5]);
        m->temp         = rx->can_rx[6];
    }
}