#ifndef _MANIPULATORS_H_
#define _MANIPULATORS_H_

#include "stm32f4xx_ll_usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "terminal_cmds.h"

#define STM_DRIVER_BUF_SIZE             256
#define STM_DRIVER_STACK_DEPTH          1024

#define MAX_DYN_COMMANDS                8
#define DEFAULT_DELAY                   200
#define RELAXATION_TIME                 10

#define DYN_POS_PER_REV                 1023/300
#define DYN_POS_TO_REV_PER_MS           6686

/*
 * Set dynamixel angle command
 */
#define DYN_SET_ANGLE(manip_ctrl, num, id, angle, speed) \
        (manip_ctrl)->dyn_ctrl[(num)].cmd_buff[0] = (0x01); \
        (manip_ctrl)->dyn_ctrl[(num)].cmd_buff[1] = (id); \
        (manip_ctrl)->dyn_ctrl[(num)].cmd_buff[2] = (uint8_t) ((angle) & 0xff); \
        (manip_ctrl)->dyn_ctrl[(num)].cmd_buff[3] = (uint8_t) (((angle) >> 8) \
                                                             & 0xff); \

/*
 * Memory for terminal task
 */
StackType_t manipulators_ts[STM_DRIVER_STACK_DEPTH];
StaticTask_t manipulators_tb;

/*
 * Flags for dynamixel
 */
#define DYN_BUSY_POS            (0U)

#define is_dyn_flag_set(manip_ctrl, bit) \
        (manip_ctrl->dyn_status &  (1 << (bit)))

#define dyn_set_flag(manip_ctrl, bit) \
        manip_ctrl->dyn_status |= (1 << (bit))

#define dyn_clr_flag(manip_ctrl, bit) \
        manip_ctrl->dyn_status &= (~(1 << (bit)))

/*
 * Dynamixel command structure
 */
typedef struct {
        uint8_t id;
        uint16_t current_pos;
        uint16_t goal_pos;
        uint16_t speed;
        uint32_t delay_ms;
        uint32_t start_time;
        uint8_t cmd_buff[10];
} dyn_ctrl_t;


/*
 * Manipulators control structure defenition
 */
typedef struct {
        uint8_t total_cmd;
        uint8_t current_cmd;
        uint8_t completed_cmd;
        uint8_t dyn_status;
        uint32_t current_time;
        dyn_ctrl_t sequence_cmd[MAX_DYN_COMMANDS];
        uint8_t stm_dr_buff[10];
        TaskHandle_t manip_notify;
} manip_ctrl_t;

/*
 * Main manager for processing incoming commands
 */
void manipulators_manager(void *arg);

#endif
