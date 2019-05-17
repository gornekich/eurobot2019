#ifndef _MANIPULATORS_H_
#define _MANIPULATORS_H_

#include "stm32f4xx_ll_usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "dev_map.h"

#define STM_DRIVER_BUF_SIZE             256
#define STM_DRIVER_STACK_DEPTH          1024

#define BAR_ADC_SAMPLES_SIZE            20
#define BAR_PACK_CHECK_TRESHOLD         100 // TODO change

#define MAX_DYN_COMMANDS                8
#define DEFAULT_DELAY                   200
#define RELAXATION_TIME                 (float) 10
#define CONVERT_COEF_MS                 (float) 436
#define SET_PUMP_PLATFORM_DELAY         (uint32_t) 600
#define GRAB_PACK_DELAY                 (uint32_t) 400
#define RELEASER_THROW_DELAY            (uint32_t) 400

#define DYN_SPEEDS      0x0200, 0x0200, 0x0300, 0x03ff
#define DYN_INIT_POS    0x015a, 0x020d, 0x01cc, 0x01eb

/*
 * Set dynamixel angle command
 */
#define DYN_SET_ANGLE(manip_ctrl, num, id, angle, speed) \
        (manip_ctrl)->dyn_ctrl[(num)].cmd_buff[0] = (0x01); \
        (manip_ctrl)->dyn_ctrl[(num)].cmd_buff[1] = (id); \
        (manip_ctrl)->dyn_ctrl[(num)].cmd_buff[2] = (uint8_t) ((angle) & 0xff); \
        (manip_ctrl)->dyn_ctrl[(num)].cmd_buff[3] = (uint8_t) (((angle) >> 8) \
                                                             & 0xff);

/*
 * Memory for terminal task
 */
StackType_t manipulators_ts[STM_DRIVER_STACK_DEPTH];
StaticTask_t manipulators_tb;

/*
 * Flags for dynamixel
 */
#define DYN_BUSY_POS                    (0U)
#define DYN_BUSY                        (0x01 << DYN_BUSY_POS)
#define BLOCK_PUMP_POS                  (1U)
#define BLOCK_PUMP                      (0x01 << BLOCK_PUMP_POS)
#define BLOCK_DYN_POS                   (2U)
#define BLOCK_DYN                       (0x01 << BLOCK_DYN_POS)
#define BLOCK_STEPPER_POS               (3U)
#define BLOCK_STEPPER                   (0x01 << BLOCK_STEPPER_POS)

/*
 * Manipulators flags
 */
#define is_manip_flag_set(manip_ctrl, bit) \
        (manip_ctrl->flags & bit)

#define manip_set_flag(manip_ctrl, bit) \
        manip_ctrl->flags |= bit

#define manip_clr_flag(manip_ctrl, bit) \
        manip_ctrl->flags &= (~bit)

/*
 * Dynamixel flags
 */
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
        uint8_t cmd_started;
        uint8_t cmd_completed;
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
        uint8_t flags;
        uint32_t current_time;
        uint16_t dyn_pos[NUMBER_OF_DYNAMIXELS];
        uint16_t dyn_speeds[NUMBER_OF_DYNAMIXELS];
        dyn_ctrl_t sequence_cmd[MAX_DYN_COMMANDS];
        uint8_t bar_check;
        uint8_t bar_adc_samples[BAR_ADC_SAMPLES_SIZE];
        uint8_t stm_dr_buff[10];
        TaskHandle_t manip_notify;
} manip_ctrl_t;

/*
 * Public function for blocking all manipulators
 */
void manipulators_block(void);
/*
 * Main manager for processing incoming commands
 */
void manipulators_manager(void *arg);

#endif
