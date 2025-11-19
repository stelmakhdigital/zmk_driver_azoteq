/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ZEPHYR_DRIVERS_INPUT_TPS43_H_
#define ZEPHYR_DRIVERS_INPUT_TPS43_H_

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

/* TPS43 with IQS572 controller register definitions - Based on IQS5xx datasheet */
/* All registers are 16-bit addresses */
#define TPS43_REG_DEVICE_INFO       0x0000
#define TPS43_REG_SYSTEM_INFO_0     0x000F
#define TPS43_REG_SYSTEM_INFO_1     0x0010
#define TPS43_REG_NUM_FINGERS       0x0011
#define TPS43_REG_REL_X             0x0012  /* 2 bytes - relative X movement */
#define TPS43_REG_REL_Y             0x0014  /* 2 bytes - relative Y movement */
#define TPS43_REG_ABS_X             0x0016  /* 2 bytes - absolute X position */
#define TPS43_REG_ABS_Y             0x0018  /* 2 bytes - absolute Y position */
#define TPS43_REG_TOUCH_STRENGTH     0x001A  /* 2 bytes */
#define TPS43_REG_TOUCH_AREA        0x001C

/* System control and configuration */
#define TPS43_REG_SYSTEM_CONTROL_0  0x0431
#define TPS43_REG_SYSTEM_CONFIG_0   0x058E
#define TPS43_REG_SYSTEM_CONFIG_1   0x058F
#define TPS43_REG_FILTER_SETTINGS   0x0632
#define TPS43_REG_XY_CONFIG_0       0x0669
#define TPS43_REG_BOTTOM_BETA       0x0637
#define TPS43_REG_STATIONARY_THRESH 0x0672
#define TPS43_REG_SINGLE_FINGER_GESTURES_CONF 0x06B7
#define TPS43_REG_MULTI_FINGER_GESTURES_CONF  0x06B8
#define TPS43_REG_HOLD_TIME         0x06BD

/* Gesture events */
#define TPS43_REG_GESTURE_EVENTS_0  0x000D
#define TPS43_REG_GESTURE_EVENTS_1  0x000E

/* End communication window - MUST be called after each read */
#define TPS43_REG_END_COMM_WINDOW   0xEEEE

/* IQS572 specific values for TPS43 */
#define TPS43_MAX_X                 2048
#define TPS43_MAX_Y                 1792
#define TPS43_MAX_TOUCH_POINTS      5

/* IQS572 device ID */
#define TPS43_DEVICE_ID             0x41  /* Expected device ID for IQS572 */

/* System Info 0 bits */
#define TPS43_SHOW_RESET            BIT(7)
#define TPS43_ALP_REATI_OCCURRED    BIT(6)
#define TPS43_ALP_ATI_ERROR         BIT(5)
#define TPS43_REATI_OCCURRED        BIT(4)
#define TPS43_ATI_ERROR             BIT(3)

/* System Info 1 bits */
#define TPS43_SWITCH_STATE          BIT(5)
#define TPS43_SNAP_TOGGLE           BIT(4)
#define TPS43_RR_MISSED             BIT(3)
#define TPS43_TOO_MANY_FINGERS      BIT(2)
#define TPS43_PALM_DETECT           BIT(1)
#define TPS43_TP_MOVEMENT           BIT(0)

/* System Config 0 bits */
#define TPS43_MANUAL_CONTROL        BIT(7)
#define TPS43_SETUP_COMPLETE        BIT(6)
#define TPS43_WDT                   BIT(5)

/* System Config 1 bits */
#define TPS43_EVENT_MODE            BIT(0)
#define TPS43_GESTURE_EVENT         BIT(1)
#define TPS43_TP_EVENT              BIT(2)

/* XY Config 0 bits */
#define TPS43_FLIP_X                BIT(0)
#define TPS43_FLIP_Y                BIT(1)
#define TPS43_SWITCH_XY_AXIS        BIT(2)

/* System Control 0 bits */
#define TPS43_ACK_RESET             BIT(7)

/* Filter settings bits */
#define TPS43_IIR_FILTER            BIT(0)
#define TPS43_MAV_FILTER            BIT(1)
#define TPS43_IIR_SELECT            BIT(2)
#define TPS43_ALP_COUNT_FILTER      BIT(3)

/* Gesture events */
#define TPS43_SINGLE_TAP            BIT(0)
#define TPS43_PRESS_AND_HOLD        BIT(1)
#define TPS43_TWO_FINGER_TAP        BIT(0)  /* In GESTURE_EVENTS_1 */
#define TPS43_SCROLL                BIT(1)  /* In GESTURE_EVENTS_1 */

/* Error recovery thresholds */
#define TPS43_MAX_ERROR_COUNT       5

/* Communication timeouts and retry counts */
#define TPS43_I2C_TIMEOUT_MS        100
#define TPS43_INIT_TIMEOUT_MS       500
#define TPS43_MAX_RETRIES           3

struct tps43_config {
    struct i2c_dt_spec i2c;
    struct gpio_dt_spec int_gpio;
    struct gpio_dt_spec rst_gpio;
    uint16_t resolution_x;
    uint16_t resolution_y;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
    uint8_t sensitivity;  /* Sensitivity multiplier (percentage, default 100) */
};

struct tps43_data {
    struct k_work_delayable work;
    struct k_mutex lock;
    const struct device *dev;
    
    /* Touch data */
    int16_t x;
    int16_t y;
    int16_t last_x;  /* Previous X position for delta calculation */
    int16_t last_y;  /* Previous Y position for delta calculation */
    uint8_t touch_state;
    uint8_t touch_strength;
    
    /* Device state */
    bool device_ready;
    bool initialized;
    uint8_t error_count;
    
    /* GPIO callback */
    struct gpio_callback gpio_cb;
};

#endif /* ZEPHYR_DRIVERS_INPUT_TPS43_H_ */
