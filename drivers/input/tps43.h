/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

 #ifndef ZEPHYR_DRIVERS_SENSOR_TPS43_H_
 #define ZEPHYR_DRIVERS_SENSOR_TPS43_H_
 
 #include <zephyr/device.h>
 #include <zephyr/drivers/gpio.h>
 #include <zephyr/drivers/i2c.h>
 #include <zephyr/drivers/sensor.h>
 
 /* TPS43 with IQS572 controller register definitions - Based on actual IQS572 datasheet */
 #define TPS43_REG_DEVICE_INFO       0x00
 #define TPS43_REG_SYS_INFO_0        0x01
 #define TPS43_REG_SYS_INFO_1        0x02
 #define TPS43_REG_VERSION_INFO      0x03
 #define TPS43_REG_XY_INFO_0         0x10
 #define TPS43_REG_XY_INFO_1         0x11
 #define TPS43_REG_TOUCH_STRENGTH    0x12
 #define TPS43_REG_TOUCH_AREA        0x13
 #define TPS43_REG_COORDINATES_X     0x14
 #define TPS43_REG_COORDINATES_Y     0x16
 #define TPS43_REG_PROX_STATUS       0x20
 #define TPS43_REG_TOUCH_STATUS      0x21
 #define TPS43_REG_COUNTS            0x22
 #define TPS43_REG_LTA               0x23
 #define TPS43_REG_DELTAS            0x24
 #define TPS43_REG_MULTIPLIERS       0x25
 #define TPS43_REG_COMPENSATION      0x26
 #define TPS43_REG_PROX_SETTINGS     0x40
 #define TPS43_REG_THRESHOLDS        0x41
 #define TPS43_REG_TIMINGS_0         0x42
 #define TPS43_REG_TIMINGS_1         0x43
 #define TPS43_REG_GESTURE_TIMERS    0x44
 #define TPS43_REG_ACTIVE_CHANNELS   0x45
 
/* IQS572 specific configuration registers */
#define TPS43_REG_SYS_CONFIG        0x50
#define TPS43_REG_SYS_CONFIG_0      0x50  /* System configuration register 0 */
#define TPS43_REG_FILTER_SETTINGS   0x51
#define TPS43_REG_POWER_MODE        0x52
#define TPS43_REG_X_RESOLUTION      0x6E  /* X-axis resolution register */
#define TPS43_REG_Y_RESOLUTION      0x70  /* Y-axis resolution register */
 
 /* IQS572 specific values for TPS43 */
 #define TPS43_MAX_X                 2048
 #define TPS43_MAX_Y                 1792
 #define TPS43_MAX_TOUCH_POINTS      5
 
 /* IQS572 device ID */
 #define TPS43_DEVICE_ID             0x41  /* Expected device ID for IQS572 */
 
 /* IQS572 system configuration flags */
 #define TPS43_SYS_CFG_FLIP_X        BIT(0)
 #define TPS43_SYS_CFG_FLIP_Y        BIT(1)
 #define TPS43_SYS_CFG_SWITCH_XY     BIT(2)
 #define TPS43_SYS_CFG_TP_EVENT      BIT(3)
 #define TPS43_SYS_CFG_PROX_EVENT    BIT(4)
 
 /* Touch detection flags */
 #define TPS43_XY_INFO_TOUCH_MASK    0x01
 
 /* IQS572 expected product ID */
 #define TPS43_EXPECTED_PRODUCT_ID   0x58  /* Expected device ID for IQS572 */
 
 /* Error recovery thresholds */
 #define TPS43_MAX_ERROR_COUNT       5
 
 /* Device state flags */
 #define TPS43_SHOW_RESET            BIT(7)
 #define TPS43_TOUCH_EVENT           BIT(0)
 #define TPS43_PROX_EVENT            BIT(1)
 
 /* Communication timeouts and retry counts */
 #define TPS43_I2C_TIMEOUT_MS        100
 #define TPS43_INIT_TIMEOUT_MS       500
 #define TPS43_MAX_RETRIES           3
 
 struct tps43_config {
     struct i2c_dt_spec i2c;
     struct gpio_dt_spec ready_gpio;
     struct gpio_dt_spec reset_gpio;
     uint16_t max_x;
     uint16_t max_y;
     uint8_t max_touch_points;
 };
 
 struct tps43_data {
     const struct device *dev;
     struct k_work work;
     struct k_timer poll_timer;
     struct k_sem sem;
     struct gpio_callback ready_cb;
 
     uint8_t num_touches;
     uint16_t x[TPS43_MAX_TOUCH_POINTS];
     uint16_t y[TPS43_MAX_TOUCH_POINTS];
     uint8_t touch_strength[TPS43_MAX_TOUCH_POINTS];
     uint8_t touch_area[TPS43_MAX_TOUCH_POINTS];
     
     /* Error tracking and recovery */
     uint8_t error_count;
     bool device_ready;
     int64_t last_success_time;
     
     /* Power management */
     bool is_sleeping;
 };
 
 void tps43_ready_callback(const struct device *gpio_dev,
               struct gpio_callback *cb, uint32_t pins);
 
 /* Forward declaration for GPIO callback structure */
 struct gpio_callback;
 
 #endif /* ZEPHYR_DRIVERS_SENSOR_TPS43_H_ */ 