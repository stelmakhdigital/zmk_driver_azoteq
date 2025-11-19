/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT azoteq_tps43

#include "tps43.h"

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(tps43, CONFIG_INPUT_LOG_LEVEL);

/* Forward declarations */
static int tps43_device_init(const struct device *dev);
static int tps43_device_reset(const struct device *dev);
static int tps43_verify_device_id(const struct device *dev);
static int tps43_configure_device(const struct device *dev);

/* I2C helper functions - IQS5xx uses 16-bit register addresses */
static int tps43_i2c_read_reg16(const struct device *dev, uint16_t reg, uint16_t *val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[2];
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;
    
    ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to read register 0x%04x: %d", reg, ret);
        return ret;
    }
    
    /* IQS5xx returns data in big-endian format (MSB first) */
    *val = (buf[0] << 8) | buf[1];
    return 0;
}

static int tps43_i2c_write_reg16(const struct device *dev, uint16_t reg, uint16_t val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};
    int ret;
    
    ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to write register 0x%04x: %d", reg, ret);
        return ret;
    }
    
    return 0;
}

static int tps43_i2c_read_reg8(const struct device *dev, uint16_t reg, uint8_t *val)
{
    const struct tps43_config *config = dev->config;
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;
    
    ret = i2c_write_read_dt(&config->i2c, reg_buf, sizeof(reg_buf), val, 1);
    if (ret < 0) {
        LOG_ERR("Failed to read register 0x%04x: %d", reg, ret);
        return ret;
    }
    
    return 0;
}

static int tps43_i2c_write_reg8(const struct device *dev, uint16_t reg, uint8_t val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};
    int ret;
    
    ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to write register 0x%04x: %d", reg, ret);
        return ret;
    }
    
    return 0;
}

static int tps43_end_comm_window(const struct device *dev)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[3] = {TPS43_REG_END_COMM_WINDOW >> 8, TPS43_REG_END_COMM_WINDOW & 0xFF, 0x00};
    int ret;
    
    ret = i2c_write_dt(&config->i2c, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Failed to end comm window: %d", ret);
        return ret;
    }
    
    return 0;
}

static int tps43_verify_device_id(const struct device *dev)
{
    uint16_t device_info;
    int ret;
    
    ret = tps43_i2c_read_reg16(dev, TPS43_REG_DEVICE_INFO, &device_info);
    if (ret < 0) {
        LOG_ERR("Failed to read device info");
        return ret;
    }
    
    LOG_INF("Device Product ID: 0x%04x", device_info);
    
    /* IQS5xx Product ID - typically 0x003a or similar */
    if (device_info == 0x003a || device_info == 0x3a00) {
        LOG_INF("Product ID verified: 0x%04x", device_info);
    } else {
        LOG_WRN("Unexpected Product ID: 0x%04x (expected 0x003a or 0x3a00)", device_info);
    }
    
    /* End communication window */
    tps43_end_comm_window(dev);
    
    return 0;
}

static int tps43_device_reset(const struct device *dev)
{
    const struct tps43_config *config = dev->config;
    struct tps43_data *data = dev->data;
    
    if (!gpio_is_ready_dt(&config->rst_gpio)) {
        LOG_ERR("Reset GPIO not ready");
        return -ENODEV;
    }
    
    LOG_INF("Performing hardware reset");
    
    /* Reset sequence: LOW -> wait -> HIGH -> wait */
    gpio_pin_set_dt(&config->rst_gpio, 0);
    k_msleep(10);
    gpio_pin_set_dt(&config->rst_gpio, 1);
    k_msleep(50); /* Allow device to boot */
    
    /* Reset driver state */
    data->device_ready = false;
    data->error_count = 0;
    data->last_x = 0;
    data->last_y = 0;
    data->x = 0;
    data->y = 0;
    data->touch_state = 0;
    
    return 0;
}

static int tps43_configure_device(const struct device *dev)
{
    const struct tps43_config *config = dev->config;
    int ret;
    
    /* Enable event mode and trackpad events */
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONFIG_1,
                            TPS43_EVENT_MODE | TPS43_TP_EVENT | TPS43_GESTURE_EVENT);
    if (ret < 0) {
        LOG_ERR("Failed to configure event mode: %d", ret);
        return ret;
    }
    
    /* Configure filter settings */
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_FILTER_SETTINGS,
                            TPS43_IIR_FILTER | TPS43_MAV_FILTER | TPS43_ALP_COUNT_FILTER);
    if (ret < 0) {
        LOG_ERR("Failed to configure filter settings: %d", ret);
        return ret;
    }
    
    /* Configure axes */
    uint8_t xy_config = 0;
    xy_config |= config->invert_x ? TPS43_FLIP_X : 0;
    xy_config |= config->invert_y ? TPS43_FLIP_Y : 0;
    xy_config |= config->swap_xy ? TPS43_SWITCH_XY_AXIS : 0;
    
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_XY_CONFIG_0, xy_config);
    if (ret < 0) {
        LOG_ERR("Failed to configure axes: %d", ret);
        return ret;
    }
    
    /* Configure system settings */
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONFIG_0, TPS43_SETUP_COMPLETE | TPS43_WDT);
    if (ret < 0) {
        LOG_ERR("Failed to configure system: %d", ret);
        return ret;
    }
    
    /* End communication window */
    ret = tps43_end_comm_window(dev);
    if (ret < 0) {
        LOG_ERR("Failed to end comm window during initialization: %d", ret);
        return ret;
    }
    
    LOG_INF("Device configured successfully");
    return 0;
}

static int tps43_device_init(const struct device *dev)
{
    struct tps43_data *data = dev->data;
    int ret;
    
    /* Reset device first */
    ret = tps43_device_reset(dev);
    if (ret < 0) {
        LOG_ERR("Device reset failed");
        return ret;
    }
    
    /* Verify device ID */
    ret = tps43_verify_device_id(dev);
    if (ret < 0) {
        LOG_ERR("Device verification failed");
        return ret;
    }
    
    /* Configure device */
    ret = tps43_configure_device(dev);
    if (ret < 0) {
        LOG_ERR("Device configuration failed");
        return ret;
    }
    
    data->device_ready = true;
    data->initialized = true;
    
    LOG_INF("TPS43 device initialized successfully");
    return 0;
}
static int tps43_read_touch_data(const struct device *dev)
{
    struct tps43_data *data = dev->data;
    const struct tps43_config *config = dev->config;
    uint8_t sys_info_0, sys_info_1;
    int ret;
    int16_t rel_x = 0, rel_y = 0;
    
    if (!data->device_ready) {
        return -ENODEV;
    }
    
    /* Read system info registers */
    ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_0, &sys_info_0);
    if (ret < 0) {
        LOG_DBG("Failed to read system info 0: %d (device may be busy)", ret);
        goto end_comm;
    }
    
    ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_1, &sys_info_1);
    if (ret < 0) {
        LOG_DBG("Failed to read system info 1: %d (device may be busy)", ret);
        goto end_comm;
    }
    
    LOG_DBG("System info: SYS_INFO_0=0x%02x, SYS_INFO_1=0x%02x", sys_info_0, sys_info_1);
    
    /* Handle reset indication */
    if (sys_info_0 & TPS43_SHOW_RESET) {
        LOG_INF("Device reset detected");
        /* Acknowledge reset */
        tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_0, TPS43_ACK_RESET);
        goto end_comm;
    }
    
    bool tp_movement = (sys_info_1 & TPS43_TP_MOVEMENT) != 0;
    
    if (tp_movement) {
        /* Read relative movement directly */
        ret = tps43_i2c_read_reg16(dev, TPS43_REG_REL_X, (uint16_t *)&rel_x);
        if (ret < 0) {
            LOG_ERR("Failed to read relative X: %d", ret);
            goto end_comm;
        }
        
        ret = tps43_i2c_read_reg16(dev, TPS43_REG_REL_Y, (uint16_t *)&rel_y);
        if (ret < 0) {
            LOG_ERR("Failed to read relative Y: %d", ret);
            goto end_comm;
        }
        
        LOG_DBG("Touch movement: rel_x=%d, rel_y=%d", rel_x, rel_y);
        
        /* Apply coordinate transformations */
        if (config->swap_xy) {
            int16_t tmp = rel_x;
            rel_x = rel_y;
            rel_y = tmp;
        }
        
        if (config->invert_x) {
            rel_x = -rel_x;
        }
        
        if (config->invert_y) {
            rel_y = -rel_y;
        }
        
        /* Scale delta based on sensitivity */
        rel_x = (rel_x * (int32_t)config->sensitivity) / 100;
        rel_y = (rel_y * (int32_t)config->sensitivity) / 100;
        
        /* Clamp to prevent overflow */
        rel_x = CLAMP(rel_x, INT16_MIN, INT16_MAX);
        rel_y = CLAMP(rel_y, INT16_MIN, INT16_MAX);
        
        /* Report relative movement if there's any */
        if (rel_x != 0 || rel_y != 0) {
            LOG_INF("Reporting movement: dx=%d, dy=%d", rel_x, rel_y);
            if (rel_x != 0) {
                input_report_rel(dev, INPUT_REL_X, rel_x, false, K_NO_WAIT);
                LOG_DBG("Reported INPUT_REL_X: %d", rel_x);
            }
            if (rel_y != 0) {
                input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_NO_WAIT);
                LOG_DBG("Reported INPUT_REL_Y: %d", rel_y);
            }
        } else {
            LOG_DBG("No movement (rel_x=%d, rel_y=%d)", rel_x, rel_y);
        }
    }
    
end_comm:
    /* End communication window - MUST be called after each read */
    tps43_end_comm_window(dev);
    
    /* Reset error count on successful read */
    data->error_count = 0;
    
    return 0;
}

 static void tps43_work_handler(struct k_work *work)
 {
     struct k_work_delayable *delayable_work = k_work_delayable_from_work(work);
     struct tps43_data *data = CONTAINER_OF(delayable_work, struct tps43_data, work);
     const struct device *dev = data->dev;
     int ret;
     
     LOG_DBG("Work handler called");
     
     k_mutex_lock(&data->lock, K_FOREVER);
     
     /* Reinitialize device if needed */
     if (!data->device_ready) {
         LOG_INF("Reinitializing device");
         ret = tps43_device_init(dev);
         if (ret < 0) {
             LOG_ERR("Device reinitialization failed");
             k_mutex_unlock(&data->lock);
             /* Retry after longer delay */
             k_work_reschedule(&data->work, K_MSEC(1000));
             return;
         }
     }
     
     /* Read touch data and report input events */
     ret = tps43_read_touch_data(dev);
     if (ret < 0) {
         /* Don't reschedule immediately on error - let interrupts handle it */
         k_mutex_unlock(&data->lock);
         return;
     }
     
     k_mutex_unlock(&data->lock);
     
     /* Don't reschedule periodic polling - rely on GPIO interrupts instead */
     /* This reduces I2C bus load and prevents errors when device is not ready */
 }

static void tps43_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct tps43_data *data = CONTAINER_OF(cb, struct tps43_data, gpio_cb);
    
    LOG_DBG("GPIO interrupt triggered");
    /* Schedule work to handle interrupt in work queue context */
    k_work_reschedule(&data->work, K_NO_WAIT);
}

static int tps43_init(const struct device *dev)
{
    struct tps43_data *data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret;
    
    LOG_INF("=== TPS43 input driver init called for device %s ===", dev->name);
    LOG_DBG("I2C bus: %s", config->i2c.bus ? config->i2c.bus->name : "NULL");
    LOG_DBG("I2C address: 0x%02x", config->i2c.addr);
    
    data->dev = dev;
    
    /* Initialize mutex */
    k_mutex_init(&data->lock);
    
    /* Initialize work queue */
    k_work_init_delayable(&data->work, tps43_work_handler);
    
    /* Check I2C bus readiness */
    if (!i2c_is_ready_dt(&config->i2c)) {
        LOG_ERR("I2C bus not ready");
        return -ENODEV;
    }
    
    /* Configure reset GPIO */
    if (gpio_is_ready_dt(&config->rst_gpio)) {
        ret = gpio_pin_configure_dt(&config->rst_gpio, GPIO_OUTPUT_ACTIVE);
        if (ret < 0) {
            LOG_ERR("Failed to configure reset GPIO: %d", ret);
            return ret;
        }
    }
    
    /* Configure interrupt GPIO */
    if (gpio_is_ready_dt(&config->int_gpio)) {
        ret = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);
        if (ret < 0) {
            LOG_ERR("Failed to configure interrupt GPIO: %d", ret);
            return ret;
        }
        
        gpio_init_callback(&data->gpio_cb, tps43_gpio_callback, BIT(config->int_gpio.pin));
        ret = gpio_add_callback(config->int_gpio.port, &data->gpio_cb);
        if (ret < 0) {
            LOG_ERR("Failed to add GPIO callback: %d", ret);
            return ret;
        }
        
        /* Enable interrupt for edge detection (rising edge when data ready) */
        ret = gpio_pin_interrupt_configure_dt(&config->int_gpio, GPIO_INT_EDGE_RISING);
        if (ret < 0) {
            LOG_ERR("Failed to enable GPIO interrupt: %d", ret);
            return ret;
        }
        LOG_DBG("GPIO interrupt enabled for pin %d (rising edge)", config->int_gpio.pin);
    }
    
    /* Wait for device to be ready after reset */
    k_msleep(100);
    
    /* Initialize device */
    ret = tps43_device_init(dev);
    if (ret < 0) {
        LOG_ERR("Device initialization failed: %d", ret);
        return ret;
    }
    
    /* Don't start periodic polling - rely on GPIO interrupts only */
    /* This prevents I2C errors when device is not ready */
    LOG_DBG("Driver initialized - using GPIO interrupts only");
    
    LOG_INF("TPS43 input driver initialized successfully");
    return 0;
}

#define TPS43_DEFINE(inst)                                                    \
    static struct tps43_data tps43_data_##inst;                             \
                                                                              \
    static const struct tps43_config tps43_config_##inst = {                \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                 \
        .int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),        \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {0}),        \
        .resolution_x = DT_INST_PROP_OR(inst, resolution_x, 0),            \
        .resolution_y = DT_INST_PROP_OR(inst, resolution_y, 0),            \
        .invert_x = DT_INST_PROP_OR(inst, invert_x, false),                \
        .invert_y = DT_INST_PROP_OR(inst, invert_y, false),                \
        .swap_xy = DT_INST_PROP_OR(inst, swap_xy, false),                  \
        .sensitivity = DT_INST_PROP_OR(inst, sensitivity, 100),            \
    };                                                                       \
                                                                              \
    DEVICE_DT_INST_DEFINE(inst, tps43_init, NULL,                          \
                         &tps43_data_##inst, &tps43_config_##inst,        \
                         POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);   \
    BUILD_ASSERT(DT_INST_REG_ADDR(inst) == 0x74, "I2C address mismatch");

DT_INST_FOREACH_STATUS_OKAY(TPS43_DEFINE)
