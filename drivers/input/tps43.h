#pragma once

#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Адрес I2C TPS43 */
#define TPS43_I2C_ADDR              0x74

/*
 * Регистры IQS5xx-B000 (TPS43 / TPS65)
 * https://www.azoteq.com/images/stories/pdf/iqs5xx-b000_trackpad_datasheet.pdf
 * Адреса сверены с IQS5xx_addresses.h из примеров Azoteq и драйвером Linux iqs5xx.c
 */

/* --- Данные (только чтение) --- */
#define TPS43_REG_GESTURE_EVENTS_0  0x000D
#define TPS43_REG_GESTURE_EVENTS_1  0x000E
#define TPS43_REG_SYSTEM_INFO_0     0x000F
#define TPS43_REG_SYSTEM_INFO_1     0x0010
#define TPS43_REG_NUM_FINGERS       0x0011
#define TPS43_REG_REL_X             0x0012  /* 2 байта, int16 BE */
#define TPS43_REG_REL_Y             0x0014  /* 2 байта, int16 BE */
#define TPS43_REG_ABS_X             0x0016
#define TPS43_REG_ABS_Y             0x0018
#define TPS43_REG_TOUCH_STRENGTH    0x001A
#define TPS43_REG_TOUCH_AREA        0x001C

/* Пакетное чтение одним I2C-запросом: 0x000D..0x0015 */
#define TPS43_FRAME_START           TPS43_REG_GESTURE_EVENTS_0
#define TPS43_FRAME_LEN             9

/* --- Управление --- */
#define TPS43_REG_SYSTEM_CONTROL_0  0x0431
#define TPS43_REG_SYSTEM_CONTROL_1  0x0432

/* --- Тайминги / частота опроса --- */
#define TPS43_REG_ACTIVE_RR         0x057A  /* 2 байта, мс */
#define TPS43_REG_IDLE_TOUCH_RR     0x057C  /* 2 байта, мс */
#define TPS43_REG_IDLE_RR           0x057E  /* 2 байта, мс */
#define TPS43_REG_ACTIVE_TIMEOUT    0x0584  /* 1 байт, с */

/* --- Конфигурация системы --- */
#define TPS43_REG_SYSTEM_CONFIG_0   0x058E
#define TPS43_REG_SYSTEM_CONFIG_1   0x058F

/* --- Фильтры --- */
#define TPS43_REG_FILTER_SETTINGS   0x0632

/* --- XY --- */
#define TPS43_REG_XY_CONFIG_0       0x0669
#define TPS43_REG_PALM_REJECT_THR   0x066C  /* 1 байт */
#define TPS43_REG_PALM_REJECT_TO    0x066D  /* 1 байт */

/* --- Жесты --- */
#define TPS43_REG_SINGLE_FINGER_GESTURES 0x06B7
#define TPS43_REG_MULTI_FINGER_GESTURES  0x06B8
#define TPS43_REG_TAP_TIME          0x06B9  /* 2 байта, мс */
#define TPS43_REG_TAP_DISTANCE      0x06BB  /* 2 байта, px */
#define TPS43_REG_HOLD_TIME         0x06BD  /* 2 байта, мс */
#define TPS43_REG_SCROLL_INIT_DIST  0x06C8  /* 2 байта, px */

/* Закрытие окна связи (запись по адресу 0xEEEE) */
#define TPS43_REG_END_COMM_WINDOW   0xEEEE

/* ---------- System Info 0 (0x000F) ---------- */
#define TPS43_SHOW_RESET            BIT(7)  /* было BIT(0) — это бит режима, а не сброса */
#define TPS43_ALP_REATI_OCCURRED    BIT(6)
#define TPS43_ALP_ATI_ERROR         BIT(5)
#define TPS43_REATI_OCCURRED        BIT(4)
#define TPS43_ATI_ERROR             BIT(3)
#define TPS43_CHARGING_MODE_MASK    GENMASK(2, 0)

/* ---------- System Info 1 (0x0010) ---------- */
#define TPS43_SWITCH_STATE          BIT(5)
#define TPS43_SNAP_TOGGLE           BIT(4)
#define TPS43_RR_MISSED             BIT(3)
#define TPS43_TOO_MANY_FINGERS      BIT(2)
#define TPS43_PALM_DETECT           BIT(1)
#define TPS43_TP_MOVEMENT           BIT(0)

/* ---------- System Control 0 (0x0431) ---------- */
#define TPS43_ACK_RESET             BIT(7)

/* ---------- System Control 1 (0x0432) ---------- */
#define TPS43_RESET                 BIT(1)
#define TPS43_SUSPEND               BIT(0)

/* ---------- System Config 0 (0x058E) ---------- */
#define TPS43_MANUAL_CONTROL        BIT(7)
#define TPS43_SETUP_COMPLETE        BIT(6)
#define TPS43_WDT_ENABLE            BIT(5)
#define TPS43_ALP_REATI             BIT(3)
#define TPS43_REATI                 BIT(2)

/* ---------- System Config 1 (0x058F) ---------- */
#define TPS43_EVENT_MODE            BIT(0)
#define TPS43_GESTURE_EVENT         BIT(1)
#define TPS43_TP_EVENT              BIT(2)
#define TPS43_REATI_EVENT           BIT(3)
#define TPS43_TOUCH_EVENT           BIT(6)

/* ---------- XY Config 0 (0x0669) ---------- */
#define TPS43_FLIP_X                BIT(0)
#define TPS43_FLIP_Y                BIT(1)
#define TPS43_SWITCH_XY_AXIS        BIT(2)
#define TPS43_PALM_REJECT           BIT(3)

/* ---------- Gesture Events 0 (0x000D) ---------- */
#define TPS43_SINGLE_TAP            BIT(0)
#define TPS43_PRESS_AND_HOLD        BIT(1)
#define TPS43_SWIPE_X_NEG           BIT(2)
#define TPS43_SWIPE_X_POS           BIT(3)
#define TPS43_SWIPE_Y_POS           BIT(4)
#define TPS43_SWIPE_Y_NEG           BIT(5)

/* ---------- Gesture Events 1 (0x000E) ---------- */
#define TPS43_TWO_FINGER_TAP        BIT(0)
#define TPS43_SCROLL                BIT(1)
#define TPS43_ZOOM                  BIT(2)

/* ---------- Filter Settings (0x0632) ---------- */
#define TPS43_IIR_FILTER            BIT(0)
#define TPS43_MAV_FILTER            BIT(1)
#define TPS43_IIR_SELECT            BIT(2)  /* 1 = фиксированный IIR, 0 = динамический */
#define TPS43_ALP_COUNT_FILTER      BIT(3)

enum tps43_scroll_axis {
    TPS43_AXIS_NONE = 0,
    TPS43_AXIS_X,
    TPS43_AXIS_Y,
};

struct tps43_config {
    struct i2c_dt_spec i2c_bus;
    struct gpio_dt_spec rdy_gpio;
    struct gpio_dt_spec rst_gpio;

    bool single_tap;
    bool press_and_hold;
    bool two_finger_tap;
    bool scroll;
    bool swipes;
    bool invert_x;
    bool invert_y;
    bool switch_xy;
    bool invert_scroll_x;
    bool invert_scroll_y;
    bool palm_reject;
    bool disable_reati;

    int16_t sensitivity;
    int16_t scroll_sensitivity;

    bool enable_power_management;

    uint8_t filter_settings;

    /* Подавление «сдвига при касании» и защита от ложных кликов */
    uint16_t movement_start_threshold;
    uint16_t typing_tap_timeout_ms;
    uint16_t typing_move_timeout_ms;
    uint16_t swipe_threshold;

    /* Необязательные аппаратные параметры (0 = не трогать значение из прошивки) */
    uint16_t active_report_rate_ms;
    uint16_t idle_touch_report_rate_ms;
    uint8_t  active_timeout_s;
    uint16_t tap_time_ms;
    uint16_t tap_distance;
    uint16_t hold_time_ms;
    uint16_t scroll_initial_distance;
    uint8_t  palm_reject_threshold;

    /* Произвольные записи регистров: пары <адрес значение>, 8 бит */
    const uint16_t *register_writes;
    size_t register_writes_len;
};

struct tps43_drv_data {
    const struct device *dev;
    struct k_sem lock;
    struct gpio_callback rdy_cb;
    struct k_work work;

    bool device_ready;
    bool initialized;
    bool drag_active;
    bool suspended;

    /* Состояние текущего касания */
    uint8_t prev_fingers;
    bool moving;                 /* порог старта движения пройден */
    uint32_t start_travel;       /* накопленный путь до старта */
    bool swipe_fired;
    int32_t swipe_acc;

    /* Остатки от масштабирования — чтобы медленное движение не терялось */
    int32_t rem_x;
    int32_t rem_y;
    int32_t rem_scroll_x;
    int32_t rem_scroll_y;
    enum tps43_scroll_axis scroll_axis;
};

int tps43_set_sleep(const struct device *dev, bool sleep);

/* Вызывается при нажатии клавиши на той же половине (disable-while-typing) */
void tps43_notify_keypress(void);

#ifdef __cplusplus
}
#endif
