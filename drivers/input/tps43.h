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

/* Определения регистров контроллера TPS43 с IQS572 - На основе даташита IQS5xx-B000 */
/* https://www.azoteq.com/images/stories/pdf/iqs5xx-b000_trackpad_datasheet.pdf */

/* События жестов */
// Только чтение
#define TPS43_REG_GESTURE_EVENTS_0  0x000D  /* 1 байт */
#define TPS43_REG_GESTURE_EVENTS_1  0x000E  /* 1 байт */

/* Системная информация */
// Только чтение
#define TPS43_REG_SYSTEM_INFO_0     0x000F  /* 1 байт - Флаги состояния */
#define TPS43_REG_SYSTEM_INFO_1     0x0010  /* 1 байт - Флаги состояния */


/* Управление и конфигурация системы */
// Чтение-запись
#define TPS43_REG_SYSTEM_CONTROL_0  0x0431  /* 1 байт - Основное управление (ACK_RESET, AUTO_ATI) */
#define TPS43_REG_SYSTEM_CONTROL_1  0x0432  /* 1 байт - Управление режимом событий */
// Чтение-запись
#define TPS43_REG_SYSTEM_CONFIG_0   0x058E  /* 1 байт - Конфигурация системы */
#define TPS43_REG_SYSTEM_CONFIG_1   0x058F  /* 1 байт - Дополнительная конфигурация */

/* Данные касания - основные координаты */
// Только чтение
#define TPS43_REG_NUM_FINGERS       0x0011  /* 1 байт */
#define TPS43_REG_REL_X             0x0012  /* 2 байта - относительное движение по X */
#define TPS43_REG_REL_Y             0x0014  /* 2 байта - относительное движение по Y */
#define TPS43_REG_ABS_X             0x0016  /* 2 байта - абсолютная позиция X */
#define TPS43_REG_ABS_Y             0x0018  /* 2 байта - абсолютная позиция Y */
#define TPS43_REG_TOUCH_STRENGTH    0x001A  /* 2 байта */
#define TPS43_REG_TOUCH_AREA        0x001C  /* 1 байт */

/* Конфигурация XY */
// Чтение-запись
#define TPS43_REG_XY_CONFIG_0       0x0669  /* 1 байт */

/* Конфигурация жестов */
// Чтение-запись // Низкоуровневая конфигурация жестов // наверное стоит убрать
#define TPS43_REG_SINGLE_FINGER_GESTURES 0x06B7  /* 1 байт */
#define TPS43_REG_MULTI_FINGER_GESTURES  0x06B8  /* 1 байт */


/* Настройки фильтра */
// Чтение-запись
#define TPS43_REG_FILTER_SETTINGS   0x0632  /* 1 байт */


/* ============================================================ */
/* Системная информация 0 (0x000F) - Флаги состояния - 8 бит */
/* ============================================================ */

#define TPS43_SHOW_RESET            BIT(0)      /* Проверка флага при сбросе системы */

/* ============================================================ */
/* Системная информация 1 (0x0010) - Флаги состояния - 8 бит */
/* ============================================================ */

#define TPS43_SWITCH_STATE          BIT(5)      /* Состояние пина SW_IN */
#define TPS43_SNAP_TOGGLE           BIT(4)      /* Переключение канала Snap */
#define TPS43_RR_MISSED             BIT(3)      /* Пропущена частота отчетов */
#define TPS43_TOO_MANY_FINGERS      BIT(2)      /* Превышено максимальное количество точек касания */
#define TPS43_PALM_DETECT           BIT(1)      /* Обнаружена ладонь */
#define TPS43_TP_MOVEMENT           BIT(0)      /* Движение тачпада */


/* ============================================================ */
/* Управление системой 0 (0x0431) - 8 бит */
/* ============================================================ */

#define TPS43_ACK_RESET             BIT(7)      /* Подтверждение сброса */

/* ============================================================ */
/* Управление системой 1 (0x0432) - 8 бит */
/* ============================================================ */

#define TPS43_RESET                 BIT(1)
#define TPS43_SUSPEND               BIT(0)      /* Режим сна */


/* ============================================================ */
/* Конфигурация системы 0 (0x058E) - 8 бит */
/* ============================================================ */

#define TPS43_SETUP_COMPLETE        BIT(6)      /* Флаг завершения настройки */
#define TPS43_WDT_ENABLE            BIT(5)      /* Включение таймера сторожевого пса */


/* ============================================================ */
/* Конфигурация системы 1 (0x058F) - 8 бит */
/* ============================================================ */

#define TPS43_EVENT_MODE            BIT(0)
#define TPS43_GESTURE_EVENT         BIT(1)
#define TPS43_TP_EVENT              BIT(2)
#define TPS43_TOUCH_EVENT           BIT(6)


/* ============================================================ */
/* Конфигурация XY 0 (0x0669) - 8 бит */
/* ============================================================ */

#define TPS43_FLIP_X                BIT(0)      /* Отразить ось X */
#define TPS43_FLIP_Y                BIT(1)      /* Отразить ось Y */
#define TPS43_SWITCH_XY_AXIS        BIT(2)      /* Поменять местами оси X и Y */


/* ============================================================ */
/* События жестов 0 (0x000D) - 8 бит */
/* ============================================================ */

#define TPS43_SINGLE_TAP            BIT(0)      /* Обнаружено касание одним пальцем */
#define TPS43_PRESS_AND_HOLD        BIT(1)      /* Обнаружено нажатие и удержание */
#define TPS43_SWIPE_UP              BIT(2)      /* Проведение одним пальцем вверх */
#define TPS43_SWIPE_DOWN            BIT(3)      /* Проведение одним пальцем вниз */
#define TPS43_SWIPE_LEFT            BIT(4)      /* Проведение одним пальцем влево */
#define TPS43_SWIPE_RIGHT           BIT(5)      /* Проведение одним пальцем вправо */

/* ============================================================ */
/* События жестов 1 (0x000E) - 8 бит */
/* ============================================================ */

#define TPS43_TWO_FINGER_TAP        BIT(0)      /* Касание двумя пальцами */
#define TPS43_SCROLL                BIT(1)      /* Прокрутка вверх */
#define TPS43_ZOOM                  BIT(2)      /* Жест масштабирования */

/* ============================================================ */
/* Настройки фильтра 0 (0x0632) - 8 бит */
/* ============================================================ */

#define TPS43_IIR_FILTER           BIT(0)      /* Фильтр IIR */
#define TPS43_MAV_FILTER           BIT(1)      /* Фильтр MAV */
#define TPS43_IIR_SELECT           BIT(2)      /* Выбор IIR */
#define TPS43_ALP_COUNT_FILTER     BIT(3)      /* Фильтр подсчета ALP */


/* Окончание окна связи - ОБЯЗАТЕЛЬНО вызывать после каждого чтения (запись в недопустимый адрес 0xEEEE вызывает NACK) */
#define TPS43_REG_END_COMM_WINDOW   0xEEEE

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

    int16_t sensitivity;
    int16_t scroll_sensitivity;

    bool enable_power_management;

    uint8_t filter_settings;
};

struct tps43_drv_data {
    const struct device *dev;
    struct k_sem lock;
    struct gpio_callback rdy_cb;
    struct k_work work;

    bool device_ready;
    bool initialized;
    bool scroll_active;
    bool drag_active;
    bool suspended;         
};

int tps43_set_sleep(const struct device *dev, bool sleep);

#ifdef __cplusplus
}
#endif

