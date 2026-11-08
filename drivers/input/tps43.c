#include <stdint.h>
#define DT_DRV_COMPAT azoteq_tps43

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <errno.h>

#include "tps43.h"

LOG_MODULE_REGISTER(tps43, CONFIG_INPUT_LOG_LEVEL);

/* ------------------------------------------------------------------ */
/* Disable-while-typing: время последнего нажатия клавиши (мс аптайма) */
/* ------------------------------------------------------------------ */

static atomic_t last_keypress_ms;
static atomic_t keypress_seen;

void tps43_notify_keypress(void) {
    atomic_set(&last_keypress_ms, (atomic_val_t)k_uptime_get_32());
    atomic_set(&keypress_seen, 1);
}

static bool typed_within(uint16_t timeout_ms) {
    if (timeout_ms == 0 || !atomic_get(&keypress_seen)) {
        return false;
    }
    uint32_t elapsed = k_uptime_get_32() - (uint32_t)atomic_get(&last_keypress_ms);
    return elapsed < timeout_ms;
}

/* ------------------------------------------------------------------ */
/* I2C                                                                 */
/* ------------------------------------------------------------------ */

/**
 * Закрывает окно связи (обязательно после каждого цикла обмена).
 * NACK в ответ на запись по 0xEEEE — штатное поведение.
 */
static void tps43_end_communication_window(const struct device *dev) {
    const struct tps43_config *config = dev->config;
    uint8_t end_buf[3] = {TPS43_REG_END_COMM_WINDOW >> 8, TPS43_REG_END_COMM_WINDOW & 0xFF, 0x00};

    int ret = i2c_write_dt(&config->i2c_bus, end_buf, sizeof(end_buf));
    if (ret != 0 && ret != -EIO) {
        LOG_DBG("Закрытие окна связи вернуло: %d", ret);
    }
}

static int tps43_read_regs(const struct device *dev, uint16_t reg, uint8_t *val, size_t len) {
    const struct tps43_config *config = dev->config;
    uint8_t addr_buf[2];

    sys_put_be16(reg, addr_buf);
    return i2c_write_read_dt(&config->i2c_bus, addr_buf, sizeof(addr_buf), val, len);
}

static int tps43_i2c_read_reg8_w_err(const struct device *dev, uint16_t reg, uint8_t *val,
                                     bool with_err) {
    int ret = tps43_read_regs(dev, reg, val, 1);

    if (ret != 0) {
        if (with_err) {
            LOG_ERR("Ошибка чтения регистра 0x%04x: %d", reg, ret);
        } else {
            LOG_DBG("Ожидаемая ошибка чтения регистра 0x%04x: %d", reg, ret);
        }
    }
    /* В оригинале здесь не было return — функция возвращала мусор при успехе */
    return ret;
}

static inline int tps43_i2c_read_reg8(const struct device *dev, uint16_t reg, uint8_t *val) {
    return tps43_i2c_read_reg8_w_err(dev, reg, val, true);
}

static int tps43_i2c_write_reg8(const struct device *dev, uint16_t reg, uint8_t val) {
    const struct tps43_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};

    int ret = i2c_write_dt(&config->i2c_bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Ошибка записи регистра 0x%04x: %d", reg, ret);
    }
    return ret;
}

static int tps43_i2c_write_reg16(const struct device *dev, uint16_t reg, uint16_t val) {
    const struct tps43_config *config = dev->config;
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};

    int ret = i2c_write_dt(&config->i2c_bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Ошибка записи регистра 0x%04x: %d", reg, ret);
    }
    return ret;
}

/** Чтение-модификация-запись: меняем только биты из mask, остальное оставляем как в прошивке */
static int tps43_update_reg8(const struct device *dev, uint16_t reg, uint8_t mask, uint8_t val) {
    uint8_t cur = 0;
    int ret = tps43_i2c_read_reg8(dev, reg, &cur);
    if (ret != 0) {
        return ret;
    }
    uint8_t next = (cur & ~mask) | (val & mask);
    LOG_DBG("RMW 0x%04x: 0x%02x -> 0x%02x", reg, cur, next);
    if (next == cur) {
        return 0;
    }
    return tps43_i2c_write_reg8(dev, reg, next);
}

/* ------------------------------------------------------------------ */
/* Состояние касания                                                   */
/* ------------------------------------------------------------------ */

static void tps43_reset_touch_state(struct tps43_drv_data *data) {
    data->moving = false;
    data->start_travel = 0;
    data->swipe_fired = false;
    data->swipe_acc = 0;
    data->rem_x = 0;
    data->rem_y = 0;
    data->rem_scroll_x = 0;
    data->rem_scroll_y = 0;
    data->scroll_axis = TPS43_AXIS_NONE;
}

static void tps43_release_drag(const struct device *dev) {
    struct tps43_drv_data *data = dev->data;

    if (data->drag_active) {
        LOG_DBG("Отпускание перетаскивания");
        data->drag_active = false;
        input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);
    }
}

/**
 * Масштабирование в процентах с сохранением остатка.
 * Без остатка (value * pct) / 100 обнуляет мелкие движения и даёт
 * «ступенчатый» курсор и рваный скролл.
 */
static int32_t tps43_scale(int32_t value, int32_t pct, int32_t *rem) {
    int32_t total = value * pct + *rem;
    int32_t out = total / 100;
    *rem = total - out * 100;
    return out;
}

/* ------------------------------------------------------------------ */
/* Конфигурация                                                        */
/* ------------------------------------------------------------------ */

static int tps43_configure_device(const struct device *dev) {
    const struct tps43_config *config = dev->config;
    int ret;

    /* Подтверждаем сброс всегда (как в драйвере Linux) */
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_0, TPS43_ACK_RESET);
    if (ret != 0) {
        return ret;
    }

    /* Оси + режекция ладони: меняем только биты 0..3, остальные оставляем */
    uint8_t xy = 0;
    xy |= config->invert_x ? TPS43_FLIP_X : 0;
    xy |= config->invert_y ? TPS43_FLIP_Y : 0;
    xy |= config->switch_xy ? TPS43_SWITCH_XY_AXIS : 0;
    xy |= config->palm_reject ? TPS43_PALM_REJECT : 0;
    ret = tps43_update_reg8(dev, TPS43_REG_XY_CONFIG_0,
                            TPS43_FLIP_X | TPS43_FLIP_Y | TPS43_SWITCH_XY_AXIS | TPS43_PALM_REJECT,
                            xy);
    if (ret != 0) {
        return ret;
    }

    if (config->palm_reject_threshold) {
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_PALM_REJECT_THR, config->palm_reject_threshold);
        if (ret != 0) {
            return ret;
        }
    }

    /*
     * Жесты пишем ВСЕГДА, иначе остаются умолчания прошивки модуля.
     * Однопальцевые свайпы в железе больше не включаем: свайпы делаются
     * программно тремя пальцами.
     */
    uint8_t single = 0;
    single |= config->single_tap ? TPS43_SINGLE_TAP : 0;
    single |= config->press_and_hold ? TPS43_PRESS_AND_HOLD : 0;
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SINGLE_FINGER_GESTURES, single);
    if (ret != 0) {
        return ret;
    }

    uint8_t multi = 0;
    multi |= config->two_finger_tap ? TPS43_TWO_FINGER_TAP : 0;
    multi |= config->scroll ? TPS43_SCROLL : 0;
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_MULTI_FINGER_GESTURES, multi);
    if (ret != 0) {
        return ret;
    }

    /* Необязательные параметры жестов */
    if (config->tap_time_ms &&
        (ret = tps43_i2c_write_reg16(dev, TPS43_REG_TAP_TIME, config->tap_time_ms)) != 0) {
        return ret;
    }
    if (config->tap_distance &&
        (ret = tps43_i2c_write_reg16(dev, TPS43_REG_TAP_DISTANCE, config->tap_distance)) != 0) {
        return ret;
    }
    if (config->hold_time_ms &&
        (ret = tps43_i2c_write_reg16(dev, TPS43_REG_HOLD_TIME, config->hold_time_ms)) != 0) {
        return ret;
    }
    if (config->scroll_initial_distance &&
        (ret = tps43_i2c_write_reg16(dev, TPS43_REG_SCROLL_INIT_DIST,
                                     config->scroll_initial_distance)) != 0) {
        return ret;
    }

    /* Частота опроса */
    if (config->active_report_rate_ms &&
        (ret = tps43_i2c_write_reg16(dev, TPS43_REG_ACTIVE_RR, config->active_report_rate_ms)) !=
            0) {
        return ret;
    }
    if (config->idle_touch_report_rate_ms &&
        (ret = tps43_i2c_write_reg16(dev, TPS43_REG_IDLE_TOUCH_RR,
                                     config->idle_touch_report_rate_ms)) != 0) {
        return ret;
    }
    if (config->active_timeout_s &&
        (ret = tps43_i2c_write_reg8(dev, TPS43_REG_ACTIVE_TIMEOUT, config->active_timeout_s)) !=
            0) {
        return ret;
    }

    /* Фильтры */
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_FILTER_SETTINGS, config->filter_settings);
    if (ret != 0) {
        return ret;
    }

    /* Произвольные регистры из devicetree: <адрес значение> */
    for (size_t i = 0; i + 1 < config->register_writes_len; i += 2) {
        uint16_t reg = config->register_writes[i];
        uint8_t val = (uint8_t)config->register_writes[i + 1];
        LOG_INF("register-writes: 0x%04x = 0x%02x", reg, val);
        ret = tps43_i2c_write_reg8(dev, reg, val);
        if (ret != 0) {
            return ret;
        }
    }

    /* События: event mode + движение + жесты + касание/отпускание */
    uint8_t events = TPS43_EVENT_MODE | TPS43_TP_EVENT | TPS43_TOUCH_EVENT;
    if (config->single_tap || config->press_and_hold || config->scroll || config->two_finger_tap) {
        events |= TPS43_GESTURE_EVENT;
    }
    ret = tps43_update_reg8(dev, TPS43_REG_SYSTEM_CONFIG_1, 0xFF, events);
    if (ret != 0) {
        return ret;
    }

    /*
     * System Config 0 — ТОЛЬКО через чтение-модификацию-запись.
     * Оригинал писал сюда 0x40 и тем самым выключал Re-ATI, ALP Re-ATI и WDT.
     * Без Re-ATI базовая линия ёмкости «уплывает» (температура, влажность,
     * рука рядом) — это классическая причина фантомных касаний и дрожания.
     */
    uint8_t cfg0 = TPS43_SETUP_COMPLETE | TPS43_WDT_ENABLE;
    if (!config->disable_reati) {
        cfg0 |= TPS43_REATI | TPS43_ALP_REATI;
    }
    ret = tps43_update_reg8(dev, TPS43_REG_SYSTEM_CONFIG_0,
                            TPS43_SETUP_COMPLETE | TPS43_WDT_ENABLE | TPS43_REATI |
                                TPS43_ALP_REATI,
                            cfg0);
    if (ret != 0) {
        return ret;
    }

    LOG_INF("Конфигурация применена: события=0x%02X, фильтр=0x%02X, жесты=0x%02X/0x%02X", events,
            config->filter_settings, single, multi);
    return 0;
}

static int check_reset_and_reconfigure(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;
    int ret;
    uint8_t sys_info = 0;
    int wait_count = 0;
    const int max_wait_count = 50;

    do {
        ret = tps43_i2c_read_reg8_w_err(dev, TPS43_REG_SYSTEM_INFO_0, &sys_info, false);
        if (ret < 0) {
            k_sleep(K_MSEC(100));
            if (++wait_count >= max_wait_count) {
                LOG_ERR("Устройство не отвечает после %d мс", wait_count * 100);
                return -ETIMEDOUT;
            }
        }
    } while (ret < 0);

    LOG_INF("Устройство ответило через %d мс, SYSTEM_INFO_0=0x%02X", wait_count * 100, sys_info);

    ret = tps43_configure_device(dev);
    tps43_end_communication_window(dev);
    if (ret != 0) {
        LOG_ERR("Ошибка конфигурации устройства: %d", ret);
        return ret;
    }

    drv_data->device_ready = true;
    return 0;
}

/* ------------------------------------------------------------------ */
/* Suspend / resume                                                    */
/* ------------------------------------------------------------------ */

static int tps43_set_suspend_internal(const struct device *dev, bool suspend, bool lock_held) {
    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret = 0;

    if (!config->enable_power_management || !drv_data->initialized) {
        return 0;
    }
    if (suspend == drv_data->suspended) {
        return 0;
    }

    if (!lock_held) {
        if (k_sem_take(&drv_data->lock, K_MSEC(100)) != 0) {
            LOG_WRN("Не удалось захватить семафор для suspend/resume");
            return -EBUSY;
        }
    }

    if (suspend && config->rdy_gpio.port != NULL) {
        gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_DISABLE);
        tps43_release_drag(dev);
    }

    uint8_t control_reg = 0;

    if (drv_data->suspended && !suspend) {
        /* Первая транзакция после suspend вернёт NACK (п.7.3.1) — это и есть пробуждение */
        (void)tps43_i2c_read_reg8_w_err(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg, false);
        for (int i = 0; i < 10; i++) {
            k_sleep(K_MSEC(20));
            ret = tps43_i2c_read_reg8_w_err(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg, false);
            if (ret == 0) {
                break;
            }
        }
    } else {
        ret = tps43_i2c_read_reg8_w_err(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg, false);
        if (ret == -EIO && suspend) {
            drv_data->suspended = true;
            ret = 0;
            goto done;
        }
    }
    if (ret != 0) {
        LOG_ERR("Ошибка чтения SYSTEM_CONTROL_1: %d", ret);
        goto done;
    }

    if (suspend) {
        control_reg |= TPS43_SUSPEND;
    } else {
        control_reg &= ~TPS43_SUSPEND;
    }

    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, control_reg);
    if (ret != 0) {
        if (ret == -EIO && suspend) {
            drv_data->suspended = true;
            ret = 0;
        }
        goto done;
    }

    drv_data->suspended = suspend;
    if (!suspend) {
        tps43_reset_touch_state(drv_data);
    }
    LOG_INF("Тачпад: %s", suspend ? "suspend" : "resume");

done:
    if (!drv_data->suspended && config->rdy_gpio.port != NULL) {
        gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
    }
    tps43_end_communication_window(dev);
    if (!lock_held) {
        k_sem_give(&drv_data->lock);
    }
    return ret;
}

/* ------------------------------------------------------------------ */
/* Обработка кадра                                                     */
/* ------------------------------------------------------------------ */

static void tps43_click(const struct device *dev, uint16_t btn) {
    input_report_key(dev, btn, 1, true, K_FOREVER);
    input_report_key(dev, btn, 0, true, K_FOREVER);
}

static void tps43_handle_scroll(const struct device *dev, int16_t rel_x, int16_t rel_y) {
    struct tps43_drv_data *data = dev->data;
    const struct tps43_config *config = dev->config;

    /* Ось фиксируется на всё касание двумя пальцами — без «прыжков» между H/V */
    if (data->scroll_axis == TPS43_AXIS_NONE) {
        if (rel_x == 0 && rel_y == 0) {
            return;
        }
        data->scroll_axis = (abs(rel_x) > abs(rel_y)) ? TPS43_AXIS_X : TPS43_AXIS_Y;
    }

    if (data->scroll_axis == TPS43_AXIS_X) {
        int32_t v = config->invert_scroll_x ? -rel_x : rel_x;
        int32_t wheel = tps43_scale(v, config->scroll_sensitivity, &data->rem_scroll_x);
        if (wheel != 0) {
            input_report_rel(dev, INPUT_REL_HWHEEL, wheel, true, K_FOREVER);
        }
    } else {
        int32_t v = config->invert_scroll_y ? -rel_y : rel_y;
        int32_t wheel = tps43_scale(v, config->scroll_sensitivity, &data->rem_scroll_y);
        if (wheel != 0) {
            input_report_rel(dev, INPUT_REL_WHEEL, wheel, true, K_FOREVER);
        }
    }
}

static void tps43_handle_swipe(const struct device *dev, int16_t rel_x) {
    struct tps43_drv_data *data = dev->data;
    const struct tps43_config *config = dev->config;

    if (data->swipe_fired) {
        return;
    }
    data->swipe_acc += rel_x;
    if (abs(data->swipe_acc) >= config->swipe_threshold) {
        /* BTN_3/BTN_4 = «Назад»/«Вперёд». BTN_6/7 из оригинала ZMK молча отбрасывает */
        uint16_t btn = (data->swipe_acc < 0) ? INPUT_BTN_3 : INPUT_BTN_4;
        LOG_DBG("Свайп 3 пальцами: %s", data->swipe_acc < 0 ? "назад" : "вперёд");
        tps43_click(dev, btn);
        data->swipe_fired = true; /* один клик на жест, а не на каждый кадр */
    }
}

static void tps43_handle_cursor(const struct device *dev, int16_t rel_x, int16_t rel_y) {
    struct tps43_drv_data *data = dev->data;
    const struct tps43_config *config = dev->config;

    if (typed_within(config->typing_move_timeout_ms)) {
        return;
    }

    /*
     * Порог старта: пока палец не сдвинулся на N единиц от точки касания,
     * движение не отправляем. Убирает сдвиг курсора при тапе (клик «мимо»)
     * и дрожание лежащего пальца.
     */
    if (!data->moving && !data->drag_active && config->movement_start_threshold) {
        data->start_travel += abs(rel_x) + abs(rel_y);
        if (data->start_travel < config->movement_start_threshold) {
            return;
        }
    }
    data->moving = true;

    int32_t dx = tps43_scale(rel_x, config->sensitivity, &data->rem_x);
    int32_t dy = tps43_scale(rel_y, config->sensitivity, &data->rem_y);
    if (dx == 0 && dy == 0) {
        return;
    }
    input_report_rel(dev, INPUT_REL_X, dx, false, K_FOREVER);
    input_report_rel(dev, INPUT_REL_Y, dy, true, K_FOREVER);
}

static void tps43_work_handler(struct k_work *work) {
    struct tps43_drv_data *data = CONTAINER_OF(work, struct tps43_drv_data, work);
    const struct device *dev = data->dev;
    const struct tps43_config *config = dev->config;
    uint8_t f[TPS43_FRAME_LEN];
    int ret;

    if (data->suspended) {
        return;
    }

    k_sem_take(&data->lock, K_FOREVER);

    /*
     * Один I2C-запрос на весь кадр (0x000D..0x0015) вместо 4–5 отдельных:
     * согласованный снимок данных и короче окно связи → меньше RR_MISSED.
     */
    ret = tps43_read_regs(dev, TPS43_FRAME_START, f, sizeof(f));
    if (ret < 0) {
        LOG_ERR("Ошибка чтения кадра: %d", ret);
        goto done;
    }

    const uint8_t g0 = f[0];
    const uint8_t g1 = f[1];
    const uint8_t si0 = f[2];
    const uint8_t si1 = f[3];
    const uint8_t fingers = f[4];
    const int16_t rel_x = (int16_t)sys_get_be16(&f[5]);
    const int16_t rel_y = (int16_t)sys_get_be16(&f[7]);

    /* Неожиданный сброс контроллера (ESD, просадка питания от RGB и т.п.) → переконфигурировать */
    if (si0 & TPS43_SHOW_RESET) {
        LOG_WRN("Контроллер сбросился — повторная конфигурация");
        tps43_release_drag(dev);
        tps43_reset_touch_state(data);
        data->prev_fingers = 0;
        ret = tps43_configure_device(dev);
        if (ret != 0) {
            LOG_ERR("Повторная конфигурация не удалась: %d", ret);
        }
        goto done;
    }

    if (si0 & (TPS43_ATI_ERROR | TPS43_ALP_ATI_ERROR)) {
        LOG_WRN("Ошибка ATI (SYSTEM_INFO_0=0x%02X) — возможны ложные касания", si0);
    }

    LOG_DBG("g0=%02X g1=%02X si0=%02X si1=%02X n=%u dx=%d dy=%d", g0, g1, si0, si1, fingers,
            rel_x, rel_y);

    /* Начало/конец касания или смена количества пальцев → сброс состояния жеста */
    if (fingers == 0 || fingers != data->prev_fingers) {
        tps43_reset_touch_state(data);
    }
    data->prev_fingers = fingers;

    /* Ладонь или слишком много пальцев — ничего не отправляем */
    if (si1 & (TPS43_PALM_DETECT | TPS43_TOO_MANY_FINGERS)) {
        tps43_release_drag(dev);
        goto done;
    }

    /* --- Жесты --- */
    const bool typing = typed_within(config->typing_tap_timeout_ms);

    if (config->single_tap && (g0 & TPS43_SINGLE_TAP)) {
        if (typing) {
            LOG_DBG("Тап проигнорирован (набор текста)");
        } else {
            tps43_click(dev, INPUT_BTN_0);
        }
    }
    if (config->two_finger_tap && (g1 & TPS43_TWO_FINGER_TAP)) {
        if (typing) {
            LOG_DBG("Тап двумя пальцами проигнорирован (набор текста)");
        } else {
            tps43_click(dev, INPUT_BTN_1);
        }
    }

    /* Удержание: проверяем КАЖДЫЙ кадр (в оригинале отпускание терялось → залипшая ЛКМ) */
    if (config->press_and_hold) {
        if ((g0 & TPS43_PRESS_AND_HOLD) && !data->drag_active && !typing) {
            data->drag_active = true;
            input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
        } else if (!(g0 & TPS43_PRESS_AND_HOLD) && data->drag_active) {
            tps43_release_drag(dev);
        }
    }
    if (fingers == 0) {
        tps43_release_drag(dev);
    }

    /* --- Движение --- */
    if (!(si1 & TPS43_TP_MOVEMENT) || (rel_x == 0 && rel_y == 0)) {
        goto done;
    }

    if (config->scroll && (g1 & TPS43_SCROLL)) {
        /* Скролл обрабатывается в ЭТОМ ЖЕ кадре — признак больше не «переезжает» в следующий */
        tps43_handle_scroll(dev, rel_x, rel_y);
    } else if (config->swipes && fingers >= 3) {
        tps43_handle_swipe(dev, rel_x);
    } else if (fingers == 1 || data->drag_active) {
        tps43_handle_cursor(dev, rel_x, rel_y);
    }
    /* 2 пальца без признака скролла — курсор не двигаем (подготовка к скроллу/тапу) */

done:
    tps43_end_communication_window(dev);
    k_sem_give(&data->lock);
}

static void tps43_rdy_callback(const struct device *port, struct gpio_callback *cb,
                               uint32_t pins) {
    struct tps43_drv_data *drv_data = CONTAINER_OF(cb, struct tps43_drv_data, rdy_cb);

    k_work_submit(&drv_data->work);
}

/* ------------------------------------------------------------------ */
/* Инициализация                                                       */
/* ------------------------------------------------------------------ */

static int tps43_init(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret;

    drv_data->dev = dev;
    drv_data->device_ready = false;
    drv_data->initialized = false;
    drv_data->drag_active = false;
    drv_data->suspended = false;
    drv_data->prev_fingers = 0;
    tps43_reset_touch_state(drv_data);

    /* Семафор и work — ДО включения прерывания (в оригинале было наоборот) */
    k_sem_init(&drv_data->lock, 1, 1);
    k_work_init(&drv_data->work, tps43_work_handler);

    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_ERR("Шина I2C не доступна");
        return -ENODEV;
    }

    if (config->rst_gpio.port) {
        ret = gpio_pin_configure_dt(&config->rst_gpio, GPIO_OUTPUT_INACTIVE);
        if (ret != 0) {
            LOG_ERR("Ошибка конфигурации RST GPIO: %d", ret);
            return ret;
        }
        gpio_pin_set_dt(&config->rst_gpio, 0);
        k_sleep(K_MSEC(10));
        gpio_pin_set_dt(&config->rst_gpio, 1);
        k_sleep(K_MSEC(610));
    }

    ret = check_reset_and_reconfigure(dev);
    if (ret != 0) {
        return ret;
    }

    if (config->rdy_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
        if (ret != 0) {
            LOG_ERR("Ошибка конфигурации RDY GPIO: %d", ret);
            return ret;
        }
        gpio_init_callback(&drv_data->rdy_cb, tps43_rdy_callback, BIT(config->rdy_gpio.pin));
        ret = gpio_add_callback(config->rdy_gpio.port, &drv_data->rdy_cb);
        if (ret != 0) {
            LOG_ERR("Ошибка добавления callback RDY: %d", ret);
            return ret;
        }
        ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
        if (ret != 0) {
            LOG_ERR("Ошибка настройки прерывания RDY: %d", ret);
            return ret;
        }
    } else {
        LOG_WRN("rdy-gpios не задан — драйвер не будет получать события");
    }

    drv_data->initialized = true;
    LOG_INF("TPS43 инициализирован");
    return 0;
}

#define TPS43_REGW_NAME(inst) tps43_##inst##_register_writes

#define TPS43_REGW_DEFINE(inst)                                                                    \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, register_writes),                                      \
                (static const uint16_t TPS43_REGW_NAME(inst)[] =                                   \
                     DT_INST_PROP(inst, register_writes);),                                        \
                ())

#define TPS43_REGW_PTR(inst)                                                                       \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, register_writes), (TPS43_REGW_NAME(inst)), (NULL))

#define TPS43_REGW_LEN(inst)                                                                       \
    COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, register_writes),                                      \
                (ARRAY_SIZE(TPS43_REGW_NAME(inst))), (0))

#define TPS43_INIT(inst)                                                                           \
    TPS43_REGW_DEFINE(inst)                                                                        \
    BUILD_ASSERT(TPS43_REGW_LEN(inst) % 2 == 0,                                                    \
                 "register-writes должен состоять из пар <адрес значение>");                       \
    static struct tps43_drv_data tps43_##inst##_drvdata;                                           \
                                                                                                   \
    static const struct tps43_config tps43_##inst##_config = {                                     \
        .i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                                     \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rdy_gpios, {0}),                                \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {0}),                                \
        .single_tap = DT_INST_PROP(inst, single_tap),                                              \
        .press_and_hold = DT_INST_PROP(inst, press_and_hold),                                      \
        .two_finger_tap = DT_INST_PROP(inst, two_finger_tap),                                      \
        .scroll = DT_INST_PROP(inst, scroll),                                                      \
        .swipes = DT_INST_PROP(inst, swipes),                                                      \
        .invert_x = DT_INST_PROP(inst, invert_x),                                                  \
        .invert_y = DT_INST_PROP(inst, invert_y),                                                  \
        .switch_xy = DT_INST_PROP(inst, switch_xy),                                                \
        .invert_scroll_x = DT_INST_PROP(inst, invert_scroll_x),                                    \
        .invert_scroll_y = DT_INST_PROP(inst, invert_scroll_y),                                    \
        .palm_reject = DT_INST_PROP(inst, palm_reject),                                            \
        .disable_reati = DT_INST_PROP(inst, disable_reati),                                        \
        .sensitivity = DT_INST_PROP(inst, sensitivity),                                            \
        .scroll_sensitivity = DT_INST_PROP(inst, scroll_sensitivity),                              \
        .enable_power_management = DT_INST_PROP(inst, enable_power_management),                    \
        .filter_settings = DT_INST_PROP(inst, filter_settings),                                    \
        .movement_start_threshold = DT_INST_PROP(inst, movement_start_threshold),                  \
        .typing_tap_timeout_ms = DT_INST_PROP(inst, typing_tap_timeout_ms),                        \
        .typing_move_timeout_ms = DT_INST_PROP(inst, typing_move_timeout_ms),                      \
        .swipe_threshold = DT_INST_PROP(inst, swipe_threshold),                                    \
        .active_report_rate_ms = DT_INST_PROP(inst, active_report_rate_ms),                        \
        .idle_touch_report_rate_ms = DT_INST_PROP(inst, idle_touch_report_rate_ms),                \
        .active_timeout_s = DT_INST_PROP(inst, active_timeout_s),                                  \
        .tap_time_ms = DT_INST_PROP(inst, tap_time_ms),                                            \
        .tap_distance = DT_INST_PROP(inst, tap_distance),                                          \
        .hold_time_ms = DT_INST_PROP(inst, hold_time_ms),                                          \
        .scroll_initial_distance = DT_INST_PROP(inst, scroll_initial_distance),                    \
        .palm_reject_threshold = DT_INST_PROP(inst, palm_reject_threshold),                        \
        .register_writes = TPS43_REGW_PTR(inst),                                                   \
        .register_writes_len = TPS43_REGW_LEN(inst),                                               \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(inst, tps43_init, NULL, &tps43_##inst##_drvdata,                         \
                          &tps43_##inst##_config, POST_KERNEL, CONFIG_INPUT_TPS43_INIT_PRIORITY,   \
                          NULL);                                                                   \
    BUILD_ASSERT(DT_INST_REG_ADDR(inst) == TPS43_I2C_ADDR, "Несоответствие адреса I2C");

DT_INST_FOREACH_STATUS_OKAY(TPS43_INIT)

int tps43_set_sleep(const struct device *dev, bool sleep) {
    if (dev == NULL) {
        return -EINVAL;
    }
    return tps43_set_suspend_internal(dev, sleep, false);
}
