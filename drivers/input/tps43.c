#include <stdint.h>
#define DT_DRV_COMPAT azoteq_tps43

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
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
 
/**
 * @brief Завершает окно связи с тачпадом
 * 
 * После каждого чтения регистров тачпада необходимо завершить окно связи,
 * записав специальный адрес 0xEEEE, что вызывает NACK от устройства.
 * Это обязательный шаг согласно протоколу IQS5xx.
 * 
 * @param dev Указатель на устройство тачпада
 */
static void tps43_end_communication_window(const struct device *dev) {
    const struct tps43_config *config = dev->config;
    uint8_t end_buf[2];

    sys_put_be16(TPS43_REG_END_COMM_WINDOW, end_buf);

    int ret = i2c_write_dt(&config->i2c_bus, end_buf, sizeof(end_buf));
    if (ret != 0 && ret != -EIO) {
        LOG_INF("Запись окончания окна связи вернула: %d (ожидается NACK)", ret);
    }
}

/**
 * @brief Читает последовательность регистров тачпада
 * 
 * Выполняет чтение нескольких байт из последовательных регистров тачпада,
 * начиная с указанного адреса. Используется для чтения связанных регистров,
 * таких как события жестов (GESTURE_EVENTS_0 и GESTURE_EVENTS_1).
 * 
 * @param dev Указатель на устройство тачпада
 * @param reg Адрес начального регистра (16-битный)
 * @param val Указатель на буфер для данных
 * @param len Количество байт для чтения
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int read_sequence_registers(const struct device *dev, uint16_t reg, void *val, size_t len) {
    const struct tps43_config *config = dev->config;
    uint8_t addr_buf[2];
    addr_buf[0] = (uint8_t)((reg >> 8) & 0xFF);
    addr_buf[1] = (uint8_t)(reg & 0xFF);

    return i2c_write_read_dt(&config->i2c_bus, addr_buf, 2, val, len);
}

/**
 * @brief Читает 16-битный регистр тачпада через I2C
 * 
 * Выполняет чтение 16-битного значения из указанного регистра тачпада.
 * Данные интерпретируются как big-endian (MSB first).
 * 
 * @param dev Указатель на устройство тачпада
 * @param reg Адрес регистра (16-битный)
 * @param val Указатель на переменную для сохранения прочитанного значения
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int tps43_i2c_read_reg16(const struct device *dev, uint16_t reg, uint16_t *val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[2];
    // формирует 2-байтовый адрес регистра: (MSB, LSB)
    // MSB: сдвиг на 8 бит вправо (0x2F00 -> 0x2F)
    // LSB: побитовое И с маской - маска, оставляет только младший байт (0x2F00 -> 0x00)
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;
    
    // записывает адрес регистра (reg_buf) и читает 2 байта данных (в буфер buf)
    ret = i2c_write_read_dt(&config->i2c_bus, reg_buf, sizeof(reg_buf), buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Ошибка чтения регистра 0x%04x: %d", reg, ret);
        return ret;
    }
    
    // преобразует big-endian данные (MSB first) обратно в 16-битное значение
    *val = (buf[0] << 8) | buf[1];
    return 0;
}

/**
 * @brief Записывает 16-битное значение в регистр тачпада через I2C
 * 
 * Выполняет запись 16-битного значения в указанный регистр тачпада.
 * Данные передаются как big-endian (MSB first).
 * 
 * @param dev Указатель на устройство тачпада
 * @param reg Адрес регистра (16-битный)
 * @param val Значение для записи (16-битное)
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int tps43_i2c_write_reg16(const struct device *dev, uint16_t reg, uint16_t val)
{
    const struct tps43_config *config = dev->config;
    // формирует 4-байтовый адрес регистра: (MSB, LSB, MSB_VALUE, LSB_VALUE)
    uint8_t buf[4] = {reg >> 8, reg & 0xFF, val >> 8, val & 0xFF};
    int ret;
    
    ret = i2c_write_dt(&config->i2c_bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Ошибка записи регистра 0x%04x: %d", reg, ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Читает 8-битный регистр тачпада через I2C
 * 
 * Выполняет чтение 8-битного значения из указанного регистра тачпада.
 * Используется для чтения большинства регистров конфигурации и статуса.
 * 
 * @param dev Указатель на устройство тачпада
 * @param reg Адрес регистра (16-битный)
 * @param val Указатель на переменную для сохранения прочитанного значения
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int tps43_i2c_read_reg8(const struct device *dev, uint16_t reg, uint8_t *val)
{
    const struct tps43_config *config = dev->config;
    // формирует 2-байтовый адрес регистра: (MSB, LSB)
    uint8_t reg_buf[2] = {reg >> 8, reg & 0xFF};
    int ret;
    
    ret = i2c_write_read_dt(&config->i2c_bus, reg_buf, sizeof(reg_buf), val, 1);
    if (ret < 0) {
        LOG_ERR("Ошибка записи регистра 0x%04x: %d", reg, ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Записывает 8-битное значение в регистр тачпада через I2C
 * 
 * Выполняет запись 8-битного значения в указанный регистр тачпада.
 * Используется для записи регистров конфигурации и управления.
 * 
 * @param dev Указатель на устройство тачпада
 * @param reg Адрес регистра (16-битный)
 * @param val Значение для записи (8-битное)
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int tps43_i2c_write_reg8(const struct device *dev, uint16_t reg, uint8_t val)
{
    const struct tps43_config *config = dev->config;
    uint8_t buf[3] = {reg >> 8, reg & 0xFF, val};
    int ret;
    
    ret = i2c_write_dt(&config->i2c_bus, buf, sizeof(buf));
    if (ret < 0) {
        LOG_ERR("Ошибка записи регистра 0x%04x: %d", reg, ret);
        return ret;
    }
    
    return 0;
}

/**
 * @brief Callback обработчик прерывания от пина RDY тачпада
 * 
 * Вызывается при изменении состояния пина RDY (Ready) тачпада,
 * который сигнализирует о наличии новых данных для чтения.
 * Планирует выполнение обработчика работы для чтения данных.
 * 
 * @param dev Указатель на устройство тачпада
 * @param cb Указатель на структуру callback GPIO
 * @param pins Маска пинов, вызвавших прерывание
 */
 static void tps43_rdy_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins) {
     struct tps43_drv_data *drv_data = CONTAINER_OF(cb, struct tps43_drv_data, rdy_cb);
 
     k_work_submit(&drv_data->work);
 }
 
/**
 * @brief Внутренняя функция для перевода тачпада в режим suspend/resume
 * 
 * Управляет регистром SYSTEM_CONTROL_1 (0x0432), устанавливая или снимая бит SUSPEND.
 * В режиме suspend тачпад переходит в состояние низкого энергопотребления и не обрабатывает
 * касания до пробуждения.
 * 
 * @param dev Указатель на устройство тачпада
 * @param suspend true - перевести в suspend, false - вывести из suspend
 * @param lock_held true если семафор уже захвачен (для внутреннего использования)
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int tps43_set_suspend_internal(const struct device *dev, bool suspend, bool lock_held) {
    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret = 0;

    // Если управление питанием отключено, ничего не делаем
    if (!config->enable_power_management) {
        return 0;
    }

    // Захватываем семафор, если он еще не захвачен
    if (!lock_held) {
        if (k_sem_take(&drv_data->lock, K_MSEC(100)) != 0) {
            LOG_WRN("Не удалось захватить семафор для suspend/resume");
            return -EBUSY;
        }
    }

    // Проверяем, не пытаемся ли установить то же состояние
    if (drv_data->suspended == suspend) {
        if (!lock_held) {
            k_sem_give(&drv_data->lock);
        }
        return 0;
    }

    // Читаем текущее значение регистра SYSTEM_CONTROL_1
    uint8_t control_reg = 0;
    ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, &control_reg);
    if (ret != 0) {
        LOG_ERR("Ошибка чтения регистра SYSTEM_CONTROL_1: %d", ret);
        goto done;
    }

    // Устанавливаем или снимаем бит SUSPEND
    if (suspend) {
        control_reg |= TPS43_SUSPEND;
        LOG_INF("Перевод тачпада в режим suspend (низкое энергопотребление)");
    } else {
        control_reg &= ~TPS43_SUSPEND;
        LOG_INF("Пробуждение тачпада из режима suspend");
    }

    // Записываем обновленное значение регистра
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_1, control_reg);
    if (ret != 0) {
        LOG_ERR("Ошибка записи регистра SYSTEM_CONTROL_1: %d", ret);
        goto done;
    }

    // Обновляем внутреннее состояние
    drv_data->suspended = suspend;
    
    // Если пробуждаем, обновляем время активности
    if (!suspend) {
        drv_data->last_activity_time = k_uptime_get_32();
    }

done:
    // Освобождаем семафор, если мы его захватывали
    if (!lock_held) {
        k_sem_give(&drv_data->lock);
    }
    // Завершаем окно связи (обязательно после каждой операции I2C)
    tps43_end_communication_window(dev);
    return 0;
}

/**
 * @brief Основной обработчик работы для обработки событий тачпада
 * 
 * Выполняется при получении прерывания от тачпада (RDY pin).
 * Читает и обрабатывает события жестов, движения курсора и прокрутки,
 * преобразуя их в события ввода для системы ZMK.
 * Также управляет пробуждением тачпада из режима suspend при обнаружении активности.
 * 
 * Защищен семафором для предотвращения прерывания другими операциями I2C,
 * что обеспечивает плавное движение курсора без прерываний.
 * 
 * @param work Указатель на структуру работы
 */
static void tps43_work_handler(struct k_work *work) {
    struct tps43_drv_data *drv_data = CONTAINER_OF(work, struct tps43_drv_data, work);
    const struct device *dev = drv_data->dev;
    const struct tps43_config *config = dev->config;
    int ret;

    // Захватываем семафор для защиты всех операций I2C от прерывания
    // Это предотвращает конфликты при одновременном доступе к тачпаду
    k_sem_take(&drv_data->lock, K_FOREVER);

    // Обновляем время последней активности для управления питанием
    drv_data->last_activity_time = k_uptime_get_32();
    
    // Если устройство было в suspend, пробуждаем его
    // Используем lock_held=true, так как семафор уже захвачен
    if (drv_data->suspended && config->enable_power_management) {
        tps43_set_suspend_internal(dev, false, true);
    }

    // Определяем события
    bool is_scroll_active = drv_data->scroll_active;
    bool is_drag_active = drv_data->drag_active;

    uint8_t sys_info = 0;
    ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_1, &sys_info);
    if (ret < 0) {
        LOG_ERR("Ошибка чтения системной информации: %d", ret);
        goto done;
    }

    uint8_t gestures_events[2];
    ret = read_sequence_registers(dev, TPS43_REG_GESTURE_EVENTS_0, &gestures_events, 2);
    if (ret < 0) {
        LOG_ERR("Ошибка чтения событий жестов: %d", ret);
        goto done;
    }
    
    if (gestures_events[0] != 0 || gestures_events[1] != 0) {

        LOG_INF("Жесты: Одиночное=0x%02X, Мульти=0x%02X", gestures_events[0], gestures_events[1]);

        if (gestures_events[0] & TPS43_SINGLE_TAP) {
            LOG_INF("Одиночное касание → ЛЕВАЯ КНОПКА");
            input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER);
            input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);  
        }
        if (gestures_events[1] & TPS43_TWO_FINGER_TAP) {
            LOG_INF("Касание двумя пальцами → ПРАВАЯ КНОПКА");
            input_report_key(dev, INPUT_BTN_1, 1, true, K_FOREVER);  
            input_report_key(dev, INPUT_BTN_1, 0, true, K_FOREVER); 
        }
        if ((gestures_events[0] & TPS43_PRESS_AND_HOLD) && (!(is_drag_active))) {
            LOG_INF("Обнаружено нажатие и удержание - ПЕРЕТАСКИВАНИЕ (УДЕРЖАНИЕ ЛЕВОЙ КНОПКИ)");
            // установить внутренний флаг на перетаскивание и нажимает левую кнопку мыши
            is_drag_active = true;
            input_report_key(dev, INPUT_BTN_0, 1, true, K_FOREVER); 
        }
        if ((!(gestures_events[0] & TPS43_PRESS_AND_HOLD)) && (is_drag_active)) {
            LOG_INF("Обнаружено окончание нажатия и удержания - ОТПУСКАНИЕ (ОТПУСК ЛЕВОЙ КНОПКИ)");
            // установить внутренний флаг на перетаскивание и нажимает левую кнопку мыши
            is_drag_active = false;
            input_report_key(dev, INPUT_BTN_0, 0, true, K_FOREVER);   // отпускание + синхронизация
        }
        if (gestures_events[1] & TPS43_SCROLL) {
            LOG_INF("Обнаружена прокрутка - Прокрутка");
            // устанавливаем признак скролла для обработки в блоке tp_movement
            is_scroll_active = true;
        }
    }

    if (sys_info & TPS43_TP_MOVEMENT) {
        int16_t rel_x = 0, rel_y = 0;
        ret = tps43_i2c_read_reg16(dev, TPS43_REG_REL_X, (uint16_t*)&rel_x);
        if (ret < 0) {
            LOG_ERR("Ошибка чтения REL_X: %d", ret);
            goto done;
        }
        ret = tps43_i2c_read_reg16(dev, TPS43_REG_REL_Y, (uint16_t*)&rel_y);
        if (ret < 0) {
            LOG_ERR("Ошибка чтения REL_Y: %d", ret);
            goto done;
        }
        // Отправляем движение курсора
        if (rel_x != 0 || rel_y != 0) {
            if (rel_x != 0 ) {
                int32_t scaled_x = ((int32_t)rel_x * config->sensitivity) / 100;
                rel_x = (int16_t)CLAMP(scaled_x, INT16_MIN, INT16_MAX);
            }
            if (rel_y != 0) { 
                int32_t scaled_y = ((int32_t)rel_y * config->sensitivity) / 100;
                rel_y = (int16_t)CLAMP(scaled_y, INT16_MIN, INT16_MAX);
            }
            LOG_INF("Отправка движения: dx=%d, dy=%d", rel_x, rel_y);

            // Обработка свайпов тремя пальцами
            if (config->swipes) {
                uint8_t num_fingers = 0;
                ret = tps43_i2c_read_reg8(dev, TPS43_REG_NUM_FINGERS, &num_fingers);
                if (ret < 0) {
                    LOG_ERR("Ошибка чтения NUM_FINGERS: %d", ret);
                    goto done;
                }
                if (num_fingers == 3) {
                    if (rel_x < 0) {
                        LOG_INF("Свайп 3 пальцами влево - кнопка 6 мыши");
                        input_report_key(dev, INPUT_BTN_6, 1, true, K_FOREVER);
                        input_report_key(dev, INPUT_BTN_6, 0, true, K_FOREVER);
                    }
                    if (rel_x > 0) {
                        LOG_INF("Свайп 3 пальцами вправо - кнопка 7 мыши");
                        input_report_key(dev, INPUT_BTN_7, 1, true, K_FOREVER);
                        input_report_key(dev, INPUT_BTN_7, 0, true, K_FOREVER);
                    }
                }
            }

        
            if (is_scroll_active) {
                // Обработка скролла: оставляем только доминирующую ось
                if (abs(rel_x) > abs(rel_y)) {
                    // Горизонтальный скролл
                    if (config->invert_scroll_x) {
                        rel_x = -rel_x;
                    }
                    int16_t wheel = (rel_x * config->scroll_sensitivity) / 100;
                    input_report_rel(dev, INPUT_REL_HWHEEL, wheel, true, K_FOREVER);
                } else {
                    // Вертикальный скролл
                    if (config->invert_scroll_y) {
                        rel_y = -rel_y;
                    }
                    int16_t wheel = (rel_y * config->scroll_sensitivity) / 100;
                    input_report_rel(dev, INPUT_REL_WHEEL, wheel, true, K_FOREVER);
                }
                is_scroll_active = false;
            } else {
                // Обычное движение курсора
                input_report_rel(dev, INPUT_REL_X, rel_x, false, K_FOREVER);
                input_report_rel(dev, INPUT_REL_Y, rel_y, true, K_FOREVER);
            }
        }
    }

done:
    // Сохраняем для следующего вызова
    drv_data->scroll_active = is_scroll_active;
    drv_data->drag_active = is_drag_active;
    tps43_end_communication_window(dev);
    
    // Освобождаем семафор после завершения всех операций I2C
    k_sem_give(&drv_data->lock);
}

/**
 * @brief Сбрасывает внутренние значения состояния драйвера
 * 
 * Инициализирует все флаги состояния драйвера в начальные значения.
 * Используется при инициализации и сбросе устройства.
 * 
 * @param dev Указатель на устройство тачпада
 * @return 0 при успехе
 */
static int tps43_reset_values(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;

    drv_data->device_ready = false;
    drv_data->initialized = false;
    drv_data->scroll_active = false;
    drv_data->drag_active = false;

    LOG_INF("Сброс значений");
    return 0;
}

/**
 * @brief Конфигурирует системные регистры тачпада для работы
 * 
 * Настраивает регистры тачпада для отслеживания событий касания, жестов и движения.
 * Включает необходимые жесты (single tap, press and hold, scroll, two finger tap),
 * настраивает инверсию осей и устанавливает флаг завершения настройки.
 * 
 * @param dev Указатель на устройство тачпада
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int tps43_configure_device(const struct device *dev) {

    const struct tps43_config *config = dev->config;
    int ret;

    // запись в TPS43_REG_SYSTEM_CONFIG_1 событий для отслеживания  
    uint8_t events_to_track = TPS43_TP_EVENT | TPS43_EVENT_MODE;
    
    // Жесты (single_tap, press_and_hold, scroll, two_finger_tap)
    if (config->single_tap || config->press_and_hold || 
        config->scroll || config->two_finger_tap) {
        events_to_track |= TPS43_GESTURE_EVENT;
    }
    
    // Touch events для абсолютных координат
    events_to_track |= TPS43_TOUCH_EVENT;
    
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONFIG_1, events_to_track);
    if (ret != 0) {
        LOG_WRN("Ошибка записи событий для отслеживания: %d", ret);
        return ret;
    }
    LOG_INF("События сконфигурированы: 0x%02X", events_to_track);

    // конфигурация осей
    uint8_t xy_config = 0;
    xy_config |= config->invert_x ? TPS43_FLIP_X : 0;
    xy_config |= config->invert_y ? TPS43_FLIP_Y : 0;
    xy_config |= config->switch_xy ? TPS43_SWITCH_XY_AXIS : 0;
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_XY_CONFIG_0, xy_config);
    if (ret != 0) {
        LOG_WRN("Ошибка записи конфигурации XY: %d", ret);
        return ret;
    }

    // включение одиночных жестов на уровне железа
    if (config->single_tap || config->press_and_hold || config->swipes) {
        uint8_t single_gestures = 0;
        single_gestures |= config->single_tap ? TPS43_SINGLE_TAP : 0;
        single_gestures |= config->press_and_hold ? TPS43_PRESS_AND_HOLD : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_UP : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_DOWN : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_LEFT : 0;
        single_gestures |= config->swipes ? TPS43_SWIPE_RIGHT : 0;
        
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SINGLE_FINGER_GESTURES, single_gestures);
        if (ret != 0) {
            LOG_WRN("Ошибка конфигурации одиночных жестов: %d", ret);
            return ret;
        }
        LOG_INF("Одиночные жесты включены: 0x%02X", single_gestures);
    }

    // включение мульти-жестов
    if (config->two_finger_tap || config->scroll) {
        uint8_t multi_gestures = 0;
        multi_gestures |= config->two_finger_tap ? TPS43_TWO_FINGER_TAP : 0;
        multi_gestures |= config->scroll ? TPS43_SCROLL : 0;
        
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_MULTI_FINGER_GESTURES, multi_gestures);
        if (ret != 0) {
            LOG_WRN("Ошибка конфигурации мульти-жестов: %d", ret);
            return ret;
        }
        LOG_INF("Мульти-жесты включены: 0x%02X", multi_gestures);
    }

    // конфигурация фильтров
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_FILTER_SETTINGS, config->filter_settings);
    if (ret != 0) {
        LOG_WRN("Ошибка записи настроек фильтров: %d", ret);
        return ret;
    }
    LOG_INF("Настройки фильтров установлены: 0x%02X", config->filter_settings);

    // установка признака завершения конфигурации
    ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONFIG_0, TPS43_SETUP_COMPLETE);
    if (ret != 0) {
        LOG_WRN("Ошибка записи флага завершения настройки: %d", ret);
        return ret;
    }

    return 0;
}

/**
 * @brief Проверяет состояние сброса устройства и выполняет реконфигурацию
 * 
 * Ожидает готовности устройства после сброса, проверяет флаг SHOW_RESET
 * и отправляет подтверждение сброса (ACK_RESET) при необходимости.
 * Затем выполняет полную конфигурацию устройства.
 * 
 * @param dev Указатель на устройство тачпада
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int check_reset_and_reconfigure(const struct device *dev) {
    struct tps43_drv_data *drv_data = dev->data;
    int ret;
    uint8_t sys_info = 0;
    uint8_t wait_count = 0;
    const uint8_t max_wait_count = 50;

    // Ожидание готовности устройства
    do {
        ret = tps43_i2c_read_reg8(dev, TPS43_REG_SYSTEM_INFO_0, &sys_info);
        if (ret < 0) {
            k_sleep(K_MSEC(100));
            wait_count++;
            if (wait_count >= max_wait_count) {
                LOG_ERR("Устройство не отвечает после %d мс", wait_count * 100);
                return -ETIMEDOUT;
            }
        }
    } while (ret < 0);
    
    LOG_INF("Устройство готово через %d мс", wait_count * 100);

    // после сброса устанавливаем флаг на подтверждение что сброс был выполнен
    if (sys_info & TPS43_SHOW_RESET) {
        LOG_INF("Обнаружен SHOW_RESET, отправка ACK_RESET");
        ret = tps43_i2c_write_reg8(dev, TPS43_REG_SYSTEM_CONTROL_0, TPS43_ACK_RESET);
        if (ret != 0) {
            LOG_ERR("Ошибка отправки ACK_RESET: %d", ret);
            return ret;
        }
        k_sleep(K_MSEC(10));
    }

    ret = tps43_configure_device(dev);
    if (ret != 0) {
        LOG_ERR("Ошибка конфигурации устройства: %d", ret);
        return ret;
    }

    drv_data->device_ready = true;
    
    return 0;
}

/**
 * @brief Публичная функция для перевода тачпада в suspend/resume
 * 
 * @param dev Указатель на устройство тачпада
 * @param suspend true - перевести в suspend, false - вывести из suspend
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int tps43_set_suspend(const struct device *dev, bool suspend) {
    return tps43_set_suspend_internal(dev, suspend, false);
}
 
/**
 * @brief Обработчик отложенной работы для автоматического перевода в suspend
 * 
 * Эта функция периодически проверяет время бездействия тачпада и автоматически
 * переводит его в режим suspend, если прошло достаточно времени без активности.
 * Также пробуждает тачпад, если обнаружена активность.
 * 
 * Работает только если suspend_timeout_ms > 0 и enable_power_management = true.
 * 
 * @param work Указатель на отложенную работу
 */
static void tps43_suspend_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct tps43_drv_data *drv_data = CONTAINER_OF(dwork, struct tps43_drv_data, suspend_work);
    const struct device *dev = drv_data->dev;
    const struct tps43_config *config = dev->config;

    // Если управление питанием отключено или таймаут не задан, не делаем ничего
    if (!config->enable_power_management || config->suspend_timeout_ms <= 0) {
        return;
    }

    // Вычисляем время бездействия
    int32_t now = k_uptime_get_32();
    int32_t inactive_time = now - drv_data->last_activity_time;

    // Если время бездействия превысило таймаут и устройство не в suspend - переводим в suspend
    if (inactive_time >= config->suspend_timeout_ms && !drv_data->suspended) {
        LOG_INF("Автоматический переход в suspend после %d мс бездействия", inactive_time);
        tps43_set_suspend(dev, true);
    }
    // Если время бездействия меньше таймаута и устройство в suspend - пробуждаем
    // (это может произойти, если активность была обнаружена через прерывание RDY)
    else if (inactive_time < config->suspend_timeout_ms && drv_data->suspended) {
        LOG_INF("Автоматическое пробуждение из suspend (обнаружена активность)");
        tps43_set_suspend(dev, false);
    }

    // Планируем следующую проверку через 10 секунд (если таймаут задан)
    if (config->suspend_timeout_ms > 0) {
        k_work_schedule(&drv_data->suspend_work, K_MSEC(10000));
    }
}

/**
 * @brief Инициализирует драйвер тачпада TPS43
 * 
 * Выполняет полную инициализацию драйвера: проверяет доступность I2C шины,
 * выполняет аппаратный сброс через GPIO RST (если подключен), ожидает готовности
 * устройства, конфигурирует регистры тачпада и настраивает прерывания GPIO RDY.
 * Также инициализирует систему управления питанием при необходимости.
 * 
 * @param dev Указатель на устройство тачпада
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
static int tps43_init(const struct device *dev) {

    struct tps43_drv_data *drv_data = dev->data;
    const struct tps43_config *config = dev->config;
    int ret;

    drv_data->dev = dev;

    LOG_INF("=== Драйвер Azoteq tps43 для устройства %s ===", dev->name);
    
    // Проверка I2C шины
    if (!device_is_ready(config->i2c_bus.bus)) {
        LOG_ERR("Шина I2C не доступна");
        return -ENODEV;
    }
    
    LOG_INF("I2C шина: %s", config->i2c_bus.bus->name);
    LOG_INF("I2C адрес: 0x%02x", config->i2c_bus.addr);

    ret = tps43_reset_values(dev);
    if (ret != 0) {
        LOG_ERR("Ошибка сброса значений: %d", ret);
        return ret;
    }

    // GPIO сброс через hardware RST
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
        
        LOG_INF("Аппаратный сброс завершен");
    }

    // проверка SHOW_RESET и конфигурация
    ret = check_reset_and_reconfigure(dev);
    if (ret != 0) {
        LOG_ERR("Ошибка конфигурации устройства: %d", ret);
        return ret;
    }

    // прерывания RDY настраиваем только ПОСЛЕ конфигурации устройства!
    if (config->rdy_gpio.port != NULL) {
        ret = gpio_pin_configure_dt(&config->rdy_gpio, GPIO_INPUT);
        if (ret != 0) {
            LOG_WRN("Ошибка конфигурации RDY GPIO: %d", ret);
        } else {
            ret = gpio_pin_interrupt_configure_dt(&config->rdy_gpio, 
                                                    GPIO_INT_EDGE_TO_ACTIVE);
            if (ret == 0) {
                gpio_init_callback(&drv_data->rdy_cb, tps43_rdy_callback, 
                                    BIT(config->rdy_gpio.pin));
                ret = gpio_add_callback(config->rdy_gpio.port, &drv_data->rdy_cb);
                if (ret == 0) {
                    LOG_INF("Прерывание RDY сконфигурировано");
                } else {
                    LOG_WRN("Ошибка добавления callback RDY: %d", ret);
                }
            }
        }
    }

    drv_data->initialized = true;
    drv_data->suspended = false;
    drv_data->last_activity_time = k_uptime_get_32();

    // Инициализируем семафор для защиты операций I2C
    // Первый параметр - начальное количество (1 = доступен)
    // Второй параметр - максимальное количество (1 = бинарный семафор)
    k_sem_init(&drv_data->lock, 1, 1);

    k_work_init(&drv_data->work, tps43_work_handler);
    
    // Инициализируем отложенную работу для автоматического suspend
    // если включено управление питанием и задан таймаут
    if (config->enable_power_management && config->suspend_timeout_ms > 0) {
        k_work_init_delayable(&drv_data->suspend_work, tps43_suspend_work_handler);
        // Запускаем первую проверку через 10 секунд
        k_work_schedule(&drv_data->suspend_work, K_MSEC(10000));
        LOG_INF("Автоматическое управление питанием включено (таймаут: %d мс)", 
                config->suspend_timeout_ms);
    }
    
    LOG_INF("Драйвер TPS43 успешно инициализирован");
    return 0;
}

 
#define TPS43_INIT(inst)                                                                             \
    static struct tps43_drv_data tps43_##inst##_drvdata = {                                          \
        .device_ready = false,                                                                       \
        .initialized = false,                                                                        \
        .scroll_active = false,                                                                      \
        .drag_active = false,                                                                        \
        .suspended = false,                                                                          \
        .last_activity_time = 0,                                                                     \
    };                                                                                               \
                                                                                                     \
    static const struct tps43_config tps43_##inst##_config = {                                       \
        .i2c_bus = I2C_DT_SPEC_INST_GET(inst),                                                       \
        .rdy_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rdy_gpios, {0}),                                  \
        .rst_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, rst_gpios, {0}),                                  \
        .single_tap = DT_INST_PROP(inst, single_tap),                                                \
        .press_and_hold = DT_INST_PROP(inst, press_and_hold),                                        \
        .two_finger_tap = DT_INST_PROP(inst, two_finger_tap),                                        \
        .scroll = DT_INST_PROP(inst, scroll),                                                        \
        .swipes = DT_INST_PROP(inst, swipes),                                                        \
        .invert_x = DT_INST_PROP(inst, invert_x),                                                    \
        .invert_y = DT_INST_PROP(inst, invert_y),                                                    \
        .switch_xy = DT_INST_PROP(inst, switch_xy),                                                  \
        .invert_scroll_x = DT_INST_PROP(inst, invert_scroll_x),                                      \
        .invert_scroll_y = DT_INST_PROP(inst, invert_scroll_y),                                      \
        .sensitivity = DT_INST_PROP_OR(inst, sensitivity, 100),                                      \
        .scroll_sensitivity = DT_INST_PROP_OR(inst, scroll_sensitivity, 50),                         \
        .enable_power_management = DT_INST_PROP_OR(inst, enable_power_management, true),             \
        .suspend_timeout_ms = DT_INST_PROP_OR(inst, suspend_timeout_ms, 0),                          \
        .filter_settings = DT_INST_PROP_OR(inst, filter_settings, 0x0F),                             \
    };                                                                                               \
                                                                                                     \
    DEVICE_DT_INST_DEFINE(inst, tps43_init, NULL, &tps43_##inst##_drvdata, &tps43_##inst##_config,   \
                        POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY, NULL);                              \
    BUILD_ASSERT(DT_INST_REG_ADDR(inst) == TPS43_I2C_ADDR, "Несоответствие адреса I2C");


DT_INST_FOREACH_STATUS_OKAY(TPS43_INIT)

/**
 * @brief Публичная функция для управления режимом сна тачпада
 * 
 * Эта функция используется системой управления питанием ZMK (через tps43_idle_sleeper)
 * для перевода тачпада в режим сна при переходе клавиатуры в состояние idle/sleep.
 * 
 * @param dev Указатель на устройство тачпада
 * @param sleep true - перевести в режим сна, false - пробудить
 * @return 0 при успехе, отрицательный код ошибки при неудаче
 */
int tps43_set_sleep(const struct device *dev, bool sleep) {
    if (dev == NULL) {
        return -EINVAL;
    }
    return tps43_set_suspend(dev, sleep);
}
