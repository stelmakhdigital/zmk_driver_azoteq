/**
 * @file tps43_idle_sleeper.c
 * @brief Интеграция управления питанием тачпада TPS43 с системой управления питанием ZMK
 * 
 * Этот модуль подписывается на события изменения состояния активности ZMK и автоматически
 * переводит тачпад в режим сна при переходе клавиатуры в состояние idle/sleep, что позволяет
 * значительно снизить энергопотребление.
 * 
 * Работает совместно с автоматическим управлением питанием в основном драйвере (tps43.c),
 * которое отслеживает время бездействия тачпада.
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include "tps43.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(tps43_sleeper, CONFIG_INPUT_LOG_LEVEL);

/**
 * @brief Макрос для получения указателя на устройство из devicetree
 */
#define GET_TPS43_DEV(node_id) DEVICE_DT_GET(node_id),

/**
 * @brief Массив указателей на все устройства TPS43 в системе
 * 
 * Автоматически заполняется из devicetree для всех устройств с совместимостью azoteq_tps43
 */
static const struct device *tps43_devs[] = {DT_FOREACH_STATUS_OKAY(azoteq_tps43, GET_TPS43_DEV)};

/**
 * @brief Обработчик события изменения состояния активности ZMK
 * 
 * Эта функция вызывается при изменении состояния активности клавиатуры:
 * - ZMK_ACTIVITY_ACTIVE - клавиатура активна, тачпад должен быть пробужден
 * - ZMK_ACTIVITY_IDLE - клавиатура в режиме ожидания, тачпад переводится в sleep
 * - ZMK_ACTIVITY_SLEEP - клавиатура в режиме сна, тачпад переводится в sleep
 * 
 * @param eh Указатель на событие изменения состояния активности
 * @return 0 при успешной обработке
 */
static int on_activity_state(const zmk_event_t *eh) {
    const struct zmk_activity_state_changed *state_ev = as_zmk_activity_state_changed(eh);
    if (!state_ev) {
        LOG_WRN("Событие не найдено, игнорируем");
        return 0;
    }

    /* Переводим тачпад в sleep если состояние не ACTIVE, иначе пробуждаем */
    bool should_sleep = (state_ev->state != ZMK_ACTIVITY_ACTIVE);
    
    LOG_INF("Изменение состояния активности ZMK: %d -> тачпад %s", 
            state_ev->state, should_sleep ? "sleep" : "active");
    
    // Применяем изменение ко всем устройствам TPS43 в системе
    for (size_t i = 0; i < ARRAY_SIZE(tps43_devs); i++) {
        int ret = tps43_set_sleep(tps43_devs[i], should_sleep);
        if (ret != 0) {
            LOG_WRN("Ошибка управления питанием тачпада %zu: %d", i, ret);
        }
    }

    return 0;
}

// Регистрируем слушатель событий ZMK
ZMK_LISTENER(tps43_idle_sleeper, on_activity_state);
// Подписываемся на события изменения состояния активности
ZMK_SUBSCRIPTION(tps43_idle_sleeper, zmk_activity_state_changed);