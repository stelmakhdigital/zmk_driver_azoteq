/**
 * @file tps43_typing.c
 * @brief Disable-while-typing: сообщает драйверу о нажатиях клавиш на этой половине.
 *
 * Драйвер игнорирует тапы (typing-tap-timeout-ms) и, опционально, движение
 * (typing-move-timeout-ms) в течение заданного времени после нажатия клавиши.
 * На сплите видны только клавиши той половины, где стоит тачпад.
 */

#include <zephyr/kernel.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include "tps43.h"

static int tps43_on_position(const zmk_event_t *eh) {
    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);
    if (ev && ev->state) {
        tps43_notify_keypress();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(tps43_typing, tps43_on_position);
ZMK_SUBSCRIPTION(tps43_typing, zmk_position_state_changed);
