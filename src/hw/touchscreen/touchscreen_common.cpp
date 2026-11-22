#include "touchscreen_common.hpp"
#include "touchscreen.hpp"
#include <gui/od3d/gesture_od3d.hpp>
#include <timing.h>
#include <option/has_side_leds.h>
#include <logging/log.hpp>
#include <bsod/bsod.h>

#if HAS_SIDE_LEDS()
    #include <leds/side_strip_handler.hpp>
#endif

LOG_COMPONENT_DEF(Touch, logging::Severity::info);

METRIC_DEF(metric_touch_event_, "touch_evt", METRIC_VALUE_STRING, 0, METRIC_ENABLED);
METRIC_DEF(metric_touch_pos, "touch_pos", METRIC_VALUE_CUSTOM, 0, METRIC_ENABLED);

metric_t *metric_touch_event() {
    return &metric_touch_event_;
}

Touchscreen_Base::LenientClickGuard::LenientClickGuard() {
    touchscreen.lenient_click_allowed_++;
}
Touchscreen_Base::LenientClickGuard::~LenientClickGuard() {
    touchscreen.lenient_click_allowed_--;
}

bool Touchscreen_Base::is_enabled() const {
    // !!! is_disabled_till_reset_ check must be before config_store
    return !is_disabled_till_reset_ && config_store().touch_enabled.get();
}

void Touchscreen_Base::set_enabled(bool set) {
    if (set == is_enabled()) {
        return;
    }

    config_store().touch_enabled.set(set);
    metric_record_string(metric_touch_event(), "set_enabled %i", set);
    log_info(Touch, "set enabled %i", set);
}

TouchscreenEvent Touchscreen_Base::get_event() {
    if (is_last_event_consumed_) {
        return TouchscreenEvent();
    }

#if HAS_SIDE_LEDS()
    // We cannot put this into recognize_gesture or such,
    // because those are in the timer interrupt and ActivityPing uses mutex.
    leds::SideStripHandler::instance().activity_ping();
#endif

    is_last_event_consumed_ = true;
    return last_event_;
}

TouchscreenEvent Touchscreen_Base::get_last_event() const {
    return last_event_;
}

void Touchscreen_Base::update() {
    if (!is_hw_ok_) {
        return;
    }

    TouchState touch_state = last_touch_state_;
    touch_state.invalidate = false;
    update_impl(touch_state);

    if (touch_state.multitouch_point_count > 0) {
        metric_record_custom(&metric_touch_pos, " x=%i,y=%i", touch_state.multitouch_points[0].position.x, touch_state.multitouch_points[0].position.y);
    }

    // Gesture reported as invalid -> invalidate
    if (touch_state.invalidate) {
        gesture_recognition_state_ = GestureRecognitionState::invalid;
    }

    // Touch start -> set up gesture recognition
    else if (touch_state.multitouch_point_count == 1 && gesture_recognition_state_ == GestureRecognitionState::idle) {
        gesture_recognition_state_ = GestureRecognitionState::active;
        gesture_start_pos_ = touch_state.multitouch_points[0].position;
        gesture_start_ms_ = ticks_ms();
    }

    // Multiple touch points -> invalid gesture state
    else if (touch_state.multitouch_point_count > 1) {
        gesture_recognition_state_ = GestureRecognitionState::invalid;
    }

    // Touch end - recognize gesture
    else if (touch_state.multitouch_point_count == 0 && gesture_recognition_state_ != GestureRecognitionState::idle) {
        recognize_gesture();
        gesture_recognition_state_ = GestureRecognitionState::idle;
    }

    last_touch_state_ = touch_state;
}

void Touchscreen_Base::recognize_gesture() {
    if (gesture_recognition_state_ == GestureRecognitionState::invalid) {
        return;
    }

    debug_assert(last_touch_state_.multitouch_point_count > 0);
    const point_ui16_t last_touch_pos = last_touch_state_.multitouch_points[0].position;

    const point_i16_t touch_pos_diff = point_i16_t::from_point(last_touch_pos) - point_i16_t::from_point(gesture_start_pos_);

    /// Distance from the gesture_start_pos that is still considered a click
    const int16_t click_max_diff = lenient_click_allowed_ ? 3 : 0;

    TouchscreenEvent event {
        .pos_x = gesture_start_pos_.x,
        .pos_y = gesture_start_pos_.y,
    };

    // OneDrop3D (interface G) : balayage horizontal (retour) plus exigeant que chez Prusa : 60 px, en moins de 600 ms
    // (_count, rien, est déjà la valeur d'un événement vide)
    event.type = od3d::gesture::classify(touch_pos_diff.x, touch_pos_diff.y, ticks_ms() - gesture_start_ms_, click_max_diff);

    metric_record_string(metric_touch_event(), "evt t%i x%i y%i", static_cast<int>(event.type), event.pos_x, event.pos_y);

    if (event) {
        last_event_ = event;
        is_last_event_consumed_ = false;
    }
}
