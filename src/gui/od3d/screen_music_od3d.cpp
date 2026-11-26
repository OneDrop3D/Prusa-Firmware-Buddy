/// @file
#include "screen_music_od3d.hpp"
#include "menu_items_od3d.hpp"
#include "menu_od3d.hpp"
#include "value_editor_od3d.hpp"

#include <cmath>
#include <common/marlin_vars.hpp>
#include <config_store/store_instance.hpp>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <filepath_operation.h>
#include <span>
#include <tuple>
#include <utility>
#include <i18n.h>
#include <img_resources.hpp>
#include <marlin_client.hpp>
#include <strings.h>
#include <window_msgbox.hpp>

namespace od3d {

namespace {

    // Trois bandeaux comme les moteurs du MMU (screen_mmu_od3d.cpp) : fichier, moteurs, lecture
    constexpr Rect16 panel_rect(int i) {
        return Rect16(style::margin, style::top + 78 * i, 464, 71);
    }
    constexpr Rect16 note_rect(int i) {
        return Rect16(150, panel_rect(i).Top() + 7, 310, height(Font::small));
    }
    constexpr int16_t row_top(int i) {
        return panel_rect(i).Top() + 28;
    }
    constexpr Rect16 button_body(int row, int i) {
        return Rect16(18 + 150 * i, row_top(row), 142, 36);
    }
    constexpr Rect16 previous_body = Rect16(18, row_top(0), 68, 36);
    constexpr Rect16 next_body = Rect16(392, row_top(0), 68, 36);
    constexpr Rect16 name_rect = Rect16(94, row_top(0), 290, 36);
    static_assert(style::inside(panel_rect(0), style::with_ring(next_body)) && style::inside(panel_rect(2), style::with_ring(button_body(2, 2))));
    static_assert(name_rect.Left() > style::with_ring(previous_body).Right() && name_rect.Right() < style::with_ring(next_body).Left());
    static_assert(style::inside(style::content, panel_rect(2)));

    enum Tag : uint8_t {
        tag_previous = 0,
        tag_next,
        tag_voices,
        tag_tempo,
        tag_pitch,
        tag_play,
        tag_stop,
        tag_after_print,
    };

    constexpr NumericInputConfig tempo_config {
        .min_value = music::min_tempo,
        .max_value = music::max_tempo,
        .step = 5,
        .unit = Unit::percent,
    };

    constexpr NumericInputConfig pitch_config {
        .min_value = -music::max_transpose,
        .max_value = music::max_transpose,
    };

    void setup_note(window_text_t &note) {
        note.set_font(Font::small);
        note.SetAlignment(Align_t::RightCenter());
        note.SetTextColor(style::text_muted);
        note.SetBackColor(style::surface);
        note.SetPadding({ 0, 0, 0, 0 });
    }

    /// Un fichier MIDI d'après son nom long (.mid ou .midi, sans tenir compte de la casse)
    bool is_midi(const char *name) {
        const char *dot = strrchr(name, '.');
        return dot && (strcasecmp(dot, ".mid") == 0 || strcasecmp(dot, ".midi") == 0);
    }

    /// Parcourt les .mid de la racine de la clé dans l'ordre du dossier ; pour le n° wanted (ou celui dont le nom
    /// court est wanted_short), copie le nom long et le nom court. Rend le nombre de fichiers et le n° trouvé.
    std::pair<int16_t, int16_t> list_midi(int16_t wanted, const char *wanted_short, std::span<char> name, std::span<char> short_name) {
        DIR *dir = opendir("/usb");
        if (!dir) {
            return { 0, -1 };
        }
        int16_t count = 0;
        int16_t found = -1;
        while (const dirent *ent = readdir(dir)) {
            const char *long_name = dirent_lfn(ent);
            // Fichiers seulement, sans les « ._x.mid » cachés que macOS laisse sur la clé
            if (!(ent->d_type & DT_REG) || (ent->d_type & DT_DIR) || long_name[0] == '.' || !is_midi(long_name) || strlen(ent->d_name) >= short_name.size()) {
                continue;
            }
            const bool match = wanted_short ? strcasecmp(ent->d_name, wanted_short) == 0 : count == wanted;
            if (match && found < 0) {
                found = count;
                strlcpy(name.data(), long_name, name.size());
                strlcpy(short_name.data(), ent->d_name, short_name.size());
            }
            if (count < INT16_MAX) {
                ++count;
            }
        }
        closedir(dir);
        return { count, found };
    }

    string_view_utf8 status_text(music::Status s) {
        switch (s) {
        case music::Status::idle:
            return string_view_utf8::MakeNULLSTR();
        case music::Status::playing:
            return _("Playing...");
        case music::Status::done:
            return _("Finished");
        case music::Status::stopped:
            return _("Stopped");
        case music::Status::no_file:
            return _("File not found");
        case music::Status::bad_file:
            return _("Unreadable MIDI file");
        case music::Status::no_notes:
            return _("No notes to play");
        case music::Status::printing:
            return _("Not while printing.");
        case music::Status::no_room:
            return _("No room on this axis");
        case music::Status::no_memory:
            return _("Not enough memory, try again later.");
        }
        return string_view_utf8::MakeNULLSTR();
    }

    // Retour du 26/11 : les voix (moteurs du haut A et B, plateau en Z) au lieu d'un seul axe
    constexpr const char *motors_labels[music::motors_count] = { N_("3 voices"), N_("2 voices"), N_("Melody") };

} // namespace

ScreenMusic::ScreenMusic()
    : Screen(true)
    , header(this, _("Motor music"), true)
    , file_panel(this, panel_rect(0), _("File"))
    , file_note(this, note_rect(0), is_multiline::no)
    , previous(this, previous_body, tag_previous, Button::Look::secondary, string_view_utf8::MakeCPUFLASH("<"), nullptr, style::surface, 10)
    , file_name(this, name_rect, is_multiline::no)
    , next(this, next_body, tag_next, Button::Look::secondary, string_view_utf8::MakeCPUFLASH(">"), nullptr, style::surface, 10)
    , sound_panel(this, panel_rect(1), _("Sound settings"))
    , voices_button(this, button_body(1, 0), tag_voices, Button::Look::secondary, {}, nullptr, style::surface, 10)
    , tempo_button(this, button_body(1, 1), tag_tempo, Button::Look::secondary, {}, nullptr, style::surface, 10)
    , pitch_button(this, button_body(1, 2), tag_pitch, Button::Look::secondary, {}, nullptr, style::surface, 10)
    , play_panel(this, panel_rect(2), _("Playback"))
    , status_note(this, note_rect(2), is_multiline::no)
    , play_button(this, button_body(2, 0), tag_play, Button::Look::primary, _("Play"), &img::print_16x16, style::surface, 10)
    , stop_button(this, button_body(2, 1), tag_stop, Button::Look::outline, _("Stop"), &img::cancel_16x16, style::surface, 10)
    , after_print(this, button_body(2, 2), tag_after_print, Button::Look::secondary, {}, nullptr, style::surface, 10)
    , tabs(this, section_tab()) {
    ClrMenuTimeoutClose(); // l'écran reste ouvert pendant la lecture
    setup_note(file_note);
    setup_note(status_note);
    file_name.set_font(Font::normal);
    file_name.SetAlignment(Align_t::Center());
    file_name.SetTextColor(style::text_strong);
    file_name.SetBackColor(style::surface);
    file_name.SetPadding({ 0, 0, 0, 0 });

    media = marlin_vars().media_inserted.get();
    scan();
    show_after_print();
    shown_status = music::status();
    shown_playing = music::is_playing();
    show_status();
    update();
    if (play_button.IsEnabled()) {
        play_button.SetFocus();
    } else if (window_t *first = GetFirstEnabledSubWin()) {
        first->SetFocus();
    }
}

void ScreenMusic::scan() {
    const auto saved = config_store().od3d_music_file.get();
    const bool has_saved = saved[0] != '\0' && memchr(saved.data(), '\0', saved.size());
    auto [n, found] = has_saved ? list_midi(-1, saved.data(), name, short_name) : std::pair<int16_t, int16_t> { 0, -1 };
    if (found < 0) {
        std::tie(n, found) = list_midi(0, nullptr, name, short_name);
    }
    count = n;
    index = found;
    if (index < 0) {
        name[0] = '\0';
        short_name[0] = '\0';
    }
    tuning = index >= 0 ? music::load_tuning(music::tuning_path, short_name.data()) : music::Tuning {};
    show_tuning();

    if (count == 0) {
        file_note.SetText(string_view_utf8::MakeNULLSTR());
        file_name.SetText(marlin_vars().media_inserted.get() ? _("No .mid file") : _("Insert a USB drive"));
        file_name.SetTextColor(style::text_muted);
    } else {
        snprintf(count_text.data(), count_text.size(), "%d / %d", index + 1, count);
        file_note.SetText(string_view_utf8::MakeRAM(count_text.data()));
        file_name.SetText(string_view_utf8::MakeRAM(name.data()));
        file_name.SetTextColor(style::text_strong);
    }
    file_note.Invalidate();
    file_name.Invalidate();
}

void ScreenMusic::select(int wanted) {
    if (count <= 0) {
        return;
    }
    wanted = (wanted % count + count) % count;
    const auto [n, found] = list_midi(int16_t(wanted), nullptr, name, short_name);
    if (found >= 0) {
        // Retenu tout de suite : c'est aussi le morceau joué après chaque impression
        config_store().od3d_music_file.set(short_name);
    }
    scan();
    (void)n;
}

void ScreenMusic::show_tuning() {
    const uint8_t chosen = config_store().od3d_music_motors.get();
    voices_button.set(Button::Look::secondary, _(motors_labels[chosen < music::motors_count ? chosen : 0]));
    tempo_button.set(Button::Look::secondary, _("Speed %d%%").formatted(tempo_params, int(tuning.tempo)));
    pitch_button.set(Button::Look::secondary, _("Pitch %+d").formatted(pitch_params, int(tuning.transpose)));
}

void ScreenMusic::edit_tuning(bool pitch) {
    // Chaque pas s'entend tout de suite si le morceau joue ; Annuler remet la valeur de départ
    const auto v = pitch
        ? ValueEditor::exec(_("Pitch (semitones)"), tuning.transpose, pitch_config, [](float v) {
              auto t = music::live_tuning();
              t.transpose = int8_t(std::lround(v));
              music::set_live_tuning(t);
          })
        : ValueEditor::exec(_("Speed"), tuning.tempo, tempo_config, [](float v) {
              auto t = music::live_tuning();
              t.tempo = uint8_t(std::lround(v));
              music::set_live_tuning(t);
          });
    if (!v) {
        return;
    }
    if (pitch) {
        tuning.transpose = int8_t(std::lround(*v));
    } else {
        tuning.tempo = uint8_t(std::lround(*v));
    }
    show_tuning();
    if (!music::save_tuning(music::tuning_path, short_name.data(), tuning)) {
        MsgBoxWarning(_("Not saved: the USB drive refused the file."), Responses_Ok);
    }
}

void ScreenMusic::show_after_print() {
    const bool on = config_store().od3d_music_at_print_end.get();
    after_print.set(on ? Button::Look::selected : Button::Look::secondary, on ? _("After print: on") : _("After print: off"));
}

void ScreenMusic::show_status() {
    if (shown_pending) {
        status_note.SetText(_("Starting..."));
    } else {
        status_note.SetText(status_text(shown_playing ? music::Status::playing : shown_status));
    }
    status_note.Invalidate();
}

void ScreenMusic::update() {
    const bool inserted = marlin_vars().media_inserted.get();
    if (inserted != media) {
        media = inserted;
        scan();
    }

    const bool now_playing = music::is_playing();
    if (pending && (now_playing || music::run_count() != runs_at_request)) {
        pending = false;
    }
    const music::Status s = music::status();
    if (now_playing != shown_playing || s != shown_status || pending != shown_pending) {
        shown_playing = now_playing;
        shown_status = s;
        shown_pending = pending;
        show_status();
    }

    const bool busy = now_playing || pending;
    const bool printing = marlin_client::is_printing();
    previous.enable(!busy && count > 1);
    next.enable(!busy && count > 1);
    voices_button.enable(!busy);
    // Vitesse et hauteur restent réglables pendant l'écoute
    tempo_button.enable(index >= 0);
    pitch_button.enable(index >= 0);
    play_button.enable(!busy && !printing && index >= 0);
    stop_button.enable(busy);

    // Un élément désactivé ne garde pas le focus : il va à « Arrêter » au lancement, à « Jouer » à la fin, pas au
    // retour de l'en-tête
    if (window_t *focused = get_focused_child_window(); !focused || !focused->IsEnabled()) {
        if (window_t *to = busy ? &stop_button : play_button.IsEnabled() ? &play_button : GetFirstEnabledSubWin()) {
            to->SetFocus();
        }
    }
}

void ScreenMusic::play() {
    if (marlin_client::is_printing()) {
        MsgBoxWarning(_("Not while printing."), Responses_Ok);
        return;
    }
    if (index < 0) {
        return;
    }
    config_store().od3d_music_file.set(short_name);
    const uint16_t before = music::run_count();
    if (marlin_client::gcode_try("M1991") == marlin_client::GcodeTryResult::Submitted) {
        runs_at_request = before;
        pending = true;
    }
}

void ScreenMusic::windowEvent(window_t *sender, GUI_event_t event, void *param) {
    switch (event) {
    case GUI_event_t::LOOP:
        update();
        break;

    case GUI_event_t::CHILD_CLICK: {
        const uint8_t tag = static_cast<uint8_t>(reinterpret_cast<uintptr_t>(param));
        if (tag == tag_previous) {
            select(index - 1);
        } else if (tag == tag_next) {
            select(index + 1);
        } else if (tag == tag_voices) {
            config_store().od3d_music_motors.set(uint8_t((config_store().od3d_music_motors.get() + 1) % music::motors_count));
            show_tuning();
        } else if (tag == tag_tempo || tag == tag_pitch) {
            edit_tuning(tag == tag_pitch);
        } else if (tag == tag_play) {
            play();
        } else if (tag == tag_stop) {
            music::request_stop();
        } else if (tag == tag_after_print) {
            const bool on = !config_store().od3d_music_at_print_end.get();
            config_store().od3d_music_at_print_end.set(on);
            if (on && index >= 0) {
                config_store().od3d_music_file.set(short_name);
            }
            show_after_print();
        }
        update();
        return;
    }

    default:
        break;
    }
    Screen::windowEvent(sender, event, param);
}

// ---- Contrôle › Musique des moteurs ----

MI_MOTOR_MUSIC::MI_MOTOR_MUSIC()
    : IWindowMenuItem(_("Motor music"), &img::music_16x16, is_enabled_t::yes, is_hidden_t::no, expands_t::yes) {}

void MI_MOTOR_MUSIC::click(IWindowMenu &) {
    open_screen<ScreenMusic>();
}

} // namespace od3d
