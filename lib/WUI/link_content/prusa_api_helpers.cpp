#include "prusa_api_helpers.hpp"
#include "wui_api.h"

#include <transfers/changed_path.hpp>
#include <common/path_utils.h>
#include <print_utils.hpp>

#include <unistd.h>
#include <buddy/filename_defs.hpp>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <filename_type.hpp>
#include <lfn.h>
#include <marlin_client.hpp>
#include <state/printer_state.hpp>

namespace nhttp::link_content {

using handler::ConnectionState;
using handler::RequestParser;
using handler::StatusPage;
using http::Status;
using std::nullopt;
using std::optional;
using std::string_view;
using transfers::ChangedPath;

using Type = ChangedPath::Type;
using Incident = ChangedPath::Incident;

optional<string_view> remove_prefix(string_view input, string_view prefix) {
    if (input.size() < prefix.size() || input.substr(0, prefix.size()) != prefix) {
        return nullopt;
    }

    return input.substr(prefix.size());
}

bool parse_file_url(const RequestParser &parser, const size_t prefix_len, char *filename, const size_t filename_len, RemapPolicy remapPolicy, handler::Step &out) {
    /*
     * We do *not* use the uri with prefix removed. We need the safe
     * transformation into a file name (removal of query params,
     * forbidding of '..', etc).
     */
    /*
     * TODO:
     * We currently don't read the URI parameters. That is, we don't
     * understand "?recursive=true" and therefore don't do recursive.
     *
     * The FileInfo handler doesn't know recursive yet either.
     */
    const char *fname_real = filename + prefix_len;
    if (parser.uri_filename(filename, filename_len)) {
        size_t len = strlen(filename);
        if (prefix_len > len) {
            out.next = StatusPage(Status::NotFound, parser);
            return false;
        }

        if (remapPolicy == RemapPolicy::Octoprint) {
            /*
             * The octoprint API gives special meaning to /local and
             * /sdcard. For us, everything lives in the USB (/usb). We
             * remap these. Nevertheless, we never _generate_ these /local
             * or such URIs, so we don't remap anything else but the "root".
             */
            static const char *const roots[] = {
                "",
                "/",
                "/local",
                "/local/",
                "/sdcard",
                "/sdcard/",
            };

            for (size_t i = 0; i < sizeof roots / sizeof roots[0]; i++) {
                if (strcmp(fname_real, roots[i]) == 0) {
                    fname_real = "/usb/";
                    break;
                }
            }
        }

        /*
         * Now, we make 100% sure the user won't get a file that's not
         * on the USB drive (eg. our xflash).
         */
        if (strncmp(fname_real, "/usb/", 5) != 0 && strcmp(fname_real, "/usb") != 0) {
            out.next = StatusPage(Status::Forbidden, parser);
            return false;
        }

        // Consumers strip a fixed "/usb/" prefix; normalise the accepted bare "/usb".
        if (strcmp(fname_real, "/usb") == 0) {
            fname_real = "/usb/";
        }

        // We need to use memmove, because fname_real points into filename
        // and memmove explicitly allowes for overlapping buffer to be used
        size_t fname_real_len = strlen(fname_real);
        memmove(filename, fname_real, fname_real_len);
        filename[fname_real_len] = '\0';
        // Slicer sometimes produces duplicate slashes in the URL and
        // this may confuse eg. marlin.
        dedup_slashes(filename);
        return true;
    } else {
        out.next = StatusPage(Status::NotFound, parser);
        return false;
    }
}

StatusPage delete_file(const char *filename, const RequestParser &parser) {
    auto result = remove_file(filename);
    if (result == DeleteResult::Busy) {
        return StatusPage(Status::Conflict, parser, "File is busy");
    } else if (result == DeleteResult::ActiveTransfer) {
        return StatusPage(Status::Conflict, parser, "File is being transferred");
    } else if (result == DeleteResult::GeneralError) {
        return StatusPage(Status::NotFound, parser);
    } else {
        ChangedPath::instance.changed_path(filename, Type::File, Incident::Deleted);
        return StatusPage(Status::NoContent, parser);
    }
}

StatusPage create_folder(const char *filename, const RequestParser &parser) {
    if (mkdir(filename, 0777) != 0) {
        if (errno == EEXIST) {
            return StatusPage(Status::Conflict, parser, "Already exists");
        } else {
            return StatusPage(Status::InternalServerError, parser, "Error creating directory");
        }
    }

    ChangedPath::instance.changed_path(filename, Type::Folder, Incident::Created);
    return StatusPage(Status::Created, parser);
}

StatusPage print_file(std::span<char> filename, const RequestParser &parser) {
    // if not exists
    if (access(filename.data(), R_OK) != 0) {
        return StatusPage(Status::NotFound, StatusPage::CloseHandling::ErrorClose, parser.accepts_json);
    }

    // OneDrop3D : un POST sur un .bbf l'installe (PrintForge). M997 redémarre aussitôt : seulement au repos.
    if (filename_is_firmware(filename.data())) {
        get_SFN_path(filename);
        char gcode[filename_defs::path_buffer_size + 8];
        snprintf(gcode, sizeof(gcode), "M997 %s", filename.data());
        // Un « S » suivi d'un chiffre dans le nom serait lu comme le numéro de module de M997, qui ne flasherait rien.
        bool s_digit = false;
        for (const char *c = filename.data(); *c; ++c) {
            s_digit |= (c[0] == 'S' && isdigit(static_cast<unsigned char>(c[1])));
        }
        const bool in_root = strncmp(filename.data(), "/usb/", 5) == 0 && !strchr(filename.data() + 5, '/'); // M997 ne garde que 12 caractères
        if (s_digit || !in_root || !printer_state::remote_print_ready(false) || marlin_client::gcode_try(gcode) != marlin_client::GcodeTryResult::Submitted) {
            return StatusPage(Status::Conflict, StatusPage::CloseHandling::ErrorClose, parser.accepts_json);
        }
        return StatusPage(Status::NoContent, StatusPage::CloseHandling::ErrorClose, parser.accepts_json);
    }

    if (wui_start_print(filename, true) == StartPrintResult::PrintStarted) {
        // We ErrorClose the connection here, because theoretically
        // according to the API there could be a body, that we should
        // ignore. To ignore it explicitly, but with empty POST still working
        // ,would requier a lot more work. Closing it is not a big deal, because
        // starting a print does not happen very often.
        return StatusPage(Status::NoContent, StatusPage::CloseHandling::ErrorClose, parser.accepts_json);
    } else {
        return StatusPage(Status::Conflict, StatusPage::CloseHandling::ErrorClose, parser.accepts_json);
    }
}

void get_only(handler::ConnectionState state, const handler::RequestParser &parser, handler::Step &out) {
    if (parser.method == http::Method::Get) {
        out.next = move(state);
    } else {
        // Drop the connection in fear there might be a body we don't know about.
        out.next = StatusPage(Status::MethodNotAllowed, StatusPage::CloseHandling::ErrorClose, parser.accepts_json);
    }
}
} // namespace nhttp::link_content
