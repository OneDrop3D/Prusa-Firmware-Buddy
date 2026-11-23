/// @file
#include "mmu_flash_od3d.hpp"
#include "avr109_od3d.hpp"
#include "mmu_usb_od3d.hpp"

#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <shared_mutex>

#include <common/async_job/async_job.hpp>
#include <common/freertos_shared_mutex.hpp>
#include <logging/log.hpp>
#include <raii/scope_guard.hpp>
#include <timing.h>
#include "usbh_core.h"

LOG_COMPONENT_REF(MMU2);

extern USBH_HandleTypeDef hUsbHostHS;

namespace od3d::mmu_flash {

namespace {

    constexpr const char *firmware_path = "/system/res/mmu/fw.bin";
    constexpr uint32_t transfer_timeout_ms = 1'000;
    constexpr uint32_t bootloader_wait_ms = 30'000; ///< le temps d'appuyer sur reset, comme PrusaSlicer (avr109_wait_for_bootloader)

    // Écrits par la tâche de l'hôte USB (init, deinit), lus sous le verrou partagé class_mutex comme la clé
    // (usbh_diskio.cpp) : l'hôte prend le verrou exclusif pour défaire la classe quand l'appareil part
    struct Pipes {
        uint8_t in = 0; ///< 0 : pas de tuyau (le 0 est celui des commandes)
        uint8_t out = 0;
        uint16_t in_size = 0;
        uint16_t out_size = 0;
        uint8_t comm_interface = 0;
        Device device = Device::none;
    };
    Pipes pipes;
    std::atomic<Device> connected = Device::none;

    // Tampons du DMA de l'hôte USB : ni sur la pile (CCMRAM, hors de portée du DMA), ni à une adresse non alignée
    alignas(4) uint8_t tx[64];
    alignas(4) uint8_t rx[160];
    alignas(4) uint8_t line_coding[7];
    size_t rx_len = 0;
    size_t rx_pos = 0;

    std::atomic<Phase> phase = Phase::idle;
    std::atomic<uint8_t> percent = 0;
    std::atomic<Error> error = Error::none;
    std::atomic<char> failed_command = 0;
    std::atomic<uint16_t> failed_address = 0;
    AsyncJob job;

    USBH_StatusTypeDef class_init(USBH_HandleTypeDef *phost) {
        const auto &dev = phost->device.DevDesc;
        Pipes p;
        p.device = usb_device(dev.idVendor, dev.idProduct);
        if (p.device == Device::none) {
            return USBH_FAIL; // autre appareil CDC : ignoré, comme avant
        }
        uint8_t in_ep = 0;
        uint8_t out_ep = 0;
        const auto &cfg = phost->device.CfgDesc;
        for (uint8_t i = 0; i < std::min<uint8_t>(cfg.bNumInterfaces, USBH_MAX_NUM_INTERFACES); ++i) {
            const auto &itf = cfg.Itf_Desc[i];
            if (itf.bInterfaceClass == 0x02) {
                p.comm_interface = itf.bInterfaceNumber;
            }
            if (itf.bInterfaceClass != 0x0A) { // CDC Data : les deux points de terminaison en bloc
                continue;
            }
            for (uint8_t e = 0; e < std::min<uint8_t>(itf.bNumEndpoints, USBH_MAX_NUM_ENDPOINTS); ++e) {
                const auto &ep = itf.Ep_Desc[e];
                if ((ep.bmAttributes & 0x03) != USB_EP_TYPE_BULK) {
                    continue;
                }
                if (ep.bEndpointAddress & 0x80) {
                    in_ep = ep.bEndpointAddress;
                    p.in_size = ep.wMaxPacketSize;
                } else {
                    out_ep = ep.bEndpointAddress;
                    p.out_size = ep.wMaxPacketSize;
                }
            }
        }
        if (!in_ep || !out_ep || !p.in_size || p.in_size > sizeof(rx) / 2 || !p.out_size || p.out_size > sizeof(tx)) {
            return USBH_FAIL;
        }
        p.out = USBH_AllocPipe(phost, out_ep);
        p.in = USBH_AllocPipe(phost, in_ep);
        if (p.out == 0xFF || p.in == 0xFF) {
            USBH_FreePipe(phost, p.out);
            USBH_FreePipe(phost, p.in);
            return USBH_FAIL;
        }
        USBH_OpenPipe(phost, p.out, out_ep, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, p.out_size);
        USBH_OpenPipe(phost, p.in, in_ep, phost->device.address, phost->device.speed, USB_EP_TYPE_BULK, p.in_size);
#if defined(USBH_USE_IN_CHANNEL_BULK_NACK_INTERRUPTS) && (USBH_USE_IN_CHANNEL_BULK_NACK_INTERRUPTS == 0U)
        // Comme la clé (usbh_msc.c) : avec le DMA, le canal relance seul après un NAK, l'interruption ne ferait qu'occuper le CPU
        uint32_t USBx_BASE = (uint32_t)((HCD_HandleTypeDef *)phost->pData)->Instance;
        USBx_HC((uint32_t)p.in)->HCINTMSK &= ~USB_OTG_HCINTMSK_NAKM;
#endif
        USBH_LL_SetToggle(phost, p.in, 0);
        USBH_LL_SetToggle(phost, p.out, 0);
        pipes = p;
        rx_len = rx_pos = 0;
        return USBH_OK;
    }

    USBH_StatusTypeDef class_deinit(USBH_HandleTypeDef *phost) {
        connected = Device::none;
        for (const uint8_t pipe : { pipes.in, pipes.out }) {
            if (pipe) {
                USBH_ClosePipe(phost, pipe);
                USBH_FreePipe(phost, pipe);
            }
        }
        pipes = {};
        return USBH_OK;
    }

    USBH_StatusTypeDef class_requests(USBH_HandleTypeDef *) {
        connected = pipes.device;
        return USBH_OK;
    }

    USBH_StatusTypeDef class_idle(USBH_HandleTypeDef *) {
        return USBH_OK;
    }

    USBH_ClassTypeDef usbh_class = { "MMU", 0x02, class_init, class_deinit, class_requests, class_idle, class_idle, nullptr };

    bool gone() {
        return connected == Device::none || hUsbHostHS.device.is_disconnected;
    }

    /// Fin du transfert en cours sur `pipe` : faux si le MMU part ou ne répond pas avant `timeout_ms`
    USBH_URBStateTypeDef wait(uint8_t pipe, uint32_t timeout_ms) {
        for (const uint32_t start = ticks_ms();;) {
            const auto state = USBH_LL_GetURBState(&hUsbHostHS, pipe);
            if (state != USBH_URB_IDLE) {
                return state;
            }
            if (gone() || ticks_diff(ticks_ms(), start) > int32_t(timeout_ms)) {
                USBH_ClosePipe(&hUsbHostHS, pipe); // arrête le canal : il ne doit plus écrire dans le tampon
                return USBH_URB_ERROR;
            }
            osDelay(1);
        }
    }

    /// Des octets du chargeur dans `rx` (0 : le paquet vide qui suit un paquet plein)
    bool fill(uint32_t timeout_ms) {
        rx_len = rx_pos = 0;
        USBH_BulkReceiveData(&hUsbHostHS, rx, sizeof(rx) / pipes.in_size * pipes.in_size, pipes.in);
        if (wait(pipes.in, timeout_ms) != USBH_URB_DONE) {
            return false;
        }
        rx_len = std::min<size_t>(USBH_LL_GetLastXferSize(&hUsbHostHS, pipes.in), sizeof(rx));
        return true;
    }

    /// Ligne à 1200 bauds (SET_LINE_CODING) : le firmware du MMU redémarre dans son chargeur (usb_cdc.cpp du MMU)
    bool touch() {
        freertos::SharedMutexProxy mutex { &hUsbHostHS.class_mutex };
        std::shared_lock lock { mutex };
        if (connected != Device::firmware) {
            return false;
        }
        static constexpr uint8_t coding[] = { 0xB0, 0x04, 0, 0, 0, 0, 8 }; // 1200 bauds, 1 bit d'arrêt, sans parité, 8 bits
        memcpy(line_coding, coding, sizeof(coding));
        for (const uint32_t start = ticks_ms();;) {
            auto &setup = hUsbHostHS.Control.setup.b;
            setup.bmRequestType = USB_H2D | USB_REQ_TYPE_CLASS | USB_REQ_RECIPIENT_INTERFACE;
            setup.bRequest = 0x20; // SET_LINE_CODING
            setup.wValue.w = 0;
            setup.wIndex.w = pipes.comm_interface;
            setup.wLength.w = sizeof(line_coding);
            const auto status = USBH_CtlReq(&hUsbHostHS, line_coding, sizeof(line_coding));
            if (status != USBH_BUSY) {
                return status == USBH_OK;
            }
            if (gone() || ticks_diff(ticks_ms(), start) > int32_t(transfer_timeout_ms)) {
                return false;
            }
            osDelay(1);
        }
    }

    class Flasher final : public avr109::Port {
    public:
        Flasher(FILE *file, size_t size)
            : file(file)
            , size(size) {}

        bool send(std::span<const uint8_t> bytes) override {
            freertos::SharedMutexProxy mutex { &hUsbHostHS.class_mutex };
            std::shared_lock lock { mutex };
            while (!bytes.empty()) {
                if (connected != Device::bootloader) {
                    return false;
                }
                // Un paquet par transfert : refusé (NAK, le chargeur écrit une page), il repart tel quel sans doublon
                const size_t n = std::min<size_t>(bytes.size(), pipes.out_size);
                memcpy(tx, bytes.data(), n);
                for (const uint32_t start = ticks_ms();;) {
                    USBH_BulkSendData(&hUsbHostHS, tx, n, pipes.out, 0);
                    const auto state = wait(pipes.out, transfer_timeout_ms);
                    if (state == USBH_URB_DONE) {
                        break;
                    }
                    if (state != USBH_URB_NOTREADY || ticks_diff(ticks_ms(), start) > int32_t(transfer_timeout_ms)) {
                        return false;
                    }
                }
                bytes = bytes.subspan(n);
            }
            return true;
        }

        bool receive(std::span<uint8_t> bytes, uint32_t timeout_ms) override {
            freertos::SharedMutexProxy mutex { &hUsbHostHS.class_mutex };
            std::shared_lock lock { mutex };
            const uint32_t start = ticks_ms();
            for (auto &byte : bytes) {
                while (rx_pos == rx_len) {
                    const int32_t left = int32_t(timeout_ms) - ticks_diff(ticks_ms(), start);
                    if (connected != Device::bootloader || left <= 0 || !fill(left)) {
                        return false;
                    }
                }
                byte = rx[rx_pos++];
            }
            return true;
        }

        void drain() override {
            freertos::SharedMutexProxy mutex { &hUsbHostHS.class_mutex };
            std::shared_lock lock { mutex };
            if (connected == Device::bootloader) {
                fill(20); // une réponse tardive à un essai précédent
            }
            rx_len = rx_pos = 0;
        }

        bool read_page(size_t index, std::span<uint8_t, avr109::page_size> page) override {
            std::fill(page.begin(), page.end(), 0xFF);
            const size_t offset = index * avr109::page_size;
            if (offset >= size) {
                return true;
            }
            const size_t n = std::min(size - offset, avr109::page_size);
            return fseek(file, offset, SEEK_SET) == 0 && fread(page.data(), 1, n, file) == n;
        }

        void progress(size_t done, size_t total) override {
            percent = done * 100 / total;
            phase = Phase::writing;
        }

    private:
        FILE *file;
        size_t size;
    };

    void fail(Error e, char command = 0, uint16_t address = 0) {
        failed_command = command;
        failed_address = address;
        error = e;
        phase = Phase::failed;
        log_error(MMU2, "MMU flash failed: %d '%c' 0x%04X", int(e), command ? command : ' ', address);
    }

    void run() {
        FILE *file = fopen(firmware_path, "rb");
        if (!file) {
            return fail(Error::file);
        }
        ScopeGuard close_file([&] { fclose(file); });
        // Vérifié avant d'écrire : taille d'un firmware de MMU et saut vers le programme (jmp) en tête
        uint8_t head[2] = {};
        const bool sized = fseek(file, 0, SEEK_END) == 0;
        const long size = sized ? ftell(file) : -1;
        if (size < 0x100 || size > long(avr109::app_size) || fseek(file, 0, SEEK_SET) != 0 || fread(head, 1, 2, file) != 2 || head[0] != 0x0C || head[1] != 0x94) {
            return fail(Error::file);
        }

        // Un firmware vu est renvoyé dans son chargeur (1200 bauds), une fois ; sans MMU vu, l'écran demande son bouton reset
        bool touched = false;
        for (const uint32_t start = ticks_ms(); device() != Device::bootloader; osDelay(100)) {
            if (device() == Device::firmware && !touched) {
                touched = true;
                if (!touch()) {
                    return fail(Error::no_mmu);
                }
            }
            if (ticks_diff(ticks_ms(), start) > int32_t(bootloader_wait_ms)) {
                return fail(touched ? Error::no_bootloader : Error::not_seen);
            }
        }

        Flasher flasher(file, size);
        const auto report = avr109::flash(flasher, (size + avr109::page_size - 1) / avr109::page_size);
        switch (report.result) {
        case avr109::Result::ok:
            percent = 100;
            phase = Phase::done;
            log_info(MMU2, "MMU flashed and verified (%ld B)", size);
            return;
        case avr109::Result::no_answer:
            return fail(gone() ? Error::no_mmu : Error::no_answer, report.command);
        case avr109::Result::refused:
            return fail(Error::refused, report.command);
        case avr109::Result::not_paged:
        case avr109::Result::wrong_chip:
            return fail(Error::wrong_chip, report.command);
        case avr109::Result::file_error:
            return fail(Error::file, 0, report.address);
        case avr109::Result::verify_failed:
            return fail(Error::verify, report.command, report.address);
        }
    }

} // namespace

Device device() {
    return connected;
}

bool bundled(Version &version) {
    FILE *file = fopen(firmware_path, "rb");
    if (!file) {
        return false;
    }
    uint8_t v[6];
    const bool ok = fseek(file, 0xAC, SEEK_SET) == 0 && fread(v, 1, sizeof(v), file) == sizeof(v);
    fclose(file);
    if (ok) {
        version = { v[0], v[1], uint8_t(v[2] | v[3] << 8), uint16_t(v[4] | v[5] << 8) };
    }
    return ok;
}

Status status() {
    return { phase, percent, error, failed_command, failed_address };
}

bool start() {
    const Phase now = phase;
    if (!mmu_usb::active() || job.is_active() || now == Phase::connecting || now == Phase::writing) {
        return false;
    }
    error = Error::none;
    percent = 0;
    phase = Phase::connecting;
    job.issue([](AsyncJobExecutionControl &) { run(); });
    return true;
}

bool busy() {
    return mmu_usb::active() || job.is_active();
}

void forget_result() {
    Phase p = Phase::done;
    if (!phase.compare_exchange_strong(p, Phase::idle)) {
        p = Phase::failed;
        phase.compare_exchange_strong(p, Phase::idle);
    }
}

void register_usb_class() {
    USBH_RegisterClass(&hUsbHostHS, &usbh_class);
}

} // namespace od3d::mmu_flash
