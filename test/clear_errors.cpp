// test/clear_errors.cpp
//
// Reads the Hardware_Error_Status register of every arm motor and reboots
// any that have a latched error (indicated by a blinking LED).
//
// Usage:  sudo ./clear_errors

#include "config.hpp"

#include <DynamixelWorkbench.h>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

using namespace std::chrono_literals;

static constexpr uint8_t ALL_IDS[] = {
    DXL_ID_J1, DXL_ID_J2, DXL_ID_J3, DXL_ID_J4, DXL_ID_GRIPPER
};

// Hardware_Error_Status bit meanings (XM430 e-Manual)
static void print_error_bits(uint8_t err)
{
    if (err & 0x01) std::printf("    bit0: Input voltage error\n");
    if (err & 0x04) std::printf("    bit2: Overheating\n");
    if (err & 0x08) std::printf("    bit3: Motor encoder error\n");
    if (err & 0x10) std::printf("    bit4: Electrical shock\n");
    if (err & 0x20) std::printf("    bit5: Overload\n");
}

int main()
{
    const char* log = nullptr;
    DynamixelWorkbench wb;

    if (!wb.init(DXL_PORT, DXL_BAUD, &log)) {
        std::fprintf(stderr, "init failed: %s\n", log ? log : "?");
        return 1;
    }

    uint16_t model = 0;
    for (uint8_t id : ALL_IDS) {
        bool ping_ok = wb.ping(id, &model, &log);
        if (!ping_ok) {
            // If the motor replied with a hardware error the ping still fails,
            // but the motor is alive — reboot it directly.
            if (log && strstr(log, "Hardware error")) {
                std::printf("ID %u: hardware error flagged in ping response – rebooting\n", id);
            } else {
                std::fprintf(stderr, "ID %u: no response – %s\n", id, log ? log : "?");
                continue;
            }
        }

        // If ping succeeded, check the error register; otherwise skip the read
        // (motor is in error state and may not respond to item reads).
        int32_t hw_err = 0;
        if (ping_ok) {
            wb.itemRead(id, "Hardware_Error_Status", &hw_err, &log);
        }

        if (ping_ok && hw_err == 0) {
            std::printf("ID %u: OK\n", id);
        } else {
            if (hw_err)
                std::printf("ID %u: error 0x%02X\n", id, (unsigned)hw_err);
            print_error_bits((uint8_t)hw_err);
            std::printf("  Rebooting...\n");
            wb.reboot(id, &log);
            std::this_thread::sleep_for(500ms);
            wb.ping(id, &model, &log);  // re-discover after reboot

            int32_t after = 0;
            wb.itemRead(id, "Hardware_Error_Status", &after, &log);
            std::printf("  Status after reboot: %s\n", after == 0 ? "CLEARED" : "STILL ERROR");
        }
    }

    return 0;
}
