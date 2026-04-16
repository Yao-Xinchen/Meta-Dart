// test/scan_motors.cpp
//
// Scans for Dynamixel motors across multiple baud rates and IDs.
// Uses the low-level SDK directly (no DynamixelWorkbench) for maximum visibility.
//
// Usage:
//   sudo ./scan_motors [port]          # default port: /dev/ttyUSB0
//
// Tries Protocol 1.0 and 2.0, baud rates: 9600, 57600, 115200, 1000000, 2000000
// Scans IDs 1–253 at each combination.

#include <dynamixel_sdk.h>

#include <cstdio>

static constexpr int    ID_MIN  = 1;
static constexpr int    ID_MAX  = 30;   // raise to 253 for a full sweep (slower)

static constexpr int BAUDS[] = { 9600, 57600, 115200, 1000000, 2000000 };

static void scan(const char* port, float protocol, int baud)
{
    auto* ph  = dynamixel::PortHandler::getPortHandler(port);
    auto* pkt = dynamixel::PacketHandler::getPacketHandler(protocol);

    if (!ph->openPort()) {
        std::fprintf(stderr, "  [!] Cannot open %s\n", port);
        return;
    }
    if (!ph->setBaudRate(baud)) {
        std::fprintf(stderr, "  [!] Cannot set baud %d\n", baud);
        ph->closePort();
        return;
    }

    int found = 0;
    for (int id = ID_MIN; id <= ID_MAX; ++id) {
        uint16_t model = 0;
        uint8_t  err   = 0;
        int res = pkt->ping(ph, (uint8_t)id, &model, &err);
        if (res == COMM_SUCCESS) {
            std::printf("  *** FOUND  ID=%-3d  model=%5d  protocol=%.1f  baud=%d\n",
                        id, model, protocol, baud);
            ++found;
        }
    }
    if (found == 0)
        std::printf("  (no motors)\n");

    ph->closePort();
    // PortHandler leaks by design in the SDK; acceptable for a scan tool.
}

int main(int argc, char* argv[])
{
    const char* port = (argc > 1) ? argv[1] : "/dev/ttyUSB0";

    std::printf("=== Dynamixel motor scan ===\n");
    std::printf("Port: %s  |  ID range: %d–%d\n\n", port, ID_MIN, ID_MAX);

    for (int baud : BAUDS) {
        for (float proto : {2.0f, 1.0f}) {
            std::printf("Protocol %.1f  baud %-8d  ...\n", proto, baud);
            std::fflush(stdout);
            scan(port, proto, baud);
        }
    }

    std::printf("\nDone.\n");
    return 0;
}
