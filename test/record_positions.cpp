// test/record_positions.cpp
//
// Interactive joint-position recorder.
//
// Torque is never enabled, so the arm can be moved freely by hand.
// Press Enter when the arm is in position, then type a name.
// All positions are saved to an XML file after each recording.
//
// Usage:  sudo ./record_positions [output.xml]
//         (default: positions.xml)

#include "config.hpp"

#include <DynamixelWorkbench.h>

#include <array>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <vector>

static constexpr uint8_t JOINT_IDS[4] = {DXL_ID_J1, DXL_ID_J2, DXL_ID_J3, DXL_ID_J4};

// ── XML I/O ───────────────────────────────────────────────────────────────────
// Ordered list of names (preserves insertion order) + fast lookup map.
using PositionList = std::vector<std::string>;
using PositionMap  = std::map<std::string, std::array<float, 4>>;

static void load_xml(const char* path, PositionList& order, PositionMap& map)
{
    std::ifstream f(path);
    if (!f) return;  // file doesn't exist yet — start fresh
    std::string line;
    while (std::getline(f, line)) {
        if (line.find("<position") == std::string::npos) continue;
        auto get = [&](const std::string& attr) -> std::string {
            auto p = line.find(attr + "=\"");
            if (p == std::string::npos) return {};
            p += attr.size() + 2;
            return line.substr(p, line.find('"', p) - p);
        };
        std::string name = get("name");
        if (name.empty()) continue;
        std::array<float, 4> j{};
        j[0] = std::stof(get("j1"));
        j[1] = std::stof(get("j2"));
        j[2] = std::stof(get("j3"));
        j[3] = std::stof(get("j4"));
        if (!map.count(name)) order.push_back(name);
        map[name] = j;
    }
}

static void write_xml(const PositionList& order, const PositionMap& map, const char* path)
{
    FILE* f = std::fopen(path, "w");
    if (!f) {
        std::fprintf(stderr, "ERROR: cannot write to %s\n", path);
        return;
    }
    std::fprintf(f, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    std::fprintf(f, "<positions>\n");
    for (const auto& name : order) {
        const auto& j = map.at(name);
        std::fprintf(f,
            "  <position name=\"%s\" j1=\"%.4f\" j2=\"%.4f\" j3=\"%.4f\" j4=\"%.4f\"/>\n",
            name.c_str(), j[0], j[1], j[2], j[3]);
    }
    std::fprintf(f, "</positions>\n");
    std::fclose(f);
}

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char* argv[])
{
    const char* out_path = (argc > 1) ? argv[1] : "positions.xml";

    std::printf("=== Position recorder ===\n");
    std::printf("Output: %s\n", out_path);
    std::printf("Port:   %s  Baud: %d\n\n", DXL_PORT, DXL_BAUD);

    // Initialise hardware — torque is never enabled so the arm stays compliant.
    DynamixelWorkbench dxl_wb;
    const char* log = nullptr;
    if (!dxl_wb.init(DXL_PORT, DXL_BAUD, &log)) {
        std::fprintf(stderr, "ERROR: init failed: %s\n", log ? log : "?");
        return 1;
    }
    uint16_t model = 0;
    for (uint8_t id : JOINT_IDS) {
        if (!dxl_wb.ping(id, &model, &log))
            std::fprintf(stderr, "WARN: ping failed for ID %u: %s\n", id, log ? log : "?");
    }
    // Set Extended Position Mode so readings are multi-turn, matching what
    // arm.cpp commands. Torque stays off — arm remains compliant.
    for (uint8_t id : JOINT_IDS)
        dxl_wb.setExtendedPositionControlMode(id, &log);

    // Load existing file so we don't lose previously recorded positions.
    PositionList order;
    PositionMap  map;
    load_xml(out_path, order, map);
    std::printf("Hardware ready. Move the arm freely.\n");
    if (!order.empty()) {
        std::printf("Loaded %zu existing position(s) from %s.\n", order.size(), out_path);
        for (const auto& n : order)
            std::printf("  %s\n", n.c_str());
    }
    std::printf("Press Enter to record. Empty name = discard. Ctrl+D = finish.\n\n");

    while (true) {
        std::printf("[ %zu total ] Press Enter to capture...\n", order.size());

        std::string line;
        if (!std::getline(std::cin, line)) break;   // Ctrl+D → done

        // Read joint positions
        std::array<float, 4> joints{};
        for (int i = 0; i < 4; ++i)
            dxl_wb.getRadian(JOINT_IDS[i], &joints[i]);

        std::printf("  captured: [%.4f, %.4f, %.4f, %.4f] rad\n",
                    joints[0], joints[1], joints[2], joints[3]);

        std::printf("  name (empty to discard): ");
        std::string name;
        if (!std::getline(std::cin, name) || name.empty()) {
            std::printf("  discarded.\n\n");
            continue;
        }

        bool is_update = map.count(name) > 0;
        if (!is_update) order.push_back(name);
        map[name] = joints;
        write_xml(order, map, out_path);
        std::printf("  %s '%s'.\n\n", is_update ? "updated" : "saved as", name.c_str());
    }

    write_xml(order, map, out_path);
    std::printf("\n%zu position(s) in %s\n", order.size(), out_path);
    return 0;
}
