#include <cstdlib>
#include <cstdint>
#include <iostream>
#include <string>

#include "stablecops/app/CanopenApplication.hpp"
#include "stablecops/app/MotorConfig.hpp"

int main(int argc, char** argv) {
    stablecops::app::MotorConfig config;

    bool run = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--run") {
            run = true;
        } else if (arg == "--inspect") {
            config.inspect_on_boot = true;
        } else if (arg == "--can" && i + 1 < argc) {
            config.can_interface = argv[++i];
        } else if (arg == "--dcf" && i + 1 < argc) {
            config.master_dcf_path = argv[++i];
        } else if (arg == "--master-node" && i + 1 < argc) {
            config.master_node_id = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else if (arg == "--node" && i + 1 < argc) {
            config.node_id = static_cast<uint8_t>(std::stoi(argv[++i]));
        } else {
            std::cerr << "usage: stablecops_master [--can can0] [--dcf dcf/master.dcf] "
                         "[--master-node 127] [--node 1] [--inspect] [--run]\n";
            return EXIT_FAILURE;
        }
    }

    std::cout
        << "stableCOPS CANopen master\n"
        << "CAN interface: " << config.can_interface << '\n'
        << "Master Node ID: " << static_cast<int>(config.master_node_id) << '\n'
        << "Node ID: " << static_cast<int>(config.node_id) << '\n'
        << "Inspect on boot: " << (config.inspect_on_boot ? "yes" : "no") << '\n'
        << "Master DCF: " << config.master_dcf_path << '\n';

    if (!run) {
        std::cout << "\nPass --run to open SocketCAN and start the Lely master.\n";
        return EXIT_SUCCESS;
    }

    stablecops::app::CanopenApplication app(config);
    app.resetMaster();
    app.run();

    return EXIT_SUCCESS;
}
