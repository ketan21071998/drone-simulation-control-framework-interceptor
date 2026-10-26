// =========================================================
// Drone Interceptor Simulator - C++ / OpenGL / SFML
// Mirrors the Python Panda3D reference exactly.
//
// Coordinate system: EUS  (X=East/Right, Y=Up, Z=South/Back)
// Motor order: [FR, FL, BR, BL]
// Front (red) motors point toward -Z
//
// Build: see CMakeLists.txt
// Run:   drone_sim [port]        (default 8080)
// =========================================================

#include "physics.h"
#include "renderer.h"
#include "tcp_server.h"

#include <iostream>
#include <cstdlib>

int main(int argc, char* argv[])
{
    int port = 8080;
    if (argc >= 2) port = std::atoi(argv[1]);

    std::cout << "[SIM] Starting C++ Interceptor Simulator\n";
    std::cout << "[SIM] Physics rate: 120 Hz\n";
    std::cout << "[SIM] Socket rate:   50 Hz\n";
    std::cout << "[SIM] Coordinates: EUS  X=right  Y=up  Z=south/back\n";
    std::cout << "[SIM] Motor order: [FR, FL, BR, BL]\n";
    std::cout << "[SIM] Front (red) motors point toward -Z\n";
    std::cout << "[SIM] Press R to reset\n";

    // Shared simulation state
    SimState state;
    state.reset();

    // Background TCP server (50 Hz telemetry, accepts PWM)
    TcpServer server(port, state);
    server.start();

    // Blocking render + physics loop (main thread)
    Renderer renderer(state);
    renderer.run();   // returns when window is closed

    server.stop();
    return 0;
}
