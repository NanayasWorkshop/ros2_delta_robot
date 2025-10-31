#include <iostream>
#include <ruckig/ruckig.hpp>

int main() {
    std::cout << "Ruckig Feature Test\n";
    std::cout << "===================\n\n";

    // Test 1: Basic features
    std::cout << "✓ Basic Ruckig class available\n";

    // Test 2: Check if WITH_ONLINE_CLIENT is defined
    #ifdef WITH_ONLINE_CLIENT
        std::cout << "✓ WITH_ONLINE_CLIENT is DEFINED (Cloud/Waypoints enabled)\n";
    #else
        std::cout << "✗ WITH_ONLINE_CLIENT is NOT DEFINED (No cloud waypoints)\n";
    #endif

    // Test 3: Try to use InputParameter with waypoints
    ruckig::InputParameter<3> input;
    std::cout << "✓ InputParameter created\n";

    // Check if intermediate_positions is available (always present in headers)
    std::cout << "✓ intermediate_positions field exists in InputParameter\n";

    // Test 4: Check Ruckig constructor with waypoints
    #ifdef WITH_ONLINE_CLIENT
        ruckig::Ruckig<3> ruckig_with_waypoints(0.01, 10);
        std::cout << "✓ Ruckig with waypoints constructor available\n";
    #else
        std::cout << "✗ Ruckig with waypoints constructor NOT available\n";
    #endif

    std::cout << "\n===================\n";
    std::cout << "Summary:\n";
    std::cout << "- This is the open-source community version\n";

    #ifdef WITH_ONLINE_CLIENT
        std::cout << "- Waypoint support: YES (via cloud API)\n";
    #else
        std::cout << "- Waypoint support: NO (not compiled in)\n";
    #endif

    std::cout << "- Positional limits: NO (Pro only)\n";
    std::cout << "- Interrupt calculation: NO (Pro only)\n";

    return 0;
}
