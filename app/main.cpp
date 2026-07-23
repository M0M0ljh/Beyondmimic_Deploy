#include "hardware/unitree/UnitreeController.h"

#include <exception>
#include <iostream>

int main(int argc, char **argv)
{
    if (argc > 2)
    {
        std::cerr << "Usage: Beyondmimic_Deploy [network-interface]\n"
                  << "  no argument: Unitree MuJoCo simulation (domain 1, lo)\n"
                  << "  interface:   real robot (domain 0, interface)\n";
        return 1;
    }

    try
    {
        const bool simulation = argc == 1;
        deploy_real::UnitreeController controller(
            simulation ? 1 : 0, simulation ? "lo" : argv[1]);
        controller.Run();
    }
    catch (const std::exception &error)
    {
        std::cerr << "Beyondmimic_Deploy: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
