#include "hardware/unitree/UnitreeController.h"

#include <exception>
#include <iostream>

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: Beyondmimic_Deploy <network-interface>\n";
        return 1;
    }

    try
    {
        deploy_real::UnitreeController controller(argv[1]);
        controller.Run();
    }
    catch (const std::exception &error)
    {
        std::cerr << "Beyondmimic_Deploy: " << error.what() << '\n';
        return 1;
    }
    return 0;
}
