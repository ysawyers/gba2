#include <iostream>

#include "local/window.hpp"


int main()
{
    try
    {
        Window window;
        window.run("/Users/yondensawyers/Desktop/gameboyadvanced/roms/panda.gba");
    }
    catch (const std::exception& e)
    {
        std::cout << "[main] " << e.what() << "\n";
    }
    return 0;
}
