#include <iostream>

#include "backend/cpu.hpp"

int main()
{
    try
    {
        backend::CPU cpu;
        cpu.loadROM("/Users/yondensawyers/Desktop/gameboyadvanced/roms/panda.gba");
        cpu.run();
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << "\n";
    }
    return 0;
}
