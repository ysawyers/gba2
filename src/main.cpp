#include <iostream>

#include "backend/cpu.hpp"


int main()
{
    try
    {
        // backend::CPU cpu;
        // cpu.loadROM("/Users/yondensawyers/Desktop/gameboyadvanced/roms/panda.gba");
        // cpu.run([x = 0](const std::uint16_t* framebuffer) mutable
        // {
        //     assert(framebuffer != nullptr);

        //     std::cout << "Frame " << x++ << " rendered!\n";
        //     printf("%08X\n", framebuffer[0]);
        // });
    }
    catch (const std::exception& e)
    {
        std::cout << e.what() << "\n";
    }
    return 0;
}
