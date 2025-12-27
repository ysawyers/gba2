#include "memory.hpp"

#include <cassert>
#include <fstream>


namespace backend
{
    Memory::Memory(std::string_view filepath) :
        m_mmio(new uint8_t[0x3FF]{}),
        m_vram(new std::uint8_t[0x18000]{}),
        m_ppu(std::bit_cast<PPU::Registers*>(m_mmio), m_vram)
    {
        std::ifstream file(filepath, std::ios::binary);

        if (!file.is_open())
        {
            throw std::runtime_error("unable to open ROM file!");
        }

        // TODO improve in the future
        std::copy(
            std::istreambuf_iterator<char>(file),
            std::istreambuf_iterator<char>(),
            std::back_inserter(m_rom)
        );
    }

    Memory::~Memory()
    {
        delete[] m_mmio;
        delete[] m_vram;
    }

    std::string_view Memory::getGamePakTitle() const noexcept
    {
        return "GBA";
    }

    void Memory::registerHandlers(PPU::FramebufferHandler ppu) noexcept
    {
        m_ppu.registerHandler(ppu);
    }

    template <typename T>
    T Memory::read(std::uint32_t address) const noexcept
    {
        if (sizeof(T) == 4)
        {
            address &= ~3;
        }
        else if (sizeof(T) == 2)
        {
            address &= ~1;
        }

        std::uint8_t region = (address >> 24) & 0xFF;
        address &= 0x00FFFFFF;

        switch (region)
        {
            case 0x08:
            case 0x09:
            case 0xA0 ... 0xAF:
            case 0xB0 ... 0xBF:
            case 0xC0 ... 0xCF:
            case 0xD0 ... 0xDF:
            {
                if (address < m_rom.size()) [[likely]]
                {
                    return *std::bit_cast<T*>(m_rom.data() + address);
                }
                assert(false && "critical error read outside of ROM!");
            }
            default:
            {
                printf("read: %08X\n", address);
                exit(0);
            }
        }

        return 0;
    }

    template <typename T>
    void Memory::write(std::uint32_t address, T value) noexcept
    {
        if (sizeof(T) == 4)
        {
            address &= ~3;
        }
        else if (sizeof(T) == 2)
        {
            address &= ~1;
        }

        std::uint8_t region = (address >> 24) & 0xFF;
        address &= 0x00FFFFFF;

        switch (region)
        {
            case 0x04:
            {
                if (address < 0x3FF)
                {
                    *std::bit_cast<T*>(m_mmio + address) = value;
                    return;
                }
                assert(false && "MMIO UNUSED REGION ACCESS!\n");
                break;
            }
            case 0x06:
            {
                if (address < 0x18000)
                {
                    *std::bit_cast<T*>(m_vram + address) = value;
                    return;
                }
                assert(false && "VRAM UNUSED REGION ACCCESS!");
                break;
            }
            case 0x08:
            case 0x09:
            case 0xA0 ... 0xAF:
            case 0xB0 ... 0xBF:
            case 0xC0 ... 0xCF:
            case 0xD0 ... 0xDF:
            {
                assert(false && "critical error write to ROM!");
            }
            default:
            {
                printf("write: %08X %08X\n", (static_cast<std::uint32_t>(region) << 24) | address, value);
                exit(0);
            }
        }
    };

    void Memory::tick(int cycles) noexcept
    {
        m_ppu.tick(cycles);
    }

    template std::int8_t Memory::read<std::int8_t>(std::uint32_t address) const noexcept;
    template std::uint16_t Memory::read<std::uint16_t>(std::uint32_t address) const noexcept;
    template std::uint32_t Memory::read<std::uint32_t>(std::uint32_t address) const noexcept;

    template void Memory::write<std::uint16_t>(std::uint32_t address, std::uint16_t value) noexcept;
    template void Memory::write<std::uint32_t>(std::uint32_t address, std::uint32_t value) noexcept;
}
