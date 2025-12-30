#include "memory.hpp"

#include <cassert>
#include <fstream>

namespace
{
    constexpr int IRAM_SIZE = 0x8000;
    constexpr int MMIO_SIZE = 0x400;
    constexpr int PRAM_SIZE = 0x400;
    constexpr int VRAM_SIZE = 0x18000;
}

namespace backend
{
    Memory::Memory(std::string_view filepath) :
        m_iram(new std::uint8_t[IRAM_SIZE]{}),
        m_mmio(new uint8_t[MMIO_SIZE]{}),
        m_pram(new std::uint8_t[PRAM_SIZE]{}),
        m_vram(new std::uint8_t[VRAM_SIZE]{}),
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
        delete[] m_iram;
        delete[] m_mmio;
        delete[] m_pram;
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
        std::uint32_t alignedAddress = address & 0x00FFFFFF;
        if (sizeof(T) == 4)
        {
            alignedAddress &= ~3;
        }
        else if (sizeof(T) == 2)
        {
            alignedAddress &= ~1;
        }

        switch ((address >> 24) & 0xFF)
        {
            case 0x03:
            {
                if (alignedAddress < IRAM_SIZE)
                {
                    return *std::bit_cast<T*>(m_iram + alignedAddress);
                }
                assert(false && "INTERNAL RAM OUTSIDE BOUNDS!");
                break;
            }
            case 0x08 ... 0x0D:
            {
                if ((alignedAddress + sizeof(T) - 1) < m_rom.size()) [[likely]]
                {
                    return *std::bit_cast<T*>(m_rom.data() + alignedAddress);
                }
                assert(false && "critical error read outside of ROM!");
            }
            default: return getOpenBusValue(address);
        }

        return 0;
    }

    template <typename T>
    void Memory::write(std::uint32_t address, T value) noexcept
    {
        std::uint32_t alignedAddress = address & 0x00FFFFFF;
        if (sizeof(T) == 4)
        {
            alignedAddress &= ~3;
        }
        else if (sizeof(T) == 2)
        {
            alignedAddress &= ~1;
        }

        switch ((address >> 24) & 0xFF)
        {
            case 0x03:
            {
                if (alignedAddress < IRAM_SIZE)
                {
                    *std::bit_cast<T*>(m_iram + alignedAddress) = value;
                    break;
                }
                assert(false && "INTERNAL RAM OUTSIDE BOUNDS!");
                break;
            }
            case 0x04:
            {
                if (alignedAddress < MMIO_SIZE)
                {
                    *std::bit_cast<T*>(m_mmio + alignedAddress) = value;
                    break;
                }
                assert(false && "MMIO UNUSED REGION ACCESS!");
                break;
            }
            case 0x05:
            {
                if (alignedAddress < PRAM_SIZE)
                {
                    *std::bit_cast<T*>(m_pram + alignedAddress) = value;
                    break;
                }
                assert(false && "PRAM UNUSED REGION ACCESS!");
                break;
            }
            case 0x06:
            {
                if (alignedAddress < VRAM_SIZE)
                {
                    *std::bit_cast<T*>(m_vram + alignedAddress) = value;
                    break;
                }
                assert(false && "VRAM UNUSED REGION ACCCESS!");
                break;
            }
            case 0x08 ... 0x0D:
            {
                assert(false && "critical error write to ROM!");
            }
            default:
            {
                printf("write: %08X %08X\n", address, value);
                exit(0);
            }
        }
    };

    void Memory::tick(int cycles) noexcept
    {
        m_ppu.tick(cycles);
    }

    std::uint32_t Memory::getOpenBusValue(std::uint32_t address) const noexcept
    {
        // TODO

        printf("read: %08X\n", address);
        exit(0);

        return 0;
    }

    template std::int8_t Memory::read<std::int8_t>(std::uint32_t address) const noexcept;
    template std::uint16_t Memory::read<std::uint16_t>(std::uint32_t address) const noexcept;
    template std::uint32_t Memory::read<std::uint32_t>(std::uint32_t address) const noexcept;

    template void Memory::write<std::uint16_t>(std::uint32_t address, std::uint16_t value) noexcept;
    template void Memory::write<std::uint32_t>(std::uint32_t address, std::uint32_t value) noexcept;
}
