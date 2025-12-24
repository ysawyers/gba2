#pragma once

#include <cassert>
#include <format>
#include <fstream>
#include <string_view>
#include <vector>


namespace backend
{
    //! https://gbadev.net/gbadoc/memory.html
    class Memory
    {
    public:
        Memory()
        {
            m_mmio.resize(0x3FF);
            m_vram.resize(0x18000);
        }

        /*!
            \brief Loads ROM from a file

            /param[in] filepath Path to a ROM file
        */
        void loadROM(std::string_view filepath)
        {
            std::ifstream file(filepath, std::ios::binary);

            if (!file.is_open())
            {
                throw std::runtime_error(std::format("unable to open ROM: {}", filepath));
            }

            // TODO improve in the future
            std::copy(
                std::istreambuf_iterator<char>(file),
                std::istreambuf_iterator<char>(),
                std::back_inserter(m_rom)
            );
        }

        /*!
            \brief Checks if a ROM is loaded into memory

            \return true if loaded otherwise false
        */
        bool loadedROM() const noexcept
        {
            return !m_rom.empty();
        }

        /*!
            \brief Reads a value from memory

            \tparam T Word/Halfword/Byte type

            \param[in] address address read from

            \return Word/Halfword/Byte value
        */
        template <std::integral T>
        T read(std::uint32_t address) const noexcept
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

        /*!
            \brief Writes a value to memory

            \tparam T Word/Halfword/Byte type

            \param[in] address address written to
            \param[in] value Word/Halfword/Byte value
        */
        template <std::integral T>
        void write(std::uint32_t address, T value) noexcept
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
                    if (address < m_mmio.size())
                    {
                        *std::bit_cast<T*>(m_mmio.data() + address) = value;
                        return;
                    }
                    assert(false && "unused!\n");
                    break;
                }
                case 0x06:
                {
                    if (address < m_vram.size())
                    {
                        *std::bit_cast<T*>(m_vram.data() + address) = value;
                        return;
                    }
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

    private:
        //! ...
        std::vector<std::uint8_t> m_mmio{};

        //! ...
        std::vector <std::uint8_t> m_vram{};

        //! ...
        std::vector<std::uint8_t> m_rom{};
    };
}
