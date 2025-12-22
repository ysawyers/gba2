#include <format>
#include <fstream>
#include <string_view>
#include <vector>

#include "common/concepts.hpp"


namespace backend
{
    //! https://problemkaputt.de/gbatek-gba-memory-map.htm
    class Memory
    {
    public:
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
        template <common::Word_c T>
        T read(std::uint32_t address) const noexcept
        {
            if (std::is_same_v<T, std::uint32_t>)
            {
                address &= ~0x3;
            }
            else if (std::is_same_v<T, std::uint16_t>)
            {
                address &= ~0x1;
            }

            if (address <= 0x00003FFF)
            {
                if (address < m_rom.size()) [[likely]]
                {
                    return *std::bit_cast<T*>(m_rom.data() + address);
                }
                // TODO: this is a critical error, propogate this to caller
            }

            return 0;
        }

        /*!
            \brief Writes a value to memory

            \tparam T Word/Halfword/Byte type

            \param[in] address address written to
            \param[in] value Word/Halfword/Byte value
        */
        template <common::Word_c T>
        void write(std::uint32_t address, T value) noexcept
        {
            if (std::is_same_v<T, std::uint32_t>)
            {
                address &= ~0x3;
            }
            else if (std::is_same_v<T, std::uint16_t>)
            {
                address &= ~0x1;
            }

        };

    private:
        std::vector<std::uint8_t> m_rom{};
    };
}
