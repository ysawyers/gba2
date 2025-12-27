#pragma once

#include <string_view>
#include <vector>

#include "ppu.hpp"


namespace backend
{
    /*!
        - https://gbadev.net/gbadoc/memory.html
        - https://problemkaputt.de/gbatek.htm#gbamemorymap
    */
    class Memory
    {
    public:
        Memory(std::string_view filepath);
        Memory(Memory&) = delete;
        Memory& operator=(Memory&) = delete;
        Memory(Memory&&) = delete;
        Memory& operator=(Memory&&) = delete;
        ~Memory();

        /*!
            \return GamePak title from ROM
        */
        std::string_view getGamePakTitle() const noexcept;

        /*!
            \brief Registers handlers for emulator components

            \param[in] ppu PPU handler
        */
        void registerHandlers(PPU::FramebufferHandler ppu) noexcept;

        /*!
            \brief Reads a value from memory

            \tparam T Word/Halfword/Byte type

            \param[in] address address read from

            \return Word/Halfword/Byte value
        */
        template <typename T>
        T read(std::uint32_t address) const noexcept;

        /*!
            \brief Writes a value to memory

            \tparam T Word/Halfword/Byte type

            \param[in] address address written to
            \param[in] value Word/Halfword/Byte value
        */
        template <typename T>
        void write(std::uint32_t address, T value) noexcept;

        /*!
            \brief Ticks emulator components

            \param[in] cycles number of cycles each component is ticked
        */
        void tick(int cycles) noexcept;

    private:
        //! ...
        std::vector<std::uint8_t> m_rom{};

        //! ...
        std::uint8_t* m_mmio{nullptr};

        //! ...
        std::uint8_t* m_vram{nullptr};

        //! ...
        PPU m_ppu;
    };
}
