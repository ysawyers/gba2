#pragma once

#include <functional>


namespace backend
{
    /*!
        ...
    */
    class PPU
    {
    public:
        static constexpr int FRAME_WIDTH = 240;
        static constexpr int FRAME_HEIGHT = 160;

        using FramebufferHandler = std::function<void(const std::uint16_t*)>;

        /*!
            MMIO registers
        */
        struct [[gnu::packed]] Registers
        {
            std::uint32_t m_dispcnt;
            std::uint32_t m_dispstat;
            std::uint16_t m_vcount;
            std::uint32_t m_bgcnt[4];
            std::uint16_t m_bgofs[4][2];
        };

        PPU(Registers* mmio, std::uint8_t* vram);
        PPU(PPU&) = delete;
        PPU& operator=(PPU&) = delete;
        PPU(PPU&&) = delete;
        PPU& operator=(PPU&&) = delete;
        ~PPU();

        /*!
            \brief registers framebuffer handler

            \param[in] handler handler function that processes the framebuffer
        */
        void registerHandler(FramebufferHandler handler) noexcept;

        /*!
            ...
        */
        void renderTextBackground();

        /*!
            ...
        */
        void renderScaleRotateBackground();

        /*!
            \brief https://gbadev.net/gbadoc/graphics.html#mode-3
        */
        void renderBitmapBackground3();

        /*!
            ...
        */
        void renderScanline();

        /*!
            \brief ticks the PPU

            \param[in] cycles number of cycles ticked
        */
        void tick(int cycles) noexcept;

    private:
        /*!
            PPU periods
        */
        enum class Period
        {
            VBLANK,
            HDRAW,
            HBLANK
        };

        /*!
            PPU state machine
        */
        struct State
        {
            //! Current period the PPU is in
            Period m_period{Period::HDRAW};

            //! Number of cycles spent in the current period
            int m_cycles{};
        };

        //! ...
        FramebufferHandler m_handler{};

        //! ...
        State m_state{};

        //! ...
        std::uint16_t* m_framebuffer{nullptr};

        //! ...
        Registers* m_mmio{nullptr};

        //! ...
        std::uint8_t* m_vram{nullptr};
    };
}
