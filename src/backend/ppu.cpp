#include "ppu.hpp"

#include <cassert>


namespace
{
    static constexpr int HDRAW_CYCLES = 1004;
    static constexpr int HBLANK_CYCLES = 228;
    static constexpr int SCANLINE_CYCLES = HDRAW_CYCLES + HBLANK_CYCLES;
}

namespace backend
{
    PPU::PPU(Registers* mmio, std::uint8_t* vram) :
        m_framebuffer(new std::uint16_t[FRAME_WIDTH * FRAME_HEIGHT]{}),
        m_mmio(mmio),
        m_vram(vram)
    {
    }

    PPU::~PPU()
    {
        delete[] m_framebuffer;
    }

    void PPU::registerHandler(PPU::FramebufferHandler handler) noexcept
    {
        m_handler = handler;
    }

    void PPU::renderTextBackground()
    {
        assert(false && "render text background");
    }

    void PPU::renderScaleRotateBackground()
    {
        assert(false && "render scale/rotate background");
    }

    void PPU::renderBitmapBackground3()
    {
        assert(m_mmio->m_vcount < FRAME_HEIGHT);

        for (int col = 0; col < FRAME_WIDTH; col++)
        {
            m_framebuffer[m_mmio->m_vcount * FRAME_WIDTH + col] =
                *std::bit_cast<std::uint16_t*>(m_vram + (m_mmio->m_vcount * (FRAME_WIDTH * 2)) + (col * 2));
        }
    }

    void PPU::renderScanline()
    {
        if ((m_mmio->m_dispcnt >> 7) & 1)
        {
            // either do entire frame or just scanline ?
            assert(false && "clear screen");
        }
        else
        {
            switch (m_mmio->m_dispcnt & 7)
            {
                case 0:
                {
                    assert(false && "mode 0");
                }
                case 1:
                {
                    assert(false && "mode 1");
                }
                case 2:
                {
                    assert(false && "mode 2");
                }
                case 3:
                {
                    renderBitmapBackground3();
                    break;
                }
                case 4:
                {
                    assert(false && "mode 4");
                }
                case 5:
                {
                    assert(false && "mode 5");
                }
                case 6:
                case 7:
                {
                    assert(false && "invalid video mode");
                }
                default: std::unreachable();
            }
        }
    }

    void PPU::tick(int cycles) noexcept
    {
        assert(m_mmio != nullptr);

        for (int i = 0; i < cycles; i++)
        {
            switch (m_state.m_period)
            {
                case Period::HDRAW:
                {
                    assert(m_state.m_cycles <= HDRAW_CYCLES);

                    if (m_state.m_cycles < HDRAW_CYCLES)
                    {
                        if (m_state.m_cycles == 960)
                        {
                            renderScanline();
                        }
                    }
                    else
                    {
                        m_state.m_period = Period::HBLANK;
                        m_state.m_cycles = 0;
                    }
                    break;
                }
                case Period::HBLANK:
                {
                    assert(m_state.m_cycles <= HBLANK_CYCLES);

                    if (m_state.m_cycles == HBLANK_CYCLES)
                    {
                        assert(m_mmio->m_vcount < 160);

                        if (++m_mmio->m_vcount == 160)
                        {
                            m_handler(m_framebuffer);
                            m_state.m_period = Period::VBLANK;
                        }
                        else
                        {
                            m_state.m_period = Period::HDRAW;
                        }
                        m_state.m_cycles = 0;
                    }
                    break;
                }
                case Period::VBLANK:
                {
                    assert(m_state.m_cycles <= SCANLINE_CYCLES);

                    if (m_state.m_cycles == SCANLINE_CYCLES)
                    {
                        assert(m_mmio->m_vcount < 228);

                        if (++m_mmio->m_vcount == 228)
                        {
                            m_mmio->m_vcount = 0;
                            m_state.m_period = Period::HDRAW;
                        }
                        m_state.m_cycles = 0;
                    }
                    break;
                }
                default: std::unreachable();
            }

            m_state.m_cycles++;
        }
    }
}
