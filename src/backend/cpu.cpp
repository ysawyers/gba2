#include "cpu.hpp"

#include <cassert>
#include <cstdint>
#include <format>
#include <iostream>


namespace
{
    constexpr auto CPSR_T_BIT = 5;
}

namespace backend
{
    int CPU::execute() noexcept
    {
        std::uint32_t instr = m_pipeline[1];
        m_pipeline[1] = m_pipeline[0];

        if ((m_registers.m_cpsr >> CPSR_T_BIT) & 1)
        {
            m_pipeline[0] = m_memory.read<std::uint16_t>(m_registers[15]);

            return (this->*m_thumbLUT[LUTIndexTHUMB(instr)])(instr);
        }
        else
        {
            std::uint8_t instrFlagBits = (instr >> 28) & 0x0F;
            std::uint8_t cpsrFlagBits = (m_registers.m_cpsr >> 28) & 0x0F;

            m_pipeline[0] = m_memory.read<std::uint32_t>(m_registers[15]);

            if (m_conditionLUT[(instrFlagBits << 4) | cpsrFlagBits])
            {
                return (this->*m_armLUT[LUTIndexARM(instr)])(instr);
            }
            else
            {
                m_registers[15] += 4;
            }
        }

        return 1;
    }

    int CPU::mul_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] mul_opcode TODO");

        return 1;
    }

    int CPU::swp_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] swp_opcode TODO");

        return 1;
    }

    int CPU::ldr_str_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] ldr_str_opcode TODO");

        return 1;
    }

    int CPU::mrs_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] mrs_opcode TODO");

        return 1;
    }

    int CPU::msr_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] msr_opcode TODO");

        return 1;
    }

    int CPU::bx_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] bx_opcode TODO");

        return 1;
    }

    int CPU::ldm_stm_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] ldm_stm_opcode TODO");

        return 1;
    }

    int CPU::swi_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] swi_opcode TODO");

        return 1;
    }

    int CPU::trap_opcode(std::uint32_t instr) noexcept
    {
        std::cout << std::format("[cpu] ARM opcode executed: {:012b}", LUTIndexARM(instr)).c_str() << "\n";
        exit(0);

        m_trap = true;
        return 0;
    }

    int CPU::trap_opcode(std::uint16_t instr) noexcept
    {
        std::cout << std::format("[cpu] THUMB opcode executed: {:010b}", LUTIndexTHUMB(instr)).c_str() << "\n";
        exit(0);

        m_trap = true;
        return 0;
    }

    void CPU::start()
    {
        assert(m_memory.loadedROM());
        m_trap = false;

        while (!m_trap)
        {
            if (execute() == -1) [[unlikely]]
            {
                assert(false && "malformed instruction");
            }
        }
    }
}
