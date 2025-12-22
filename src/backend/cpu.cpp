#include "cpu.hpp"

#include <cassert>
#include <cstdint>
#include <format>
#include <iostream>
#include <utility>


namespace
{
    constexpr auto CPSR_T_BIT = 5;
}

namespace backend
{
    inline void CPU::flushPipelineARM() noexcept
    {
        m_pipeline[1] = m_memory.read<std::uint32_t>(m_registers[15]);
        m_pipeline[0] = m_memory.read<std::uint32_t>(m_registers[15] + 4);
        m_registers[15] += 8;
    }

    inline void CPU::flushPipelineTHUMB() noexcept
    {
        m_pipeline[1] = m_memory.read<std::uint16_t>(m_registers[15]);
        m_pipeline[0] = m_memory.read<std::uint16_t>(m_registers[15] + 2);
        m_registers[15] += 4;
    }

    std::uint32_t CPU::barrelShifter(
        std::uint8_t shiftType,
        std::uint8_t shiftAmount,
        std::uint32_t operand
    ) const noexcept
    {
        assert(shiftType <= 3);

        switch (shiftType)
        {
            case 0: // LSL
            {
                assert(false && "LSL");
            }
            case 1: // LSR
            {
                assert(false && "LSR");
            }
            case 2: // ASR
            {
                assert(false && "ASR");
            }
            case 3: // ROR
            {
                assert(false && "ROR");
            }
            default: std::unreachable();
        }
    }

    int CPU::execute() noexcept
    {
        std::uint32_t instr = m_pipeline[1];
        m_pipeline[1] = m_pipeline[0];

        if ((m_registers.m_cpsr >> CPSR_T_BIT) & 1)
        {
            m_pipeline[0] = m_memory.read<std::uint16_t>(m_registers[15]);
            m_registers[15] += 2;

            (this->*m_thumbLUT[LUTIndexTHUMB(instr)])(instr);
        }
        else
        {
            std::uint8_t instrFlagBits = (instr >> 28) & 0x0F;
            std::uint8_t cpsrFlagBits = (m_registers.m_cpsr >> 28) & 0x0F;

            m_pipeline[0] = m_memory.read<std::uint32_t>(m_registers[15]);
            m_registers[15] += 4;

            if (m_conditionLUT[(instrFlagBits << 4) | cpsrFlagBits])
            {
                (this->*m_armLUT[LUTIndexARM(instr)])(instr);
            }
        }

        return 1;
    }

    int CPU::alu_opcode(std::uint32_t instr) noexcept
    {
        std::uint32_t oldPC = m_registers[15];

        bool S = (instr >> 20) & 1;
        bool I = (instr >> 25) & 1;
        std::uint8_t rd = (instr >> 12) & 0xF;
        std::uint8_t rn = (instr >> 16) & 0xF;
        std::uint32_t operand2;

        if (S == 1 && rd == 15)
        {
            // ensure not used in user mode

            assert(false && "[cpu] alu_opcode TODO: set cpsr from spsr");
        }

        if (I)
        {
            std::uint8_t Is = ((instr >> 8) & 0xF) * 2;
            std::uint8_t nn = instr & 0xFF;
            operand2 = barrelShifter(3, Is, nn);
        }
        else
        {
            assert(false && "register as second operand");
        }

        switch ((instr >> 21) & 0xF)
        {
            case 0x0:
            {
                assert(false && "[cpu] alu_opcode (AND) TODO");
                break;
            }
            case 0x1:
            {
                assert(false && "[cpu] alu_opcode (EOR) TODO");
                break;
            }
            case 0x2:
            {
                assert(false && "[cpu] alu_opcode (SUB) TODO");
                break;
            }
            case 0x3:
            {
                assert(false && "[cpu] alu_opcode (RSB) TODO");
                break;
            }
            case 0x4:
            {
                assert(false && "[cpu] alu_opcode (ADD) TODO");
                break;
            }
            case 0x5:
            {
                assert(false && "[cpu] alu_opcode (ADC) TODO");
                break;
            }
            case 0x6:
            {
                assert(false && "[cpu] alu_opcode (SBC) TODO");
                break;
            }
            case 0x7:
            {
                assert(false && "[cpu] alu_opcode (RSC) TODO");
                break;
            }
            case 0x8:
            {
                assert(false && "[cpu] alu_opcode (TST) TODO");
                break;
            }
            case 0x9:
            {
                assert(false && "[cpu] alu_opcode (TEQ) TODO");
                break;
            }
            case 0xA:
            {
                assert(false && "[cpu] alu_opcode (CMP) TODO");
                break;
            }
            case 0xB:
            {
                assert(false && "[cpu] alu_opcode (CMN) TODO");
                break;
            }
            case 0xC:
            {
                assert(false && "[cpu] alu_opcode (ORR) TODO");
                break;
            }
            case 0xD:
            {
                if (rn == 0) [[likely]]
                {
                    m_registers[rd] = operand2;
                }
                else
                {
                    assert(false && "malformed instruction");
                }
                break;
            }
            case 0xE:
            {
                assert(false && "[cpu] alu_opcode (BIC) TODO");
                break;
            }
            case 0xF:
            {
                assert(false && "[cpu] alu_opcode (MVN) TODO");
                break;
            }
            default: std::unreachable();
        }

        if (m_registers[15] != oldPC)
        {
            flushPipelineARM();
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

    int CPU::psr_transfer_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] psr_transfer_opcode TODO");

        return 1;
    }

    int CPU::branch_opcode(std::uint32_t instr) noexcept
    {
        bool bl = (instr >> 24) & 1;
        if (bl)
        {
            m_registers[14] = m_registers[15] - 4;
        }
        auto nn = static_cast<std::int32_t>((instr & 0xFFFFFF) << 8) >> 8;
        m_registers[15] = nn * 4;
        flushPipelineARM();
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
            execute();
        }
    }
}
