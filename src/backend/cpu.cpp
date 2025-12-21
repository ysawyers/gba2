#include "cpu.hpp"

#include <cassert>
#include <format>


namespace backend
{
    int CPU::execute() noexcept
    {
        return 1;
    }

    int CPU::alu_opcode(std::uint32_t instr) noexcept
    {
        assert(false && "[cpu] alu_opcode TODO");

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
        assert(false && "[cpu] branch_opcode TODO");

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
        assert(false && std::format("[cpu] ARM opcode executed: {:012b}", get_mask_bits(instr)).c_str());

        m_trap = true;
        return 0;
    }

    int CPU::trap_opcode(std::uint16_t instr) noexcept
    {
        assert(false && std::format("[cpu] THUMB opcode executed: {:010b}", get_mask_bits(instr)).c_str());

        m_trap = true;
        return 0;
    }

    // TODO conditional noexcept for future passed in lambda
    void CPU::start()
    {
        m_trap = false;

        while (!m_trap)
        {
            execute();
        }
    }
}
