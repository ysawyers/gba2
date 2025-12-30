#pragma once

#include <array>
#include <cassert>
#include <string_view>

#include "common/bits.hpp"
#include "common/meta.hpp"
#include "memory.hpp"


namespace backend
{
    /*!
        ...
    */
    class CPU
    {
    public:
        CPU(std::string_view filepath) : m_memory(filepath) {}

        /*!
            \return GamePak title from ROM
        */
        std::string_view getGamePakTitle() const noexcept
        {
            return m_memory.getGamePakTitle();
        }

        /*!
            ...
        */
        bool run(PPU::FramebufferHandler ppu, bool& shutdown) noexcept(noexcept(ppu(nullptr)))
        {
            resetState();
            m_memory.registerHandlers(ppu);

            while (!shutdown)
            {
                #ifndef NDEBUG
                    std::uint32_t oldPC = m_registers[15];
                    int cycles = execute();
                    if (oldPC == m_registers[15]) [[unlikely]]
                    {
                        fprintf(stderr, "[cpu] pc value stuck at: %08X\n", oldPC);
                        return false;
                    }
                #else
                    int cycles = execute();
                #endif

                if (cycles != -1) [[likely]]
                {
                    // currently 1 cpi
                    assert(cycles == 1);

                    m_memory.tick(cycles);
                }
                else
                {
                    return false;
                }
            }
            return true;
        }

    private:
        /*!
            ...
        */
        void resetState() noexcept
        {
            m_registers.m_cpsr = 0xD3;
            // SP_svc=03007FE0h
            // SP_irq=03007FA0h
            m_registers[13] = 0x03007F00;
            m_registers[14] = 0x08000000;
            m_registers[15] = 0x08000000;
            flushPipelineARM();
        }

        /*!
            ...
        */
        void printState() noexcept
        {
            printf(
                "R0: %08X R1: %08X R2: %08X R3: %08X\n"
                "R4: %08X R5: %08X R6: %08X R7: %08X\n"
                "R8: %08X R9: %08X R10:%08X R11:%08X\n"
                "R12:%08X SP: %08X LR: %08X PC: %08X\n"
                "CPSR: %08X\n",
                m_registers[0], m_registers[1], m_registers[2], m_registers[3],
                m_registers[4], m_registers[5], m_registers[6], m_registers[7],
                m_registers[8], m_registers[9], m_registers[10], m_registers[11],
                m_registers[12], m_registers[13], m_registers[14], m_registers[15],
                m_registers.m_cpsr
            );
        }

        /*!
            \brief Performs barrel shifter operations

            \tparam ShiftType 0=LSL, 1=LSR, 2=ASR, 3=ROR
            \tparam RegisterShiftedByImmediate
                true if operand is a register that is being shifted by an immediate value

            \param[in] shiftAmount amount operand is shifted by
            \param[in] operand value to be shifted

            \return std::pair<result, carry out>
        */
        template <bool RegisterShiftedByImmediate, std::uint8_t ShiftType>
        std::pair<std::uint32_t, bool> barrelShifter(
            std::uint8_t shiftAmount,
            std::uint32_t operand
        ) const noexcept
        {
            switch (ShiftType)
            {
                case 0: // LSL
                {
                    if constexpr (RegisterShiftedByImmediate)
                    {
                        if (shiftAmount == 0)
                        {
                            return std::make_pair(operand, m_registers.getCarryFlag());
                        }
                    }

                    if (shiftAmount > 31)
                    {
                        return std::make_pair(0, (shiftAmount == 32) & (operand & 1));
                    }
                    else
                    {
                        return std::make_pair(operand << shiftAmount, operand >> ((32 - shiftAmount) & 1));
                    }
                }
                case 1: // LSR
                {
                    if constexpr (RegisterShiftedByImmediate)
                    {
                        if (shiftAmount == 0)
                        {
                            assert(false && "handle LSR#0 special case!");
                        }
                    }
                    assert(false && "LSR");
                }
                case 2: // ASR
                {
                    if constexpr (RegisterShiftedByImmediate)
                    {
                        if (shiftAmount == 0)
                        {
                            assert(false && "handle ASR#0 special case!");
                        }
                    }
                    assert(false && "ASR");
                }
                case 3: // ROR
                {
                    if constexpr (RegisterShiftedByImmediate)
                    {
                        if (shiftAmount == 0)
                        {
                            // ROR#0 implemented as RRX#1
                            assert(false && "handle ROR#0 special case!");
                        }
                    }
                    operand = ROR(operand, shiftAmount);
                    return std::make_pair(operand, operand >> 31);
                }
            }
        }

        std::uint32_t ROR(std::uint32_t operand, std::uint8_t shiftAmount) const noexcept
        {
            return (operand >> (shiftAmount & 31)) | (operand << ((-shiftAmount) & 31));
        }

        /*!
            \brief Extracts relevant bits for LUT indexing
        */
        std::uint16_t LUTIndexARM(std::uint32_t instr) const noexcept
        {
            return ((instr & 0x0FF00000) >> 16) | ((instr & 0x000000F0) >> 4);
        }

        /*!
            \brief Extracts relevant bits for LUT indexing
        */
        std::uint16_t LUTIndexTHUMB(std::uint16_t instr) const noexcept
        {
            return instr >> 6;
        }

        /*!
            \brief Flushes instruction pipeline
        */
        inline void flushPipelineARM() noexcept
        {
            m_pipeline[1] = m_memory.read<std::uint32_t>(m_registers[15]);
            m_pipeline[0] = m_memory.read<std::uint32_t>(m_registers[15] + 4);
            m_registers[15] += 8;
        }

        /*!
            \brief Flushes instruction pipeline
        */
        inline void flushPipelineTHUMB() noexcept
        {
            m_pipeline[1] = m_memory.read<std::uint16_t>(m_registers[15]);
            m_pipeline[0] = m_memory.read<std::uint16_t>(m_registers[15] + 2);
            m_registers[15] += 4;
        }

        /*!
            \brief https://problemkaputt.de/gbatek.htm#armopcodesdataprocessingalu

            \tparam I true=immediate operand2, false=register operand2
            \tparam Opcode ALU opcode
            \tparam S if true set condition flags
            \tparam ShiftType 0=LSL, 1=LSR, 2=ASR, 3=ROR
            \tparam R 1=shift by register, 0=shift by immediate
        */
        template <bool I, std::uint8_t Opcode, bool S, std::uint8_t ShiftType, bool R>
        int alu_opcode(std::uint32_t instr) noexcept
        {
            std::uint32_t oldPC = m_registers[15];

            /*
                TODO rn must be 0 for MOV/MVN and rd must be 0 for CMP/CMN/TST/TEQ
                what happens when this is violated?
            */

            std::uint8_t rd = (instr >> 12) & 0xF;
            std::uint8_t rn = (instr >> 16) & 0xF;
            std::uint32_t operand2;
            bool carryFlag;

            if constexpr (S)
            {
                if (rd == 15)
                {
                    assert(false && "[cpu] alu_opcode TODO: set cpsr from spsr");
                }
            }

            if constexpr (I)
            {
                std::uint8_t Is = ((instr >> 8) & 0xF) * 2;
                std::uint8_t nn = instr & 0xFF;
                auto [result, carryOut] = barrelShifter<false, 3>(Is, nn);
                operand2 = result;
                carryFlag = carryOut;
            }
            else
            {
                std::uint8_t rm = instr & 0xF;
                if constexpr (R)
                {
                    assert(false && "shift by register");
                }
                else
                {
                    auto [result, carryOut] = barrelShifter<true, ShiftType>(
                        (instr >> 7) & 0x1F,
                        m_registers[rm]
                    );
                    operand2 = result;
                    carryFlag = carryOut;
                }
            }

            switch (Opcode)
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
                case 0x4: // ADD
                {
                    std::uint32_t result = m_registers[rn] + operand2;
                    if constexpr (S)
                    {
                        m_registers.setNegativeFlag(result >> 31);
                        m_registers.setZeroFlag(result == 0);
                        m_registers.setCarryFlag((m_registers[rn] >> 31) + (operand2 >> 31) > (result >> 31));
                        m_registers.setOverflowFlag(
                            ((m_registers[rn] >> 31) == (operand2 >> 31)) &&
                            ((m_registers[rn] >> 31) != (result >> 31))
                        );
                    }
                    m_registers[rd] = result;
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
                case 0x9: // TEQ
                {
                    assert(false && "TEQ");
                    if constexpr (S)
                    {

                    }
                    else
                    {
                        assert(false && "instruction decoding bug: PSR Transfer");
                    }
                    break;
                }
                case 0xA: // CMP
                {
                    if constexpr (S)
                    {
                        std::uint32_t result = m_registers[rn] - operand2;
                        m_registers.setNegativeFlag(result >> 31);
                        m_registers.setZeroFlag(result == 0);
                        m_registers.setCarryFlag(m_registers[rn] >= operand2);
                        m_registers.setOverflowFlag(
                            ((m_registers[rn] >> 31) != (operand2 >> 31)) &&
                            ((m_registers[rn] >> 31) != (result >> 31))
                        );
                    }
                    else
                    {
                        assert(false && "instruction decoding bug: PSR Transfer");
                    }
                    break;
                }
                case 0xB:
                {
                    assert(false && "[cpu] alu_opcode (CMN) TODO");
                    break;
                }
                case 0xC: // ORR
                {
                    std::uint32_t result = m_registers[rn] | operand2;
                    if constexpr (S)
                    {
                        m_registers.setNegativeFlag(result >> 31);
                        m_registers.setZeroFlag(!result);
                        m_registers.setCarryFlag(carryFlag);
                    }
                    m_registers[rd] = result;
                    break;
                }
                case 0xD: // MOV
                {
                    m_registers[rd] = operand2;
                    if constexpr (S)
                    {
                        m_registers.setNegativeFlag(operand2 >> 31);
                        m_registers.setZeroFlag(operand2 == 0);
                        m_registers.setCarryFlag(carryFlag);
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
            }

            if (m_registers[15] != oldPC)
            {
                flushPipelineARM();
            }
            else
            {
                m_registers[15] += 4;
            }
            return 1;
        }

        //! https://problemkaputt.de/gbatek.htm#armopcodesmultiplyandmultiplyaccumulatemulmla
        int mul_opcode(std::uint32_t instr) noexcept
        {
            assert(false && "[cpu] mul_opcode TODO");

            return 1;
        }

        //! https://problemkaputt.de/gbatek.htm#armopcodesmemorysingledataswapswp
        int swp_opcode(std::uint32_t instr) noexcept
        {
            assert(false && "[cpu] swp_opcode TODO");

            return 1;
        }

        /*!
            \brief https://problemkaputt.de/gbatek.htm#armopcodesmemorysingledatatransferldrstrpld

            \tparam I 0=Immediate, 1=Shifted Register
            \tparam P 0=post; add offset after transfer, 1=pre; before trans.
            \tparam U 0=down; subtract offset from base, 1=up; add to base
            \tparam B 0=transfer 32bit/word, 1=transfer 8bit/byte
            \tparam L 0=Store to memory, 1=Load from memory
            \tparam TW if P=0, [T](0=Normal, 1=Force non-privileged access) otherwise [W](0=no write-back, 1=write address into base)
            \tparam ShiftType 0=LSL, 1=LSR, 2=ASR, 3=ROR
        */
        template <bool I, bool P, bool U, bool B, bool TW, bool L, std::uint8_t ShiftType>
        int ldr_str_single_transfer_opcode(std::uint32_t instr) noexcept
        {
            std::uint32_t offset;
            if constexpr (I)
            {
                offset = m_registers[instr & 0xFFF];
            }
            else
            {
                std::uint8_t rm = instr & 0xF;
                if (rm != 15)
                {
                    auto [result, carryOut] = barrelShifter<true, ShiftType>(
                        (instr >> 7) & 0x1F,
                        m_registers[rm]
                    );
                    offset = result;
                }
                else
                {
                    assert(false && "rm == 15 for ldr/str");
                }
            }

            std::uint8_t rd = (instr >> 12) & 0xF;
            std::uint8_t rn = (instr >> 16) & 0xF;
            std::uint32_t address = m_registers[rn];

            m_registers[15] += 4;
            std::uint32_t oldPC = m_registers[15];
            // PC+12 after this point due to pipeline timings

            if constexpr (P)
            {
                if constexpr (U)
                {
                    address += offset;
                }
                else
                {
                    address -= offset;
                }
            }

            if constexpr (!P || (P && TW))
            {
                if (rn != 15)
                {
                    std::uint32_t writebackAddress = address;
                    if constexpr (!P)
                    {
                        if constexpr (U)
                        {
                            writebackAddress += offset;
                        }
                        else
                        {
                            writebackAddress -= offset;
                        }
                    }

                    if (rd != rn)
                    {
                        m_registers[rn] = writebackAddress;
                    }
                }
                else
                {
                    assert(false && "writeback should not be specified?");
                }
            }

            if constexpr (L)
            {
                if constexpr (B)
                {
                    assert(false && "byte load");
                }
                else
                {
                    if (address & 1)
                    {
                        assert(false && "misaligned word load");
                    }
                    else
                    {
                        assert(false && "word load");
                    }
                }
            }
            else
            {
                if constexpr (B)
                {
                    assert(false && "byte store");
                }
                else
                {
                    assert(false && "word store");
                }
            }

            if (oldPC != m_registers[15])
            {
                flushPipelineARM();
            }
            return 1;
        }

        /*!
            \brief https://problemkaputt.de/gbatek.htm#armopcodesmemoryhalfworddoublewordandsigneddatatransfer

            \tparam P 0=post; add offset after transfer, 1=pre; before trans.
            \tparam U 0=down; subtract offset from base, 1=up; add to base
            \tparam I 0=Register Offset, 1=Immediate Offset
            \tparam W 0=no write-back, 1=write address into base
            \tparam L 0=Store to memory, 1=Load from memory
            \tparam Opcode Type of load/store performed
        */
        template <bool P, bool U, bool I, bool W, bool L, std::uint8_t Opcode>
        int ldr_str_halfword_signed_transfer_opcode(std::uint32_t instr) noexcept
        {
            std::uint32_t offset;
            if constexpr (I)
            {
                offset = (((instr >> 8) & 0xF) << 4) | (instr & 0xF);
            }
            else
            {
                std::uint8_t rm = instr & 0xF;
                if (rm != 15)
                {
                    offset = m_registers[rm];
                }
                else
                {
                    assert(false && "rm == 15 where it shouldn't?");
                }
            }

            std::uint8_t rd = (instr >> 12) & 0xF;
            std::uint8_t rn = (instr >> 16) & 0xF;
            std::uint32_t address = m_registers[rn];

            m_registers[15] += 4;
            std::uint32_t oldPC = m_registers[15];
            // PC+12 after this point due to pipeline timings

            if constexpr (P)
            {
                if constexpr (U)
                {
                    address += offset;
                }
                else
                {
                    address -= offset;
                }
            }

            if constexpr (W || !P)
            {
                if (rn != 15)
                {
                    std::uint32_t writebackAddress = address;
                    if constexpr (!P)
                    {
                        if constexpr (U)
                        {
                            writebackAddress += offset;
                        }
                        else
                        {
                            writebackAddress -= offset;
                        }
                    }

                    if (rd != rn)
                    {
                        m_registers[rn] = writebackAddress;
                    }
                }
                else
                {
                    assert(false && "writeback should not be specified?");
                }
            }

            if constexpr (L)
            {
                switch (Opcode)
                {
                    case 0:
                    {
                        assert(false && "reserved opcode. bug?");
                    }
                    case 1: // LDRH
                    {
                        if (address & 1)
                        {
                            assert(false && "unaligned LDRH");
                        }
                        else
                        {
                            m_registers[rd] = m_memory.read<std::uint16_t>(address);
                        }
                        break;
                    }
                    case 2: // LDRSB
                    {
                        m_registers[rd] = static_cast<std::int32_t>(m_memory.read<std::int8_t>(address));
                        break;
                    }
                    case 3: // LDRSH
                    {
                        if (address & 1)
                        {
                            assert(false && "unaligned LDRSH");
                        }
                        else
                        {
                            assert(false && "LDRSH");
                        }
                    }
                }
            }
            else
            {
                m_memory.write<std::uint16_t>(address, m_registers[rd]);
            }

            if (m_registers[15] != oldPC)
            {
                flushPipelineARM();
            }
            return 1;
        }

        /*!
            \brief https://problemkaputt.de/gbatek.htm#armopcodespsrtransfermrsmsr

            \tparam I 0=immediate, 1=register
            \tparam PSR 0=CPSR, 1=SPSR
            \tparam MSR 0=MRS, 1=MSR
        */
        template <bool I, bool PSR, bool MSR>
        int psr_transfer_opcode(std::uint32_t instr) noexcept
        {
            // constexpr std::uint32_t MSR_MASK = 0xFF0000FF;
            std::uint32_t operand;

            if constexpr (I)
            {
                auto [result, carryOut] = barrelShifter<false, 3>(
                    ((instr >> 8) & 0xF) * 2,
                    instr & 0xFF
                );
                operand = result;
            }
            else
            {
                assert(false && "[cpu] msr_opcode (register) TODO");
            }

            if constexpr (MSR)
            {
                if ((instr >> 19) & 1) // flag bits
                {
                    m_registers.setNegativeFlag((operand >> 31) & 1);
                    m_registers.setZeroFlag((operand >> 30) & 1);
                    m_registers.setCarryFlag((operand >> 29) & 1);
                    m_registers.setOverflowFlag((operand >> 28) & 1);
                }

                if ((instr >> 16) & 1) // control bits
                {
                    // The T-bit must not change
                    operand &= ~(1 << CPSR_T_BIT);

                    assert(false && "[cpu] msr_opcode control mask");
                }
            }
            else
            {
                assert(false && "[cpu] mrs_opcode TODO");
            }

            m_registers[15] += 4;
            return 1;
        }

        /*!
            \brief https://problemkaputt.de/gbatek.htm#armopcodesbranchandbranchwithlinkbblbxblxswibkpt

            \tparam Opcode if true branch with link
        */
        template <bool Opcode>
        int branch_opcode(std::uint32_t instr) noexcept
        {
            if constexpr (Opcode)
            {
                m_registers[14] = m_registers[15] - 4;
            }
            auto nn = static_cast<std::int32_t>((instr & 0xFFFFFF) << 8) >> 8;
            m_registers[15] += nn * 4;
            flushPipelineARM();
            return 1;
        }

        //! https://problemkaputt.de/gbatek.htm#armopcodesbranchandbranchwithlinkbblbxblxswibkpt
        int bx_opcode(std::uint32_t instr) noexcept
        {
            std::uint32_t address = m_registers[instr & 0xF];
            bool enableThumb = address & 1;

            m_registers.m_cpsr = (m_registers.m_cpsr & ~(1 << CPSR_T_BIT)) | (enableThumb << CPSR_T_BIT);
            m_registers[15] = address;
            if (enableThumb)
            {
                flushPipelineTHUMB();
            }
            else
            {
                flushPipelineARM();
            }

            printState();
            exit(0);

            return 1;
        }

        /*!
            \brief https://problemkaputt.de/gbatek.htm#armopcodesmemoryblockdatatransferldmstm

            \tparam P 0=post; add offset after transfer, 1=pre; before trans.
            \tparam U 0=down; subtract offset from base, 1=up; add to base
            \tparam S 0=No, 1=load PSR or force user mode
            \tparam W 0=no write-back, 1=write address into base
            \tparam L 0=Store to memory, 1=Load from memory
        */
        template <bool P, bool U, bool S, bool W, bool L>
        int ldm_stm_opcode(std::uint32_t instr) noexcept
        {
            std::uint8_t rn = (instr >> 16) & 0xF;
            std::uint32_t address = m_registers[rn];
            std::uint16_t reglist = instr & 0xFFFF;
            bool baseInRegList = (reglist >> rn) & 1;

            if (rn == 15) [[unlikely]]
            {
                assert(false && "rn == 15 what happens???");
            }

            m_registers[15] += 4;
            std::uint32_t oldPC = m_registers[15];
            // PC+12 after this point due to pipeline timings

            int offset = U ? 4 : -4;

            if (reglist == 0) [[unlikely]]
            {
                reglist |= 0x8000;
                offset = U ? 64 : -64;
            }

            // std::uint8_t prevMode = 0;
            if constexpr (S)
            {
                if constexpr (L)
                {
                    if ((reglist >> 15) & 1)
                    {
                        assert(false && "TODO cpsr=spsr_<current mode>");
                    }
                }
                else
                {
                    assert(false && "TODO user bank transfer");
                }
            }

            int start;
            int stop;
            int stride;

            if constexpr (U)
            {
                start = __builtin_ffs(reglist) - 1;
                stop = 16;
                stride = 1;
            }
            else
            {
                start = 31 - __builtin_clz(reglist);
                stop = -1;
                stride = -1;
            }

            if constexpr (L)
            {
                for (int reg = start; reg != stop; reg += stride)
                {
                    if ((reglist >> reg) & 1)
                    {
                        if constexpr (P)
                        {
                            address += offset;
                            m_registers[reg] = m_memory.read<std::uint32_t>(address);
                        }
                        else
                        {
                            m_registers[reg] = m_memory.read<std::uint32_t>(address);
                            address += offset;
                        }
                    }
                }
            }
            else
            {
                if constexpr (P)
                {
                    /*
                        If the base register is the first transfer in the register list,
                        the unmodified value is written back instead of base + offset.
                    */
                    if (start == rn)
                    {
                        m_memory.write<std::uint32_t>(address, m_registers[start]);
                        address += offset;
                        start += stride;
                    }
                }

                for (int reg = start; reg != stop; reg += stride)
                {
                    if ((reglist >> reg) & 1)
                    {
                        if constexpr (P)
                        {
                            address += offset;
                            m_memory.write<std::uint32_t>(address, m_registers[reg]);
                        }
                        else
                        {
                            m_memory.write<std::uint32_t>(address, m_registers[reg]);
                            address += offset;
                        }
                    }
                }
            }

            if constexpr (S)
            {
                assert(false && "return to previous mode");
            }

            if constexpr (W)
            {
                /*
                    No writeback should occur if the base register
                    was used in the register list.
                */
                if (!baseInRegList)
                {
                    m_registers[rn] = address;
                }
            }

            if (oldPC != m_registers[15])
            {
                flushPipelineARM();
            }
            return 1;
        }

        //! https://problemkaputt.de/gbatek.htm#armopcodesbranchandbranchwithlinkbblbxblxswibkpt
        int swi_opcode(std::uint32_t instr) noexcept
        {
            assert(false && "[cpu] swi_opcode TODO");

            return 1;
        }

        //! handles undefined opcodes (ARM)
        int trap_opcode(std::uint32_t instr) noexcept
        {
            return -1;
        }

        //! handles undefined opcodes (THUMB)
        int trap_opcode(std::uint16_t instr) noexcept
        {
            return -1;
        }

        /*!
            \brief Executes a single instruction

            \return number of cycles taken
        */
        int execute() noexcept
        {
            // TODO handle IRQ here

            std::uint32_t instr = m_pipeline[1];
            m_pipeline[1] = m_pipeline[0];

            m_registers[15] &= ~1;

            /*
                The PC offset (+2/+4) fetching into pipeline[0] happens
                concurrently during execution in hardware so that is simulated
                by handling the PC offset inside each instruction respectively
            */
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

    private:
        static constexpr auto CPSR_T_BIT = 5;

        enum class Mode
        {
            SYSUSER,
            FIQ,
            IRQ,
            SVC,
            ABT,
            UND
        };

        enum class Opcode
        {
            ALU,
            MUL,
            SWP,
            LDR_STR_SINGLE_TRANSFER,
            LDR_STR_SIGNED_TRANSFER,
            BRANCH,
            BX,
            LDM_STM,
            SWI,
            PSR,
            TRAP
        };

        using ARMOpcodeHandler = int(CPU::*)(std::uint32_t);
        using THUMBOpcodeHandler = int(CPU::*)(std::uint16_t);

        //! https://problemkaputt.de/gbatek.htm#armcpuregisterset
        struct Registers
        {
        public:
            /*!
                \brief Access registers 0-15 by index
            */
            std::uint32_t& operator[](std::size_t index) noexcept
            {
                return m_regs[index];
            }

            std::array<std::uint32_t, 17>& getBank(Mode mode) noexcept
            {
                return m_bankedRegs[std::to_underlying(mode)];
            }

            bool getNegativeFlag() const noexcept
            {
                return (m_cpsr >> 31) & 1;
            }

            bool getZeroFlag() const noexcept
            {
                return (m_cpsr >> 30) & 1;
            }

            bool getCarryFlag() const noexcept
            {
                return (m_cpsr >> 29) & 1;
            }

            bool getOverflowFlag() const noexcept
            {
                return (m_cpsr >> 28) & 1;
            }

            void setNegativeFlag(bool n) noexcept
            {
                m_cpsr = (m_cpsr & ~(1 << 31)) | (static_cast<std::uint32_t>(n) << 31);
            }

            void setZeroFlag(bool z) noexcept
            {
                m_cpsr = (m_cpsr & ~(1 << 30)) | (static_cast<std::uint32_t>(z) << 30);
            }

            void setCarryFlag(bool c) noexcept
            {
                m_cpsr = (m_cpsr & ~(1 << 29)) | (static_cast<std::uint32_t>(c) << 29);
            }

            void setOverflowFlag(bool v) noexcept
            {
                m_cpsr = (m_cpsr & ~(1 << 28)) | (static_cast<std::uint32_t>(v) << 28);
            }

        public:
            std::uint32_t m_cpsr{};
            std::uint32_t m_spsr{};

        private:
            //! active registers
            std::array<std::uint32_t, 16> m_regs{};

            //! banked registers + spsr
            std::array<std::array<std::uint32_t, 17>, 6> m_bankedRegs{};
        };

        //! lookup table for ARM opcodes
        std::array<ARMOpcodeHandler, 4096> m_armLUT = [] consteval
        {
            std::array<Opcode, 4096> opcodeLUT{};
            opcodeLUT.fill(Opcode::TRAP);

            auto registerOpcodes =
            [&](Opcode opcode, std::uint16_t bitmask, std::uint16_t wildcard)
            {
                auto bitmasks = common::generatePermutations<std::uint16_t>(bitmask, wildcard);
                for (const auto bitmask : bitmasks)
                {
                    opcodeLUT[bitmask] = opcode;
                }
            };

            /* GBA does not use coprocessor instructions */

            // SWI
            registerOpcodes(Opcode::SWI, 0b111100000000, 0b000011111111);
            // B, BL
            registerOpcodes(Opcode::BRANCH, 0b101000000000, 0b000111111111);
            // LDM, STM
            registerOpcodes(Opcode::LDM_STM, 0b100000000000, 0b000111111111);
            // LDR, STR (register offset)
            registerOpcodes(Opcode::LDR_STR_SINGLE_TRANSFER, 0b011000000000, 0b000111111110);
            // LDR, STR (immediate offset)
            registerOpcodes(Opcode::LDR_STR_SINGLE_TRANSFER, 0b010000000000, 0b000111111111);
            // ALU (immediate value)
            registerOpcodes(Opcode::ALU, 0b001000000000, 0b000111111111);
            // ALU (register shift)
            registerOpcodes(Opcode::ALU, 0b000000000001, 0b000111110110);
            // ALU (immediate shift)
            registerOpcodes(Opcode::ALU, 0b000000000000, 0b000111111110);
            // BX
            registerOpcodes(Opcode::BX, 0b000100100001, 0b000000000000);
            // MSR (immediate)
            registerOpcodes(Opcode::PSR, 0b001100100000, 0b000001001111);
            // MSR (register)
            registerOpcodes(Opcode::PSR, 0b000100100000, 0b000001000000);
            // MRS
            registerOpcodes(Opcode::PSR, 0b000100000000, 0b000001000000);
            // LDRSB, LDRSH
            registerOpcodes(Opcode::LDR_STR_SIGNED_TRANSFER, 0b000000011101, 0b000111100010);
            // LDRH, STRH
            registerOpcodes(Opcode::LDR_STR_SIGNED_TRANSFER, 0b000000001011, 0b000111110000);
            // SWP
            registerOpcodes(Opcode::SWP, 0b000100001001, 0b000001000000);
            // MULL, MLAL
            registerOpcodes(Opcode::MUL, 0b000010001001, 0b000001110000);
            // MUL, MLA
            registerOpcodes(Opcode::MUL, 0b000000001001, 0b000000110000);

            std::array<ARMOpcodeHandler, 4096> lut;
            lut.fill(&CPU::trap_opcode);

            common::staticFor<std::uint16_t, 4096>([&](auto hash)
            {
                /*
                    hash (12-bit value):
                    11 | 10 |  9 |  8 |  7 |  6 |  5 |  4 | 3 | 2 | 1 | 0

                    corresponding bits in instruction:
                    27 | 26 | 25 | 24 | 23 | 22 | 21 | 20 | 7 | 6 | 5 | 4
                */
                switch (opcodeLUT[hash])
                {
                    case Opcode::ALU:
                    {
                        lut[hash] = &CPU::alu_opcode<
                            (hash >> 9) & 1,
                            (hash >> 5) & 0xF,
                            (hash >> 4) & 1,
                            (hash >> 1) & 3,
                            hash & 1
                        >;
                        break;
                    }
                    case Opcode::MUL:
                    {
                        lut[hash] = &CPU::mul_opcode;
                        break;
                    }
                    case Opcode::SWP:
                    {
                        lut[hash] = &CPU::swp_opcode;
                        break;
                    }
                    case Opcode::LDR_STR_SINGLE_TRANSFER:
                    {
                        lut[hash] = &CPU::ldr_str_single_transfer_opcode<
                            (hash >> 9) & 1,
                            (hash >> 8) & 1,
                            (hash >> 7) & 1,
                            (hash >> 6) & 1,
                            (hash >> 5) & 1,
                            (hash >> 4) & 1,
                            (hash >> 1) & 3
                        >;
                        break;
                    }
                    case Opcode::LDR_STR_SIGNED_TRANSFER:
                    {
                        lut[hash] = &CPU::ldr_str_halfword_signed_transfer_opcode<
                            (hash >> 8) & 1,
                            (hash >> 7) & 1,
                            (hash >> 6) & 1,
                            (hash >> 5) & 1,
                            (hash >> 4) & 1,
                            (hash >> 1) & 3
                        >;
                        break;
                    }
                    case Opcode::BRANCH:
                    {
                        lut[hash] = &CPU::branch_opcode<(hash >> 8) & 1>;
                        break;
                    }
                    case Opcode::BX:
                    {
                        lut[hash] = &CPU::bx_opcode;
                        break;
                    }
                    case Opcode::LDM_STM:
                    {
                        lut[hash] = &CPU::ldm_stm_opcode<
                            (hash >> 8) & 1,
                            (hash >> 7) & 1,
                            (hash >> 6) & 1,
                            (hash >> 5) & 1,
                            (hash >> 4) & 1
                        >;
                        break;
                    }
                    case Opcode::SWI:
                    {
                        lut[hash] = &CPU::swi_opcode;
                        break;
                    }
                    case Opcode::PSR:
                    {
                        lut[hash] = &CPU::psr_transfer_opcode<
                            (hash >> 9) & 1,
                            (hash >> 6) & 1,
                            (hash >> 5) & 1
                        >;
                        break;
                    }
                    case Opcode::TRAP:
                    {
                        lut[hash] = &CPU::trap_opcode;
                        break;
                    }
                }
            });

            return lut;
        }();

        //! lookup table for THUMB opcodes
        std::array<THUMBOpcodeHandler, 1024> m_thumbLUT = [] consteval
        {
            std::array<Opcode, 1024> opcodeLUT;
            opcodeLUT.fill(Opcode::TRAP);

            // auto registerOpcodes =
            // [&](Opcode opcode, std::uint16_t bitmask, std::uint16_t wildcard)
            // {
            //     auto bitmasks = common::generatePermutations<std::uint16_t>(bitmask, wildcard);
            //     for (const auto bitmask : bitmasks)
            //     {
            //         opcodeLUT[bitmask] = opcode;
            //     }
            // };

            // TODO

            std::array<THUMBOpcodeHandler, 1024> lut;
            lut.fill(&CPU::trap_opcode);

            return lut;
        }();

        //! lookup table for condition codes
        std::array<bool, 256> m_conditionLUT = [] consteval
        {
            std::array<bool, 256> lut{};

            for (int flags = 0b0000; flags <= 0b1111; flags++)
            {
                bool n = flags & 8;
                bool z = flags & 4;
                bool c = flags & 2;
                bool v = flags & 1;

                lut[(0b0000 << 4) | flags] = z;                // COND EQ
                lut[(0b0001 << 4) | flags] = !z;               // COND NE
                lut[(0b0010 << 4) | flags] = c;                // COND CS
                lut[(0b0011 << 4) | flags] = !c;               // COND CC
                lut[(0b0100 << 4) | flags] = n;                // COND MI
                lut[(0b0101 << 4) | flags] = !n;               // COND PL
                lut[(0b0110 << 4) | flags] = v;                // COND VS
                lut[(0b0111 << 4) | flags] = !v;               // COND VC
                lut[(0b1000 << 4) | flags] = c && !z;          // COND HI
                lut[(0b1001 << 4) | flags] = !c || z;          // COND LS
                lut[(0b1010 << 4) | flags] = n == v;           // COND GE
                lut[(0b1011 << 4) | flags] = n != v;           // COND LT
                lut[(0b1100 << 4) | flags] = !(z || (n != v)); // COND GT
                lut[(0b1101 << 4) | flags] = z || (n != v);    // COND LE
                lut[(0b1110 << 4) | flags] = true;             // COND AL
            }

            return lut;
        }();

        //! memory interface
        Memory m_memory;

        //! active registers
        Registers m_registers{};

        //! 3-stage pipeline ([0]=fetch, [1]=decode)
        std::array<std::uint32_t, 2> m_pipeline
        {
            0xF0000000,
            0xF0000000
        };
    };
}
