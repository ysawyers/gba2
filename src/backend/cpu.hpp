#pragma once

#include <array>
#include <cassert>
#include <cstdint>
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
        /*!
            \brief Loads ROM from a file

            /param[in] filepath Path to a ROM file
        */
        void loadROM(std::string_view filepath)
        {
            m_memory.loadROM(filepath);
        }

        /*!
            TODO conditional noexcept for future passed in lambda
        */
        void start();

    private:
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

        std::uint32_t ROR(std::uint32_t operand, std::uint8_t shiftAmount) const noexcept
        {
            return (operand >> (shiftAmount & 31)) | (operand << ((-shiftAmount) & 31));
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
            \brief Performs barrel shifter operations

            \tparam RegisterShiftedByImmediate
                true if operand is a register that is being shifted by an immediate value

            \param[in] shiftType 0=LSL, 1=LSR, 2=ASR, 3=ROR
            \param[in] shiftAmount amount operand is shifted by
            \param[in] operand value to be shifted

            \return result of shift
        */
        template <bool RegisterShiftedByImmediate>
        std::uint32_t barrelShifter(
            std::uint8_t shiftType,
            std::uint8_t shiftAmount,
            std::uint32_t operand
        ) const noexcept
        {
            std::uint32_t result = 0;

            assert(shiftType <= 3);
            switch (shiftType)
            {
                case 0: // LSL
                {
                    if constexpr (RegisterShiftedByImmediate)
                    {
                        if (shiftAmount == 0)
                        {
                            return operand;
                        }
                    }
                    assert(false && "LSL");
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
                        assert(false && "handle ROR#0 special case!");
                    }
                    assert(false && "ROR");
                }
                default: std::unreachable();
            }

            return result;
        }

        /*!
            \brief Executes a single instruction

            \return number of cycles taken
        */
        int execute() noexcept;

        /*!
            \brief https://problemkaputt.de/gbatek.htm#armopcodesdataprocessingalu

            \tparam I true=immediate operand2, false=register operand2
            \tparam ALU opcode
            \tparam S true=set condition flags, false=do not set condition flags
        */
        template <bool I, std::uint8_t Opcode, bool S>
        int alu_opcode(std::uint32_t instr) noexcept
        {
            std::uint32_t oldPC = m_registers[15];

            std::uint8_t rd = (instr >> 12) & 0xF;
            std::uint8_t rn = (instr >> 16) & 0xF;
            std::uint32_t operand2;

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
                operand2 = barrelShifter<false>(3, Is, nn);
            }
            else
            {
                assert(false && "register as second operand");
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
                    return -1;
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
            else
            {
                m_registers[15] += 4;
            }

            return 1;
        }

        //! https://problemkaputt.de/gbatek.htm#armopcodesmultiplyandmultiplyaccumulatemulmla
        int mul_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesmemorysingledataswapswp
        int swp_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesmemoryhalfworddoublewordandsigneddatatransfer
        int ldr_str_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodespsrtransfermrsmsr
        int mrs_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodespsrtransfermrsmsr
        int msr_opcode(std::uint32_t instr) noexcept;

        /*!
            \brief https://problemkaputt.de/gbatek.htm#armopcodesbranchandbranchwithlinkbblbxblxswibkpt

            \tparam Opcode true=BL, false=B
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
        int bx_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesmemoryblockdatatransferldmstm
        int ldm_stm_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesbranchandbranchwithlinkbblbxblxswibkpt
        int swi_opcode(std::uint32_t instr) noexcept;

        //! handles undefined opcodes (ARM)
        int trap_opcode(std::uint32_t instr) noexcept;

        //! handles undefined opcodes (THUMB)
        int trap_opcode(std::uint16_t instr) noexcept;

    private:
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
            LDR_STR,
            BRANCH,
            BX,
            LDM_STM,
            SWI,
            MRS,
            MSR,
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
            std::uint32_t m_cpsr{0xD3};
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
            [&opcodeLUT](Opcode opcode, std::uint16_t bitmask, std::uint16_t wildcard)
            {
                auto bitmasks = common::generatePermutations<std::uint16_t>(bitmask, wildcard);
                for (const auto bitmask : bitmasks)
                {
                    opcodeLUT[bitmask] = opcode;
                }
            };

            /* GBA does not use coprocessor instructions */

            // MUL, MLA
            registerOpcodes(Opcode::MUL, 0b000000001001, 0b000000110000);
            // MULL, MLAL
            registerOpcodes(Opcode::MUL, 0b000010001001, 0b000001110000);
            // SWP
            registerOpcodes(Opcode::SWP, 0b000100001001, 0b000001000000);
            // LDRH, STRH
            registerOpcodes(Opcode::LDR_STR, 0b000000001011, 0b000111110000);
            // LDRSB, LDRSH
            registerOpcodes(Opcode::LDR_STR, 0b000000011101, 0b000111100010);
            // MRS
            registerOpcodes(Opcode::MRS, 0b000100000000, 0b000001000000);
            // MSR (register)
            registerOpcodes(Opcode::MSR, 0b000100100000, 0b000001000000);
            // MSR (immediate)
            registerOpcodes(Opcode::MSR, 0b001100100000, 0b000001000000);
            // BX
            registerOpcodes(Opcode::BX, 0b000100100001, 0b000000000000);
            // ALU (immediate shift)
            registerOpcodes(Opcode::ALU, 0b000000000000, 0b000111111110);
            // ALU (register shift)
            registerOpcodes(Opcode::ALU, 0b000000000001, 0b000111110110);
            // ALU (immediate value)
            registerOpcodes(Opcode::ALU, 0b001000000000, 0b000111111111);
            // LDR, STR (immediate offset)
            registerOpcodes(Opcode::LDR_STR, 0b010000000000, 0b000111111111);
            // LDR, STR (register offset)
            registerOpcodes(Opcode::LDR_STR, 0b011000000000, 0b000111111110);
            // LDM, STM
            registerOpcodes(Opcode::LDM_STM, 0b100000000000, 0b000111111111);
            // B, BL
            registerOpcodes(Opcode::BRANCH, 0b101000000000, 0b000111111111);
            // SWI
            registerOpcodes(Opcode::SWI, 0b111100000000, 0b000011111111);

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
                        lut[hash] = &CPU::alu_opcode<(hash >> 9) & 1, (hash >> 5) & 0xF, (hash >> 4) & 1>;
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
                    case Opcode::LDR_STR:
                    {
                        lut[hash] = &CPU::ldr_str_opcode;
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
                        lut[hash] = &CPU::ldm_stm_opcode;
                        break;
                    }
                    case Opcode::SWI:
                    {
                        lut[hash] = &CPU::swi_opcode;
                        break;
                    }
                    case Opcode::MRS:
                    {
                        lut[hash] = &CPU::mrs_opcode;
                        break;
                    }
                    case Opcode::MSR:
                    {
                        lut[hash] = &CPU::msr_opcode;
                        break;
                    }
                    case Opcode::TRAP:
                    {
                        lut[hash] = &CPU::trap_opcode;
                        break;
                    }
                    default: break;
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
            // [&lut](Opcode opcode, std::uint16_t bitmask, std::uint16_t wildcard)
            // {
            //     auto bitmasks = common::generatePermutations<std::uint16_t>(bitmask, wildcard);
            //     for (const auto bitmask : bitmasks)
            //     {
            //         lut[bitmask] = opcode;
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
        Memory m_memory{};

        //! 3-stage pipeline ([0]=fetch, [1]=decode)
        std::array<std::uint32_t, 2> m_pipeline
        {
            0xF0000000,
            0xF0000000
        };

        //! active registers
        Registers m_registers{};

        //! trap flag
        bool m_trap{false};
    };
}
