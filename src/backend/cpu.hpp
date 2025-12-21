#include <array>
#include <cassert>
#include <cstdint>
#include <string_view>

#include "common/bits.hpp"
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
            /param[in] filepath Path to ROM file
        */
        CPU(std::string_view filepath) :
            m_memory(filepath)
        {
        }

        /*!
            ...
        */
        void start();

    private:
        /*!
            \brief Extracts relevant bits for LUT indexing
        */
        std::uint16_t get_mask_bits(std::uint32_t instr) const noexcept
        {
            return ((instr & 0x00FF0000) >> 12) | ((instr & 0x000000F0) >> 4);
        }

        /*!
            \brief Extracts relevant bits for LUT indexing
        */
        std::uint16_t get_mask_bits(std::uint16_t instr) const noexcept
        {
            return instr >> 6;
        }

        /*!
            \brief Executes a single instruction

            \return number of cycles taken
        */
        int execute() noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesdataprocessingalu
        int alu_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesmultiplyandmultiplyaccumulatemulmla
        int mul_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesmemorysingledataswapswp
        int swp_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesmemoryhalfworddoublewordandsigneddatatransfer
        int ldr_str_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodespsrtransfermrsmsr
        int psr_transfer_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesbranchandbranchwithlinkbblbxblxswibkpt
        int branch_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesmemoryblockdatatransferldmstm
        int ldm_stm_opcode(std::uint32_t instr) noexcept;

        //! https://problemkaputt.de/gbatek.htm#armopcodesbranchandbranchwithlinkbblbxblxswibkpt
        int swi_opcode(std::uint32_t instr) noexcept;

        //! handles undefined opcodes (ARM)
        int trap_opcode(std::uint32_t instr) noexcept;

        //! handles undefined opcodes (THUMB)
        int trap_opcode(std::uint16_t instr) noexcept;

    private:
        using ARMOpcodeHandler = int(CPU::*)(std::uint32_t);
        using THUMBOpcodeHandler = int(CPU::*)(std::uint16_t);

        //! https://problemkaputt.de/gbatek.htm#armcpuregisterset
        enum class Mode
        {
            System     = 0,
            User       = 0,
            FIQ        = 1,
            SVC        = 2,
            ABT        = 3,
            IRQ        = 4,
            UND        = 5,
        };

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

            /*!
                \brief Swaps out banked registers when switching modes

                \param[in] currentMode Current CPU mode
                \param[in] newMode New CPU mode
            */
            void swapBanks(std::uint8_t currentMode, std::uint8_t newMode) noexcept
            {
                // TODO
            }

        public:
            std::uint32_t m_cpsr{};
            std::uint32_t m_spsr{};

        private:
            //! active registers
            std::array<std::uint32_t, 16> m_regs{};

            //! banked registers + spsr
            // std::array<std::array<std::uint32_t, 17>, 6> m_bankedRegs{};
        };

        //! lookup table for ARM opcodes
        std::array<ARMOpcodeHandler, 4096> m_armLUT = [] consteval
        {
            std::array<ARMOpcodeHandler, 4096> lut;
            lut.fill(&CPU::trap_opcode);

            auto registerHandler =
            [&lut](ARMOpcodeHandler handler, std::uint16_t bitmask, std::uint16_t wildcard)
            {
                auto bitmasks = common::generatePermutations<std::uint16_t>(bitmask, wildcard);
                for (const auto bitmask : bitmasks)
                {
                    lut[bitmask] = handler;
                }
            };

            /* GBA does not use coprocessor instructions */

            // MUL, MLA
            registerHandler(&CPU::mul_opcode, 0b000000001001, 0b000000110000);
            // MULL, MLAL
            registerHandler(&CPU::mul_opcode, 0b000010001001, 0b000001110000);
            // SWP
            registerHandler(&CPU::swp_opcode, 0b000100001001, 0b000001000000);
            // LDRH, STRH
            registerHandler(&CPU::ldr_str_opcode, 0b000000001011, 0b000111110000);
            // LDRSB, LDRSH
            registerHandler(&CPU::ldr_str_opcode, 0b000000011101, 0b000111100010);
            // MRS
            registerHandler(&CPU::psr_transfer_opcode, 0b000100000000, 0b000001000000);
            // MSR (register)
            registerHandler(&CPU::psr_transfer_opcode, 0b000100100000, 0b000001000000);
            // MSR (immediate)
            registerHandler(&CPU::psr_transfer_opcode, 0b001100100000, 0b000001000000);
            // BX
            registerHandler(&CPU::branch_opcode, 0b000100100001, 0b000000000000);
            // ALU (immediate shift)
            registerHandler(&CPU::alu_opcode, 0b000000000000, 0b000111111110);
            // ALU (register shift)
            registerHandler(&CPU::alu_opcode, 0b000000000001, 0b000111110110);
            // ALU (immediate value)
            registerHandler(&CPU::alu_opcode, 0b001000000000, 0b000111111111);
            // LDR, STR (immediate offset)
            registerHandler(&CPU::ldr_str_opcode, 0b010000000000, 0b000111111111);
            // LDR, STR (register offset)
            registerHandler(&CPU::ldr_str_opcode, 0b011000000000, 0b000111111110);
            // LDM, STM
            registerHandler(&CPU::ldm_stm_opcode, 0b100000000000, 0b000111111111);
            // B, BL
            registerHandler(&CPU::branch_opcode, 0b101000000000, 0b000111111111);
            // SWI
            registerHandler(&CPU::swi_opcode, 0b111100000000, 0b000011111111);

            return lut;
        }();

        //! lookup table for THUMB opcodes
        std::array<THUMBOpcodeHandler, 1024> m_thumbLUT = [] consteval
        {
            std::array<THUMBOpcodeHandler, 1024> lut;
            lut.fill(&CPU::trap_opcode);

            // auto registerHandler =
            // [&lut](std::uint16_t bitmask, std::uint16_t wildcard, THUMBOpcodeHandler handler)
            // {
            //     auto bitmasks = common::generatePermutations<std::uint16_t>(bitmask, wildcard);
            //     for (const auto bitmask : bitmasks)
            //     {
            //         lut[bitmask] = handler;
            //     }
            // };

            return lut;
        }();

        //! memory interface
        Memory m_memory;

        //! 3-stage pipeline
        std::array<std::uint32_t, 2> m_pipeline
        {
            0xF0000000, // fetch
            0xF0000000  // decode
        };

        //! active registers
        Registers m_registers{};

        //! trap flag
        bool m_trap{false};
    };
}
