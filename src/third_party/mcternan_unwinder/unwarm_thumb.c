/***************************************************************************
 * ARM Stack Unwinder, Michael.McTernan.2001@cs.bris.ac.uk
 *
 * This program is PUBLIC DOMAIN.
 * This means that there is no copyright and anyone is able to take a copy
 * for free and use it as they wish, with or without modifications, and in
 * any context, commercially or otherwise. The only limitation is that I
 * don't guarantee that the software is fit for any purpose or accept any
 * liability for it's use or misuse - this software is without warranty.
 ***************************************************************************
 * File Description: Abstract interpretation for Thumb mode.
 **************************************************************************/

#define MODULE_NAME "UNWARM_THUMB"

#define UPGRADE_ARM_STACK_UNWIND

/***************************************************************************
 * Include Files
 **************************************************************************/

#if defined(UPGRADE_ARM_STACK_UNWIND)
#include <asm/sigcontext.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/ucontext.h>
#include "unwarm.h"

/***************************************************************************
 * Manifest Constants
 **************************************************************************/


/***************************************************************************
 * Type Definitions
 **************************************************************************/


/***************************************************************************
 * Variables
 **************************************************************************/


/***************************************************************************
 * Macros
 **************************************************************************/


/***************************************************************************
 * Local Functions
 **************************************************************************/

/** Sign extend an 11 bit value.
 * This function simply inspects bit 11 of the input \a value, and if
 * set, the top 5 bits are set to give a 2's compliment signed value.
 * \param value   The value to sign extend.
 * \return The signed-11 bit value stored in a 16bit data type.
 */
static SignedInt16 signExtend11(Int16 value)
{
    if(value & 0x400)
    {
        value |= 0xf800;
    }

    return value;
}


/***************************************************************************
 * Global Functions
 **************************************************************************/


UnwResult UnwStartThumb(UnwState * const state)
{
    Boolean  found = FALSE;
    Int16    t = UNW_MAX_INSTR_COUNT;

    do
    {
        Int16 instr;

        /* Attempt to read the instruction */
        if(!state->cb->readH(state->regData[15].v & (~0x1), &instr))
        {
            return UNWIND_IREAD_H_FAIL;
        }

        UnwPrintd5("T %x %x (LR %x) %04x:",
                   state->regData[13].v, state->regData[15].v, state->regData[14].v, instr);

        /* Check that the PC is still on Thumb alignment */
        if(!(state->regData[15].v & 0x1))
        {
            UnwPrintd1("\nError: PC misalignment\n");
            return UNWIND_INCONSISTENT;
        }

        /* Check that the SP and PC have not been invalidated */
        if(!M_IsOriginValid(state->regData[13].o) || !M_IsOriginValid(state->regData[15].o))
        {
            UnwPrintd1("\nError: PC or SP invalidated\n");
            return UNWIND_INCONSISTENT;
        }

        /* Format 1: Move shifted register
         *  LSL Rd, Rs, #Offset5
         *  LSR Rd, Rs, #Offset5
         *  ASR Rd, Rs, #Offset5
         */
        if((instr & 0xe000) == 0x0000 && (instr & 0x1800) != 0x1800)
        {
            Boolean signExtend;
            Int8    op      = (instr & 0x1800) >> 11;
            Int8    offset5 = (instr & 0x07c0) >>  6;
            Int8    rs      = (instr & 0x0038) >>  3;
            Int8    rd      = (instr & 0x0007);

            switch(op)
            {
                case 0: /* LSL */
                    UnwPrintd6("LSL r%d, r%d, #%d\t; r%d %s", rd, rs, offset5, rs, M_Origin2Str(state->regData[rs].o));
                    state->regData[rd].v = state->regData[rs].v << offset5;
                    state->regData[rd].o = state->regData[rs].o;
                    state->regData[rd].o |= REG_VAL_ARITHMETIC;
                    break;

                case 1: /* LSR */
                    UnwPrintd6("LSR r%d, r%d, #%d\t; r%d %s", rd, rs, offset5, rs, M_Origin2Str(state->regData[rs].o));
                    state->regData[rd].v = state->regData[rs].v >> offset5;
                    state->regData[rd].o = state->regData[rs].o;
                    state->regData[rd].o |= REG_VAL_ARITHMETIC;
                    break;

                case 2: /* ASR */
                    UnwPrintd6("ASL r%d, r%d, #%d\t; r%d %s", rd, rs, offset5, rs, M_Origin2Str(state->regData[rs].o));

                    signExtend = (state->regData[rs].v & 0x8000) ? TRUE : FALSE;
                    state->regData[rd].v = state->regData[rs].v >> offset5;
                    if(signExtend)
                    {
                        state->regData[rd].v |= 0xffffffff << (32 - offset5);
                    }
                    state->regData[rd].o = state->regData[rs].o;
                    state->regData[rd].o |= REG_VAL_ARITHMETIC;
                    break;
            }
        }
        /* Format 2: add/subtract
         *  ADD Rd, Rs, Rn
         *  ADD Rd, Rs, #Offset3
         *  SUB Rd, Rs, Rn
         *  SUB Rd, Rs, #Offset3
         */
        else if((instr & 0xf800) == 0x1800)
        {
            Boolean I  = (instr & 0x0400) ? TRUE : FALSE;
            Boolean op = (instr & 0x0200) ? TRUE : FALSE;
            Int8    rn = (instr & 0x01c0) >> 6;
            Int8    rs = (instr & 0x0038) >> 3;
            Int8    rd = (instr & 0x0007);

            /* Print decoding */
            UnwPrintd6("%s r%d, r%d, %c%d\t;",
                       op ? "SUB" : "ADD",
                       rd, rs,
                       I ? '#' : 'r',
                       rn);
            UnwPrintd5("r%d %s, r%d %s",
                       rd, M_Origin2Str(state->regData[rd].o),
                       rs, M_Origin2Str(state->regData[rs].o));
            if(!I)
            {
                UnwPrintd3(", r%d %s", rn, M_Origin2Str(state->regData[rn].o));

                /* Perform calculation */
                if(op)
                {
                    state->regData[rd].v = state->regData[rs].v - state->regData[rn].v;
                }
                else
                {
                    state->regData[rd].v = state->regData[rs].v + state->regData[rn].v;
                }

                /* Propagate the origin */
                if(M_IsOriginValid(state->regData[rs].v) &&
                   M_IsOriginValid(state->regData[rn].v))
                {
                    state->regData[rd].o = state->regData[rs].o;
                    state->regData[rd].o |= REG_VAL_ARITHMETIC;
                }
                else
                {
                    state->regData[rd].o = REG_VAL_INVALID;
                }
            }
            else
            {
                /* Perform calculation */
                if(op)
                {
                    state->regData[rd].v = state->regData[rs].v - rn;
                }
                else
                {
                    state->regData[rd].v = state->regData[rs].v + rn;
                }

                /* Propagate the origin */
                state->regData[rd].o = state->regData[rs].o;
                state->regData[rd].o |= REG_VAL_ARITHMETIC;
            }
        }
        /* Format 3: move/compare/add/subtract immediate
         *  MOV Rd, #Offset8
         *  CMP Rd, #Offset8
         *  ADD Rd, #Offset8
         *  SUB Rd, #Offset8
         */
        else if((instr & 0xe000) == 0x2000)
        {
            Int8    op      = (instr & 0x1800) >> 11;
            Int8    rd      = (instr & 0x0700) >>  8;
            Int8    offset8 = (instr & 0x00ff);

            switch(op)
            {
                case 0: /* MOV */
                    UnwPrintd3("MOV r%d, #0x%x", rd, offset8);
                    state->regData[rd].v = offset8;
                    state->regData[rd].o = REG_VAL_FROM_CONST;
                    break;

                case 1: /* CMP */
                    /* Irrelevant to unwinding */
                    UnwPrintd1("CMP ???");
                    break;

                case 2: /* ADD */
                    UnwPrintd5("ADD r%d, #0x%x\t; r%d %s",
                               rd, offset8, rd, M_Origin2Str(state->regData[rd].o));
                    state->regData[rd].v += offset8;
                    state->regData[rd].o |= REG_VAL_ARITHMETIC;
                    break;

                case 3: /* SUB */
                    UnwPrintd5("SUB r%d, #0x%d\t; r%d %s",
                               rd, offset8, rd, M_Origin2Str(state->regData[rd].o));
                    state->regData[rd].v += offset8;
                    state->regData[rd].o |= REG_VAL_ARITHMETIC;
                    break;
            }
        }
        /* Format 4: ALU operations
         *  AND Rd, Rs
         *  EOR Rd, Rs
         *  LSL Rd, Rs
         *  LSR Rd, Rs
         *  ASR Rd, Rs
         *  ADC Rd, Rs
         *  SBC Rd, Rs
         *  ROR Rd, Rs
         *  TST Rd, Rs
         *  NEG Rd, Rs
         *  CMP Rd, Rs
         *  CMN Rd, Rs
         *  ORR Rd, Rs
         *  MUL Rd, Rs
         *  BIC Rd, Rs
         *  MVN Rd, Rs
         */
        else if((instr & 0xfc00) == 0x4000)
        {
            Int8 op = (instr & 0x03c0) >> 6;
            Int8 rs = (instr & 0x0038) >> 3;
            Int8 rd = (instr & 0x0007);
#if defined(UNW_DEBUG)
            static const char * const mnu[16] =
            { "AND", "EOR", "LSL", "LSR",
              "ASR", "ADC", "SBC", "ROR",
              "TST", "NEG", "CMP", "CMN",
              "ORR", "MUL", "BIC", "MVN" };
#endif
            /* Print the mnemonic and registers */
            switch(op)
            {
                case 0: /* AND */
                case 1: /* EOR */
                case 2: /* LSL */
                case 3: /* LSR */
                case 4: /* ASR */
                case 7: /* ROR */
                case 9: /* NEG */
                case 12: /* ORR */
                case 13: /* MUL */
                case 15: /* MVN */
                    UnwPrintd8("%s r%d ,r%d\t; r%d %s, r%d %s",
                               mnu[op],
                               rd, rs,
                               rd, M_Origin2Str(state->regData[rd].o),
                               rs, M_Origin2Str(state->regData[rs].o));
                    break;

                case 5: /* ADC */
                case 6: /* SBC */
                    UnwPrintd4("%s r%d, r%d", mnu[op], rd, rs);
                    break;

                case 8: /* TST */
                case 10: /* CMP */
                case 11: /* CMN */
                    /* Irrelevant to unwinding */
                    UnwPrintd2("%s ???", mnu[op]);
                    break;

                case 14: /* BIC */
                    UnwPrintd5("r%d ,r%d\t; r%d %s",
                                rd, rs,
                                rs, M_Origin2Str(state->regData[rs].o));
                    state->regData[rd].v &= !state->regData[rs].v;
                    break;
            }


            /* Perform operation */
            switch(op)
            {
                case 0: /* AND */
                    state->regData[rd].v &= state->regData[rs].v;
                    break;

                case 1: /* EOR */
                    state->regData[rd].v ^= state->regData[rs].v;
                    break;

                case 2: /* LSL */
                    state->regData[rd].v <<= state->regData[rs].v;
                    break;

                case 3: /* LSR */
                    state->regData[rd].v >>= state->regData[rs].v;
                    break;

                case 4: /* ASR */
                    if(state->regData[rd].v & 0x80000000)
                    {
                        state->regData[rd].v >>= state->regData[rs].v;
                        state->regData[rd].v |= 0xffffffff << (32 - state->regData[rs].v);
                    }
                    else
                    {
                        state->regData[rd].v >>= state->regData[rs].v;
                    }

                    break;

                case 5: /* ADC */
                case 6: /* SBC */
                case 8: /* TST */
                case 10: /* CMP */
                case 11: /* CMN */
                    break;
                case 7: /* ROR */
                    state->regData[rd].v = (state->regData[rd].v >> state->regData[rs].v) |
                                    (state->regData[rd].v << (32 - state->regData[rs].v));
                    break;

                case 9: /* NEG */
                    state->regData[rd].v = -state->regData[rs].v;
                    break;

                case 12: /* ORR */
                    state->regData[rd].v |= state->regData[rs].v;
                    break;

                case 13: /* MUL */
                    state->regData[rd].v *= state->regData[rs].v;
                    break;

                case 14: /* BIC */
                    state->regData[rd].v &= !state->regData[rs].v;
                    break;

                case 15: /* MVN */
                    state->regData[rd].v = !state->regData[rs].v;
                    break;
            }

            /* Propagate data origins */
            switch(op)
            {
                case 0: /* AND */
                case 1: /* EOR */
                case 2: /* LSL */
                case 3: /* LSR */
                case 4: /* ASR */
                case 7: /* ROR */
                case 12: /* ORR */
                case 13: /* MUL */
                case 14: /* BIC */
                    if(M_IsOriginValid(state->regData[rs].o) && M_IsOriginValid(state->regData[rs].o))
                    {
                        state->regData[rd].o = state->regData[rs].o;
                        state->regData[rd].o |= REG_VAL_ARITHMETIC;
                    }
                    else
                    {
                        state->regData[rd].o = REG_VAL_INVALID;
                    }
                    break;

                case 5: /* ADC */
                case 6: /* SBC */
                    /* C-bit not tracked */
                    state->regData[rd].o = REG_VAL_INVALID;
                    break;

                case 8: /* TST */
                case 10: /* CMP */
                case 11: /* CMN */
                    /* Nothing propagated */
                    break;

                case 9: /* NEG */
                case 15: /* MVN */
                    state->regData[rd].o = state->regData[rs].o;
                    state->regData[rd].o |= REG_VAL_ARITHMETIC;
                    break;

            }

        }
        /* Format 5: Hi register operations/branch exchange
         *  ADD Rd, Hs
         *  ADD Hd, Rs
         *  ADD Hd, Hs
         */
        else if((instr & 0xfc00) == 0x4400)
        {
            Int8    op  = (instr & 0x0300) >> 8;
            Boolean h1  = (instr & 0x0080) ? TRUE: FALSE;
            Boolean h2  = (instr & 0x0040) ? TRUE: FALSE;
            Int8    rhs = (instr & 0x0038) >> 3;
            Int8    rhd = (instr & 0x0007);

            /* Adjust the register numbers */
            if(h2) rhs += 8;
            if(h1) rhd += 8;

            /* Thumb-1 required h1 or h2 set, but Thumb-2 doesn't. */

            switch(op)
            {
                case 0: /* ADD */
                    UnwPrintd5("ADD r%d, r%d\t; r%d %s",
                               rhd, rhs, rhs, M_Origin2Str(state->regData[rhs].o));
                    state->regData[rhd].v += state->regData[rhs].v;
                    state->regData[rhd].o =  state->regData[rhs].o;
                    state->regData[rhd].o |= REG_VAL_ARITHMETIC;
                    break;

                case 1: /* CMP */
                    /* Irrelevant to unwinding */
                    UnwPrintd1("CMP ???");
                    break;

                case 2: /* MOV */
                    UnwPrintd5("MOV r%d, r%d\t; r%d %s",
                               rhd, rhs, rhd, M_Origin2Str(state->regData[rhs].o));
                    state->regData[rhd].v += state->regData[rhs].v;
                    state->regData[rhd].o  = state->regData[rhd].o;
                    break;

                case 3: /* BX */
                    UnwPrintd4("BX r%d\t; r%d %s\n",
                               rhs, rhs, M_Origin2Str(state->regData[rhs].o));

                    /* Only follow BX if the data was from the stack */
                    if(state->regData[rhs].o == REG_VAL_FROM_STACK)
                    {
                        UnwPrintd2(" Return PC=0x%x\n", state->regData[rhs].v & (~0x1));

                        /* Report the return address, including mode bit */
                        if(!UnwReportRetAddr(state, state->regData[rhs].v))
                        {
                            return UNWIND_TRUNCATED;
                        }

                        /* Update the PC */
                        state->regData[15].v = state->regData[rhs].v;

                        /* Determine the new mode */
                        if(state->regData[rhs].v & 0x1)
                        {
                            /* Branching to THUMB */

                            /* Account for the auto-increment which isn't needed */
                            state->regData[15].v -= 2;
                        }
                        else
                        {
                            /* Branch to ARM */
                            return UnwStartArm(state);
                        }
                    }
                    else
                    {
                        UnwPrintd4("\nError: BX to invalid register: r%d = 0x%x (%s)\n",
                                   rhs, state->regData[rhs].o, M_Origin2Str(state->regData[rhs].o));
                        return UNWIND_FAILURE;
                    }
            }
        }
        /* Format 9: PC-relative load
         *  LDR Rd,[PC, #imm]
         */
        else if((instr & 0xf800) == 0x4800)
        {
            Int8  rd    = (instr & 0x0700) >> 8;
            Int8  word8 = (instr & 0x00ff);
            Int32 address;

            /* Compute load address, adding a word to account for prefetch */
            address = (state->regData[15].v & (~0x3)) + 4 + (word8 << 2);

            UnwPrintd3("LDR r%d, 0x%08x", rd, address);

            if(!UnwMemReadRegister(state, address, &state->regData[rd]))
            {
                return UNWIND_DREAD_W_FAIL;
            }
        }
        /* Format 13: add offset to Stack Pointer
         *  ADD sp,#+imm
         *  ADD sp,#-imm
         */
        else if((instr & 0xff00) == 0xB000)
        {
            /* McTernan's original used int8, but that is too small for many stack frames... */
            Int16 value = (instr & 0x7f) * 4;

            /* Check the negative bit */
            if((instr & 0x80) != 0)
            {
                UnwPrintd2("SUB sp,#0x%x", value);
                state->regData[13].v -= value;
            }
            else
            {
                UnwPrintd2("ADD sp,#0x%x", value);
                state->regData[13].v += value;
            }
        }
        /* Format 14: push/pop registers
         *  PUSH {Rlist}
         *  PUSH {Rlist, LR}
         *  POP {Rlist}
         *  POP {Rlist, PC}
         */
        else if((instr & 0xf600) == 0xb400)
        {
            Boolean  L     = (instr & 0x0800) ? TRUE : FALSE;
            Boolean  R     = (instr & 0x0100) ? TRUE : FALSE;
            Int8     rList = (instr & 0x00ff);

            if(L)
            {
                Int8 r;

                /* Load from memory: POP */
                UnwPrintd2("POP {Rlist%s}\n", R ? ", PC" : "");

                for(r = 0; r < 8; r++)
                {
                    if(rList & (0x1 << r))
                    {
                        /* Read the word */
                        if(!UnwMemReadRegister(state, state->regData[13].v, &state->regData[r]))
                        {
                            return UNWIND_DREAD_W_FAIL;
                        }

                        /* Alter the origin to be from the stack if it was valid */
                        if(M_IsOriginValid(state->regData[r].o))
                        {
                            state->regData[r].o = REG_VAL_FROM_STACK;
                        }

                        state->regData[13].v += 4;

                        UnwPrintd3("  r%d = 0x%08x\n", r, state->regData[r].v);
                    }
                }

                /* Check if the PC is to be popped */
                if(R)
                {
                    /* Get the return address */
                    if(!UnwMemReadRegister(state, state->regData[13].v, &state->regData[15]))
                    {
                        return UNWIND_DREAD_W_FAIL;
                    }

                    /* Alter the origin to be from the stack if it was valid */
                    if(!M_IsOriginValid(state->regData[15].o))
                    {
                        /* Return address is not valid */
                        UnwPrintd1("PC popped with invalid address\n");
                        return UNWIND_FAILURE;
                    }
                    else
                    {
                        /* The bottom bit should have been set to indicate that
                         *  the caller was from Thumb.  This would allow return
                         *  by BX for interworking APCS.
                         */
                        if((state->regData[15].v & 0x1) == 0)
                        {
                            UnwPrintd2("Warning: Return address not to Thumb: 0x%08x\n",
                                       state->regData[15].v);

                            /* Pop into the PC will not switch mode */
                            return UNWIND_INCONSISTENT;
                        }

                        /* Store the return address */
                        if(!UnwReportRetAddr(state, state->regData[15].v))
                        {
                            return UNWIND_TRUNCATED;
                        }

                        /* Now have the return address */
                        UnwPrintd2(" Return PC=%x\n", state->regData[15].v);

                        /* Update the pc */
                        state->regData[13].v += 4;

                        /* Compensate for the auto-increment, which isn't needed here */
                        state->regData[15].v -= 2;
                    }
                }

            }
            else
            {
                SignedInt8 r;

                /* Store to memory: PUSH */
                UnwPrintd2("PUSH {Rlist%s}", R ? ", LR" : "");

                /* Check if the LR is to be pushed */
                if(R)
                {
                    UnwPrintd3("\n  lr = 0x%08x\t; %s",
                               state->regData[14].v, M_Origin2Str(state->regData[14].o));

                    state->regData[13].v -= 4;

                    /* Write the register value to memory */
                    if(!UnwMemWriteRegister(state, state->regData[13].v, &state->regData[14]))
                    {
                        return UNWIND_DWRITE_W_FAIL;
                    }
                }

                for(r = 7; r >= 0; r--)
                {
                    if(rList & (0x1 << r))
                    {
                        UnwPrintd4("\n  r%d = 0x%08x\t; %s",
                                   r, state->regData[r].v, M_Origin2Str(state->regData[r].o));

                        state->regData[13].v -= 4;

                        if(!UnwMemWriteRegister(state, state->regData[13].v, &state->regData[r]))
                        {
                            return UNWIND_DWRITE_W_FAIL;
                        }
                    }
                }
            }
        }
        /* Format 17: service call
         *  SVC #imm
         */
        else if((instr & 0xff00) == 0xdf00)
        {
            Int8 call = instr & 0xff;

            /* Handle Linux sigreturn. */
            if (call == 0 && state->regData[7].v == 0xad) {
                UnwPrintd2("SVC #%d sigreturn\n", (int)call);

                Int32 sigframeAddr = state->regData[13].v;
                if (sigframeAddr & 7) {
                    UnwPrintd2("Warning: Misaligned SP at sigreturn syscall: %08x", sigframeAddr);
                    return UNWIND_INCONSISTENT;
                }

                UnwPrintd2("Stack sigframe addr: %08x\n", sigframeAddr);
#if 0
                for (int i = 0; i < 128; i++) {
                    Int32 newAddr = sigframeAddr + (i * 4);
                    UnwPrintd4("Addr %08x (%x): %08x\n", newAddr, i, *(uint32_t *)newAddr);
                }
#endif

                UnwPrintd3("LINUX_UC_MCONTEXT_OFF=%x LINUX_SC_LR_OFF=%x\n",
                           offsetof(struct ucontext, uc_mcontext),
                           offsetof(struct sigcontext, arm_lr));

                Int8 r;
                for (r = 0; r <= 15; r++) {
                    if (!UnwMemReadRegister(state,
                                            sigframeAddr +
                                                sizeof(siginfo_t) +
                                                offsetof(struct ucontext, uc_mcontext) +
                                                offsetof(struct sigcontext, arm_r0) +
                                                (uint32_t)r * 4,
                                            &state->regData[r])) {
                        return UNWIND_DREAD_W_FAIL;
                    }

                    /* Alter the origin to be from the stack if it was valid */
                    if (M_IsOriginValid(state->regData[r].o)) {
                        state->regData[r].o = REG_VAL_FROM_STACK;
                    }
                }

                RegData cpsr = { 0 };
                if (!UnwMemReadRegister(state,
                                        sigframeAddr +
                                            sizeof(siginfo_t) +
                                            offsetof(struct ucontext, uc_mcontext) +
                                            offsetof(struct sigcontext, arm_cpsr),
                                        &cpsr)) {
                    return UNWIND_DREAD_W_FAIL;
                }

                if(!UnwReportRetAddr(state, state->regData[15].v)) {
                    return UNWIND_TRUNCATED;
                }

                /* Now have the return address */
                UnwPrintd2(" Return PC=%x\n", state->regData[15].v);
                UnwPrintd2(" Link register=%x\n", state->regData[14].v);

                /* Determine the new mode */
                if(cpsr.v & 0x20)
                {
                    /* Set THUMB mode */
                    state->regData[15].v = state->regData[15].v | 0x1;

                    /* Account for the auto-increment which isn't needed */
                    state->regData[15].v -= 2;
                }
                else
                {
                    /* Branch to ARM */
                    return UnwStartArm(state);
                }
            } else {
                UnwPrintd2("SVC #%d", (int)call);
            }
        }
        /* Format 18: unconditional branch
         *  B label
         */
        else if((instr & 0xf800) == 0xe000)
        {
            SignedInt16 branchValue = signExtend11(instr & 0x07ff);

            /* Branch distance is twice that specified in the instruction. */
            branchValue *= 2;

            UnwPrintd2("B %d \n", branchValue);

            /* Update PC */
            state->regData[15].v += branchValue;

            /* Need to advance by a word to account for pre-fetch.
             *  Advance by a half word here, allowing the normal address
             *  advance to account for the other half word.
             */
            state->regData[15].v += 2;

            /* Display PC of next instruction */
            UnwPrintd2(" New PC=%x", state->regData[15].v + 2);

        }
        /* Format 19: 32-bit Thumb-2 instruction, 0b11101 block. */
        else if((instr & 0xf800) == 0xe800)
        {
            state->regData[15].v += 2;

            Int16 instr2;
            if (!state->cb->readH(state->regData[15].v & (~0x1), &instr2)) {
                return UNWIND_IREAD_H_FAIL;
            }

            /* Subformat 5: Load and Store Multiple */
            if ((instr & 0xfe40) == 0xe800) {
                Boolean U = (instr & 0x0080) ? TRUE : FALSE;
                Boolean V = (instr & 0x0100) ? TRUE : FALSE;
                Boolean W = (instr & 0x0020) ? TRUE : FALSE;
                Boolean L = (instr & 0x0010) ? TRUE : FALSE;
                Int16 baseReg = instr & 0x000f;
                Int16 regList = instr2;
                UnwResult result = UnwProcessBlockTransferInstruction(state,
                                                                      V,
                                                                      U,
                                                                      FALSE,
                                                                      W,
                                                                      L,
                                                                      baseReg,
                                                                      regList,
                                                                      FALSE);
                if (result != UNWIND_SUCCESS) {
                    return result;
                }
            } else {
                UnwPrintd2("%04x ???? (32-bit)", (unsigned)instr2);
                /* Unknown/undecoded.  May alter some register, so invalidate file */
                UnwInvalidateRegisterFile(state->regData);
            }
        }
        /* Format 20: 32-bit Thumb-2 instruction, 0b11101 block. */
        else if((instr & 0xf000) == 0xf000)
        {
            state->regData[15].v += 2;

            Int16 instr2;
            if (!state->cb->readH(state->regData[15].v & (~0x1), &instr2)) {
                return UNWIND_IREAD_H_FAIL;
            }

            /* Subformat 1: Data processing instructions: immediate, including bitfield and
             * saturate */
            if ((instr & 0xfa00) == 0xf000) {
                Int16 imm = (instr2 & 0xff) | ((instr2 >> 12) & 0x7) |
                    (((instr & 0x400) << 5) >> 3);
                Int8 rd = (instr2 >> 8) & 0xf;
                Int8 rn = instr & 0xf;
                if ((instr & 0x1e0) >> 5 == 0x2 && rn == 0xf) {
                    /* MOV */
                    UnwPrintd3("MOV r%d, #0x%x", rd, imm);
                    state->regData[rd].v = imm;
                    state->regData[rd].o = REG_VAL_FROM_CONST;
                } else if ((instr & 0x1e0) >> 5 == 0x8) {
                    /* ADD */
                    UnwPrintd4("ADD r%d, r%d, #0x%x", rd, rn, imm);
                    state->regData[rd].v = state->regData[rn].v + imm;
                    state->regData[rd].o = state->regData[rn].o;
                } else if ((instr & 0x1e0) >> 5 == 0xd) {
                    /* SUB */
                    UnwPrintd4("SUB r%d, r%d, #0x%x", rd, rn, imm);
                    state->regData[rd].v = state->regData[rn].v - imm;
                    state->regData[rd].o = state->regData[rn].o;
                } else {
                    /* Unknown/undecoded.  May alter some register, so invalidate file */
                    UnwPrintd2("%04x ???? (32-bit)", (unsigned)instr2);
                    UnwInvalidateRegisterFile(state->regData);
                }
            } else {
                /* Unknown/undecoded.  May alter some register, so invalidate file */
                UnwPrintd2("%04x ???? (32-bit)", (unsigned)instr2);
                UnwInvalidateRegisterFile(state->regData);
            }
        }
        else
        {
            UnwPrintd1("????");

            /* Unknown/undecoded.  May alter some register, so invalidate file */
            UnwInvalidateRegisterFile(state->regData);
        }

        UnwPrintd1("\n");

        /* Should never hit the reset vector */
        if(state->regData[15].v == 0) return UNWIND_RESET;

        /* Check next address */
        state->regData[15].v += 2;

        /* Garbage collect the memory hash (used only for the stack) */
        UnwMemHashGC(state);

        t--;
        if(t == 0) return UNWIND_EXHAUSTED;

    }
    while(!found);

    return UNWIND_SUCCESS;
}

#endif /* UPGRADE_ARM_STACK_UNWIND */

/* END OF FILE */

