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
 * File Description: Utility functions and glue for ARM unwinding sub-modules.
 **************************************************************************/

#define MODULE_NAME "UNWARM"

#define UPGRADE_ARM_STACK_UNWIND

/***************************************************************************
 * Include Files
 **************************************************************************/

#if defined(UPGRADE_ARM_STACK_UNWIND)
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "unwarm.h"
#include "unwarmmem.h"

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


/***************************************************************************
 * Global Functions
 **************************************************************************/

#if defined(UNW_DEBUG)
/** Printf wrapper.
 * This is used such that alternative outputs for any output can be selected
 * by modification of this wrapper function.
 */
void UnwPrintf(const char *format, ...)
{
    va_list args;

    va_start( args, format );
    vprintf(format, args );
}
#endif

/** Invalidate all general purpose registers.
 */
void UnwInvalidateRegisterFile(RegData *regFile)
{
    /* Rust code uses r11 as the frame pointer, so don't invalidate that one. */
    Int8 t;
    for (t = 0; t < 13; t++) {
        if (t != 11)
            regFile[t].o = REG_VAL_INVALID;
    }
}


/** Initialise the data used for unwinding.
 */
void UnwInitState(UnwState * const       state,   /**< Pointer to structure to fill. */
                  const UnwindCallbacks *cb,      /**< Callbacks. */
                  void                  *rptData, /**< Data to pass to report function. */
                  Int32                  pcValue, /**< PC at which to start unwinding. */
                  Int32                  spValue) /**< SP at which to start unwinding. */
{
    UnwInvalidateRegisterFile(state->regData);

    /* Store the pointer to the callbacks */
    state->cb = cb;
    state->reportData = rptData;

    /* Setup the SP and PC */
    state->regData[13].v = spValue;
    state->regData[13].o = REG_VAL_FROM_CONST;
    state->regData[15].v = pcValue;
    state->regData[15].o = REG_VAL_FROM_CONST;

    UnwPrintd3("\nInitial: PC=0x%08x SP=0x%08x\n", pcValue, spValue);

    /* Invalidate all memory addresses */
    memset(state->memData.used, 0, sizeof(state->memData.used));
}


/** Call the report function to indicate some return address.
 * This returns the value of the report function, which if TRUE
 * indicates that unwinding may continue.
 */
Boolean UnwReportRetAddr(UnwState * const state, Int32 addr)
{
    /* Cast away const from reportData.
     *  The const is only to prevent the unw module modifying the data.
     */
    return state->cb->report((void *)state->reportData, addr);
}


/** Write some register to memory.
 * This will store some register and meta data onto the virtual stack.
 * The address for the write
 * \param state [in/out]  The unwinding state.
 * \param wAddr [in]  The address at which to write the data.
 * \param reg   [in]  The register to store.
 * \return TRUE if the write was successful, FALSE otherwise.
 */
Boolean UnwMemWriteRegister(UnwState * const      state,
                            const Int32           addr,
                            const RegData * const reg)
{
    return UnwMemHashWrite(&state->memData,
                           addr,
                           reg->v,
                           M_IsOriginValid(reg->o));
}

/** Read a register from memory.
 * This will read a register from memory, and setup the meta data.
 * If the register has been previously written to memory using
 * UnwMemWriteRegister, the local hash will be used to return the
 * value while respecting whether the data was valid or not.  If the
 * register was previously written and was invalid at that point,
 * REG_VAL_INVALID will be returned in *reg.
 * \param state [in]  The unwinding state.
 * \param addr  [in]  The address to read.
 * \param reg   [out] The result, containing the data value and the origin
 *                     which will be REG_VAL_FROM_MEMORY, or REG_VAL_INVALID.
 * \return TRUE if the address could be read and *reg has been filled in.
 *         FALSE is the data could not be read.
 */
Boolean UnwMemReadRegister(UnwState * const      state,
                           const Int32           addr,
                           RegData * const       reg)
{
    Boolean tracked;

    /* Check if the value can be found in the hash */
    if(UnwMemHashRead(&state->memData, addr, &reg->v, &tracked))
    {
        reg->o = tracked ? REG_VAL_FROM_MEMORY : REG_VAL_INVALID;
        return TRUE;
    }
    /* Not in the hash, so read from real memory */
    else if(state->cb->readW(addr, &reg->v))
    {
        reg->o = REG_VAL_FROM_MEMORY;
        return TRUE;
    }
    /* Not in the hash, and failed to read from memory */
    else
    {
        return FALSE;
    }
}

UnwResult UnwProcessBlockTransferInstruction(UnwState *state,
                                             Boolean P,
                                             Boolean U,
                                             Boolean S,
                                             Boolean W,
                                             Boolean L,
                                             Int16 baseReg,
                                             Int16 regList,
                                             Boolean fromARM) {
    Int32      addr      = state->regData[baseReg].v;
    Boolean    addrValid = M_IsOriginValid(state->regData[baseReg].o);
    SignedInt8 r;

#if defined(UNW_DEBUG)
    /* Display the instruction */
    if(L)
    {
        UnwPrintd6("LDM%c%c r%d%s, {reglist}%s\n",
                   P ? 'E' : 'F',
                   U ? 'D' : 'A',
                   baseReg,
                   W ? "!" : "",
                   S ? "^" : "");
    }
    else
    {
        UnwPrintd6("STM%c%c r%d%s, {reglist}%s\n",
                   !P ? 'E' : 'F',
                   !U ? 'D' : 'A',
                   baseReg,
                   W ? "!" : "",
                   S ? "^" : "");
    }
#endif
    /* S indicates that banked registers (untracked) are used, unless
     *  this is a load including the PC when the S-bit indicates that
     *  that CPSR is loaded from SPSR (also untracked, but ignored).
     */
    if(S && (!L || (regList & (0x01 << 15)) == 0))
    {
        UnwPrintd1("\nError:S-bit set requiring banked registers\n");
        return UNWIND_FAILURE;
    }
    else if(baseReg == 15)
    {
        UnwPrintd1("\nError: r15 used as base register\n");
        return UNWIND_FAILURE;
    }
    else if(regList == 0)
    {
        UnwPrintd1("\nError: Register list empty\n");
        return UNWIND_FAILURE;
    }

    /* Check if ascending or descending.
     *  Registers are loaded/stored in order of address.
     *  i.e. r0 is at the lowest address, r15 at the highest.
     */
    r = U ? 0 : 15;

    do
    {
        /* Check if the register is to be transferred */
        if(regList & (0x01 << r))
        {
            if(P) addr += U ? 4 : -4;

            if(L)
            {
                if(addrValid)
                {
                    if(!UnwMemReadRegister(state, addr, &state->regData[r]))
                    {
                        return UNWIND_DREAD_W_FAIL;
                    }

                    /* Update the origin if read via the stack pointer */
                    if(M_IsOriginValid(state->regData[r].o) && baseReg == 13)
                    {
                        state->regData[r].o = REG_VAL_FROM_STACK;
                    }

                    UnwPrintd5(" R%d = 0x%08x\t; r%d %s\n",
                               r,
                               state->regData[r].v,
                               r,
                               M_Origin2Str(state->regData[r].o));
                }
                else
                {
                    /* Invalidate the register as the base reg was invalid */
                    state->regData[r].o = REG_VAL_INVALID;

                    UnwPrintd2(" R%d = ???\n", r);
                }
            }
            else
            {
                if(addrValid)
                {
                    if(!UnwMemWriteRegister(state, state->regData[13].v, &state->regData[r]))
                    {
                        return UNWIND_DWRITE_W_FAIL;
                    }
                }

                UnwPrintd4(" R%d = 0x%08x (%s)\n",
                           r,
                           state->regData[r].v,
                           addrValid ? "valid" : "invalid");
            }

            if(!P) addr += U ? 4 : -4;
        }

        /* Check the next register */
        r += U ? 1 : -1;
    }
    while(r >= 0 && r <= 15);

    int i;
    for (i = 0; i < 16; i += 4) {
        RegData reg = { 0 };
        UnwMemReadRegister(state, addr + i, &reg);
        UnwPrintd3(" stack+%x = 0x%08x\n", i, reg.v);
    }

    /* Check the writeback bit */
    if(W) state->regData[baseReg].v = addr;

    /* Check if the PC was loaded */
    if(L && (regList & (0x01 << 15)))
    {
        if(!M_IsOriginValid(state->regData[15].o))
        {
            /* Return address is not valid */
            UnwPrintd1("PC popped with invalid address\n");
            return UNWIND_FAILURE;
        }
        else
        {
            /* Store the return address */
            if(!UnwReportRetAddr(state, state->regData[15].v))
            {
                return UNWIND_TRUNCATED;
            }

            UnwPrintd2("  Return PC=0x%x\n", state->regData[15].v);

            /* Determine the return mode */
            if (fromARM) {
                if(state->regData[15].v & 0x1)
                {
                    /* Branching to THUMB */
                    return UnwStartThumb(state);
                }
                else
                {
                    /* Branch to ARM */

                    /* Account for the auto-increment which isn't needed */
                    state->regData[15].v -= 4;
                }
            } else {
                if(state->regData[15].v & 0x1)
                {
                    /* Branching to THUMB. Account for the auto-increment which isn't needed. */
                    state->regData[15].v -= 2;
                }
                else
                {
                    /* Branch to ARM */
                    return UnwStartArm(state);
                }
            }
        }
    }

    return UNWIND_SUCCESS;
}

#endif /* UPGRADE_ARM_STACK_UNWIND */

/* END OF FILE */
