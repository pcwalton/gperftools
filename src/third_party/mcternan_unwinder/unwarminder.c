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
 * File Description: Implementation of the interface into the ARM unwinder.
 **************************************************************************/

#define MODULE_NAME "UNWARMINDER"

#define UPGRADE_ARM_STACK_UNWIND

/***************************************************************************
 * Include Files
 **************************************************************************/

#if defined(UPGRADE_ARM_STACK_UNWIND)
#include <stdio.h>
#include <string.h>
#include "unwarminder.h"
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


/***************************************************************************
 * Global Functions
 **************************************************************************/

UnwResult UnwindStart(Int32                  spValue,
                      const UnwindCallbacks *cb,
                      void                  *data)
{
    Int32    retAddr;
    UnwState state;

#if !defined(SIM_CLIENT)
    asm("mov %0,pc" : "=r" (retAddr) : :);
    retAddr -= 8;
    retAddr = (retAddr | 1);
    asm("mov %0,sp" : "=r" (spValue) : :);
#else
    retAddr = 0x0000a894;
    spValue = 0x7ff7edf8;
#endif

    /* Initialise the unwinding state */
    UnwInitState(&state, cb, data, retAddr, spValue);

    /* Check the Thumb bit */
    if(retAddr & 0x1)
    {
        return UnwStartThumb(&state);
    }
    else
    {
        return UnwStartArm(&state);
    }
}

#endif /* UPGRADE_ARM_STACK_UNWIND */

/* END OF FILE */

