// -*- Mode: C++; c-basic-offset: 2; indent-tabs-mode: nil -*-
// Copyright (c) 2015 Mozilla Foundation
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//     * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//     * Neither the name of the Mozilla Foundation nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef BASE_STACKTRACE_ARM_MCTERNAN_INL_H_
#define BASE_STACKTRACE_ARM_MCTERNAN_INL_H_

#define UPGRADE_ARM_STACK_UNWIND

// #define UNW_DEBUG

#include "third_party/mcternan_unwinder/unwarm.h"

#include <dlfcn.h>
#include <fcntl.h>
#include <gelf.h>
#include <gperftools/stacktrace.h>
#include <libelf.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define MAX_READABLE_SEGMENTS   256

struct readable_segment {
    uintptr_t start_addr;
    uintptr_t end_addr;
};

struct stack_trace_context {
    int skip_count;
    int max_depth;
    void **result;
    int result_length;
};

__thread readable_segment readable_segments[MAX_READABLE_SEGMENTS];
__thread unsigned readable_segments_count = 0;

static void populate_readable_segments_if_necessary() {
    if (readable_segments_count > 0)
        return;

    FILE *f = fopen("/proc/self/maps", "r");
    while (1) {
        char line[1024];
        if (fgets(line, sizeof(line), f) == NULL)
            break;
        uint32_t start_addr, end_addr;
        char read_perm = '-';
        sscanf(line, "%08x-%08x %c", &start_addr, &end_addr, &read_perm);
        if (read_perm != 'r')
            continue;
        readable_segments[readable_segments_count].start_addr = start_addr;
        readable_segments[readable_segments_count].end_addr = end_addr;
        readable_segments_count++;
    }
}

static int segment_is_in_range(const void *key_ptr, const void *readable_segment_ptr) {
    struct readable_segment *key = (struct readable_segment *)key_ptr;
    struct readable_segment *readable_segment = (struct readable_segment *)readable_segment_ptr;
    if (key->end_addr <= readable_segment->start_addr)
        return -1;
    if (key->start_addr >= readable_segment->end_addr)
        return 1;
    return 0;
}

static bool can_read_range(uintptr_t start, uintptr_t end) {
    readable_segment key;
    key.start_addr = start;
    key.end_addr = end;
    return bsearch(&key,
                   readable_segments,
                   readable_segments_count,
                   sizeof(readable_segments[0]),
                   segment_is_in_range) != NULL;
}

static Boolean arm_report_callback(void *user_data, Int32 address) {
    stack_trace_context *context = (stack_trace_context *)user_data;

    if (context->skip_count > 0) {
        context->skip_count--;
        return TRUE;
    }

    if (context->result_length == context->max_depth)
        return TRUE;

    context->result[context->result_length] = (void *)address;
    context->result_length++;
    return TRUE;
}

static Boolean arm_read32_callback(Int32 address, Int32 *val) {
    if (!can_read_range(address, address + 4))
        return FALSE;
    *val = *(int32_t *)address;
    return TRUE;
}

static Boolean arm_read16_callback(Int32 address, Int16 *val) {
    if (!can_read_range(address, address + 2))
        return FALSE;
    *val = *(int16_t *)address;
    return TRUE;
}

static Boolean arm_read8_callback(Int32 address, Int8 *val) {
    if (!can_read_range(address, address + 1))
        return FALSE;
    *val = *(int8_t *)address;
    return TRUE;
}

#ifdef UNW_DEBUG
static int arm_printf_callback(const char *format, ...) {
    va_list args;
    va_start(args, format);
    int result = vfprintf(stderr, format, args);
    va_end(args);
    return result;
}
#endif

const UnwindCallbacks arm_callbacks = {
    arm_report_callback,
    arm_read32_callback,
    arm_read16_callback,
    arm_read8_callback,
#ifdef UNW_DEBUG
    arm_printf_callback
#endif
};

#endif  // BASE_STACKTRACE_ARM_MCTERNAN_INL_H_

// Note: this part of the file is included several times.
// Do not put globals below.

// The following 4 functions are generated from the code below:
//   GetStack{Trace,Frames}()
//   GetStack{Trace,Frames}WithContext()
//
// These functions take the following args:
//   void** result: the stack-trace, as an array
//   int* sizes: the size of each stack frame, as an array
//               (GetStackFrames* only)
//   int max_depth: the size of the result (and sizes) array(s)
//   int skip_count: how many stack pointers to skip before storing in result
//   void* ucp: a ucontext_t* (GetStack{Trace,Frames}WithContext only)
static int GET_STACK_TRACE_OR_FRAMES {
    populate_readable_segments_if_necessary();

    stack_trace_context context;
    context.skip_count = skip_count;
    context.max_depth = max_depth;
    context.result = result;
    context.result_length = 0;

    UnwindStart(0, &arm_callbacks, (void *)&context);
    return context.result_length;
}

