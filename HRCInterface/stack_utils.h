// stack_utils.h
#ifndef STACK_UTILS_H
#define STACK_UTILS_H

#include <stdint.h>

// 获取当前栈指针（x86_64架构，GCC编译器）
static inline uintptr_t get_current_stack_ptr() {
    uintptr_t sp;
    __asm__ __volatile__("mov %%rsp, %0" : "=r"(sp));
    return sp;
}

#endif // STACK_UTILS_H