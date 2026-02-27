/**
 * @file dlopen_global.c
 * @brief MEX function to load a shared library with RTLD_GLOBAL flag
 *
 * MATLAB's loadlibrary() uses RTLD_NOW|RTLD_LOCAL, which means symbols from
 * the loaded library are not visible to subsequently loaded libraries.
 * This MEX function uses dlopen with RTLD_GLOBAL|RTLD_LAZY to make symbols
 * globally available, which is needed for preloading stub libraries.
 *
 * Usage: dlopen_global('/path/to/library.so')
 * Compile: mex dlopen_global.c -ldl
 *
 * Copyright (c) 2026 Guangdong Huayan Robotics Co., Ltd.
 */
#include "mex.h"
#include <dlfcn.h>
#include <string.h>
#include <stdint.h>

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
    if (nrhs < 1 || !mxIsChar(prhs[0])) {
        mexErrMsgIdAndTxt("dlopen_global:input", "Usage: dlopen_global('path/to/lib.so')");
    }
    
    char *path = mxArrayToString(prhs[0]);
    if (!path) {
        mexErrMsgIdAndTxt("dlopen_global:input", "Failed to read path string");
    }
    
    void *handle = dlopen(path, RTLD_GLOBAL | RTLD_LAZY);
    if (!handle) {
        const char *err = dlerror();
        mxFree(path);
        mexErrMsgIdAndTxt("dlopen_global:dlopen", "dlopen failed: %s", err ? err : "unknown error");
    }
    
    mexPrintf("  dlopen_global: loaded %s (RTLD_GLOBAL)\n", path);
    mxFree(path);
    
    /* Return handle as uint64 for potential future dlclose */
    if (nlhs > 0) {
        plhs[0] = mxCreateNumericMatrix(1, 1, mxUINT64_CLASS, mxREAL);
        *((uint64_t*)mxGetData(plhs[0])) = (uint64_t)handle;
    }
}
