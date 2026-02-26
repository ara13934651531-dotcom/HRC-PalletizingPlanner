/**
 * @file s50_tcp_stubs.c
 * @brief Stub implementations for undefined symbols in libHRCInterface.so
 *
 * libHRCInterface.so declares but never implements:
 *   - void initTCPPosition(const EcRealVector& p)
 *   - void updateTCPPosition(const EcRealVector& p, EcRealVector& vel)
 *
 * These are never called at runtime (optional TCP position tracking)
 * but their absence causes dlopen with RTLD_NOW to fail.
 * Compile: gcc -shared -fPIC -o s50_tcp_stubs.so s50_tcp_stubs.c
 */
void initTCPPosition(void* p) {
    /* stub — never called, only needed to resolve linker symbol */
}

void updateTCPPosition(void* p, void* vel) {
    /* stub — never called, only needed to resolve linker symbol */
}
