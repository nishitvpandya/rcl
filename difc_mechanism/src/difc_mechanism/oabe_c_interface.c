#include "difc_mechanism/oabe_c_interface.h"

#include <openabe/openabe.h>

#ifdef __cplusplus
extern "C" {
#endif

void oabe_init() {
    oabe::InitializeOpenABE();
}

void oabe_shutdown() {
    oabe::ShutdownOpenABE();
}

#ifdef __cplusplus
}
#endif