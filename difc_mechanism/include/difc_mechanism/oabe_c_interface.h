#ifndef DIFC_MECHANISM_OABE_C_INTERFACE_H
#define DIFC_MECHANISM_OABE_C_INTERFACE_H

#include <openabe/openabe.h>

#ifdef __cplusplus
extern "C" {
#endif

void oabe_init();
void oabe_shutdown();

#ifdef __cplusplus
}
#endif

#endif // DIFC_MECHANISM_OABE_C_INTERFACE_H