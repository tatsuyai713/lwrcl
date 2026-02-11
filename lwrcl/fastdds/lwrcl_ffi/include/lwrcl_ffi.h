#ifndef LWRCL_FFI_H_
#define LWRCL_FFI_H_

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32)
#if defined(LWRCL_FFI_EXPORTS)
#define LWRCL_FFI_API __declspec(dllexport)
#else
#define LWRCL_FFI_API __declspec(dllimport)
#endif
#else
#define LWRCL_FFI_API
#endif

typedef struct lwrcl_node_t lwrcl_node_t;

LWRCL_FFI_API void lwrcl_ffi_init(void);
LWRCL_FFI_API void lwrcl_ffi_init_args(int argc, const char **argv);
LWRCL_FFI_API void lwrcl_ffi_shutdown(void);
LWRCL_FFI_API int lwrcl_ffi_ok(void);

LWRCL_FFI_API lwrcl_node_t *lwrcl_ffi_node_create(const char *name, const char *ns);
LWRCL_FFI_API void lwrcl_ffi_node_destroy(lwrcl_node_t *node);
LWRCL_FFI_API void lwrcl_ffi_spin_some(lwrcl_node_t *node);

LWRCL_FFI_API void lwrcl_ffi_bytes_free(uint8_t *data);
LWRCL_FFI_API const char *lwrcl_ffi_last_error(void);

#include "lwrcl_ffi_generated.h"

#ifdef __cplusplus
}
#endif

#endif // LWRCL_FFI_H_
