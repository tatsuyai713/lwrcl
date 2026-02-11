#ifndef LWRCL_FFI_INTERNAL_H_
#define LWRCL_FFI_INTERNAL_H_

#include <memory>
#include <string>

namespace lwrcl {
class Node;
}

struct lwrcl_node_t {
  std::shared_ptr<lwrcl::Node> node;
};

namespace lwrcl_ffi_internal {
void clear_last_error();
void set_last_error(const char *message);
void set_last_error(const std::string &message);
}  // namespace lwrcl_ffi_internal

#endif  // LWRCL_FFI_INTERNAL_H_
