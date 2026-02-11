#include "lwrcl_ffi.h"

#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <string>

#include "lwrcl.hpp"
#include "lwrcl_ffi_internal.h"

namespace {
thread_local std::string g_last_error;

}  // namespace

namespace lwrcl_ffi_internal {
void clear_last_error() {
  g_last_error.clear();
}

void set_last_error(const char *message) {
  if (message == nullptr) {
    g_last_error = "unknown error";
    return;
  }
  g_last_error = message;
}

void set_last_error(const std::string &message) {
  g_last_error = message;
}
}  // namespace lwrcl_ffi_internal

using lwrcl_ffi_internal::clear_last_error;
using lwrcl_ffi_internal::set_last_error;

extern "C" {

void lwrcl_ffi_init(void) {
  clear_last_error();
  try {
    lwrcl::init(0, nullptr);
  } catch (const std::exception &e) {
    set_last_error(e.what());
  } catch (...) {
    set_last_error("lwrcl_ffi_init: unknown error");
  }
}

void lwrcl_ffi_init_args(int argc, const char **argv) {
  clear_last_error();
  try {
    lwrcl::init(argc, const_cast<char **>(argv));
  } catch (const std::exception &e) {
    set_last_error(e.what());
  } catch (...) {
    set_last_error("lwrcl_ffi_init_args: unknown error");
  }
}

void lwrcl_ffi_shutdown(void) {
  clear_last_error();
  try {
    lwrcl::shutdown();
  } catch (const std::exception &e) {
    set_last_error(e.what());
  } catch (...) {
    set_last_error("lwrcl_ffi_shutdown: unknown error");
  }
}

int lwrcl_ffi_ok(void) {
  clear_last_error();
  try {
    return lwrcl::ok() ? 1 : 0;
  } catch (const std::exception &e) {
    set_last_error(e.what());
  } catch (...) {
    set_last_error("lwrcl_ffi_ok: unknown error");
  }
  return 0;
}

lwrcl_node_t *lwrcl_ffi_node_create(const char *name, const char *ns) {
  clear_last_error();
  try {
    if (name == nullptr || std::strlen(name) == 0) {
      set_last_error("lwrcl_ffi_node_create: name is required");
      return nullptr;
    }
    auto handle = std::make_unique<lwrcl_node_t>();
    if (ns == nullptr || std::strlen(ns) == 0) {
      handle->node = lwrcl::Node::make_shared(std::string(name));
    } else {
      handle->node = lwrcl::Node::make_shared(std::string(name), std::string(ns));
    }
    return handle.release();
  } catch (const std::exception &e) {
    set_last_error(e.what());
  } catch (...) {
    set_last_error("lwrcl_ffi_node_create: unknown error");
  }
  return nullptr;
}

void lwrcl_ffi_node_destroy(lwrcl_node_t *node) {
  clear_last_error();
  try {
    delete node;
  } catch (const std::exception &e) {
    set_last_error(e.what());
  } catch (...) {
    set_last_error("lwrcl_ffi_node_destroy: unknown error");
  }
}

void lwrcl_ffi_spin_some(lwrcl_node_t *node) {
  clear_last_error();
  try {
    if (node == nullptr || !node->node) {
      set_last_error("lwrcl_ffi_spin_some: node is null");
      return;
    }
    lwrcl::spin_some(node->node);
  } catch (const std::exception &e) {
    set_last_error(e.what());
  } catch (...) {
    set_last_error("lwrcl_ffi_spin_some: unknown error");
  }
}

void lwrcl_ffi_bytes_free(uint8_t *data) {
  clear_last_error();
  try {
    std::free(data);
  } catch (const std::exception &e) {
    set_last_error(e.what());
  } catch (...) {
    set_last_error("lwrcl_ffi_bytes_free: unknown error");
  }
}

const char *lwrcl_ffi_last_error(void) {
  return g_last_error.empty() ? nullptr : g_last_error.c_str();
}

}  // extern "C"
