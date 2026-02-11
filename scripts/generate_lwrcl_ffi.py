#!/usr/bin/env python3

import argparse
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def camel_to_snake_weird(name: str) -> str:
    name = re.sub(r"([A-Z]+)([A-Z][a-z])", r"\1_\2", name)
    # Match update_headers.sh behavior: it used [a-z\d] in sed, which is [a-zd] there.
    name = re.sub(r"([a-zd])([A-Z])", r"\1_\2", name)
    return name.lower()


def snake_to_pascal(name: str) -> str:
    return "".join(part.capitalize() for part in name.split("_"))


PRIMITIVE_ALIASES: Dict[str, str] = {
    "bool": "boolean",
    "boolean": "boolean",
    "octet": "octet",
    "char": "char",
    "int8": "int8",
    "uint8": "uint8",
    "int16": "int16",
    "uint16": "uint16",
    "int32": "int32",
    "uint32": "uint32",
    "int64": "int64",
    "uint64": "uint64",
    "float": "float",
    "float32": "float",
    "double": "double",
    "float64": "double",
    "long": "int32",
    "unsigned long": "uint32",
    "long long": "int64",
    "unsigned long long": "uint64",
}

C_FFI_TYPES: Dict[str, str] = {
    "boolean": "uint8_t",
    "octet": "uint8_t",
    "char": "int8_t",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "float": "float",
    "double": "double",
}

DART_FFI_TYPES: Dict[str, str] = {
    "boolean": "Uint8",
    "octet": "Uint8",
    "char": "Int8",
    "int8": "Int8",
    "uint8": "Uint8",
    "int16": "Int16",
    "uint16": "Uint16",
    "int32": "Int32",
    "uint32": "Uint32",
    "int64": "Int64",
    "uint64": "Uint64",
    "float": "Float",
    "double": "Double",
}

DART_TYPES: Dict[str, str] = {
    "boolean": "bool",
    "octet": "int",
    "char": "int",
    "int8": "int",
    "uint8": "int",
    "int16": "int",
    "uint16": "int",
    "int32": "int",
    "uint32": "int",
    "int64": "int",
    "uint64": "int",
    "float": "double",
    "double": "double",
}


@dataclass
class FieldDef:
    name: str
    base_type: str
    is_sequence: bool
    is_array: bool
    array_len: Optional[int]
    seq_bound: Optional[int]
    is_string: bool
    is_primitive: bool
    is_nested: bool
    nested_pkg: Optional[str]
    nested_name: Optional[str]


@dataclass
class MessageDef:
    pkg: str
    name: str
    fields: List[FieldDef]


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//.*", "", text)
    return text


def parse_type(type_str: str) -> Tuple[bool, bool, bool, Optional[str], Optional[str], str]:
    type_str = type_str.strip()

    if type_str == "string":
        return True, False, False, None, None, "string"

    # Check longer forms first to avoid partial matches
    for alias_key in sorted(PRIMITIVE_ALIASES.keys(), key=len, reverse=True):
        if type_str == alias_key:
            normalized = PRIMITIVE_ALIASES[alias_key]
            return False, True, False, None, None, normalized

    if "::" in type_str:
        parts = type_str.split("::")
        if len(parts) >= 3 and parts[-2] == "msg":
            return False, False, True, parts[-3], parts[-1], type_str
        return False, False, True, parts[0], parts[-1], type_str

    return False, False, True, None, type_str, type_str


def parse_idl_message(path: Path) -> Optional[MessageDef]:
    text = strip_comments(path.read_text(encoding="utf-8"))
    match = re.search(r"struct\s+(\w+)\s*\{(.*?)\};", text, flags=re.S)
    if not match:
        return None
    name = match.group(1)
    body = match.group(2)
    fields: List[FieldDef] = []
    for raw in body.split(";"):
        line = raw.strip()
        if not line:
            continue
        if line.startswith("const "):
            continue
        seq_match = re.match(r"sequence\s*<\s*(.+?)(?:\s*,\s*(\d+))?\s*>\s+(\w+)$", line)
        if seq_match:
            type_str = seq_match.group(1).strip()
            seq_bound = seq_match.group(2)
            name_str = seq_match.group(3)
            is_string, is_primitive, is_nested, nested_pkg, nested_name, base_type = parse_type(type_str)
            fields.append(
                FieldDef(
                    name=name_str,
                    base_type=base_type,
                    is_sequence=True,
                    is_array=False,
                    array_len=None,
                    seq_bound=int(seq_bound) if seq_bound else None,
                    is_string=is_string,
                    is_primitive=is_primitive,
                    is_nested=is_nested,
                    nested_pkg=nested_pkg,
                    nested_name=nested_name,
                )
            )
            continue
        array_match = re.match(r"(.+?)\s+(\w+)\s*\[(\d+)\]$", line)
        if array_match:
            type_str = array_match.group(1).strip()
            name_str = array_match.group(2)
            length = int(array_match.group(3))
            is_string, is_primitive, is_nested, nested_pkg, nested_name, base_type = parse_type(type_str)
            fields.append(
                FieldDef(
                    name=name_str,
                    base_type=base_type,
                    is_sequence=False,
                    is_array=True,
                    array_len=length,
                    seq_bound=None,
                    is_string=is_string,
                    is_primitive=is_primitive,
                    is_nested=is_nested,
                    nested_pkg=nested_pkg,
                    nested_name=nested_name,
                )
            )
            continue
        scalar_match = re.match(r"(.+?)\s+(\w+)$", line)
        if scalar_match:
            type_str = scalar_match.group(1).strip()
            name_str = scalar_match.group(2)
            is_string, is_primitive, is_nested, nested_pkg, nested_name, base_type = parse_type(type_str)
            fields.append(
                FieldDef(
                    name=name_str,
                    base_type=base_type,
                    is_sequence=False,
                    is_array=False,
                    array_len=None,
                    seq_bound=None,
                    is_string=is_string,
                    is_primitive=is_primitive,
                    is_nested=is_nested,
                    nested_pkg=nested_pkg,
                    nested_name=nested_name,
                )
            )
            continue
    pkg = path.parent.parent.name
    for field in fields:
        if field.is_nested and field.nested_pkg is None:
            field.nested_pkg = pkg
    return MessageDef(pkg=pkg, name=name, fields=fields)


def find_messages(idl_root: Path) -> List[MessageDef]:
    messages: List[MessageDef] = []
    for path in sorted(idl_root.rglob("msg/*.idl")):
        if not path.is_file():
            continue
        msg = parse_idl_message(path)
        if msg:
            messages.append(msg)
    return messages


def dart_class_name(pkg: str, name: str) -> str:
    return f"{snake_to_pascal(pkg)}{name}"


def generate_header(messages: List[MessageDef]) -> str:
    lines: List[str] = []
    lines.append("#ifndef LWRCL_FFI_GENERATED_H_")
    lines.append("#define LWRCL_FFI_GENERATED_H_")
    lines.append("")
    lines.append("// This file is generated. Do not edit.")
    lines.append("")
    lines.append("#include <stddef.h>")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append("#ifndef LWRCL_FFI_API")
    lines.append("#define LWRCL_FFI_API")
    lines.append("#endif")
    lines.append("")
    lines.append("#ifdef __cplusplus")
    lines.append("extern \"C\" {")
    lines.append("#endif")
    lines.append("")

    for msg in messages:
        suffix = f"{msg.pkg}__msg__{msg.name}"
        lines.append(f"typedef struct lwrcl_msg__{suffix}_t lwrcl_msg__{suffix}_t;")
        lines.append(f"typedef struct lwrcl_publisher__{suffix}_t lwrcl_publisher__{suffix}_t;")
        lines.append(f"typedef struct lwrcl_subscription__{suffix}_t lwrcl_subscription__{suffix}_t;")
        lines.append("")
        lines.append(f"LWRCL_FFI_API lwrcl_msg__{suffix}_t *lwrcl_ffi_msg_create__{suffix}(void);")
        lines.append(f"LWRCL_FFI_API void lwrcl_ffi_msg_destroy__{suffix}(lwrcl_msg__{suffix}_t *msg);")
        lines.append("")
        lines.append(f"LWRCL_FFI_API lwrcl_publisher__{suffix}_t *lwrcl_ffi_publisher_create__{suffix}(lwrcl_node_t *node, const char *topic, uint16_t depth);")
        lines.append(f"LWRCL_FFI_API void lwrcl_ffi_publisher_destroy__{suffix}(lwrcl_publisher__{suffix}_t *publisher);")
        lines.append(f"LWRCL_FFI_API int lwrcl_ffi_publisher_publish__{suffix}(lwrcl_publisher__{suffix}_t *publisher, const lwrcl_msg__{suffix}_t *msg);")
        lines.append("")
        lines.append(f"LWRCL_FFI_API lwrcl_subscription__{suffix}_t *lwrcl_ffi_subscription_create__{suffix}(lwrcl_node_t *node, const char *topic, uint16_t depth);")
        lines.append(f"LWRCL_FFI_API void lwrcl_ffi_subscription_destroy__{suffix}(lwrcl_subscription__{suffix}_t *subscription);")
        lines.append(f"LWRCL_FFI_API int lwrcl_ffi_subscription_has_message__{suffix}(lwrcl_subscription__{suffix}_t *subscription);")
        lines.append(f"LWRCL_FFI_API int lwrcl_ffi_subscription_take__{suffix}(lwrcl_subscription__{suffix}_t *subscription, lwrcl_msg__{suffix}_t *out_msg);")
        lines.append("")

        for field in msg.fields:
            field_suffix = f"{suffix}__{field.name}"
            if field.is_sequence or field.is_array:
                lines.append(f"LWRCL_FFI_API size_t lwrcl_ffi_msg_get__{field_suffix}_size(lwrcl_msg__{suffix}_t *msg);")
                if field.is_sequence:
                    lines.append(f"LWRCL_FFI_API void lwrcl_ffi_msg_resize__{field_suffix}(lwrcl_msg__{suffix}_t *msg, size_t size);")
                if field.is_string:
                    lines.append(f"LWRCL_FFI_API int lwrcl_ffi_msg_get__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index, char **out_data, size_t *out_length);")
                    lines.append(f"LWRCL_FFI_API void lwrcl_ffi_msg_set__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index, const char *data, size_t length);")
                elif field.is_primitive:
                    c_type = C_FFI_TYPES[field.base_type]
                    lines.append(f"LWRCL_FFI_API {c_type} lwrcl_ffi_msg_get__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index);")
                    lines.append(f"LWRCL_FFI_API void lwrcl_ffi_msg_set__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index, {c_type} value);")
                else:
                    nested_suffix = f"{field.nested_pkg}__msg__{field.nested_name}"
                    lines.append(f"LWRCL_FFI_API lwrcl_msg__{nested_suffix}_t *lwrcl_ffi_msg_get__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index);")
                    lines.append(f"LWRCL_FFI_API void lwrcl_ffi_msg_set__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index, const lwrcl_msg__{nested_suffix}_t *value);")
                lines.append("")
            else:
                if field.is_string:
                    lines.append(f"LWRCL_FFI_API int lwrcl_ffi_msg_get__{field_suffix}(lwrcl_msg__{suffix}_t *msg, char **out_data, size_t *out_length);")
                    lines.append(f"LWRCL_FFI_API void lwrcl_ffi_msg_set__{field_suffix}(lwrcl_msg__{suffix}_t *msg, const char *data, size_t length);")
                elif field.is_primitive:
                    c_type = C_FFI_TYPES[field.base_type]
                    lines.append(f"LWRCL_FFI_API {c_type} lwrcl_ffi_msg_get__{field_suffix}(lwrcl_msg__{suffix}_t *msg);")
                    lines.append(f"LWRCL_FFI_API void lwrcl_ffi_msg_set__{field_suffix}(lwrcl_msg__{suffix}_t *msg, {c_type} value);")
                else:
                    nested_suffix = f"{field.nested_pkg}__msg__{field.nested_name}"
                    lines.append(f"LWRCL_FFI_API lwrcl_msg__{nested_suffix}_t *lwrcl_ffi_msg_get__{field_suffix}(lwrcl_msg__{suffix}_t *msg);")
                    lines.append(f"LWRCL_FFI_API void lwrcl_ffi_msg_set__{field_suffix}(lwrcl_msg__{suffix}_t *msg, const lwrcl_msg__{nested_suffix}_t *value);")
                lines.append("")

    lines.append("#ifdef __cplusplus")
    lines.append("}")
    lines.append("#endif")
    lines.append("")
    lines.append("#endif  // LWRCL_FFI_GENERATED_H_")
    lines.append("")
    return "\n".join(lines)


def generate_cpp(messages: List[MessageDef]) -> str:
    lines: List[str] = []
    lines.append("// This file is generated. Do not edit.")
    lines.append("#include \"lwrcl_ffi.h\"")
    lines.append("#include \"lwrcl_ffi_internal.h\"")
    lines.append("")
    lines.append("#include <cstdint>")
    lines.append("#include <cstdlib>")
    lines.append("#include <cstring>")
    lines.append("#include <exception>")
    lines.append("#include <memory>")
    lines.append("#include <string>")
    lines.append("")
    lines.append("#include \"lwrcl.hpp\"")
    lines.append("")
    for msg in messages:
        snake = camel_to_snake_weird(msg.name)
        lines.append(f"#include \"{msg.pkg}/msg/{snake}.hpp\"")
    lines.append("")
    lines.append("using lwrcl_ffi_internal::clear_last_error;")
    lines.append("using lwrcl_ffi_internal::set_last_error;")
    lines.append("")
    lines.append("namespace {")
    lines.append("template <typename T, typename Handle>")
    lines.append("Handle *create_publisher_handle(lwrcl_node_t *node, const char *topic, uint16_t depth, const char *func_name)")
    lines.append("{")
    lines.append("  if (node == nullptr || !node->node) {")
    lines.append("    set_last_error(std::string(func_name) + \": node is null\");")
    lines.append("    return nullptr;")
    lines.append("  }")
    lines.append("  if (topic == nullptr || std::strlen(topic) == 0) {")
    lines.append("    set_last_error(std::string(func_name) + \": topic is required\");")
    lines.append("    return nullptr;")
    lines.append("  }")
    lines.append("  auto handle = std::make_unique<Handle>();")
    lines.append("  handle->publisher = node->node->create_publisher<T>(std::string(topic), depth);")
    lines.append("  return handle.release();")
    lines.append("}")
    lines.append("")
    lines.append("template <typename T, typename Handle>")
    lines.append("Handle *create_subscription_handle(lwrcl_node_t *node, const char *topic, uint16_t depth, const char *func_name)")
    lines.append("{")
    lines.append("  if (node == nullptr || !node->node) {")
    lines.append("    set_last_error(std::string(func_name) + \": node is null\");")
    lines.append("    return nullptr;")
    lines.append("  }")
    lines.append("  if (topic == nullptr || std::strlen(topic) == 0) {")
    lines.append("    set_last_error(std::string(func_name) + \": topic is required\");")
    lines.append("    return nullptr;")
    lines.append("  }")
    lines.append("  auto handle = std::make_unique<Handle>();")
    lines.append("  handle->subscription = node->node->create_subscription<T>(std::string(topic), depth, [](std::shared_ptr<T>) {});")
    lines.append("  return handle.release();")
    lines.append("}")
    lines.append("")
    lines.append("int copy_string_to_out(const std::string &value, char **out_data, size_t *out_length, const char *func_name)")
    lines.append("{")
    lines.append("  if (out_data == nullptr || out_length == nullptr) {")
    lines.append("    set_last_error(std::string(func_name) + \": output pointers are null\");")
    lines.append("    return -1;")
    lines.append("  }")
    lines.append("  if (value.empty()) {")
    lines.append("    *out_data = nullptr;")
    lines.append("    *out_length = 0;")
    lines.append("    return 1;")
    lines.append("  }")
    lines.append("  auto *buffer = static_cast<char *>(std::malloc(value.size()));")
    lines.append("  if (buffer == nullptr) {")
    lines.append("    set_last_error(std::string(func_name) + \": malloc failed\");")
    lines.append("    return -1;")
    lines.append("  }")
    lines.append("  std::memcpy(buffer, value.data(), value.size());")
    lines.append("  *out_data = buffer;")
    lines.append("  *out_length = value.size();")
    lines.append("  return 1;")
    lines.append("}")
    lines.append("")
    lines.append("template <typename Msg, typename Handle>")
    lines.append("Handle *create_message_handle()")
    lines.append("{")
    lines.append("  auto handle = std::make_unique<Handle>();")
    lines.append("  handle->msg = std::make_shared<Msg>();")
    lines.append("  return handle.release();")
    lines.append("}")
    lines.append("")
    lines.append("template <typename Msg, typename Handle>")
    lines.append("Handle *create_message_copy_handle(const Msg &value)")
    lines.append("{")
    lines.append("  auto handle = std::make_unique<Handle>();")
    lines.append("  handle->msg = std::make_shared<Msg>(value);")
    lines.append("  return handle.release();")
    lines.append("}")
    lines.append("")
    lines.append("template <typename Msg, typename Handle>")
    lines.append("void assign_message_from_handle(Msg &target, const Handle *value)")
    lines.append("{")
    lines.append("  if (value && value->msg) {")
    lines.append("    target = *(value->msg);")
    lines.append("  } else {")
    lines.append("    target = Msg();")
    lines.append("  }")
    lines.append("}")
    lines.append("}  // namespace")
    lines.append("")
    lines.append("extern \"C\" {")
    lines.append("")

    for msg in messages:
        suffix = f"{msg.pkg}__msg__{msg.name}"
        msg_type = f"{msg.pkg}::msg::{msg.name}"

        lines.append(f"struct lwrcl_msg__{suffix}_t {{")
        lines.append(f"  std::shared_ptr<{msg_type}> msg;")
        lines.append("};")
        lines.append("")
        lines.append(f"struct lwrcl_publisher__{suffix}_t {{")
        lines.append(f"  std::shared_ptr<lwrcl::Publisher<{msg_type}>> publisher;")
        lines.append("};")
        lines.append("")
        lines.append(f"struct lwrcl_subscription__{suffix}_t {{")
        lines.append(f"  std::shared_ptr<lwrcl::Subscription<{msg_type}>> subscription;")
        lines.append("};")
        lines.append("")

        lines.append(f"lwrcl_msg__{suffix}_t *lwrcl_ffi_msg_create__{suffix}(void)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append(f"    return create_message_handle<{msg_type}, lwrcl_msg__{suffix}_t>();")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_msg_create__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("  return nullptr;")
        lines.append("}")
        lines.append("")

        lines.append(f"void lwrcl_ffi_msg_destroy__{suffix}(lwrcl_msg__{suffix}_t *msg)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append("    delete msg;")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_msg_destroy__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("}")
        lines.append("")

        lines.append(f"lwrcl_publisher__{suffix}_t *lwrcl_ffi_publisher_create__{suffix}(lwrcl_node_t *node, const char *topic, uint16_t depth)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append(f"    return create_publisher_handle<{msg_type}, lwrcl_publisher__{suffix}_t>(node, topic, depth, \"lwrcl_ffi_publisher_create__{suffix}\");")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_publisher_create__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("  return nullptr;")
        lines.append("}")
        lines.append("")

        lines.append(f"void lwrcl_ffi_publisher_destroy__{suffix}(lwrcl_publisher__{suffix}_t *publisher)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append("    delete publisher;")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_publisher_destroy__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("}")
        lines.append("")

        lines.append(f"int lwrcl_ffi_publisher_publish__{suffix}(lwrcl_publisher__{suffix}_t *publisher, const lwrcl_msg__{suffix}_t *msg)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append("    if (publisher == nullptr || !publisher->publisher) {")
        lines.append(f"      set_last_error(\"lwrcl_ffi_publisher_publish__{suffix}: publisher is null\");")
        lines.append("      return -1;")
        lines.append("    }")
        lines.append("    if (msg == nullptr || !msg->msg) {")
        lines.append(f"      set_last_error(\"lwrcl_ffi_publisher_publish__{suffix}: message is null\");")
        lines.append("      return -1;")
        lines.append("    }")
        lines.append("    publisher->publisher->publish(*msg->msg);")
        lines.append("    return 0;")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_publisher_publish__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("  return -1;")
        lines.append("}")
        lines.append("")

        lines.append(f"lwrcl_subscription__{suffix}_t *lwrcl_ffi_subscription_create__{suffix}(lwrcl_node_t *node, const char *topic, uint16_t depth)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append(f"    return create_subscription_handle<{msg_type}, lwrcl_subscription__{suffix}_t>(node, topic, depth, \"lwrcl_ffi_subscription_create__{suffix}\");")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_subscription_create__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("  return nullptr;")
        lines.append("}")
        lines.append("")

        lines.append(f"void lwrcl_ffi_subscription_destroy__{suffix}(lwrcl_subscription__{suffix}_t *subscription)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append("    delete subscription;")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_subscription_destroy__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("}")
        lines.append("")

        lines.append(f"int lwrcl_ffi_subscription_has_message__{suffix}(lwrcl_subscription__{suffix}_t *subscription)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append("    if (subscription == nullptr || !subscription->subscription) {")
        lines.append(f"      set_last_error(\"lwrcl_ffi_subscription_has_message__{suffix}: subscription is null\");")
        lines.append("      return 0;")
        lines.append("    }")
        lines.append("    return subscription->subscription->has_message() ? 1 : 0;")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_subscription_has_message__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("  return 0;")
        lines.append("}")
        lines.append("")

        lines.append(f"int lwrcl_ffi_subscription_take__{suffix}(lwrcl_subscription__{suffix}_t *subscription, lwrcl_msg__{suffix}_t *out_msg)")
        lines.append("{")
        lines.append("  clear_last_error();")
        lines.append("  try {")
        lines.append("    if (subscription == nullptr || !subscription->subscription) {")
        lines.append(f"      set_last_error(\"lwrcl_ffi_subscription_take__{suffix}: subscription is null\");")
        lines.append("      return -1;")
        lines.append("    }")
        lines.append("    if (out_msg == nullptr || !out_msg->msg) {")
        lines.append(f"      set_last_error(\"lwrcl_ffi_subscription_take__{suffix}: out_msg is null\");")
        lines.append("      return -1;")
        lines.append("    }")
        lines.append(f"    std::shared_ptr<{msg_type}> msg;")
        lines.append("    if (!subscription->subscription->take(msg)) {")
        lines.append("      return 0;")
        lines.append("    }")
        lines.append("    *(out_msg->msg) = *msg;")
        lines.append("    return 1;")
        lines.append("  } catch (const std::exception &e) {")
        lines.append("    set_last_error(e.what());")
        lines.append("  } catch (...) {")
        lines.append(f"    set_last_error(\"lwrcl_ffi_subscription_take__{suffix}: unknown error\");")
        lines.append("  }")
        lines.append("  return -1;")
        lines.append("}")
        lines.append("")

        for field in msg.fields:
            field_suffix = f"{suffix}__{field.name}"
            field_access = f"msg->msg->{field.name}()"
            if field.is_sequence or field.is_array:
                lines.append(f"size_t lwrcl_ffi_msg_get__{field_suffix}_size(lwrcl_msg__{suffix}_t *msg)")
                lines.append("{")
                lines.append("  clear_last_error();")
                lines.append("  try {")
                lines.append("    if (msg == nullptr || !msg->msg) {")
                lines.append(f"      set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}_size: message is null\");")
                lines.append("      return 0;")
                lines.append("    }")
                lines.append(f"    return {field_access}.size();")
                lines.append("  } catch (const std::exception &e) {")
                lines.append("    set_last_error(e.what());")
                lines.append("  } catch (...) {")
                lines.append(f"    set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}_size: unknown error\");")
                lines.append("  }")
                lines.append("  return 0;")
                lines.append("}")
                lines.append("")

                if field.is_sequence:
                    lines.append(f"void lwrcl_ffi_msg_resize__{field_suffix}(lwrcl_msg__{suffix}_t *msg, size_t size)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_resize__{field_suffix}: message is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    lines.append(f"    {field_access}.resize(size);")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_resize__{field_suffix}: unknown error\");")
                    lines.append("  }")
                    lines.append("}")
                    lines.append("")

                if field.is_string:
                    lines.append(f"int lwrcl_ffi_msg_get__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index, char **out_data, size_t *out_length)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}_at: message is null\");")
                    lines.append("      return -1;")
                    lines.append("    }")
                    lines.append(f"    const auto &value = {field_access}.at(index);")
                    lines.append(f"    return copy_string_to_out(value, out_data, out_length, \"lwrcl_ffi_msg_get__{field_suffix}_at\");")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}_at: unknown error\");")
                    lines.append("  }")
                    lines.append("  return -1;")
                    lines.append("}")
                    lines.append("")

                    lines.append(f"void lwrcl_ffi_msg_set__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index, const char *data, size_t length)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}_at: message is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    lines.append("    if (data == nullptr) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}_at: data is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    lines.append(f"    {field_access}.at(index) = std::string(data, length);")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}_at: unknown error\");")
                    lines.append("  }")
                    lines.append("}")
                    lines.append("")

                elif field.is_primitive:
                    c_type = C_FFI_TYPES[field.base_type]
                    lines.append(f"{c_type} lwrcl_ffi_msg_get__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}_at: message is null\");")
                    lines.append("      return 0;")
                    lines.append("    }")
                    lines.append(f"    return {field_access}.at(index);")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}_at: unknown error\");")
                    lines.append("  }")
                    lines.append("  return 0;")
                    lines.append("}")
                    lines.append("")

                    lines.append(f"void lwrcl_ffi_msg_set__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index, {c_type} value)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}_at: message is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    if field.base_type == "boolean":
                        lines.append(f"    {field_access}.at(index) = (value != 0);")
                    else:
                        lines.append(f"    {field_access}.at(index) = value;")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}_at: unknown error\");")
                    lines.append("  }")
                    lines.append("}")
                    lines.append("")
                else:
                    nested_suffix = f"{field.nested_pkg}__msg__{field.nested_name}"
                    nested_type = f"{field.nested_pkg}::msg::{field.nested_name}"
                    lines.append(f"lwrcl_msg__{nested_suffix}_t *lwrcl_ffi_msg_get__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}_at: message is null\");")
                    lines.append("      return nullptr;")
                    lines.append("    }")
                    lines.append(f"    const auto &value = {field_access}.at(index);")
                    lines.append(f"    return create_message_copy_handle<{nested_type}, lwrcl_msg__{nested_suffix}_t>(value);")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}_at: unknown error\");")
                    lines.append("  }")
                    lines.append("  return nullptr;")
                    lines.append("}")
                    lines.append("")

                    lines.append(f"void lwrcl_ffi_msg_set__{field_suffix}_at(lwrcl_msg__{suffix}_t *msg, size_t index, const lwrcl_msg__{nested_suffix}_t *value)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}_at: message is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    lines.append(f"    assign_message_from_handle<{nested_type}, lwrcl_msg__{nested_suffix}_t>({field_access}.at(index), value);")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}_at: unknown error\");")
                    lines.append("  }")
                    lines.append("}")
                    lines.append("")
            else:
                if field.is_string:
                    lines.append(f"int lwrcl_ffi_msg_get__{field_suffix}(lwrcl_msg__{suffix}_t *msg, char **out_data, size_t *out_length)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}: message is null\");")
                    lines.append("      return -1;")
                    lines.append("    }")
                    lines.append(f"    const auto &value = {field_access};")
                    lines.append(f"    return copy_string_to_out(value, out_data, out_length, \"lwrcl_ffi_msg_get__{field_suffix}\");")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}: unknown error\");")
                    lines.append("  }")
                    lines.append("  return -1;")
                    lines.append("}")
                    lines.append("")

                    lines.append(f"void lwrcl_ffi_msg_set__{field_suffix}(lwrcl_msg__{suffix}_t *msg, const char *data, size_t length)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}: message is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    lines.append("    if (data == nullptr) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}: data is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    lines.append(f"    msg->msg->{field.name}(std::string(data, length));")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}: unknown error\");")
                    lines.append("  }")
                    lines.append("}")
                    lines.append("")
                elif field.is_primitive:
                    c_type = C_FFI_TYPES[field.base_type]
                    lines.append(f"{c_type} lwrcl_ffi_msg_get__{field_suffix}(lwrcl_msg__{suffix}_t *msg)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}: message is null\");")
                    lines.append("      return 0;")
                    lines.append("    }")
                    if field.base_type == "boolean":
                        lines.append(f"    return msg->msg->{field.name}() ? 1 : 0;")
                    else:
                        lines.append(f"    return msg->msg->{field.name}();")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}: unknown error\");")
                    lines.append("  }")
                    lines.append("  return 0;")
                    lines.append("}")
                    lines.append("")

                    lines.append(f"void lwrcl_ffi_msg_set__{field_suffix}(lwrcl_msg__{suffix}_t *msg, {c_type} value)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}: message is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    if field.base_type == "boolean":
                        lines.append(f"    msg->msg->{field.name}(value != 0);")
                    else:
                        lines.append(f"    msg->msg->{field.name}(value);")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}: unknown error\");")
                    lines.append("  }")
                    lines.append("}")
                    lines.append("")
                else:
                    nested_suffix = f"{field.nested_pkg}__msg__{field.nested_name}"
                    nested_type = f"{field.nested_pkg}::msg::{field.nested_name}"
                    lines.append(f"lwrcl_msg__{nested_suffix}_t *lwrcl_ffi_msg_get__{field_suffix}(lwrcl_msg__{suffix}_t *msg)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}: message is null\");")
                    lines.append("      return nullptr;")
                    lines.append("    }")
                    lines.append(f"    const auto &value = msg->msg->{field.name}();")
                    lines.append(f"    return create_message_copy_handle<{nested_type}, lwrcl_msg__{nested_suffix}_t>(value);")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_get__{field_suffix}: unknown error\");")
                    lines.append("  }")
                    lines.append("  return nullptr;")
                    lines.append("}")
                    lines.append("")

                    lines.append(f"void lwrcl_ffi_msg_set__{field_suffix}(lwrcl_msg__{suffix}_t *msg, const lwrcl_msg__{nested_suffix}_t *value)")
                    lines.append("{")
                    lines.append("  clear_last_error();")
                    lines.append("  try {")
                    lines.append("    if (msg == nullptr || !msg->msg) {")
                    lines.append(f"      set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}: message is null\");")
                    lines.append("      return;")
                    lines.append("    }")
                    lines.append(f"    assign_message_from_handle<{nested_type}, lwrcl_msg__{nested_suffix}_t>(msg->msg->{field.name}(), value);")
                    lines.append("  } catch (const std::exception &e) {")
                    lines.append("    set_last_error(e.what());")
                    lines.append("  } catch (...) {")
                    lines.append(f"    set_last_error(\"lwrcl_ffi_msg_set__{field_suffix}: unknown error\");")
                    lines.append("  }")
                    lines.append("}")
                    lines.append("")

    lines.append("}  // extern \"C\"")
    lines.append("")
    return "\n".join(lines)


def generate_dart(messages: List[MessageDef]) -> str:
    lines: List[str] = []
    lines.append("// This file is generated. Do not edit.")
    lines.append("import 'dart:convert';")
    lines.append("import 'dart:ffi';")
    lines.append("import 'dart:typed_data';")
    lines.append("")
    lines.append("import 'package:ffi/ffi.dart';")
    lines.append("import '../lwrcl.dart';")
    lines.append("")

    for msg in messages:
        suffix = f"{msg.pkg}__msg__{msg.name}"
        class_name = dart_class_name(msg.pkg, msg.name)
        lines.append(f"class {class_name} extends MessageBase {{")
        lines.append(f"  {class_name}() : super(_create(), _destroy);")
        lines.append(f"  {class_name}._fromHandle(Pointer<Void> handle) : super.fromHandle(handle, _destroy);")
        lines.append("")
        lines.append(f"  static final MessageType type = MessageType('{msg.pkg}', '{msg.name}');")
        lines.append("")
        lines.append(f"  static final Pointer<Void> Function() _create = bindings.lookupFunction<Pointer<Void> Function(), Pointer<Void> Function()>('lwrcl_ffi_msg_create__{suffix}');")
        lines.append(f"  static final void Function(Pointer<Void>) _destroy = bindings.lookupFunction<Void Function(Pointer<Void>), void Function(Pointer<Void>)>('lwrcl_ffi_msg_destroy__{suffix}');")

        for field in msg.fields:
            field_suffix = f"{suffix}__{field.name}"
            if field.is_sequence or field.is_array:
                lines.append("")
                lines.append(f"  static final int Function(Pointer<Void>) _{field.name}Size = bindings.lookupFunction<UintPtr Function(Pointer<Void>), int Function(Pointer<Void>)>('lwrcl_ffi_msg_get__{field_suffix}_size');")
                if field.is_sequence:
                    lines.append(f"  static final void Function(Pointer<Void>, int) _{field.name}Resize = bindings.lookupFunction<Void Function(Pointer<Void>, UintPtr), void Function(Pointer<Void>, int)>('lwrcl_ffi_msg_resize__{field_suffix}');")
                if field.is_string:
                    lines.append(f"  static final int Function(Pointer<Void>, int, Pointer<Pointer<Uint8>>, Pointer<IntPtr>) _{field.name}GetAt = bindings.lookupFunction<Int32 Function(Pointer<Void>, UintPtr, Pointer<Pointer<Uint8>>, Pointer<IntPtr>), int Function(Pointer<Void>, int, Pointer<Pointer<Uint8>>, Pointer<IntPtr>)>('lwrcl_ffi_msg_get__{field_suffix}_at');")
                    lines.append(f"  static final void Function(Pointer<Void>, int, Pointer<Uint8>, int) _{field.name}SetAt = bindings.lookupFunction<Void Function(Pointer<Void>, UintPtr, Pointer<Uint8>, IntPtr), void Function(Pointer<Void>, int, Pointer<Uint8>, int)>('lwrcl_ffi_msg_set__{field_suffix}_at');")
                elif field.is_primitive:
                    ffi_type = DART_FFI_TYPES[field.base_type]
                    dart_type = DART_TYPES[field.base_type]
                    lines.append(f"  static final {dart_type} Function(Pointer<Void>, int) _{field.name}GetAt = bindings.lookupFunction<{ffi_type} Function(Pointer<Void>, UintPtr), {dart_type} Function(Pointer<Void>, int)>('lwrcl_ffi_msg_get__{field_suffix}_at');")
                    lines.append(f"  static final void Function(Pointer<Void>, int, {dart_type}) _{field.name}SetAt = bindings.lookupFunction<Void Function(Pointer<Void>, UintPtr, {ffi_type}), void Function(Pointer<Void>, int, {dart_type})>('lwrcl_ffi_msg_set__{field_suffix}_at');")
                else:
                    lines.append(f"  static final Pointer<Void> Function(Pointer<Void>, int) _{field.name}GetAt = bindings.lookupFunction<Pointer<Void> Function(Pointer<Void>, UintPtr), Pointer<Void> Function(Pointer<Void>, int)>('lwrcl_ffi_msg_get__{field_suffix}_at');")
                    lines.append(f"  static final void Function(Pointer<Void>, int, Pointer<Void>) _{field.name}SetAt = bindings.lookupFunction<Void Function(Pointer<Void>, UintPtr, Pointer<Void>), void Function(Pointer<Void>, int, Pointer<Void>)>('lwrcl_ffi_msg_set__{field_suffix}_at');")

                dart_field_type = (
                    "String" if field.is_string else
                    DART_TYPES[field.base_type] if field.is_primitive else
                    dart_class_name(field.nested_pkg, field.nested_name)
                )
                lines.append("")
                lines.append(f"  List<{dart_field_type}> get {field.name} {{")
                lines.append(f"    final size = _{field.name}Size(handle);")
                lines.append(f"    throwIfLastError('get {field.name} size');")
                lines.append(f"    final list = <{dart_field_type}>[];")
                lines.append("    for (var i = 0; i < size; i++) {")
                if field.is_string:
                    lines.append("      final outData = calloc<Pointer<Uint8>>();")
                    lines.append("      final outLen = calloc<IntPtr>();")
                    lines.append("      try {")
                    lines.append(f"        final result = _{field.name}GetAt(handle, i, outData, outLen);")
                    lines.append("        if (result < 0) {")
                    lines.append(f"          throwIfLastError('get {field.name}');")
                    lines.append("        }")
                    lines.append("        final length = outLen.value;")
                    lines.append("        if (length == 0 || outData.value.address == 0) {")
                    lines.append("          list.add('');")
                    lines.append("        } else {")
                    lines.append("          final bytes = outData.value.asTypedList(length);")
                    lines.append("          list.add(utf8.decode(bytes, allowMalformed: true));")
                    lines.append("          bindings.bytesFree(outData.value);")
                    lines.append("        }")
                    lines.append("      } finally {")
                    lines.append("        calloc.free(outData);")
                    lines.append("        calloc.free(outLen);")
                    lines.append("      }")
                elif field.is_primitive:
                    if field.base_type == "boolean":
                        lines.append(f"      final value = _{field.name}GetAt(handle, i);")
                        lines.append(f"      throwIfLastError('get {field.name}');")
                        lines.append("      list.add(value != 0);")
                    else:
                        lines.append(f"      final value = _{field.name}GetAt(handle, i);")
                        lines.append(f"      throwIfLastError('get {field.name}');")
                        lines.append("      list.add(value);")
                else:
                    nested_class = dart_class_name(field.nested_pkg, field.nested_name)
                    lines.append(f"      final ptr = _{field.name}GetAt(handle, i);")
                    lines.append("      if (ptr.address == 0) {")
                    lines.append(f"        throwIfLastError('get {field.name}');")
                    lines.append("      }")
                    lines.append(f"      list.add({nested_class}._fromHandle(ptr));")
                lines.append("    }")
                lines.append("    return list;")
                lines.append("  }")
                lines.append("")
                lines.append(f"  set {field.name}(List<{dart_field_type}> value) {{")
                if field.is_sequence:
                    lines.append(f"    _{field.name}Resize(handle, value.length);")
                    lines.append(f"    throwIfLastError('resize {field.name}');")
                else:
                    lines.append(f"    if (value.length != {field.array_len}) {{")
                    lines.append(f"      throw LwrclException('expected {field.array_len} items for {field.name}');")
                    lines.append("    }")
                lines.append("    for (var i = 0; i < value.length; i++) {")
                if field.is_string:
                    lines.append("      final units = utf8.encode(value[i]);")
                    lines.append("      final buffer = malloc<Uint8>(units.length + 1);")
                    lines.append("      try {")
                    lines.append("        buffer.asTypedList(units.length).setAll(0, units);")
                    lines.append("        buffer[units.length] = 0;")
                    lines.append(f"        _{field.name}SetAt(handle, i, buffer, units.length);")
                    lines.append(f"        throwIfLastError('set {field.name}');")
                    lines.append("      } finally {")
                    lines.append("        malloc.free(buffer);")
                    lines.append("      }")
                elif field.is_primitive:
                    if field.base_type == "boolean":
                        lines.append(f"      _{field.name}SetAt(handle, i, value[i] ? 1 : 0);")
                    else:
                        lines.append(f"      _{field.name}SetAt(handle, i, value[i]);")
                    lines.append(f"      throwIfLastError('set {field.name}');")
                else:
                    lines.append(f"      _{field.name}SetAt(handle, i, value[i].handle);")
                    lines.append(f"      throwIfLastError('set {field.name}');")
                lines.append("    }")
                lines.append("  }")
            else:
                lines.append("")
                if field.is_string:
                    lines.append(f"  static final int Function(Pointer<Void>, Pointer<Pointer<Uint8>>, Pointer<IntPtr>) _{field.name}Get = bindings.lookupFunction<Int32 Function(Pointer<Void>, Pointer<Pointer<Uint8>>, Pointer<IntPtr>), int Function(Pointer<Void>, Pointer<Pointer<Uint8>>, Pointer<IntPtr>)>('lwrcl_ffi_msg_get__{field_suffix}');")
                    lines.append(f"  static final void Function(Pointer<Void>, Pointer<Uint8>, int) _{field.name}Set = bindings.lookupFunction<Void Function(Pointer<Void>, Pointer<Uint8>, IntPtr), void Function(Pointer<Void>, Pointer<Uint8>, int)>('lwrcl_ffi_msg_set__{field_suffix}');")
                    lines.append("")
                    lines.append(f"  String get {field.name} {{")
                    lines.append("    final outData = calloc<Pointer<Uint8>>();")
                    lines.append("    final outLen = calloc<IntPtr>();")
                    lines.append("    try {")
                    lines.append(f"      final result = _{field.name}Get(handle, outData, outLen);")
                    lines.append("      if (result < 0) {")
                    lines.append(f"        throwIfLastError('get {field.name}');")
                    lines.append("      }")
                    lines.append("      final length = outLen.value;")
                    lines.append("      if (length == 0 || outData.value.address == 0) {")
                    lines.append("        return '';")
                    lines.append("      }")
                    lines.append("      final bytes = outData.value.asTypedList(length);")
                    lines.append("      final text = utf8.decode(bytes, allowMalformed: true);")
                    lines.append("      bindings.bytesFree(outData.value);")
                    lines.append("      return text;")
                    lines.append("    } finally {")
                    lines.append("      calloc.free(outData);")
                    lines.append("      calloc.free(outLen);")
                    lines.append("    }")
                    lines.append("  }")
                    lines.append("")
                    lines.append(f"  set {field.name}(String value) {{")
                    lines.append("    final units = utf8.encode(value);")
                    lines.append("    final buffer = malloc<Uint8>(units.length + 1);")
                    lines.append("    try {")
                    lines.append("      buffer.asTypedList(units.length).setAll(0, units);")
                    lines.append("      buffer[units.length] = 0;")
                    lines.append(f"      _{field.name}Set(handle, buffer, units.length);")
                    lines.append(f"      throwIfLastError('set {field.name}');")
                    lines.append("    } finally {")
                    lines.append("      malloc.free(buffer);")
                    lines.append("    }")
                    lines.append("  }")
                elif field.is_primitive:
                    ffi_type = DART_FFI_TYPES[field.base_type]
                    dart_type = DART_TYPES[field.base_type]
                    lines.append(f"  static final {dart_type} Function(Pointer<Void>) _{field.name}Get = bindings.lookupFunction<{ffi_type} Function(Pointer<Void>), {dart_type} Function(Pointer<Void>)>('lwrcl_ffi_msg_get__{field_suffix}');")
                    lines.append(f"  static final void Function(Pointer<Void>, {dart_type}) _{field.name}Set = bindings.lookupFunction<Void Function(Pointer<Void>, {ffi_type}), void Function(Pointer<Void>, {dart_type})>('lwrcl_ffi_msg_set__{field_suffix}');")
                    lines.append("")
                    lines.append(f"  {DART_TYPES[field.base_type]} get {field.name} {{")
                    lines.append(f"    final value = _{field.name}Get(handle);")
                    lines.append(f"    throwIfLastError('get {field.name}');")
                    if field.base_type == "boolean":
                        lines.append("    return value != 0;")
                    else:
                        lines.append("    return value;")
                    lines.append("  }")
                    lines.append("")
                    lines.append(f"  set {field.name}({DART_TYPES[field.base_type]} value) {{")
                    if field.base_type == "boolean":
                        lines.append(f"    _{field.name}Set(handle, value ? 1 : 0);")
                    else:
                        lines.append(f"    _{field.name}Set(handle, value);")
                    lines.append(f"    throwIfLastError('set {field.name}');")
                    lines.append("  }")
                else:
                    nested_class = dart_class_name(field.nested_pkg, field.nested_name)
                    lines.append(f"  static final Pointer<Void> Function(Pointer<Void>) _{field.name}Get = bindings.lookupFunction<Pointer<Void> Function(Pointer<Void>), Pointer<Void> Function(Pointer<Void>)>('lwrcl_ffi_msg_get__{field_suffix}');")
                    lines.append(f"  static final void Function(Pointer<Void>, Pointer<Void>) _{field.name}Set = bindings.lookupFunction<Void Function(Pointer<Void>, Pointer<Void>), void Function(Pointer<Void>, Pointer<Void>)>('lwrcl_ffi_msg_set__{field_suffix}');")
                    lines.append("")
                    lines.append(f"  {nested_class} get {field.name} {{")
                    lines.append(f"    final ptr = _{field.name}Get(handle);")
                    lines.append("    if (ptr.address == 0) {")
                    lines.append(f"      throwIfLastError('get {field.name}');")
                    lines.append("    }")
                    lines.append(f"    return {nested_class}._fromHandle(ptr);")
                    lines.append("  }")
                    lines.append("")
                    lines.append(f"  set {field.name}({nested_class} value) {{")
                    lines.append(f"    _{field.name}Set(handle, value.handle);")
                    lines.append(f"    throwIfLastError('set {field.name}');")
                    lines.append("  }")

        lines.append("}")
        lines.append("")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate lwrcl FFI bindings for all ROS 2 messages")
    parser.add_argument("--idl-root", required=True, help="Path to ros-data-types-for-fastdds/src")
    parser.add_argument("--out-h", required=True, help="Output header path")
    parser.add_argument("--out-cpp", required=True, help="Output C++ source path")
    parser.add_argument("--out-dart", required=False, help="Output Dart bindings path")
    args = parser.parse_args()

    idl_root = Path(args.idl_root).resolve()
    if not idl_root.exists():
        raise SystemExit(
            "IDL root not found: "
            f"{idl_root}\n"
            "Hint: run `git submodule update --init --recursive` and verify the path "
            "`lwrcl/data_types/src/ros-data-types-for-fastdds/src` exists."
        )

    messages = find_messages(idl_root)
    if not messages:
        raise SystemExit(
            "No message IDL files found under: "
            f"{idl_root}\n"
            "Hint: ensure the ros-data-types-for-fastdds submodule is initialized "
            "and contains `msg/*.idl` files."
        )

    out_h = Path(args.out_h)
    out_cpp = Path(args.out_cpp)
    out_h.parent.mkdir(parents=True, exist_ok=True)
    out_cpp.parent.mkdir(parents=True, exist_ok=True)

    out_h.write_text(generate_header(messages), encoding="utf-8")
    out_cpp.write_text(generate_cpp(messages), encoding="utf-8")

    if args.out_dart:
        out_dart = Path(args.out_dart)
        out_dart.parent.mkdir(parents=True, exist_ok=True)
        out_dart.write_text(generate_dart(messages), encoding="utf-8")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
