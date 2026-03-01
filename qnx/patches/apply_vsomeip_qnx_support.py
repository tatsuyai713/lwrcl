#!/usr/bin/env python3
"""
Apply QNX SDP 8.0 support patches to upstream COVESA/vsomeip 3.6.x.

vsomeip 3.6.1 already ships native QNX support (USE_RT, socket linking,
CLOEXEC wrappers, QNX base path). This patcher only adds what is still
missing for a static-Boost cross-build on QNX SDP 8.0:

  CMakeLists.txt:
    - Wrap Boost find_package to use static .a libraries on QNX
    - Replace ${Boost_LIBRARIES} in target_link_libraries with static .a
    - Wrap systemd pkg_check_modules (pkg-config may not exist on QNX)
    - Guard vsomeip3-cfg socket linking (cfg is conditional)

  C++ sources:
    - asio_tcp_socket.hpp / asio_udp_socket.hpp: split SO_BINDTODEVICE
      guard so QNX gets stubs (QNX lacks SO_BINDTODEVICE)
    - wrappers_qnx.cpp: rewrite for QNX SDP 8.0 (sys/sockmsg.h removed,
      accept4() provided natively by libsocket)

This script is idempotent: running it twice produces the same result.

Usage:
    python3 apply_vsomeip_qnx_support.py /path/to/vsomeip/CMakeLists.txt
"""

import os
import re
import sys


# Idempotency marker we inject into every file we touch.
MARKER = "# QNX_AUTOSAR_PATCHED"


def _is_patched(content: str) -> bool:
    return MARKER in content


def patch_cmakelists(filepath: str) -> None:
    with open(filepath, "r") as f:
        content = f.read()

    if _is_patched(content):
        print(f"[INFO] {filepath} already patched for QNX, skipping.")
        return

    print(f"[INFO] Applying QNX support patch to {filepath}")

    # ---------------------------------------------------------------
    # 1) Wrap systemd check for QNX (pkg-config may not be available)
    # ---------------------------------------------------------------
    if 'if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "QNX")' not in content:
        content = content.replace(
            'pkg_check_modules(SystemD "libsystemd")',
            'if (NOT ${CMAKE_SYSTEM_NAME} MATCHES "QNX")\n'
            'pkg_check_modules(SystemD "libsystemd")\n'
            'endif()',
            1,
        )

    # ---------------------------------------------------------------
    # 2) Wrap Boost find_package for QNX to use pre-built static libs.
    #    The build script passes -DBoost_FOUND=TRUE and sets paths manually,
    #    but we still need the static lib variables defined.
    # ---------------------------------------------------------------
    boost_pattern = re.compile(
        r"find_package\(\s*Boost\s+(\d+\.\d+)\s+COMPONENTS\s+"
        r"system\s+thread\s+filesystem\s+REQUIRED\s*\)"
    )
    boost_match = boost_pattern.search(content)
    if boost_match:
        old_boost = boost_match.group(0)
        ctx_before = content[max(0, boost_match.start() - 100) : boost_match.start()]
        if "BOOST_SYSTEM_LIB" not in ctx_before:
            new_boost = (
                'if (${CMAKE_SYSTEM_NAME} MATCHES "QNX")\n'
                "set(BOOST_SYSTEM_LIB ${Boost_LIBRARY_DIR}/libboost_system.a)\n"
                "set(BOOST_THREAD_LIB ${Boost_LIBRARY_DIR}/libboost_thread.a)\n"
                "set(BOOST_FILESYSTEM_LIB ${Boost_LIBRARY_DIR}/libboost_filesystem.a)\n"
                "else ()\n"
                f"{old_boost}\n"
                "endif ()"
            )
            content = content.replace(old_boost, new_boost, 1)

    # ---------------------------------------------------------------
    # 3) Replace ${Boost_LIBRARIES} in target_link_libraries with
    #    static .a references on QNX.
    # ---------------------------------------------------------------
    link_pattern = re.compile(
        r"target_link_libraries\([^)]*\$\{Boost_LIBRARIES\}[^)]*\)"
    )

    def qnx_link_replace(match: re.Match) -> str:
        original = match.group(0)
        boost_static = (
            "${BOOST_SYSTEM_LIB} ${BOOST_THREAD_LIB} ${BOOST_FILESYSTEM_LIB}"
        )
        qnx_line = original.replace("${Boost_LIBRARIES}", boost_static)
        return (
            'if (${CMAKE_SYSTEM_NAME} MATCHES "QNX")\n\t'
            + qnx_line
            + "\nelse()\n\t"
            + original
            + "\nendif()"
        )

    content = link_pattern.sub(qnx_link_replace, content)

    # ---------------------------------------------------------------
    # 4) Fix vsomeip 3.6.1 bug: the native QNX socket-linking block
    #    references vsomeip3-cfg unconditionally, but cfg is only built
    #    when VSOMEIP_ENABLE_MULTIPLE_ROUTING_MANAGERS == 0.
    # ---------------------------------------------------------------
    content = content.replace(
        "    target_link_libraries(${VSOMEIP_NAME}-cfg socket)\n",
        "    if (VSOMEIP_ENABLE_MULTIPLE_ROUTING_MANAGERS EQUAL 0)\n"
        "        target_link_libraries(${VSOMEIP_NAME}-cfg socket)\n"
        "    endif()\n",
        1,
    )

    # Stamp with our marker so we won't re-patch.
    content = MARKER + "\n" + content

    with open(filepath, "w") as f:
        f.write(content)

    print(f"[INFO] QNX support patch applied to {filepath}")


def patch_asio_socket_header(filepath: str) -> None:
    """Fix SO_BINDTODEVICE guard: QNX does not have SO_BINDTODEVICE.

    The base class declares bind_to_device() and can_read_fd_flags() as pure
    virtual, so we must provide implementations for QNX:
      - bind_to_device: return false (SO_BINDTODEVICE not supported on QNX)
      - can_read_fd_flags: use fcntl(F_GETFD) which works on QNX
    """
    if not os.path.exists(filepath):
        return

    with open(filepath, "r") as f:
        content = f.read()

    if _is_patched(content):
        print(f"[INFO] {filepath} already patched for QNX, skipping.")
        return

    old_guard = '#if defined(__linux__) || defined(__QNX__)'
    new_guard_block = (
        '#if defined(__linux__)\n'
        '    [[nodiscard]] bool bind_to_device(std::string const& _device) override {\n'
        '        return setsockopt(socket_.native_handle(), SOL_SOCKET, SO_BINDTODEVICE, _device.c_str(), static_cast<socklen_t>(_device.size()))\n'
        '                != -1;\n'
        '    }\n'
        '    [[nodiscard]] bool can_read_fd_flags() override { return fcntl(socket_.native_handle(), F_GETFD) != -1; }\n'
        '#elif defined(__QNX__)\n'
        '    // QNX: SO_BINDTODEVICE is not supported; provide stubs for pure virtuals.\n'
        '    [[nodiscard]] bool bind_to_device(std::string const& /*_device*/) override { return false; }\n'
        '    [[nodiscard]] bool can_read_fd_flags() override { return fcntl(socket_.native_handle(), F_GETFD) != -1; }\n'
        '#endif'
    )

    if old_guard not in content:
        print(f"[INFO] {filepath}: SO_BINDTODEVICE guard not found, skipping.")
        return

    # Find the full #if...#endif block to replace
    start = content.index(old_guard)
    end_tag = '#endif'
    end = content.index(end_tag, start) + len(end_tag)

    print(f"[INFO] Patching {filepath}: replacing SO_BINDTODEVICE block with Linux+QNX stubs")
    content = content[:start] + new_guard_block + content[end:]
    content = f"// {MARKER}\n" + content

    with open(filepath, "w") as f:
        f.write(content)


def patch_wrappers_qnx(filepath: str) -> None:
    """Rewrite wrappers_qnx.cpp for QNX SDP 8.0.

    The upstream file uses sys/sockmsg.h which was removed in QNX 8.0.
    In QNX 8.0, accept4() is provided natively by libsocket, so we can
    drop the custom implementation and just use the native one.
    """
    if not os.path.exists(filepath):
        return

    with open(filepath, "r") as f:
        content = f.read()

    if _is_patched(content):
        print(f"[INFO] {filepath} already patched for QNX, skipping.")
        return

    if "sys/sockmsg.h" not in content:
        print(f"[INFO] {filepath}: sys/sockmsg.h not referenced, skipping.")
        return

    print(f"[INFO] Patching {filepath}: replacing custom accept4() with QNX 8.0 native accept4()")

    new_content = f"""\
// {MARKER}
// wrappers_qnx.cpp - patched for QNX SDP 8.0
// QNX 8.0 provides accept4() natively in libsocket; sys/sockmsg.h is gone.
// We keep the __wrap_socket/__wrap_accept/__wrap_open wrappers but use the
// native accept4() instead of the custom message-passing implementation.
#ifdef __QNX__

#include <sys/socket.h>
#include <sys/types.h>
#include <fcntl.h>
#include <cerrno>
#include <pthread.h>
#include <cstdlib>
#include <string.h>
#include <unistd.h>

/*
 * These definitions MUST remain in the global namespace.
 */
extern "C" {{
/*
 * The real socket(2), renamed by GCC.
 */
int __real_socket(int domain, int type, int protocol) noexcept;

/*
 * Overrides socket(2) to set SOCK_CLOEXEC by default.
 */
int __wrap_socket(int domain, int type, int protocol) noexcept {{
    return __real_socket(domain, type | SOCK_CLOEXEC, protocol);
}}

/*
 * Overrides accept(2) to set SOCK_CLOEXEC by default.
 * QNX 8.0 provides accept4() natively in libsocket.
 */
int __wrap_accept(int sockfd, struct sockaddr* addr, socklen_t* addrlen) {{
    return accept4(sockfd, addr, addrlen, SOCK_CLOEXEC);
}}

/*
 * The real open(2), renamed by GCC.
 */
int __real_open(const char* pathname, int flags, mode_t mode);

/*
 * Overrides open(2) to set O_CLOEXEC by default.
 */
int __wrap_open(const char* pathname, int flags, mode_t mode) {{
    return __real_open(pathname, flags | O_CLOEXEC, mode);
}}
}}

#endif
"""
    with open(filepath, "w") as f:
        f.write(new_content)


def patch_so_bindtodevice(filepath: str) -> None:
    """Fix SO_BINDTODEVICE guards in TCP/UDP endpoint implementation files.

    QNX does not have SO_BINDTODEVICE (Linux-specific). Upstream guards
    SO_BINDTODEVICE calls with:
        #if defined(__linux__) || defined(ANDROID) || defined(__QNX__)

    Remove __QNX__ from only those specific guards that contain SO_BINDTODEVICE,
    leaving other QNX-inclusive guards intact.
    """
    if not os.path.exists(filepath):
        return

    with open(filepath, "r") as f:
        content = f.read()

    if _is_patched(content):
        print(f"[INFO] {filepath} already patched for QNX, skipping.")
        return

    if "SO_BINDTODEVICE" not in content:
        print(f"[INFO] {filepath}: no SO_BINDTODEVICE usage, skipping.")
        return

    OLD_GUARD = "#if defined(__linux__) || defined(ANDROID) || defined(__QNX__)"
    NEW_GUARD = "#if defined(__linux__) || defined(ANDROID)"

    if OLD_GUARD not in content:
        print(f"[INFO] {filepath}: target guard not found, skipping.")
        return

    # Split on the guard and fix only blocks that contain SO_BINDTODEVICE
    parts = content.split(OLD_GUARD)
    changed = False
    new_parts = [parts[0]]
    for part in parts[1:]:
        # Find the matching #endif for this guard (at start of line)
        endif_pos = part.find("\n#endif")
        if endif_pos != -1 and "SO_BINDTODEVICE" in part[:endif_pos]:
            new_parts.append(NEW_GUARD + part)
            changed = True
        else:
            new_parts.append(OLD_GUARD + part)

    if not changed:
        print(f"[INFO] {filepath}: SO_BINDTODEVICE not inside target guard, skipping.")
        return

    new_content = "".join(new_parts)
    new_content = f"// {MARKER}\n" + new_content

    print(f"[INFO] Patching {filepath}: removing __QNX__ from SO_BINDTODEVICE guard")
    with open(filepath, "w") as f:
        f.write(new_content)


def patch_server_endpoint_impl(filepath: str) -> None:
    """Fix server_endpoint_impl.cpp QNX template instantiation block.

    The QNX-specific template instantiation block unconditionally uses
    stream_protocol_ext regardless of the Boost version, while the Linux
    block correctly guards it with VSOMEIP_BOOST_VERSION < 106600.
    With Boost >= 1.66 (e.g. 1.86), stream_protocol_ext does not exist
    in the standard Boost installation; the file is only present in the
    vsomeip helper/ directory for older Boost builds.

    Replace:
        #ifdef __QNX__
        template class server_endpoint_impl<boost::asio::local::stream_protocol_ext>;
        #endif

    With:
        #ifdef __QNX__
        #if VSOMEIP_BOOST_VERSION < 106600
        template class server_endpoint_impl<boost::asio::local::stream_protocol_ext>;
        #else
        template class server_endpoint_impl<boost::asio::local::stream_protocol>;
        #endif
        #endif
    """
    if not os.path.exists(filepath):
        return

    with open(filepath, "r") as f:
        content = f.read()

    if _is_patched(content):
        print(f"[INFO] {filepath} already patched for QNX, skipping.")
        return

    old_block = (
        "#ifdef __QNX__\n"
        "template class server_endpoint_impl<boost::asio::local::stream_protocol_ext>;\n"
        "#endif"
    )

    if old_block not in content:
        print(f"[INFO] {filepath}: QNX stream_protocol_ext block not found, skipping.")
        return

    new_block = (
        "#ifdef __QNX__\n"
        "#if VSOMEIP_BOOST_VERSION < 106600\n"
        "template class server_endpoint_impl<boost::asio::local::stream_protocol_ext>;\n"
        "#else\n"
        "template class server_endpoint_impl<boost::asio::local::stream_protocol>;\n"
        "#endif\n"
        "#endif"
    )

    content = content.replace(old_block, new_block, 1)
    content = f"// {MARKER}\n" + content

    print(f"[INFO] Patching {filepath}: adding VSOMEIP_BOOST_VERSION guard to QNX stream_protocol_ext block")
    with open(filepath, "w") as f:
        f.write(content)


def patch_udp_pktinfo(filepath: str) -> None:
    """Patch udp_server_endpoint_impl_receive_op.hpp for QNX.

    QNX lacks struct in_pktinfo and IP_PKTINFO (Linux/Windows-specific IPv4
    packet info). QNX supports IPv6 pktinfo (in6_pktinfo / IPV6_PKTINFO) but
    not the IPv4 equivalent. Provide stubs so the file compiles on QNX.
    The IPv4 destination address detection will be silently non-functional at
    runtime on QNX (setsockopt with IP_PKTINFO will fail gracefully; no
    IP_PKTINFO cmsgs will be received), which is acceptable for AUTOSAR targets.
    """
    if not os.path.exists(filepath):
        return

    with open(filepath, "r") as f:
        content = f.read()

    if _is_patched(content):
        print(f"[INFO] {filepath} already patched for QNX, skipping.")
        return

    if "in_pktinfo" not in content and "IP_PKTINFO" not in content:
        print(f"[INFO] {filepath}: no in_pktinfo/IP_PKTINFO usage, skipping.")
        return

    compat_block = (
        "#ifdef __QNX__\n"
        "// QNX compatibility: struct in_pktinfo and IP_PKTINFO are Linux/Windows-specific.\n"
        "// QNX supports IPv6 pktinfo but not IPv4 pktinfo. These stubs allow compilation;\n"
        "// IPv4 destination address detection will be silently non-functional at runtime.\n"
        "#ifndef IP_PKTINFO\n"
        "#  define IP_PKTINFO 26\n"
        "#endif\n"
        "#ifndef __in_pktinfo_defined\n"
        "#  define __in_pktinfo_defined\n"
        "struct in_pktinfo {\n"
        "    unsigned int   ipi_ifindex;\n"
        "    struct in_addr ipi_spec_dst;\n"
        "    struct in_addr ipi_addr;\n"
        "};\n"
        "#endif\n"
        "#endif // __QNX__\n"
        "\n"
    )

    # Inject after the include guard opening
    lines = content.splitlines(keepends=True)
    insert_idx = 0
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith(("#ifndef", "#define", "#pragma once", "//")):
            insert_idx = i + 1
        elif stripped == "":
            continue
        else:
            break

    lines.insert(insert_idx, compat_block)
    new_content = f"// {MARKER}\n" + "".join(lines)

    print(f"[INFO] Patching {filepath}: adding struct in_pktinfo/IP_PKTINFO stubs for QNX")
    with open(filepath, "w") as f:
        f.write(new_content)


def patch_local_server_receive_op(filepath: str) -> None:
    """Patch local_server_endpoint_impl_receive_op.hpp for QNX.

    QNX lacks struct ucred and SCM_CREDENTIALS (Linux-specific credential
    passing via Unix domain sockets). Provide compatibility stubs so the
    file compiles on QNX. Credential passing will be silently non-functional
    at runtime on QNX, which is acceptable for embedded targets.
    """
    if not os.path.exists(filepath):
        return

    with open(filepath, "r") as f:
        content = f.read()

    if _is_patched(content):
        print(f"[INFO] {filepath} already patched for QNX, skipping.")
        return

    if "SCM_CREDENTIALS" not in content and "struct ucred" not in content:
        print(f"[INFO] {filepath}: no SCM_CREDENTIALS/ucred usage, skipping.")
        return

    compat_block = (
        "#ifdef __QNX__\n"
        "// QNX compatibility: struct ucred and SCM_CREDENTIALS are Linux-specific.\n"
        "// On QNX, credential passing via Unix domain sockets is not supported;\n"
        "// these stubs allow compilation; credential auth is skipped at runtime.\n"
        "#ifndef SCM_CREDENTIALS\n"
        "#  define SCM_CREDENTIALS 0x02\n"
        "#endif\n"
        "#ifndef __ucred_defined\n"
        "#  define __ucred_defined\n"
        "struct ucred { unsigned int uid; unsigned int gid; unsigned int pid; };\n"
        "#endif\n"
        "#endif // __QNX__\n"
        "\n"
    )

    # Inject after the include guard opening (first #ifndef/#define pair or #pragma once)
    lines = content.splitlines(keepends=True)
    insert_idx = 0
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped.startswith(("#ifndef", "#define", "#pragma once", "//")):
            insert_idx = i + 1
        elif stripped == "":
            continue
        else:
            break

    lines.insert(insert_idx, compat_block)
    new_content = f"// {MARKER}\n" + "".join(lines)

    print(f"[INFO] Patching {filepath}: adding struct ucred/SCM_CREDENTIALS stubs for QNX")
    with open(filepath, "w") as f:
        f.write(new_content)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <CMakeLists.txt>", file=sys.stderr)
        sys.exit(1)

    cmake_file = sys.argv[1]
    vsomeip_root = os.path.dirname(cmake_file)

    patch_cmakelists(cmake_file)

    # Patch C++ headers with QNX-incompatible Linux socket options
    for header in [
        "implementation/endpoints/include/asio_tcp_socket.hpp",
        "implementation/endpoints/include/asio_udp_socket.hpp",
    ]:
        patch_asio_socket_header(os.path.join(vsomeip_root, header))

    # Patch wrappers_qnx.cpp for QNX SDP 8.0 (sys/sockmsg.h removed)
    patch_wrappers_qnx(
        os.path.join(vsomeip_root, "implementation/utility/src/wrappers_qnx.cpp")
    )

    # Patch TCP/UDP endpoint .cpp files: remove __QNX__ from SO_BINDTODEVICE guards
    # (SO_BINDTODEVICE is Linux-only and not available on QNX)
    for src in [
        "implementation/endpoints/src/tcp_client_endpoint_impl.cpp",
        "implementation/endpoints/src/tcp_server_endpoint_impl.cpp",
        "implementation/endpoints/src/udp_client_endpoint_impl.cpp",
        "implementation/endpoints/src/udp_server_endpoint_impl.cpp",
    ]:
        patch_so_bindtodevice(os.path.join(vsomeip_root, src))

    # Patch server_endpoint_impl.cpp: add VSOMEIP_BOOST_VERSION guard to QNX
    # template instantiation block (upstream uses stream_protocol_ext unconditionally
    # on QNX, but Boost >= 1.66 does not have stream_protocol_ext in the standard path)
    patch_server_endpoint_impl(
        os.path.join(
            vsomeip_root,
            "implementation/endpoints/src/server_endpoint_impl.cpp",
        )
    )

    # Patch udp_server_endpoint_impl_receive_op.hpp for QNX
    # (struct in_pktinfo / IP_PKTINFO are Linux/Windows-specific)
    patch_udp_pktinfo(
        os.path.join(
            vsomeip_root,
            "implementation/endpoints/include/udp_server_endpoint_impl_receive_op.hpp",
        )
    )

    # Patch local_server_endpoint_impl_receive_op.hpp for QNX
    # (struct ucred / SCM_CREDENTIALS are Linux-specific)
    patch_local_server_receive_op(
        os.path.join(
            vsomeip_root,
            "implementation/endpoints/include/local_server_endpoint_impl_receive_op.hpp",
        )
    )
