#!/usr/bin/env python3
"""Patch generated AUTOSAR proxy/skeleton header to avoid iceoryx runtime-name collisions.

The upstream code generator currently constructs ZeroCopyPublisher/Subscriber
with a fixed default runtime name ("adaptive_autosar_ara_com"), which collides
when publisher/subscriber run in separate processes.

This patch injects:
- `ResolveIceoryxRuntimeName()` helper
- optional env overrides:
  - `ARA_COM_ICEORYX_RUNTIME_NAME`
  - `ARA_COM_ICEORYX_RUNTIME_PREFIX`
- process-id suffix fallback to guarantee uniqueness across processes
"""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


RUNTIME_HELPER = """// lwrcl patch: avoid fixed iceoryx runtime-name collisions across processes.
inline std::string ResolveIceoryxRuntimeName()
{
  const char *fixed_runtime_name = std::getenv("ARA_COM_ICEORYX_RUNTIME_NAME");
  if (fixed_runtime_name != nullptr && fixed_runtime_name[0] != '\\0')
  {
    return std::string{fixed_runtime_name};
  }

  std::string runtime_prefix{"adaptive_autosar_ara_com"};
  const char *runtime_prefix_env = std::getenv("ARA_COM_ICEORYX_RUNTIME_PREFIX");
  if (runtime_prefix_env != nullptr && runtime_prefix_env[0] != '\\0')
  {
    runtime_prefix = SanitizeIceoryxToken(std::string{runtime_prefix_env});
  }

  return runtime_prefix + "_" + std::to_string(static_cast<unsigned long>(::getpid()));
}

"""


def apply_patch_to_text(text: str) -> str:
    updated = text

    # getpid for runtime-name suffix
    if "#include <unistd.h>" not in updated:
        if "#include <cstdlib>\n" in updated:
            updated = updated.replace(
                "#include <cstdlib>\n",
                "#include <cstdlib>\n#include <unistd.h>\n",
                1,
            )
        else:
            raise RuntimeError("Failed to patch include list: '#include <cstdlib>' not found.")

    marker = "inline ara::com::zerocopy::ChannelDescriptor BuildIceoryxChannel(const TopicBinding &binding)\n{"
    if "inline std::string ResolveIceoryxRuntimeName()" not in updated:
        if marker not in updated:
            raise RuntimeError("Failed to inject runtime-name helper: BuildIceoryxChannel marker not found.")
        updated = updated.replace(marker, RUNTIME_HELPER + marker, 1)

    updated = updated.replace(
        "publisher_(BuildIceoryxChannel(binding))",
        "publisher_(BuildIceoryxChannel(binding), ResolveIceoryxRuntimeName())",
    )
    updated = updated.replace(
        "subscriber_(BuildIceoryxChannel(binding))",
        "subscriber_(BuildIceoryxChannel(binding), ResolveIceoryxRuntimeName())",
    )

    if "publisher_(BuildIceoryxChannel(binding), ResolveIceoryxRuntimeName())" not in updated:
        raise RuntimeError("Failed to patch iceoryx publisher runtime-name initialization.")
    if "subscriber_(BuildIceoryxChannel(binding), ResolveIceoryxRuntimeName())" not in updated:
        raise RuntimeError("Failed to patch iceoryx subscriber runtime-name initialization.")

    return updated


def main() -> int:
    parser = argparse.ArgumentParser(description="Patch generated autosar proxy/skeleton header.")
    parser.add_argument("--header", required=True, help="Path to generated header")
    args = parser.parse_args()

    header_path = Path(args.header)
    if not header_path.is_file():
        print(f"[autosar-proxy-patch] header not found: {header_path}", file=sys.stderr)
        return 1

    original = header_path.read_text(encoding="utf-8")
    patched = apply_patch_to_text(original)
    if patched != original:
        header_path.write_text(patched, encoding="utf-8")
        print(f"[autosar-proxy-patch] patched: {header_path}")
    else:
        print(f"[autosar-proxy-patch] already up to date: {header_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

