#!/usr/bin/env python3
import re
import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: patch_type_name.py <header>", file=sys.stderr)
        return 2

    header = Path(sys.argv[1])
    text = header.read_text()
    patched = re.sub(
        r'(return\s*")([A-Za-z0-9_]+::msg::)([A-Za-z0-9_]+)(";)',
        r'\1\2dds_::\3_\4',
        text,
    )
    if patched != text:
        header.write_text(patched)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
