#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
expose_idl_members.py

Post-process IDL-generated C++ code (FastDDS / CycloneDDS) so that struct
members are exposed as plain public members rather than via "()" accessors.

After this patcher runs, code that used to be written as

    msg->header().stamp().sec() = 1;
    msg->header().frame_id()   = "x";

can be written ROS 2-style:

    msg->header.stamp.sec = 1;
    msg->header.frame_id  = "x";

The patcher is idempotent: running it again on already-patched code is a no-op.

Usage:
    expose_idl_members.py --mode {fastdds,cyclonedds} <directory> [<directory> ...]

  --mode fastdds      Patch fastddsgen output:
                        *.h            (skips *PubSubTypes.h)
                        and matching   *.cxx
  --mode cyclonedds   Patch cyclonedds idlc -l cxx output:
                        *_cyclone.hpp

The script walks each given directory recursively.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Tuple


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_matching_brace(text: str, open_pos: int) -> int:
    """Return the index of the '}' matching the '{' at open_pos, or -1."""
    assert text[open_pos] == '{'
    depth = 0
    i = open_pos
    n = len(text)
    while i < n:
        c = text[i]
        if c == '{':
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0:
                return i
        i += 1
    return -1


def _strip_doc_comment_before(text: str, pos: int) -> int:
    """If a /*! ... */ block (with surrounding whitespace) ends just before
    pos, return the index where it starts; otherwise return pos.

    The regex uses [^/]*? (rather than .*?) so the lazy body cannot leak
    across multiple comments — C/C++ doc comments contain no '/' character.
    """
    m = re.search(r'/\*![^/]*?\*/\s*\Z', text[:pos], re.DOTALL)
    if m:
        return m.start()
    return pos


def _strip_comments(text: str) -> str:
    """Remove C/C++ comments. Used only for tokenisation/scanning, never for
    rewriting the file itself."""
    text = re.sub(r'/\*.*?\*/', '', text, flags=re.DOTALL)
    text = re.sub(r'//[^\n]*', '', text)
    return text


# ---------------------------------------------------------------------------
# FastDDS
# ---------------------------------------------------------------------------

# Field declaration inside the trailing private: block of a fastddsgen header.
# Examples:
#   builtin_interfaces::msg::Time m_stamp;
#   std::string m_frame_id;
#   std::vector<uint8_t> m_data;
#   std::array<double, 9> m_K;
_FASTDDS_FIELD_RE = re.compile(
    r'^\s*([\w:<>,\s\*]+?)\s+m_(\w+)\s*;\s*$',
    re.MULTILINE,
)


def _patch_fastdds_header(path: Path) -> List[Tuple[str, List[str]]]:
    """Patch all classes in a fastddsgen *.h file. Returns a list of
    (class_name, field_names) tuples for each class that was successfully
    patched. Returns [] if the file was not a candidate."""
    text = path.read_text()
    results: List[Tuple[str, List[str]]] = []

    pos = 0
    while True:
        # Search for next class on a comment-stripped view to avoid matching
        # `class` inside doc comments, then map back to the original text.
        stripped_tail = _strip_comments(text[pos:])
        cm = re.search(r'\bclass\s+(\w+)\b[^{;]*\{', stripped_tail)
        if not cm:
            break
        class_name = cm.group(1)
        cm_orig = re.search(
            r'\bclass\s+' + re.escape(class_name) + r'\b[^{;]*\{', text[pos:]
        )
        if not cm_orig:
            break
        body_open = pos + cm_orig.end() - 1  # position of '{'
        body_close = _find_matching_brace(text, body_open)
        if body_close < 0:
            break

        body = text[body_open + 1: body_close]
        new_body, field_names = _patch_fastdds_class_body(body)
        if field_names:
            text = text[:body_open + 1] + new_body + text[body_close:]
            # body length may have changed; recompute close
            body_close = body_open + 1 + len(new_body)
            results.append((class_name, field_names))
        pos = body_close + 1

    if results:
        path.write_text(text)
    return results


def _patch_fastdds_class_body(body: str) -> Tuple[str, List[str]]:
    """Patch a single class body. Returns (new_body, field_names)."""

    # Locate the LAST `private:` label inside the class body — that is the
    # one that contains the data members in fastddsgen output.
    priv_match = None
    for m in re.finditer(r'(?m)^\s*private\s*:\s*$', body):
        priv_match = m
    if priv_match is None:
        return body, []
    priv_section = body[priv_match.end():]

    fields = _FASTDDS_FIELD_RE.findall(priv_section)
    field_names = [name for _, name in fields]
    if not field_names:
        return body, []

    new_body = body

    # 1) Remove accessor declarations (4 forms × N fields) along with their
    #    optional /*! ... */ doc comment.
    for name in field_names:
        decl_re = re.compile(
            r'(?:/\*![^/]*?\*/\s*)?'
            r'eProsima_user_DllExport\s+[^;{}]*?\b' + re.escape(name) +
            r'\s*\([^;{}]*?\)\s*(?:const\s*)?;\s*\n',
            re.DOTALL,
        )
        new_body = decl_re.sub('', new_body)

    # 2) Convert the LAST `private:` label to `public:`.
    last_priv = list(re.finditer(r'(?m)^(\s*)private(\s*:\s*)$', new_body))
    if last_priv:
        m = last_priv[-1]
        new_body = new_body[:m.start()] + m.group(1) + 'public' + m.group(2) + new_body[m.end():]

    # 3) Rename m_<name> -> <name>.
    for name in field_names:
        new_body = re.sub(r'\bm_' + re.escape(name) + r'\b', name, new_body)

    return new_body, field_names


def _patch_fastdds_cxx(path: Path, class_name: str, field_names: List[str]) -> None:
    """Patch the matching *.cxx file."""
    if not path.exists():
        return
    text = path.read_text()
    original = text

    # 1) Remove accessor function definitions:
    #       <returnType> <ns>...::<class_name>::<field>( ... ) [const] { ... }
    #    plus optional preceding /*! ... */ doc comment.
    for name in field_names:
        sig_re = re.compile(
            r'(?:[\w:<>,\s\*&]+?)\s+'
            r'(?:\w+::)*' + re.escape(class_name) + r'::' + re.escape(name) +
            r'\s*\(',
        )
        out: List[str] = []
        pos = 0
        while True:
            m = sig_re.search(text, pos)
            if not m:
                break
            # Find the corresponding '{' after the signature.
            paren_close = text.find(')', m.end())
            if paren_close < 0:
                break
            brace_open = text.find('{', paren_close)
            if brace_open < 0:
                break
            brace_close = _find_matching_brace(text, brace_open)
            if brace_close < 0:
                break
            start = _strip_doc_comment_before(text, m.start())
            out.append(text[pos:start])
            after = brace_close + 1
            while after < len(text) and text[after] in '\r\n':
                after += 1
            pos = after
        out.append(text[pos:])
        text = ''.join(out)

    # 2) Rename m_<name> -> <name>.
    #    First handle qualified accesses (`x.m_foo` -> `x.foo`), then any
    #    remaining unqualified `m_foo` (implicit `this->`) — qualify with
    #    `this->` to avoid name collisions with parameters that have the
    #    same name as the field (fastddsgen typically names copy/move
    #    parameters `x` even when the message has a field `x`).
    for name in field_names:
        text = re.sub(r'(\.|->)m_' + re.escape(name) + r'\b', r'\1' + name, text)
        text = re.sub(r'\bm_' + re.escape(name) + r'\b', 'this->' + name, text)

    # 3) Replace `data.<name>()` and `data-><name>()` with member access.
    #    Restricted to `data` (the fastddsgen serialization parameter name)
    #    so we don't strip parens from unrelated calls like `s.size()`.
    for name in field_names:
        text = re.sub(r'(\bdata(?:\.|->))' + re.escape(name) + r'\(\)', r'\g<1>' + name, text)

    if text != original:
        path.write_text(text)


def _scan_class_name(header_path: Path) -> str:
    text = _strip_comments(header_path.read_text())
    m = re.search(r'\bclass\s+(\w+)\b[^{;]*\{', text)
    return m.group(1) if m else ''


def patch_fastdds_dir(root: Path) -> int:
    n_patched = 0
    # Map of class_name -> set of field names (for cross-file rewrites).
    class_to_fields: Dict[str, set] = {}
    patched_headers: List[Path] = []
    for h in root.rglob('*.h'):
        if h.name.endswith('PubSubTypes.h'):
            continue
        try:
            class_results = _patch_fastdds_header(h)
            if not class_results:
                continue
            cxx = h.with_suffix('.cxx')
            for class_name, field_names in class_results:
                _patch_fastdds_cxx(cxx, class_name, field_names)
                class_to_fields.setdefault(class_name, set()).update(field_names)
            n_patched += 1
            patched_headers.append(h)
            print(f"[expose_idl_members] FastDDS patched: {h}")
        except Exception as e:  # noqa: BLE001
            print(f"[expose_idl_members] WARN: failed on {h}: {e}", file=sys.stderr)

    # Second pass: rewrite `<Class>::m_<field>` and `data.m_<field>` /
    # `data->m_<field>` in companion files (PubSubTypes.h/.cxx, etc.) so
    # references to renamed members keep compiling.
    if class_to_fields:
        targets: List[Path] = []
        for ext in ('*.h', '*.cxx', '*.hpp', '*.cpp'):
            targets.extend(root.rglob(ext))
        for t in targets:
            try:
                txt = t.read_text()
            except Exception:
                continue
            new = txt
            for cname, fields in class_to_fields.items():
                for f in fields:
                    new = re.sub(
                        r'\b' + re.escape(cname) + r'::m_' + re.escape(f) + r'\b',
                        cname + '::' + f,
                        new,
                    )
                    new = re.sub(
                        r'(\bdata(?:->|\.))m_' + re.escape(f) + r'\b',
                        r'\g<1>' + f,
                        new,
                    )
            if new != txt:
                t.write_text(new)
    return n_patched


# ---------------------------------------------------------------------------
# CycloneDDS (idlc -l cxx)
# ---------------------------------------------------------------------------

# Cyclone field declarations inside the leading private: block of the class:
#    ::builtin_interfaces::msg::Time stamp_;
#    std::string frame_id_;
_CYCLONE_FIELD_RE = re.compile(
    r'^\s*((?:::)?[\w:<>,\s\*]+?)\s+(\w+)_\s*(?:=[^;]*)?;\s*$',
    re.MULTILINE,
)


def _patch_cyclonedds_file(path: Path) -> List[str]:
    text = path.read_text()
    original = text

    # Walk every class in the file (cyclone idlc emits multiple classes when
    # multiple structs are defined in one IDL).
    all_field_names: List[str] = []
    pos = 0
    while True:
        cm_orig = re.search(r'\bclass\s+(\w+)\b[^{;]*\{', text[pos:])
        if not cm_orig:
            break
        class_name = cm_orig.group(1)
        body_open = pos + cm_orig.end() - 1
        body_close = _find_matching_brace(text, body_open)
        if body_close < 0:
            break

        body = text[body_open + 1: body_close]
        new_body, field_names = _patch_cyclonedds_class_body(body)
        all_field_names.extend(field_names)
        text = text[:body_open + 1] + new_body + text[body_close:]
        # Recompute body_close because new_body may differ in length.
        body_close = body_open + 1 + len(new_body)
        pos = body_close + 1

    # Outside the class (read/write/move/max template helpers, operator==),
    # rewrite `<receiver>.<name>()` -> `<receiver>.<name>` ONLY when the
    # receiver is one of the known idlc-generated variable names
    # (`instance`, `_other`, `_val_`). This avoids accidentally stripping
    # parens from unrelated calls like `props.data()` where `props` is a
    # cyclonedds CDR `entity_properties_t` whose `data()` is a method.
    _RECEIVERS = r'(?:instance|_other|_val_)'
    for name in all_field_names:
        text = re.sub(
            r'(\b' + _RECEIVERS + r'(?:->|\.))' + re.escape(name) + r'\(\)',
            r'\g<1>' + name,
            text,
        )
        # Rename any remaining `<receiver>.<name>_` (e.g. operator== uses
        # `_other.stamp_`).
        text = re.sub(
            r'(\b' + _RECEIVERS + r'(?:->|\.))' + re.escape(name) + r'_\b',
            r'\g<1>' + name,
            text,
        )

    if text != original:
        path.write_text(text)
    return all_field_names


def _patch_cyclonedds_class_body(body: str) -> Tuple[str, List[str]]:

    # Find the FIRST private: section (where members live).
    priv_iter = list(re.finditer(r'(?m)^(\s*)private(\s*:\s*)$', body))
    if not priv_iter:
        return body, []
    priv_match = priv_iter[0]
    after_priv = body[priv_match.end():]
    # Truncate at the next access-specifier label.
    nxt = re.search(r'(?m)^\s*(public|protected|private)\s*:\s*$', after_priv)
    section = after_priv[: nxt.start()] if nxt else after_priv

    fields = _CYCLONE_FIELD_RE.findall(section)
    field_names = [name for _, name in fields]
    if not field_names:
        return body, []

    new_body = body

    # 1) Remove inline accessor definitions inside the class body.
    #    Patterns produced by idlc -l cxx (single-line):
    #      const T& stamp() const { return this->stamp_; }
    #      T& stamp() { return this->stamp_; }
    #      void stamp(const T& _val_) { this->stamp_ = _val_; }
    #      void stamp(T&& _val_) { this->stamp_ = _val_; }
    for name in field_names:
        accessor_re = re.compile(
            r'^[ \t]*(?:'
            # Getter: any return type (by value, by reference, by const reference)
            r'[\w:<>\*&][\w:<>\s\*&,]*\s+' + re.escape(name) + r'\s*\(\s*\)\s*(?:const\s*)?'
            r'|'
            # void setter (copy / move)
            r'void\s+' + re.escape(name) + r'\s*\([^)]*\)\s*'
            r')\{[^}]*\}\s*\n',
            re.MULTILINE,
        )
        new_body = accessor_re.sub('', new_body)

    # 2) Convert the FIRST private: -> public:
    m2 = re.search(r'(?m)^(\s*)private(\s*:\s*)$', new_body)
    if m2:
        new_body = new_body[:m2.start()] + m2.group(1) + 'public' + m2.group(2) + new_body[m2.end():]

    # 3) Rename <name>_ -> <name> within the class body.
    for name in field_names:
        new_body = re.sub(r'\b' + re.escape(name) + r'_\b', name, new_body)

    return new_body, field_names


def patch_cyclonedds_dir(root: Path) -> int:
    n_patched = 0
    for h in root.rglob('*_cyclone.hpp'):
        try:
            names = _patch_cyclonedds_file(h)
            if names:
                n_patched += 1
                print(f"[expose_idl_members] CycloneDDS patched: {h}")
        except Exception as e:  # noqa: BLE001
            print(f"[expose_idl_members] WARN: failed on {h}: {e}", file=sys.stderr)
    return n_patched


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv: List[str]) -> int:
    parser = argparse.ArgumentParser(
        description='Expose IDL-generated DDS class members as ROS 2-style public fields.',
    )
    parser.add_argument(
        '--mode', required=True, choices=('fastdds', 'cyclonedds'),
        help='Which generator output to patch.',
    )
    parser.add_argument(
        'roots', nargs='+', type=Path,
        help='One or more directories to walk.',
    )
    args = parser.parse_args(argv)

    total = 0
    for root in args.roots:
        if not root.is_dir():
            print(f"[expose_idl_members] WARN: not a directory: {root}", file=sys.stderr)
            continue
        if args.mode == 'fastdds':
            total += patch_fastdds_dir(root)
        else:
            total += patch_cyclonedds_dir(root)
    print(f"[expose_idl_members] done. patched {total} file(s).")
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
