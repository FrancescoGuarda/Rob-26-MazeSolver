"""Guard: no stray stdout writes in MMS-protocol-critical modules.

MMS communicates over raw stdin/stdout; a stray print() to stdout would desync
the protocol. This bug class is invisible to SimAPI-based tests (which never
touch real stdin/stdout) and only surfaces when running in the real MMS GUI.

``print(..., file=sys.stderr)`` is explicitly allowed: stderr is the intended
channel for diagnostics (wall/replanning events reported by the algorithms),
and it never touches the protocol stream. Everything else -- a bare
``print(...)`` or ``print(..., file=sys.stdout)`` -- is still forbidden.
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

_REPO_ROOT = Path(__file__).resolve().parent.parent
_GUARDED_FILES = [
    *sorted((_REPO_ROOT / "src" / "algorithms").glob("*.py")),
    _REPO_ROOT / "src" / "api" / "mms_api.py",
]


def _is_stderr_only(node: ast.Call) -> bool:
    """True iff *node* is a ``print(..., file=sys.stderr)`` call."""
    for kw in node.keywords:
        if kw.arg == "file":
            return (
                isinstance(kw.value, ast.Attribute)
                and kw.value.attr == "stderr"
                and isinstance(kw.value.value, ast.Name)
                and kw.value.value.id == "sys"
            )
    return False  # no file= kwarg -> defaults to stdout -> still forbidden


def _find_print_calls(source: str) -> list[int]:
    tree = ast.parse(source)
    lines: list[int] = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "print"
            and not _is_stderr_only(node)
        ):
            lines.append(node.lineno)
    return lines


@pytest.mark.parametrize("path", _GUARDED_FILES, ids=lambda p: p.name)
def test_no_print_calls(path: Path) -> None:
    source = path.read_text()
    offending_lines = _find_print_calls(source)
    assert not offending_lines, (
        f"{path.relative_to(_REPO_ROOT)} contains print() call(s) at "
        f"line(s) {offending_lines}, which would corrupt the MMS stdin/stdout protocol"
    )
