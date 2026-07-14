"""Guard: no stray print()/stdout writes in MMS-protocol-critical modules.

MMS communicates over raw stdin/stdout; a stray print() would desync the
protocol. This bug class is invisible to SimAPI-based tests (which never
touch real stdin/stdout) and only surfaces when running in the real MMS GUI.
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


def _find_print_calls(source: str) -> list[int]:
    tree = ast.parse(source)
    lines: list[int] = []
    for node in ast.walk(tree):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == "print"
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
