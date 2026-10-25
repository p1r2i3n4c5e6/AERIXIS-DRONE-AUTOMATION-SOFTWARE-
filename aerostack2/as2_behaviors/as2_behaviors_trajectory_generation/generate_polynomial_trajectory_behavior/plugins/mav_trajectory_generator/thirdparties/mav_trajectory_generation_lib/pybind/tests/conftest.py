"""
Pytest bootstrap for local pybind package discovery.

This test suite targets the local build-tree package
``mav_trajectory_generation_py``. In monorepo/overlay workflows, pytest may
run from a higher root and miss the package unless PYTHONPATH is exported.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path


def _prepend_if_dir(path: Path) -> None:
    """Prepend an existing directory to sys.path exactly once."""
    if path.is_dir():
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)


def _has_native_module() -> bool:
    """Return True if package and native extension are both importable."""
    pkg_spec = importlib.util.find_spec("mav_trajectory_generation_py")
    if pkg_spec is None:
        return False
    native_spec = importlib.util.find_spec(
        "mav_trajectory_generation_py._mav_trajectory_generation_bindings"
    )
    return native_spec is not None


def _ensure_local_package_discovery() -> None:
    """Make local ``mav_trajectory_generation_py`` importable for tests."""
    # If a pip-installed package is already available, keep environment untouched.
    if _has_native_module():
        return

    repo_root = Path(__file__).resolve().parents[2]

    build_python = repo_root / "build" / "pybind" / "python"

    # Always prefer the build-tree package because it contains the native .so.
    _prepend_if_dir(build_python)

    if not _has_native_module():
        raise ImportError(
            "Could not import 'mav_trajectory_generation_py'. Build with "
            "BUILD_PYBIND=ON (local tests) or install with pip so that "
            "'mav_trajectory_generation_py._mav_trajectory_generation_bindings' "
            "is available."
        )


_ensure_local_package_discovery()
