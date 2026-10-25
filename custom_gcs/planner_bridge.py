"""Local GCS adapter to the same planner the drones use to validate a plan onboard.

Planning runs entirely on the laptop. It needs no ROS, no aircraft and no radio.
The GCS never reaches into an AERIXIS core (AS2) checkout: it uses its own copy of the
planner in ``vendor/as2_rescue_planner`` (refresh it with ``tools/sync_planner.sh``), so it
works on a laptop with nothing but this folder. Search order:

1. ``AS2_RESCUE_SRC`` — an explicit development override pointing at a ``rescue_ws/src``.
2. An already-installed package (normal ``import``).
3. ``vendor/`` next to this file.

If none resolve, ``available()`` reports the reason so the UI can disable the
KML workflow honestly instead of failing mid-upload.
"""
import importlib
import os
from pathlib import Path
import sys

PACKAGES = ('as2_rescue_planner',)


def _candidate_roots():
    """Directories that may contain the planner package, most specific first."""
    roots = []
    override = os.environ.get('AS2_RESCUE_SRC', '')
    if override:
        roots.append(Path(override))
    roots.append(Path(__file__).resolve().parent / 'vendor')
    return [root for root in roots if root.is_dir()]


def version():
    """Version stamp of the vendored planner copy ('' when another copy is in use)."""
    stamp = Path(__file__).resolve().parent / 'vendor' / 'as2_rescue_planner' / 'VERSION'
    return stamp.read_text().strip() if stamp.is_file() else ''


def _load():
    try:
        return importlib.import_module('as2_rescue_planner.mission')
    except ImportError as first:
        for root in _candidate_roots():
            for name in PACKAGES:
                # ROS layout is src/<name>/<name>/__init__.py, so the outer
                # <name> directory (which has no __init__.py) is what belongs on
                # sys.path. A flat layout src/<name>/__init__.py is also allowed.
                for entry in (root / name, root):
                    if not (entry / name / '__init__.py').is_file():
                        continue
                    if str(entry) not in sys.path:
                        sys.path.insert(0, str(entry))
            try:
                return importlib.import_module('as2_rescue_planner.mission')
            except ImportError as exc:
                first = exc
            # Do not leave a half-resolved namespace package behind.
            sys.modules.pop('as2_rescue_planner', None)
            sys.modules.pop('as2_rescue_planner.mission', None)
        raise ImportError(
            'Rescue planner not found. Run tools/sync_planner.sh to copy it into vendor/ '
            '(or set AS2_RESCUE_SRC for development). Last error: %s' % first)


def available():
    try:
        _load()
        return True, ''
    except ImportError as exc:
        return False, str(exc)


def plan_mission(request):
    return _load().plan_mission(request)


def features():
    """Feature-spec composer for custom missions (as2_rescue_planner.features)."""
    _load()
    return importlib.import_module('as2_rescue_planner.features')


def trees():
    _load()
    return importlib.import_module('as2_rescue_planner.trees')
