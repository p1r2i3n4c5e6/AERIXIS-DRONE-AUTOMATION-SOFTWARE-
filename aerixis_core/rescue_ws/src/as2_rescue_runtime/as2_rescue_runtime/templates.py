"""Backward-compatible re-export of the planner-owned mission trees.

The tree definitions moved to ``as2_rescue_planner.trees`` so the dependency
graph is acyclic: the runtime and mission manager depend on the planner, never
the reverse. Existing imports of ``as2_rescue_runtime.templates`` keep working.

``templates.mission`` and ``templates.xml`` are re-exported unchanged; there is
no second copy of the tree logic here.
"""
from as2_rescue_planner.trees import MODES, mission, node, xml  # noqa: F401

__all__ = ['MODES', 'mission', 'node', 'xml']
