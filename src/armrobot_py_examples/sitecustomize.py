"""Workspace-wide Python tweaks.

This ensures that modules which expect ``xml.parsers`` to be available (e.g.
the ROS xacro CLI) can safely reference ``xml.parsers.expat`` even when no XML
parsing has happened yet. Without this, xacro aborts with an AttributeError
while trying to handle unrelated exceptions such as missing files.
"""

from importlib import import_module


def _preload_xml_parsers() -> None:
    """Import xml.parsers.expat so packages can reference it safely."""
    try:
        import_module("xml.parsers.expat")
    except ModuleNotFoundError:
        # If the standard library XML parser is unavailable, keep going.
        pass


_preload_xml_parsers()

