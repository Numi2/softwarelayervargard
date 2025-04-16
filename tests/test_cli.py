"""Tests for the *vargardctl* CLI helpers.

These tests focus on the *pure‑python* helper functions that do not require
Click to be installed or Docker to be available at runtime.  This makes the
test‑suite lightweight and suitable for constrained CI environments.
"""

from pathlib import Path


import vargard_core.cli as cli


def test_load_sensors_from_yaml(tmp_path: Path):
    """The YAML helper should return a list of sensors defined in the file."""

    cfg = {
        "sensors": [
            {"type": "usb_camera", "device_index": 0},
            {"type": "radar", "port": "/dev/ttyUSB0"},
        ]
    }

    yaml_path = tmp_path / "sensors.yaml"
    yaml_path.write_text(
        """
sensors:
  - type: usb_camera
    device_index: 0
  - type: radar
    port: /dev/ttyUSB0
""",
        encoding="utf-8",
    )

    sensors = cli._load_sensors_from_yaml(yaml_path)  # pylint: disable=protected-access

    assert sensors == cfg["sensors"]


def test_list_plugins_no_entry_points(monkeypatch):
    """list-plugins should gracefully handle missing entry points."""

    class DummyEP:  # noqa: D401 Simple dummy class mimicking pkg_resources EntryPoint
        def __init__(self, name):
            self.name = name
            self.module_name = "dummy.module"
            self.attrs = ["Plugin"]

    # Monkey‑patch *pkg_resources.iter_entry_points* to return a dummy list.
    import pkg_resources

    monkeypatch.setattr(pkg_resources, "iter_entry_points", lambda group: [DummyEP("dummy")])

    # Capture output of list_plugins command
    from io import StringIO
    import sys

    captured = StringIO()
    monkeypatch.setattr(cli, "click", cli.click)  # ensure stub or real click.

    # Monkey‑patch click.echo to capture output instead of printing
    monkeypatch.setattr(cli.click, "echo", lambda msg, **_: captured.write(str(msg)))

    cli.list_plugins.callback() if hasattr(cli.list_plugins, "callback") else cli.list_plugins()  # type: ignore

    captured_value = captured.getvalue()
    assert "dummy" in captured_value
