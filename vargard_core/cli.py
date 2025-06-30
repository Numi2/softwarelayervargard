"""vargardctl – Command line utility for operating a Vargard deployment.

This module defines the *vargardctl* CLI which is exposed as a console script
via *setup.py* so that end‑users can simply run the command after installing
the *vargard_core* package.

We rely on the *click* package because it provides an ergonomic interface, has
excellent built‑in help/validation and is already a transitive dependency for
many ROS 2 + Python setups.  The CLI is intentionally thin; the heavy‑lifting
is performed by the Vargard runtime (Docker Compose, the sensor layer, the
ROS 2 nodes, …).  This wrapper focuses on convenience and orchestration.

The following sub‑commands are currently available (Sprint 6 scope):

• list‑sensors  – Displays the sensors defined in *sensors.yaml* (or auto
                  detected via *SensorManager* as fallback).
• list‑plugins  – Enumerates inference plugins discovered through the
                  *vargard.inference_plugins* “entry‑points” group.
• start         – Starts the stack using *docker‑compose up ‑d*.
• stop          – Stops the stack using *docker‑compose down*.
• logs          – Streams the container logs (last 100 lines) via
                  *docker‑compose logs*.

Keeping with the overall design philosophy we try to fail gracefully and
provide actionable error messages instead of stack‑traces.  The helper
functions are separated out so that they can be unit‑tested easily without
invoking Docker – we use *subprocess.run* but only after checking for the
presence of *docker‑compose.yml* to avoid surprises.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import List

# ---------------------------------------------------------------------------
# Optional dependency handling
# ---------------------------------------------------------------------------

try:
    import click  # type: ignore
except ImportError:  # pragma: no cover – fallback when *click* is missing
    # Provide a *very* small subset so that importing this module and running
    # the helper functions still works in environments where *click* is not
    # installed (e.g. CI).  The shims intentionally do **not** try to emulate
    # Click’s runtime – they merely make the decorators no‑ops so that unit
    # tests can import *vargard_core.cli* without pulling the extra
    # dependency.

    def _decorator_stub(*_args, **_kwargs):  # noqa: D401
        def _inner(func):
            return func

        return _inner

    class _ClickStub:  # noqa: D401
        echo = staticmethod(print)

        # Decorators
        group = staticmethod(_decorator_stub)
        command = staticmethod(_decorator_stub)
        option = staticmethod(_decorator_stub)
        version_option = staticmethod(_decorator_stub)

    click = _ClickStub()  # type: ignore

import yaml

# Lazily import sensor manager & pkg_resources because these drag in heavier
# dependencies.  Importing them inside the functions instead of at module load
# time makes the CLI start‑up feel instant.


# ---------------------------------------------------------------------------
# Helper functions
# ---------------------------------------------------------------------------


def _load_sensors_from_yaml(config_path: Path) -> List[dict]:
    """Load sensors from YAML configuration file.

    Args:
        config_path: path to *sensors.yaml*.

    Returns:
        list with sensor dicts.  Empty list if file does not exist.
    """

    if not config_path.exists():
        return []

    try:
        with config_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return list(data.get("sensors", []))
    except (OSError, yaml.YAMLError) as exc:
        click.echo(f"Error reading sensor config {config_path}: {exc}", err=True)
        return []


def _auto_detect_sensors() -> List[object]:
    """Fallback to *SensorManager* auto‑detection.

    We import lazily so that users without OpenCV/pyserial can still use the
    CLI for commands that do not touch sensors.
    """

    try:
        from vargard_sensor_layer.sensor_manager import SensorManager  # noqa

        mgr = SensorManager(config_file=None)
        return mgr.get_sensors()
    except Exception as exc:  # pylint: disable=broad-except
        # Auto detection is *best‑effort* – never crash the CLI because a
        # sensor dependency is missing.
        click.echo(f"Auto‑detection failed: {exc}", err=True)
        return []


def _docker_compose_cmd(*compose_args: str, cwd: Path | None = None) -> int:
    """Run a docker‑compose command in a robust way.

    Args:
        *compose_args: arguments passed directly to *docker compose*.
        cwd: directory in which *docker‑compose.yml* resides (defaults to CWD).

    Returns:
        exit code from the subprocess invocation.
    """

    cwd = cwd or Path.cwd()
    compose_file = cwd / "docker-compose.yml"

    if not compose_file.exists():
        click.echo(f"docker‑compose.yml not found in {cwd}", err=True)
        return 1

    cmd = ["docker", "compose"] + list(compose_args)
    try:
        return subprocess.call(cmd, cwd=cwd)
    except FileNotFoundError:
        click.echo("Error: 'docker' executable not found. Is Docker installed?", err=True)
        return 1


# ---------------------------------------------------------------------------
# Click CLI definition
# ---------------------------------------------------------------------------


@click.group()
@click.version_option(package_name="vargard_core", prog_name="vargardctl")
def cli():  # noqa: D401  (Click commands don't need docstrings)
    """Management CLI for Vargard deployments."""


# ---------------------------------------------------------------------------
# list‑sensors
# ---------------------------------------------------------------------------


@cli.command("list-sensors")
@click.option(
    "--config",
    "config_path",
    default="sensors.yaml",
    type=click.Path(exists=False, dir_okay=False, path_type=Path),
    help="YAML configuration file to read (defaults to ./sensors.yaml).",
)
def list_sensors(config_path: Path):
    """Print sensors declared in *sensors.yaml* or auto‑detected."""

    sensors: List[str] = []

    # 1) Try config file first
    entries = _load_sensors_from_yaml(config_path)
    if entries:
        for idx, s in enumerate(entries, 1):
            stype = s.get("type", "unknown")
            ident_parts = []
            if "device_index" in s:
                ident_parts.append(str(s["device_index"]))
            if "device_path" in s:
                ident_parts.append(str(s["device_path"]))
            if "rtsp_url" in s:
                ident_parts.append(str(s["rtsp_url"]))
            if "port" in s:
                ident_parts.append(str(s["port"]))
            ident = ", ".join(ident_parts) or "n/a"
            sensors.append(f"{idx}. {stype} ({ident})")

    else:
        # 2) Auto detect via SensorManager
        autodetected = _auto_detect_sensors()
        if autodetected:
            for idx, s in enumerate(autodetected, 1):
                sensors.append(f"{idx}. {s.__class__.__name__}")

    if sensors:
        click.echo("\n".join(sensors))
    else:
        click.echo("No sensors found.")


# ---------------------------------------------------------------------------
# list‑plugins
# ---------------------------------------------------------------------------


@cli.command("list-plugins")
def list_plugins():
    """Show available inference plugins discovered via entry‑points."""

    try:
        from importlib.metadata import entry_points

        eps = entry_points(group="vargard.inference_plugins")
    except Exception as exc:  # pragma: no cover  # noqa: E722
        click.echo(f"Failed to load plugins: {exc}", err=True)
        sys.exit(1)

    if not eps:
        click.echo("No inference plugins registered.")
        return

    for ep in eps:
        click.echo(f"- {ep.name}: {ep.value}")


# ---------------------------------------------------------------------------
# start / stop / logs
# ---------------------------------------------------------------------------


@cli.command()
@click.option("--compose-dir", default=".", type=click.Path(file_okay=False, path_type=Path))
def start(compose_dir: Path):
    """Start Vargard stack using Docker Compose (detached)."""

    exit_code = _docker_compose_cmd("up", "-d", cwd=compose_dir)
    sys.exit(exit_code)


@cli.command()
@click.option("--compose-dir", default=".", type=click.Path(file_okay=False, path_type=Path))
def stop(compose_dir: Path):
    """Stop Vargard stack (docker compose down)."""

    exit_code = _docker_compose_cmd("down", cwd=compose_dir)
    sys.exit(exit_code)


@cli.command()
@click.option("--compose-dir", default=".", type=click.Path(file_okay=False, path_type=Path))
@click.option("--follow/--no-follow", default=True, help="Follow (tail -f) the logs.")
@click.option("--lines", "-n", default=100, help="Number of lines to show initially.")
def logs(compose_dir: Path, follow: bool, lines: int):
    """Stream container logs via *docker compose logs*."""

    args = ["logs", f"--tail={lines}"]
    if follow:
        args.append("-f")

    exit_code = _docker_compose_cmd(*args, cwd=compose_dir)
    sys.exit(exit_code)


# ---------------------------------------------------------------------------
# Entrypoint for *python -m vargard_core.cli* (helpful for local testing)
# ---------------------------------------------------------------------------


if __name__ == "__main__":
    cli()
