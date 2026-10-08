"""Build an isolated *client-only* MCP package for OpenShell agent runs.

Only the four bounded broker-client modules are copied into the sandbox.
No motion controller, calibration data, workstation config or SDK checkout is
mounted. This code prepares one-run configuration; it does not operate a robot.
"""

from __future__ import annotations

from pathlib import Path

from .mcp_guidance import GUIDANCE_RESOURCES

MCP_PYTHON = "/opt/soarm101-mcp/bin/python"
MCP_PACKAGE = "/sandbox/soarm101_motion"
MCP_CLIENT_MODULES = (
    "capability_profile.py",
    "mcp_guidance.py",
    "mcp_server.py",
    "robotctl.py",
)
MCP_SERVER_ARGS = '["-m", "soarm101_motion.mcp_server"]'


def client_files(root: Path) -> tuple[tuple[Path, str], ...]:
    """Enumerate a reviewed transport-only subset, never an SDK tree mount."""
    package_init = root / "mcp-client-__init__.py"
    package_init.write_text('"""Bounded robot MCP client, no hardware SDK."""\n', encoding="utf-8")
    source_dir = Path(__file__).parent
    return (
        (package_init, f"{MCP_PACKAGE}/__init__.py"),
        *(
            (source_dir / filename, f"{MCP_PACKAGE}/{filename}")
            for filename in MCP_CLIENT_MODULES
        ),
    )


def config_file(root: Path, *, agent: str, hermes_config: Path | None = None) -> tuple[Path, str]:
    """Generate a one-run MCP configuration without tokens or host paths."""
    if agent == "hermes":
        if hermes_config is None:
            raise ValueError("Hermes MCP integration requires a one-run Hermes config")
        with hermes_config.open("a", encoding="utf-8") as config:
            config.write(
                "\nmcp_servers:\n"
                "  soarm101:\n"
                f'    command: "{MCP_PYTHON}"\n'
                f"    args: {MCP_SERVER_ARGS}\n"
                "    timeout: 125\n"
                "    supports_parallel_tool_calls: false\n"
                "    tools:\n"
                "      resources: true\n"
                "      prompts: true\n"
            )
        return hermes_config, "/sandbox/.hermes/config.yaml"
    if agent == "codex":
        path = root / "codex-mcp-config.toml"
        path.write_text(
            "[mcp_servers.soarm101]\n"
            f'command = "{MCP_PYTHON}"\n'
            f"args = {MCP_SERVER_ARGS}\n"
            "startup_timeout_sec = 30\n"
            "tool_timeout_sec = 125\n",
            encoding="utf-8",
        )
        return path, "/sandbox/.codex/config.toml"
    raise ValueError(f"native MCP sandbox integration is not yet supported for {agent!r}")


def skill_text(*, agent: str) -> str:
    """Use the canonical guidance resources, with only image-viewer advice per harness."""
    if agent not in {"codex", "hermes"}:
        raise ValueError(f"unknown built-in MCP agent {agent!r}")
    viewer = "Codex's MCP image inspection" if agent == "codex" else "Hermes MCP image content"
    sections = "\n\n".join(GUIDANCE_RESOURCES.values())
    return (
        "---\n"
        f"name: soarm101-openshell-{agent}-mcp\n"
        "description: Use only the bounded MCP server to inspect and operate the SO-ARM101.\n"
        "---\n\n"
        "# Isolated SO-ARM101 MCP agent\n\n"
        "Use the discovered soarm101 MCP tools and MCP resources, not shell commands "
        "or robotctl.py, for robot/camera actions. You must not access arbitrary "
        "robot hardware, raw SDK methods, calibration, host resources or a second "
        "control plane. Tool discovery is profile-constrained at the trusted broker.\n\n"
        "Read the available MCP resources and discover robot_capabilities and "
        "robot_state before acting; if a tool is unavailable, do not attempt to "
        "work around it. Camera frames are returned as image content, and must be "
        f"inspected with {viewer} before making visual claims. If this harness "
        "cannot inspect the pixels, report that limitation rather than guessing.\n\n"
        "The broker returns actionable errors; preserve those reasons. An "
        "observation-only task must never request physical action even if the "
        "human arm authority is active. Any requested output files in TASK.md "
        "remain mandatory.\n\n"
        + sections + "\n"
    )
