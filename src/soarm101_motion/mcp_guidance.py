"""Harness-neutral guidance exposed by the optional MCP adapter.

These are semantic instructions only. They never authorize a robot action and
must not duplicate the broker's dynamic state/capability source of truth.
"""

from __future__ import annotations

GUIDANCE_SCHEMA_VERSION = 1

# Sent in MCP initialization. Host support varies, so repository AGENTS.md
# reinforces the same preference for Codex without changing broker authority.
SERVER_INSTRUCTIONS = """For live SO-ARM101 robot tasks, use the connected soarm101 MCP tools
by default for current state, capabilities, permitted cameras, saved poses, Sleep,
and bounded movement. Do not require the user to say "use MCP".
For live images call capture_camera and inspect its fresh returned pixels.
Do not infer camera availability from local /dev/video* discovery, use the host
camera CLI instead, or bypass the MCP broker when it denies a capability.
Read robot_capabilities and robot_state for dynamic configuration and authority.
Only request motion when the user asks, the human-issued lease is active, and
the broker permits it. Never arm, calibrate, or override safety through shell.
If a tool or camera is unavailable, report the restriction instead of falling
back to host hardware commands. Normal SDK CLI use remains appropriate for
explicit software development, testing, and operator-led diagnostics.
"""

CORE_GUIDANCE = """# SO-ARM101 bounded-agent operating guidance

Treat robot_capabilities and robot_state as authoritative runtime evidence.
The MCP server is a client of the authenticated broker; it does not grant
authority, calibration, or unrestricted robot access.

Use an observe -> decide -> smallest useful bounded action -> re-observe loop.
Do not infer physical success from a requested action alone. When success is
visually defined, acquire a fresh permitted camera image and inspect its pixels.

For human workspace directions, read robot_capabilities.world_directions.
The world frame is the SDK model/base frame; model +Z is not automatically
physical up. If calibrated physical-direction mapping is unavailable, do not
invent one. The tool frame rotates with the TCP/gripper.

Treat broker rejections as evidence. Do not route around an expired authority
lease, workspace/path rejection, profile restriction, or unavailable camera.
STOP means software STOP/HOLD, not a hardware emergency stop.
"""

COORDINATE_GUIDANCE = """# Coordinate movement

1. Read robot_capabilities and robot_state first.
2. For right/left, forward/back, or up/down, use the measured
   world_directions.model_delta_mm_per_physical_mm vector reported by
   robot_capabilities. Multiply that vector by the desired physical distance.
3. Use jog_cartesian(frame="world", ...) with the resulting model-frame delta.
4. For corrections intentionally expressed from the current gripper/TCP axes,
   use frame="tool" and small increments.
5. Cartesian agent jogs are translation-only. Do not infer orientation control.
6. Re-read state and re-observe the scene after consequential moves.
"""

JOINT_GUIDANCE = """# Joint-guided movement

Use jog_joint only for a deliberately selected named joint and relative angle.
Prefer small corrections, then re-read state and fresh visual evidence. Joint
motion is not a substitute for an unverified Cartesian claim. A broker/profile
limit is authoritative and must not be bypassed by decomposing a forbidden
request solely to evade policy.
"""

VISUAL_GUIDANCE = """# Visual observation

Use capture_camera only for cameras listed by robot_capabilities/profile.
Every capture is fresh broker evidence with a verified SHA-256. Inspect the
returned image pixels before making visual claims. Prefer overhead for broad
workspace/completion context and wrist for local approach, grasp, clearance,
placement, or inspection when those cameras are available. If the active
harness cannot inspect MCP image content, report that limitation rather than
guessing.
"""

RECOVERY_GUIDANCE = """# Failure and recovery

On an action error, preserve the broker's reason and classify the next step:
- authority absent/expired: observation may continue; a human must re-arm;
- profile/tool/camera denied: use only the advertised permitted surface;
- workspace/path/joint safety rejection: do not route around the guard;
- stale or uncertain scene state: acquire fresh state/camera evidence;
- ambiguous coordinate direction: reread calibrated world_directions;
- repeated/no-progress motion: STOP/HOLD and report the residual uncertainty.

Never arm, calibrate, relax torque, write servo registers, or access host
devices through an alternate path.
"""

GUIDANCE_RESOURCES = {
    "core": CORE_GUIDANCE,
    "coordinates": COORDINATE_GUIDANCE,
    "joints": JOINT_GUIDANCE,
    "vision": VISUAL_GUIDANCE,
    "recovery": RECOVERY_GUIDANCE,
}


def task_prompt(task: str, strategy: str = "auto") -> str:
    """Build a concise harness-neutral task prompt without adding authority."""
    task = task.strip()
    if not task:
        raise ValueError("task must not be empty")
    if strategy not in {"auto", "coordinates", "joints", "observe-only"}:
        raise ValueError("strategy must be auto, coordinates, joints, or observe-only")
    strategy_text = {
        "auto": "Choose among the currently advertised bounded tools from evidence.",
        "coordinates": "Prefer calibrated coordinate motion when motion is required and available.",
        "joints": "Prefer deliberate small named-joint corrections when motion is required and available.",
        "observe-only": "Do not request physical motion; use only read-only observation tools.",
    }[strategy]
    return f"""Perform this SO-ARM101 task: {task}

{strategy_text}
Before acting, read the MCP guidance resource soarm101://guidance/core plus
soarm101://guidance/coordinates or soarm101://guidance/joints when relevant.
Read robot_capabilities and robot_state. Respect the broker's active profile
and human authority state. Use the smallest useful action, re-observe after
consequential changes, and do not claim visible completion without fresh
visual evidence.
"""
