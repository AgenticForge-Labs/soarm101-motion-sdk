"""Canonical OpenShell environment for SO-ARM101 reasoning agents.

This module deliberately stays robot-specific. OpenShell owns process/filesystem/network
isolation; the Motion SDK owns the bounded broker, robot capability contract, skill, and
hardware safety. No benchmark framework is required.
"""

from __future__ import annotations

import hashlib
import json
import os
import secrets
import shutil
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping, Sequence

from .agent_adapters import HERMES
from .agent_mcp_runtime import MCP_PYTHON, client_files as mcp_client_files
from .agent_mcp_runtime import config_file as mcp_config_file, skill_text as mcp_skill_text
from .capability_profile import CapabilityProfile
from .agent_adapters import get_agent_adapter
from .agent_adapters import resolve_agent_adapter


DEFAULT_AGENT = "hermes"
DEFAULT_IMAGE = HERMES.image
DEFAULT_PROVIDER = HERMES.provider
DEFAULT_MODEL = HERMES.default_model or ""
DEFAULT_BROKER_HOST = "0.0.0.0"
DEFAULT_BROKER_CLIENT_HOST = "host.openshell.internal"
DEFAULT_BROKER_PORT = 8765
OPENSHELL_SANDBOX_NAME_MAX_LENGTH = 19


class AgentSandboxError(RuntimeError):
    """OpenShell/Hermes sandbox setup or execution failed."""


def _sandbox_name(agent_name: str) -> str:
    """Return a collision-resistant OpenShell name within the server's 19-char limit."""

    short_agent = str(agent_name).strip().lower()[:5] or "agent"
    name = f"s101-{short_agent}-{secrets.token_hex(4)}"
    if len(name) > OPENSHELL_SANDBOX_NAME_MAX_LENGTH:  # defensive invariant
        raise AgentSandboxError(
            f"generated OpenShell sandbox name exceeds "
            f"{OPENSHELL_SANDBOX_NAME_MAX_LENGTH} characters"
        )
    return name


def _sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _read_broker_events(path: Path) -> list[dict[str, object]]:
    if not path.is_file():
        return []
    events: list[dict[str, object]] = []
    for line_number, raw in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        if not raw.strip():
            continue
        try:
            payload = json.loads(raw)
        except json.JSONDecodeError as exc:
            raise AgentSandboxError(
                f"broker event log contains invalid JSON at line {line_number}"
            ) from exc
        if not isinstance(payload, dict):
            raise AgentSandboxError(
                f"broker event log line {line_number} is not a JSON object"
            )
        events.append(payload)
    return events


def _validate_read_only_evidence(
    *,
    capabilities: Mapping[str, object],
    broker_events: Path,
    event_offset: int,
    output_dir: Path,
    agent_exit_code: int,
) -> dict[str, object]:
    events = _read_broker_events(broker_events)[event_offset:]
    successful = [event for event in events if event.get("ok") is True]
    actions = [
        str(event.get("action") or "")
        for event in successful
        if str(event.get("action") or "")
    ]

    forbidden_names = {"go_pose", "joint", "jog", "gripper", "sleep", "stop"}
    forbidden_observed = sorted({action for action in actions if action in forbidden_names})
    missing_actions = [
        action for action in ("capabilities", "state") if action not in actions
    ]

    raw_cameras = capabilities.get("cameras", [])
    required_cameras = (
        [str(camera) for camera in raw_cameras if str(camera)]
        if isinstance(raw_cameras, list)
        else []
    )

    capture_sha256: dict[str, list[str]] = {camera: [] for camera in required_cameras}
    for event in successful:
        if event.get("action") != "capture":
            continue
        request = event.get("request")
        result = event.get("result")
        if not isinstance(request, dict) or not isinstance(result, dict):
            continue
        camera = str(request.get("camera") or "")
        digest = str(result.get("sha256") or "")
        if camera in capture_sha256 and digest:
            capture_sha256[camera].append(digest)

    missing_capture_cameras = sorted(
        camera for camera, digests in capture_sha256.items() if not digests
    )

    observation_dir = output_dir / "observations"
    observation_sha256: dict[str, str] = {}
    if observation_dir.is_dir():
        for path in sorted(observation_dir.rglob("*")):
            if path.is_file():
                observation_sha256[str(path.relative_to(output_dir))] = _sha256_path(path)
    downloaded_hashes = set(observation_sha256.values())
    unverified_capture_cameras = sorted(
        camera
        for camera, digests in capture_sha256.items()
        if digests and not any(digest in downloaded_hashes for digest in digests)
    )

    passed = (
        agent_exit_code == 0
        and not missing_actions
        and not missing_capture_cameras
        and not unverified_capture_cameras
        and not forbidden_observed
    )
    validation: dict[str, object] = {
        "schema_version": 1,
        "status": "passed" if passed else "failed",
        "agent_exit_code": int(agent_exit_code),
        "required_cameras": required_cameras,
        "successful_actions": actions,
        "missing_actions": missing_actions,
        "missing_capture_cameras": missing_capture_cameras,
        "unverified_capture_cameras": unverified_capture_cameras,
        "forbidden_actions_observed": forbidden_observed,
        "capture_sha256": capture_sha256,
        "observation_sha256": observation_sha256,
    }
    validation_path = output_dir / "read-only-validation.json"
    validation_path.write_text(json.dumps(validation, indent=2) + "\n", encoding="utf-8")

    if not passed:
        raise AgentSandboxError(
            "read-only sandbox validation failed; see "
            f"{validation_path}: "
            f"exit={agent_exit_code}, missing_actions={missing_actions}, "
            f"missing_capture_cameras={missing_capture_cameras}, "
            f"unverified_capture_cameras={unverified_capture_cameras}, "
            f"forbidden_actions_observed={forbidden_observed}"
        )
    return validation



def _retain_mcp_capture_evidence(
    *,
    broker_events: Path,
    event_offset: int,
    output_dir: Path,
) -> dict[str, str]:
    """Retain SHA-checked host camera evidence when MCP returns pixels in-band.

    Unlike robotctl, MCP provides images directly to the reasoning model without
    first saving them in the sandbox observation folder. Use the broker's trusted
    capture_evidence record, not a model-generated filename, to preserve frames.
    """
    saved: dict[str, str] = {}
    for event in _read_broker_events(broker_events)[event_offset:]:
        if event.get("action") != "capture_evidence" or event.get("ok") is not True:
            continue
        record = event.get("result")
        if not isinstance(record, dict):
            continue
        camera = str(record.get("name") or "")
        digest = str(record.get("sha256") or "")
        host_path = str(record.get("host_path") or "")
        request_id = str(event.get("request_id") or "")
        if not (camera in {"overhead", "wrist"} and len(digest) == 64
                and all(c in "0123456789abcdef" for c in digest)
                and len(request_id) >= 12
                and all(c in "0123456789abcdef" for c in request_id)):
            raise AgentSandboxError("broker capture evidence is incomplete or invalid")
        raw = Path(host_path).read_bytes()
        if hashlib.sha256(raw).hexdigest() != digest:
            raise AgentSandboxError("broker capture evidence image SHA-256 mismatch")
        relative = Path("observations") / "mcp" / f"{camera}-{request_id[:16]}.jpg"
        destination = output_dir / relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(raw)
        saved[str(relative)] = digest
    return saved


def dockerfile_text(*, agent: str = DEFAULT_AGENT) -> str:
    return get_agent_adapter(agent).dockerfile_text()


def provider_profile_text(
    *,
    agent: str = DEFAULT_AGENT,
    auth: str | None = None,
) -> str:
    return get_agent_adapter(agent).provider_profile_text(auth)


def skill_text(*, agent: str = DEFAULT_AGENT) -> str:
    return get_agent_adapter(agent).skill_text()


def read_only_validation_task_text(*, interface: str = "robotctl") -> str:
    """Interface-specific read-only task; never imply physical authority."""
    if interface == "mcp":
        return """Validate the isolated SO-ARM101 agent environment without commanding physical motion.

1. Read SKILL.md and discover the available SO-ARM101 MCP resources and tools.
2. Use MCP robot_capabilities and robot_state to inspect available robot state.
3. An unarmed motion lease is expected: do not attempt to arm or use motion tools.
4. Capture every configured, permitted camera with the MCP capture_camera tool.
5. Inspect the actual MCP image content and record the broker request IDs and image hashes.
   If this harness cannot inspect MCP pixels, report that limitation explicitly.
6. Do not request any saved-pose, joint, Cartesian, gripper, Sleep or STOP action.
7. Report current state, available tools, configured cameras and which fresh images
   were inspected. Trusted image files are retained on the host from broker evidence.
"""
    if interface != "robotctl":
        raise ValueError("unknown read-only agent interface")
    from importlib import resources

    return resources.files("soarm101_motion.agent_assets").joinpath(
        "TASK.read-only.md"
    ).read_text(encoding="utf-8")


def host_codex_auth_path() -> Path:
    """Return the current user's ordinary Codex auth.json location."""
    codex_home = os.environ.get("CODEX_HOME", "").strip()
    root = Path(codex_home).expanduser() if codex_home else Path.home() / ".codex"
    return root / "auth.json"


def codex_chatgpt_state_dir() -> Path:
    xdg_state = os.environ.get("XDG_STATE_HOME", "").strip()
    root = Path(xdg_state).expanduser() if xdg_state else Path.home() / ".local" / "state"
    return root / "soarm101" / "codex-chatgpt"


def codex_chatgpt_auth_path() -> Path:
    return codex_chatgpt_state_dir() / "auth.json"


def _codex_login_auth_path(auth_name: str) -> Path:
    if auth_name == "installed":
        return host_codex_auth_path()
    if auth_name == "chatgpt":
        return codex_chatgpt_auth_path()
    raise AgentSandboxError(
        f"Codex auth mode {auth_name!r} does not use stored login state"
    )


def _valid_codex_chatgpt_auth(path: Path | None = None) -> bool:
    candidate = codex_chatgpt_auth_path() if path is None else path
    if not candidate.is_file():
        return False
    try:
        payload = json.loads(candidate.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return False
    if not isinstance(payload, dict):
        return False
    return payload.get("auth_mode") == "chatgpt" and isinstance(payload.get("tokens"), dict)


def broker_policy_text(
    *,
    port: int = DEFAULT_BROKER_PORT,
    read_only: bool = False,
    agent: str = DEFAULT_AGENT,
    auth: str | None = None,
    adapter_manifest: Path | None = None,
    interface: str = "robotctl",
) -> str:
    adapter = resolve_agent_adapter(agent, adapter_manifest)
    if interface not in {"robotctl", "mcp"}:
        raise ValueError(f"unsupported robot interface {interface!r}")
    read_only_rules = (
        ("GET", "/v1/health"),
        ("GET", "/v1/capabilities"),
        ("GET", "/v1/state"),
        ("POST", "/v1/capture"),
    )
    motion_rules = (
        ("POST", "/v1/go-pose"),
        ("POST", "/v1/joint"),
        ("POST", "/v1/jog"),
        ("POST", "/v1/gripper"),
        ("POST", "/v1/sleep"),
        ("POST", "/v1/stop"),
    )
    rules = read_only_rules if read_only else read_only_rules + motion_rules
    if interface == "mcp":
        rules += (("GET", "/v1/profile"),)
        if not read_only:
            rules += (("POST", "/v1/sleep-up"),)
    lines = [
        "version: 1",
        "filesystem_policy:",
        "  include_workdir: true",
        "  read_only:",
    ]
    lines.extend(f"    - {path}" for path in adapter.read_only_paths)
    if interface == "mcp":
        # Network binary permission only authorizes executing the interpreter.
        # Landlock also needs read access to its pyvenv.cfg and site-packages;
        # otherwise Python fails before the MCP server can even initialize.
        # Scope this to the fixed SDK-owned client-only venv in MCP runs.
        lines.append("    - /opt/soarm101-mcp")
    lines.extend(
        (
            "  read_write:",
            "    - /tmp",
            "landlock:",
            "  compatibility: hard_requirement",
            "process:",
            f'  run_as_user: "{adapter.run_user}"',
            f'  run_as_group: "{adapter.run_group}"',
            "network_policies:",
            "  soarm101_robot_broker:",
            "    name: soarm101_robot_broker",
            "    endpoints:",
            f"      - host: {DEFAULT_BROKER_CLIENT_HOST}",
            f"        port: {int(port)}",
            "        protocol: rest",
            "        enforcement: enforce",
            "        rules:",
        )
    )
    for method, path in rules:
        lines.extend(
            (
                "          - allow:",
                f"              method: {method}",
                f"              path: {path}",
            )
        )
    lines.append("    binaries:")
    lines.extend(
        f"      - path: {binary}" for binary in adapter.robot_client_binaries
    )
    if interface == "mcp":
        lines.extend(("      - path: " + MCP_PYTHON, "      - path: /opt/soarm101-mcp/bin/python3"))
    lines.extend(adapter.extra_network_policy_lines(auth=auth))
    return "\n".join(lines) + "\n"


@dataclass(frozen=True)
class DoctorResult:
    openshell: bool
    gateway: bool
    docker: bool
    image: bool
    provider: bool
    details: dict[str, str]
    agent: str = DEFAULT_AGENT
    auth: str | None = None

    @property
    def ready(self) -> bool:
        return all((self.openshell, self.gateway, self.docker, self.image, self.provider))

    def as_dict(self) -> dict[str, object]:
        return {
            "ready": self.ready,
            "agent": self.agent,
            "auth": self.auth,
            "openshell": self.openshell,
            "gateway": self.gateway,
            "docker": self.docker,
            "image": self.image,
            "provider": self.provider,
            "details": dict(self.details),
        }


class OpenShellClient:
    def __init__(self, executable: str = "openshell") -> None:
        self.executable = executable

    def available(self) -> bool:
        return shutil.which(self.executable) is not None

    def run(
        self,
        args: Sequence[str],
        *,
        check: bool = True,
        timeout: int = 120,
        env: Mapping[str, str] | None = None,
    ) -> subprocess.CompletedProcess[str]:
        process_env = None if env is None else {**os.environ, **dict(env)}
        completed = subprocess.run(
            [self.executable, *args],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout,
            env=process_env,
            check=False,
        )
        if check and completed.returncode != 0:
            raise AgentSandboxError(
                "OpenShell command failed: "
                + " ".join([self.executable, *args])
                + "\nstdout:\n"
                + completed.stdout
                + "\nstderr:\n"
                + completed.stderr
            )
        return completed

    def create(
        self,
        *,
        name: str,
        image: str,
        policy: Path,
        provider: str | None,
    ) -> dict[str, object]:
        args = [
            "sandbox",
            "create",
            "--name",
            name,
            "--from",
            image,
            "--policy",
            str(policy),
        ]
        if provider:
            args.extend(("--provider", provider))
        args.extend(
            (
                "--no-auto-providers",
                "--detach",
                "--output",
                "json",
                "--",
                "/bin/sh",
                "-lc",
                "while :; do sleep 3600; done",
            )
        )
        completed = self.run(
            args,
            timeout=300,
        )
        payload = json.loads(completed.stdout)
        if not isinstance(payload, dict):
            raise AgentSandboxError("OpenShell sandbox create JSON root must be an object")
        return payload

    def upload(self, name: str, source: Path, destination: str | None = None) -> None:
        # These are individually allowlisted files chosen by deterministic SDK code;
        # repository .gitignore rules must not silently remove one from the upload.
        args = ["sandbox", "upload", name, str(source)]
        if destination:
            args.append(destination)
        args.append("--no-git-ignore")
        self.run(args, timeout=300)

    def effective_policy(self, name: str) -> str:
        completed = self.run(
            ["sandbox", "get", name, "--policy-only"],
            timeout=60,
        )
        return completed.stdout

    def logs(self, name: str, *, since: str = "2h") -> str:
        completed = self.run(
            ["logs", name, "--level", "info", "--since", since],
            check=False,
            timeout=60,
        )
        return (completed.stdout or "") + (completed.stderr or "")

    def exec(
        self,
        name: str,
        command: Sequence[str],
        *,
        workdir: str = "/sandbox",
        env: Mapping[str, str] | None = None,
        timeout: int = 1800,
    ) -> subprocess.CompletedProcess[str]:
        args = [
            "sandbox",
            "exec",
            "--name",
            name,
            "--timeout",
            str(timeout),
            "--no-login-shell",
            "--workdir",
            workdir,
        ]
        for key, value in sorted((env or {}).items()):
            args.extend(("--env", f"{key}={value}"))
        args.extend(("--", *command))
        return self.run(args, check=False, timeout=timeout + 30)

    def download(self, name: str, sandbox_path: str, destination: Path) -> None:
        destination.parent.mkdir(parents=True, exist_ok=True)
        self.run(
            ["sandbox", "download", name, sandbox_path, str(destination)],
            timeout=300,
        )

    def delete(self, name: str, *, timeout: int = 120) -> None:
        completed = self.run(["sandbox", "delete", name], check=False, timeout=timeout)
        if completed.returncode != 0:
            combined = (completed.stdout + "\n" + completed.stderr).lower()
            if "not found" in combined or "not_found" in combined:
                return
            raise AgentSandboxError(completed.stderr or completed.stdout)

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            probe = self.run(
                ["sandbox", "get", name, "--output", "json"],
                check=False,
                timeout=min(30, timeout),
            )
            if probe.returncode != 0:
                combined = (probe.stdout + "\n" + probe.stderr).lower()
                if "not found" in combined or "not_found" in combined:
                    return
                raise AgentSandboxError(probe.stderr or probe.stdout)
            time.sleep(0.5)
        raise AgentSandboxError(f"sandbox {name!r} still exists after {timeout}s")


class BrokerProcess:
    def __init__(
        self,
        *,
        port: int,
        token: str,
        event_path: Path,
        profile_path: Path | None = None,
    ) -> None:
        self.profile_path = profile_path
        self.port = int(port)
        self.token = token
        self.event_path = event_path
        self.process: subprocess.Popen[str] | None = None

    def start(self) -> None:
        env = os.environ.copy()
        env["SOARM101_BROKER_TOKEN"] = self.token
        self.event_path.parent.mkdir(parents=True, exist_ok=True)
        command = [
            sys.executable, "-m", "soarm101_motion.broker",
            "--host", DEFAULT_BROKER_HOST, "--port", str(self.port),
            "--events", str(self.event_path),
        ]
        if self.profile_path is not None:
            command.extend(["--profile", str(self.profile_path)])
        self.process = subprocess.Popen(
            command,
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                out, err = self.process.communicate(timeout=1)
                raise AgentSandboxError(
                    "robot broker exited during startup\n"
                    f"stdout:\n{out}\nstderr:\n{err}"
                )
            try:
                self.request("GET", "/v1/health")
                return
            except Exception:
                time.sleep(0.2)
        raise AgentSandboxError("robot broker did not become ready within 15 seconds")

    def request(
        self,
        method: str,
        path: str,
        payload: Mapping[str, object] | None = None,
    ) -> dict[str, object]:
        data = None if payload is None else json.dumps(dict(payload)).encode("utf-8")
        request = urllib.request.Request(
            f"http://127.0.0.1:{self.port}{path}",
            data=data,
            method=method,
            headers={
                "Authorization": f"Bearer {self.token}",
                "Content-Type": "application/json",
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=10.0) as response:
                result = json.loads(response.read().decode("utf-8"))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode("utf-8", errors="replace")
            raise AgentSandboxError(f"broker HTTP {exc.code}: {detail}") from exc
        if not isinstance(result, dict) or not result.get("ok"):
            raise AgentSandboxError(f"broker request failed: {result}")
        return result

    def require_authority(self) -> dict[str, object]:
        payload = self.request("GET", "/v1/capabilities")
        result = payload.get("result")
        if not isinstance(result, dict):
            raise AgentSandboxError("broker capabilities response is missing result")
        authority = result.get("authority")
        if not isinstance(authority, dict) or not authority.get("armed"):
            raise AgentSandboxError(
                "agent authority is not active; a human must run "
                "'soarm101 agent arm --minutes N' before sandboxed control"
            )
        return result

    def stop(self) -> tuple[str, str]:
        if self.process is None:
            return "", ""
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        stdout, stderr = self.process.communicate(timeout=1)
        return stdout or "", stderr or ""


def doctor(
    *,
    agent: str = DEFAULT_AGENT,
    auth: str | None = None,
    image: str | None = None,
    provider: str | None = None,
    adapter_manifest: Path | None = None,
    openshell: OpenShellClient | None = None,
) -> DoctorResult:
    adapter = resolve_agent_adapter(agent, adapter_manifest)
    selected_auth = adapter.auth_mode(auth)
    selected_image = image or adapter.image
    if selected_auth.uses_login_state and provider:
        raise AgentSandboxError(
            "Codex ChatGPT auth uses native Codex login state; --provider is not applicable"
        )
    selected_provider = provider or selected_auth.provider
    client = openshell or OpenShellClient()
    details: dict[str, str] = {
        "image_name": selected_image,
        "provider_name": selected_provider
        or ("<native-codex-login>" if selected_auth.uses_login_state else "<none>"),
        "auth_mode": selected_auth.name,
        "credential_env_vars": ", ".join(selected_auth.credential_env_vars),
    }
    if selected_auth.uses_login_state:
        details["login_state"] = str(_codex_login_auth_path(selected_auth.name))
        details["host_codex_auth"] = str(host_codex_auth_path())

    has_openshell = client.available()
    gateway = False
    if has_openshell:
        result = client.run(["status"], check=False, timeout=30)
        gateway = result.returncode == 0
        details["openshell_status"] = (result.stdout + result.stderr).strip()

    if adapter.managed_setup:
        docker_path = shutil.which("docker")
        has_docker = bool(docker_path)
        if has_docker:
            result = subprocess.run(
                ["docker", "version", "--format", "{{.Server.Version}}"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                timeout=30,
            )
            has_docker = result.returncode == 0
            details["docker"] = (result.stdout + result.stderr).strip()

        has_image = False
        if has_docker:
            result = subprocess.run(
                ["docker", "image", "inspect", selected_image],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
                timeout=30,
            )
            has_image = result.returncode == 0
    else:
        has_docker = True
        has_image = True
        details["docker"] = "not required for external OpenShell adapter"
        details["image"] = "resolved by OpenShell at sandbox creation"

    if selected_auth.uses_login_state:
        login_path = _codex_login_auth_path(selected_auth.name)
        has_provider = _valid_codex_chatgpt_auth(login_path)
        if selected_auth.name == "installed":
            host_codex = shutil.which("codex")
            details["host_codex"] = host_codex or "<not-found>"
            if host_codex is None:
                has_provider = False
                details["provider"] = (
                    "host Codex CLI is not installed or not on PATH; install/login with "
                    "Codex first, then retry --auth installed"
                )
            elif not has_provider:
                details["provider"] = (
                    f"host Codex ChatGPT auth is missing or invalid at {login_path}; "
                    "run 'codex login' in your normal terminal"
                )
        elif not has_provider:
            details["provider"] = (
                "dedicated Codex ChatGPT login state is missing or invalid; run "
                "'soarm101 agent sandbox setup --agent codex --auth chatgpt'"
            )
    else:
        has_provider = selected_provider is None
        if gateway and selected_provider:
            result = client.run(
                ["provider", "get", selected_provider],
                check=False,
                timeout=30,
            )
            has_provider = result.returncode == 0
            if not has_provider:
                details["provider"] = (result.stdout + result.stderr).strip()

    return DoctorResult(
        openshell=has_openshell,
        gateway=gateway,
        docker=has_docker,
        image=has_image,
        provider=has_provider,
        details=details,
        agent=adapter.name,
        auth=selected_auth.name,
    )

def _setup_codex_chatgpt_login(
    *,
    image: str,
    reauth: bool = False,
) -> None:
    """Create dedicated Codex ChatGPT state without using the host Codex login."""
    destination = codex_chatgpt_auth_path()
    if _valid_codex_chatgpt_auth(destination) and not reauth:
        return

    state_parent = destination.parent
    state_parent.mkdir(parents=True, exist_ok=True)
    try:
        state_parent.chmod(0o700)
    except OSError:
        pass

    with tempfile.TemporaryDirectory(
        prefix=".codex-chatgpt-login-",
        dir=state_parent,
    ) as login_tmp:
        login_root = Path(login_tmp)
        try:
            login_root.chmod(0o700)
        except OSError:
            pass
        # Force Codex to persist this dedicated device login as auth.json instead
        # of selecting a host/keyring backend that would not survive the container.
        (login_root / "config.toml").write_text(
            'cli_auth_credentials_store = "file"\n',
            encoding="utf-8",
        )

        command = ["docker", "run", "--rm", "-i"]
        if hasattr(os, "getuid") and hasattr(os, "getgid"):
            command.extend(
                (
                    "--user",
                    f"{os.getuid()}:{os.getgid()}",
                )
            )
        command.extend(
            (
                "-e",
                "CODEX_HOME=/auth",
                "-v",
                f"{login_root}:/auth",
                image,
                "codex",
                "login",
                "--device-auth",
            )
        )
        result = subprocess.run(
            command,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            raise AgentSandboxError(
                "Codex ChatGPT device login failed; the previous dedicated login "
                "state was left unchanged"
            )

        candidate = login_root / "auth.json"
        if not _valid_codex_chatgpt_auth(candidate):
            raise AgentSandboxError(
                "Codex device login completed without a valid ChatGPT auth.json"
            )
        try:
            candidate.chmod(0o600)
        except OSError:
            pass
        staging = state_parent / f".auth.json.{secrets.token_hex(4)}.tmp"
        shutil.copy2(candidate, staging)
        try:
            staging.chmod(0o600)
        except OSError:
            pass
        os.replace(staging, destination)



def setup(
    *,
    agent: str = DEFAULT_AGENT,
    auth: str | None = None,
    image: str | None = None,
    provider: str | None = None,
    reauth: bool = False,
    adapter_manifest: Path | None = None,
    openshell: OpenShellClient | None = None,
) -> None:
    adapter = resolve_agent_adapter(agent, adapter_manifest)
    selected_auth = adapter.auth_mode(auth)
    selected_image = image or adapter.image
    if selected_auth.uses_login_state and provider:
        raise AgentSandboxError(
            "Codex ChatGPT auth uses native Codex login state; --provider is not applicable"
        )
    selected_provider = provider or selected_auth.provider
    client = openshell or OpenShellClient()
    if not client.available():
        raise AgentSandboxError(
            "OpenShell is not installed; install it first from NVIDIA/OpenShell"
        )
    status = client.run(["status"], check=False, timeout=30)
    if status.returncode != 0:
        raise AgentSandboxError(
            "OpenShell gateway is not ready:\n" + status.stdout + status.stderr
        )

    if not adapter.managed_setup:
        if reauth:
            raise AgentSandboxError("--reauth is only available for SDK-managed auth modes")
        if selected_provider:
            existing = client.run(
                ["provider", "get", selected_provider],
                check=False,
                timeout=30,
            )
            if existing.returncode != 0:
                raise AgentSandboxError(
                    f"OpenShell provider {selected_provider!r} is not configured; "
                    "create/attach it with OpenShell before using this external adapter"
                )
        return

    if shutil.which("docker") is None:
        raise AgentSandboxError(
            f"docker is required to build the {adapter.name} sandbox image"
        )

    with tempfile.TemporaryDirectory(
        prefix=f"soarm101-{adapter.name}-setup-"
    ) as tmp:
        root = Path(tmp)
        dockerfile = root / "Dockerfile"
        profile = root / "provider.yaml"
        dockerfile.write_text(adapter.dockerfile_text(), encoding="utf-8")
        if not selected_auth.uses_login_state:
            profile.write_text(
                adapter.provider_profile_text(selected_auth.name),
                encoding="utf-8",
            )

        build = subprocess.run(
            [
                "docker",
                "build",
                "-f",
                str(dockerfile),
                "-t",
                selected_image,
                str(root),
            ],
            text=True,
            check=False,
        )
        if build.returncode != 0:
            raise AgentSandboxError(
                f"failed to build the SO-ARM101 {adapter.name} sandbox image"
            )

        if selected_auth.uses_login_state:
            if selected_auth.name == "installed":
                if reauth:
                    raise AgentSandboxError(
                        "--reauth is not used with --auth installed; run 'codex login' "
                        "in your normal host terminal instead"
                    )
                if shutil.which("codex") is None:
                    raise AgentSandboxError(
                        "host Codex CLI is not installed or not on PATH"
                    )
                installed_auth = host_codex_auth_path()
                if not _valid_codex_chatgpt_auth(installed_auth):
                    raise AgentSandboxError(
                        "host Codex ChatGPT login state is missing or invalid at "
                        f"{installed_auth}; run 'codex login' in your normal terminal"
                    )
                return
            _setup_codex_chatgpt_login(
                image=selected_image,
                reauth=reauth,
            )
            return

        if not selected_auth.provider_profile_id or not selected_provider:
            raise AgentSandboxError(
                f"auth mode {selected_auth.name!r} has no provider configuration"
            )
        # Lint the candidate file itself. OpenShell 0.1.2 treats `--global` lint
        # as a uniqueness check against the installed global profile and rejects an
        # otherwise-valid update when the same custom profile ID already exists.
        client.run(["profile", "lint", "-f", str(profile)], timeout=60)
        existing_profile = client.run(
            [
                "profile",
                "describe",
                selected_auth.provider_profile_id,
                "--global",
            ],
            check=False,
            timeout=30,
        )
        if existing_profile.returncode == 0:
            client.run(
                [
                    "profile",
                    "update",
                    selected_auth.provider_profile_id,
                    "-f",
                    str(profile),
                    "--global",
                ],
                timeout=60,
            )
        else:
            client.run(
                ["profile", "import", "-f", str(profile), "--global"],
                timeout=60,
            )

        existing = client.run(
            ["provider", "get", selected_provider],
            check=False,
            timeout=30,
        )
        if existing.returncode == 0:
            client.run(
                ["provider", "update", selected_provider, "--from-existing"],
                timeout=60,
            )
        else:
            client.run(
                [
                    "provider",
                    "create",
                    "--name",
                    selected_provider,
                    "--type",
                    selected_auth.provider_profile_id,
                    "--global-profile",
                    "--from-existing",
                ],
                timeout=60,
            )


@dataclass(frozen=True)
class RunResult:
    sandbox: str
    exit_code: int
    output_dir: Path
    capabilities: dict[str, object]
    agent: str = DEFAULT_AGENT
    auth: str | None = None
    model: str | None = None
    interface: str = "robotctl"

    def as_dict(self) -> dict[str, object]:
        return {
            "sandbox": self.sandbox,
            "agent": self.agent,
            "auth": self.auth,
            "model": self.model,
            "interface": self.interface,
            "exit_code": self.exit_code,
            "output_dir": str(self.output_dir),
            "capabilities": self.capabilities,
        }


def run_agent(
    *,
    agent: str = DEFAULT_AGENT,
    auth: str | None = None,
    task: Path | None,
    output_dir: Path,
    model: str | None = None,
    image: str | None = None,
    provider: str | None = None,
    broker_port: int = DEFAULT_BROKER_PORT,
    max_turns: int = 100,
    timeout: int = 1800,
    read_only: bool = False,
    interface: str = "robotctl",
    capability_profile: Path | None = None,
    adapter_manifest: Path | None = None,
    openshell: OpenShellClient | None = None,
) -> RunResult:
    adapter = resolve_agent_adapter(agent, adapter_manifest)
    selected_auth = adapter.auth_mode(auth)
    if interface not in {"robotctl", "mcp"}:
        raise AgentSandboxError("interface must be 'robotctl' or 'mcp'")
    if interface == "mcp" and (adapter_manifest is not None or adapter.name not in {"hermes", "codex"}):
        raise AgentSandboxError("MCP currently requires the built-in Hermes or Codex adapter")
    chosen_profile = (
        CapabilityProfile.from_file(capability_profile)
        if capability_profile is not None else CapabilityProfile.full()
    )
    if not {"robot_capabilities", "robot_state"}.issubset(chosen_profile.allowed_tools):
        raise AgentSandboxError("agent sandbox profile must permit capabilities and state")
    if read_only:
        # Read-only is broker-enforced even if the input profile permits motion.
        restricted = chosen_profile.as_dict()
        restricted["name"] = "sandbox-read-only"
        restricted["tools"] = sorted(chosen_profile.allowed_tools.intersection(
            {"robot_health", "robot_capabilities", "robot_state", "capture_camera"}
        ))
        if "capture_camera" not in restricted["tools"]:
            restricted["cameras"] = []
        restricted["limits"] = {}
        chosen_profile = CapabilityProfile.from_document(restricted)
    selected_image = image or adapter.image
    if selected_auth.uses_login_state and provider:
        raise AgentSandboxError(
            "Codex ChatGPT auth uses native Codex login state; --provider is not applicable"
        )
    selected_provider = provider or selected_auth.provider
    selected_model = model or adapter.default_model

    resolved_task: Path | None = None
    if task is not None:
        resolved_task = task.expanduser().resolve()
        if not resolved_task.is_file():
            raise FileNotFoundError(resolved_task)
    elif not read_only:
        raise AgentSandboxError("--task is required for a full-control agent run")

    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise AgentSandboxError(f"output directory must be new or empty: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    client = openshell or OpenShellClient()
    readiness = doctor(
        agent=adapter.name,
        auth=selected_auth.name,
        image=selected_image,
        provider=selected_provider,
        adapter_manifest=adapter_manifest,
        openshell=client,
    )
    if not readiness.ready:
        raise AgentSandboxError(
            "agent sandbox is not ready; run "
            f"'soarm101 agent sandbox doctor --agent {adapter.name} "
            f"--auth {selected_auth.name}' and "
            f"'soarm101 agent sandbox setup --agent {adapter.name} "
            f"--auth {selected_auth.name}' first:\n"
            + json.dumps(readiness.as_dict(), indent=2)
        )

    token = secrets.token_urlsafe(32)
    sandbox = _sandbox_name(adapter.name)
    broker_events = output_dir / "broker-events.jsonl"
    created = False
    capabilities: dict[str, object] = {}
    broker_event_offset = 0

    with tempfile.TemporaryDirectory(
        prefix=f"soarm101-{adapter.name}-run-"
    ) as tmp:
        root = Path(tmp)
        profile_file = root / "broker-profile.json"
        profile_file.write_text(
            json.dumps(chosen_profile.as_dict(), indent=2) + "\n", encoding="utf-8"
        )
        broker_kwargs: dict[str, object] = {
            "port": broker_port, "token": token, "event_path": broker_events
        }
        if read_only or capability_profile is not None or interface == "mcp":
            broker_kwargs["profile_path"] = profile_file
        broker = BrokerProcess(**broker_kwargs)
        policy = root / "policy.yaml"
        skill = root / "SKILL.md"
        task_file = resolved_task
        if task_file is None:
            task_file = root / "TASK.md"
            task_file.write_text(
                read_only_validation_task_text(interface=interface),
                encoding="utf-8",
            )

        policy.write_text(
            broker_policy_text(
                port=broker_port,
                read_only=read_only,
                agent=adapter.name,
                auth=selected_auth.name,
                adapter_manifest=adapter_manifest,
                interface=interface,
            ),
            encoding="utf-8",
        )
        skill.write_text(
            mcp_skill_text(agent=adapter.name) if interface == "mcp" else adapter.skill_text(),
            encoding="utf-8",
        )
        prepared_files = list(
            adapter.prepare_files(
                root,
                max_turns=max_turns,
                auth=selected_auth.name,
            )
        )
        if interface == "mcp":
            if adapter.name == "hermes":
                config_path = root / "hermes-config.yaml"
                if not config_path.is_file():
                    raise AgentSandboxError("Hermes one-run config is not prepared")
                mcp_config_file(root, agent="hermes", hermes_config=config_path)
            else:
                prepared_files.append(mcp_config_file(root, agent="codex"))
            prepared_files.extend(mcp_client_files(root))
        if selected_auth.uses_login_state:
            source_auth = _codex_login_auth_path(selected_auth.name)
            if not _valid_codex_chatgpt_auth(source_auth):
                if selected_auth.name == "installed":
                    raise AgentSandboxError(
                        "host Codex ChatGPT login state is missing or invalid at "
                        f"{source_auth}; run 'codex login' in your normal terminal"
                    )
                raise AgentSandboxError(
                    "dedicated Codex ChatGPT login state is missing or invalid; run "
                    "'soarm101 agent sandbox setup --agent codex --auth chatgpt'"
                )
            run_auth = root / "codex-auth.json"
            shutil.copy2(source_auth, run_auth)
            try:
                run_auth.chmod(0o600)
            except OSError:
                pass
            prepared_files.append((run_auth, "/sandbox/.codex/auth.json"))
        prepared_files = tuple(prepared_files)

        try:
            broker.start()
            if read_only:
                payload = broker.request("GET", "/v1/capabilities")
                result = payload.get("result")
                if not isinstance(result, dict):
                    raise AgentSandboxError(
                        "broker capabilities response is missing result"
                    )
                capabilities = result
                broker_event_offset = len(_read_broker_events(broker_events))
            else:
                capabilities = broker.require_authority()

            create_result = client.create(
                name=sandbox,
                image=selected_image,
                policy=policy,
                provider=selected_provider,
            )
            created = True
            (output_dir / "openshell-policy.yaml").write_text(
                policy.read_text(encoding="utf-8"),
                encoding="utf-8",
            )
            (output_dir / "openshell-effective-policy.yaml").write_text(
                client.effective_policy(sandbox),
                encoding="utf-8",
            )
            (output_dir / "openshell-create.json").write_text(
                json.dumps(create_result, indent=2) + "\n",
                encoding="utf-8",
            )
            version_probe = client.exec(
                sandbox,
                adapter.version_command(),
                timeout=30,
            )
            if version_probe.returncode != 0:
                raise AgentSandboxError(
                    f"{adapter.name} version check failed: "
                    + (version_probe.stderr or version_probe.stdout)
                )
            agent_version = (version_probe.stdout or version_probe.stderr).strip()

            robotctl = Path(__file__).with_name("robotctl.py")
            input_sha256 = {
                "task": _sha256_path(task_file),
                "skill": _sha256_path(skill),
                "robotctl": _sha256_path(robotctl),
                "policy": _sha256_path(policy),
                "broker_profile": _sha256_path(profile_file),
            }
            sensitive_inputs: list[str] = []
            for source, destination in prepared_files:
                if destination in adapter.sensitive_destinations:
                    sensitive_inputs.append(destination)
                    continue
                input_sha256[f"prepared:{destination}"] = _sha256_path(source)

            (output_dir / "run-metadata.json").write_text(
                json.dumps(
                    {
                        "schema_version": 3,
                        "agent": adapter.name,
                        "auth": selected_auth.name,
                        "agent_version": agent_version,
                        "model": selected_model,
                        "image": selected_image,
                        "provider": selected_provider,
                        "read_only": bool(read_only),
                        "interface": interface,
                        "broker_profile": chosen_profile.public(),
                        "input_sha256": input_sha256,
                        "sensitive_inputs": sensitive_inputs,
                    },
                    indent=2,
                )
                + "\n",
                encoding="utf-8",
            )

            prepare = client.exec(
                sandbox,
                [
                    "/bin/mkdir",
                    "-p",
                    *adapter.mutable_directories,
                ],
                timeout=30,
            )
            if prepare.returncode != 0:
                raise AgentSandboxError(prepare.stderr or prepare.stdout)

            client.upload(sandbox, task_file, "/sandbox/TASK.md")
            client.upload(sandbox, skill, "/sandbox/SKILL.md")
            if interface == "robotctl":
                client.upload(sandbox, robotctl, "/sandbox/robotctl.py")
            for source, destination in prepared_files:
                client.upload(sandbox, source, destination)

            prompt = (
                "Read TASK.md and SKILL.md completely before acting. "
                + (
                    "Use only the discovered SO-ARM101 MCP tools for robot/camera actions. "
                    "Inspect MCP image pixels rather than guessing; do not use shell robotctl. "
                    if interface == "mcp"
                    else "Use only robotctl.py for robot/camera actions. "
                )
                + (
                    "For visual claims, inspect the pixels returned in the fresh MCP image "
                    "content using the harness's supported image capability. "
                    if interface == "mcp"
                    else "For visual claims, inspect every relevant fresh capture using the "
                    "agent-specific local image tool described in SKILL.md. "
                )
                + (
                    "This is a hard read-only validation run: do not request any pose, joint, "
                    "Cartesian, gripper, Sleep, or STOP action. The OpenShell policy omits "
                    "those routes. Motion authority is not expected or required for read-only "
                    "observation. Inspect capabilities and state, capture every configured "
                    "camera, inspect fresh images using the method in SKILL.md when available, "
                    "and report results."
                    if read_only
                    else "Complete the task safely and report the result."
                )
            )
            command = adapter.command(
                prompt=prompt,
                model=selected_model,
                max_turns=max_turns,
                auth=selected_auth.name,
                interface=interface,
            )
            environment = {
                **adapter.environment(auth=selected_auth.name),
                "SOARM101_BROKER_URL": (
                    f"http://{DEFAULT_BROKER_CLIENT_HOST}:{int(broker_port)}"
                ),
                "SOARM101_BROKER_TOKEN": token,
            }
            if interface == "mcp":
                # Fail before handing motion authority to the model if the
                # packaged MCP runtime or authenticated profile is unavailable.
                probe = client.exec(
                    sandbox,
                    [
                        MCP_PYTHON, "-c",
                        "from soarm101_motion.mcp_server import create_server, broker_visible_tools; "
                        "create_server(allowed_tools=broker_visible_tools()); "
                        "print('soarm101-mcp-ready')",
                    ],
                    env=environment,
                    timeout=30,
                )
                if probe.returncode != 0 or "soarm101-mcp-ready" not in probe.stdout:
                    # Preserve the actual preflight failure while keeping the
                    # one-run broker token out of CLI output and artifacts.
                    # An installed image is not proof the MCP import/network
                    # handshake succeeds inside an OpenShell sandbox.
                    def diagnostic(value: str) -> str:
                        safe = (value or "").replace(
                            token, "<redacted-soarm101-broker-token>"
                        ).strip()
                        return safe[:2000] or "<empty>"

                    raise AgentSandboxError(
                        "isolated MCP tool handshake failed before Codex started "
                        f"(exit={probe.returncode});\n"
                        f"stdout:\n{diagnostic(probe.stdout)}\n"
                        f"stderr:\n{diagnostic(probe.stderr)}\n"
                        "If the MCP Python executable or dependency is absent, "
                        "rebuild with 'soarm101 agent sandbox setup --agent codex "
                        "--auth installed'. Otherwise inspect the preflight error "
                        "and the retained openshell-effective-policy.yaml."
                    )
            process = client.exec(
                sandbox,
                command,
                env=environment,
                timeout=timeout,
            )

            (output_dir / f"{adapter.name}-stdout.jsonl").write_text(
                (process.stdout or "").replace(token, "<redacted-soarm101-broker-token>"),
                encoding="utf-8",
            )
            (output_dir / f"{adapter.name}-stderr.txt").write_text(
                (process.stderr or "").replace(token, "<redacted-soarm101-broker-token>"),
                encoding="utf-8",
            )

            openshell_logs = client.logs(sandbox).replace(
                token,
                "<redacted-soarm101-broker-token>",
            )
            (output_dir / "openshell-logs.txt").write_text(
                openshell_logs,
                encoding="utf-8",
            )

            downloads: list[tuple[str, Path]] = [
                ("/sandbox/observations", output_dir / "observations"),
            ]
            if not read_only:
                downloads.append(
                    ("/sandbox/task-result.json", output_dir / "task-result.json")
                )
            for source, destination in downloads:
                try:
                    client.download(sandbox, source, destination)
                except Exception as exc:
                    with (output_dir / "download-errors.txt").open(
                        "a",
                        encoding="utf-8",
                    ) as handle:
                        handle.write(f"{source}: {exc}\n")

            if interface == "mcp":
                # Retain the broker's verified camera bytes independently of
                # whether a harness writes its own observation file.
                _retain_mcp_capture_evidence(
                    broker_events=broker_events,
                    event_offset=broker_event_offset,
                    output_dir=output_dir,
                )

            if read_only:
                _validate_read_only_evidence(
                    capabilities=capabilities,
                    broker_events=broker_events,
                    event_offset=broker_event_offset,
                    output_dir=output_dir,
                    agent_exit_code=int(process.returncode),
                )

            return RunResult(
                sandbox=sandbox,
                exit_code=int(process.returncode),
                output_dir=output_dir,
                capabilities=capabilities,
                agent=adapter.name,
                auth=selected_auth.name,
                model=selected_model,
                interface=interface,
            )
        finally:
            active_error = sys.exc_info()[1]
            cleanup_error: Exception | None = None
            if created:
                try:
                    client.delete(sandbox)
                except Exception as exc:
                    cleanup_error = exc
                    (output_dir / "sandbox-cleanup-error.txt").write_text(
                        str(exc) + "\n",
                        encoding="utf-8",
                    )
            broker_stdout, broker_stderr = broker.stop()
            (output_dir / "broker-stdout.txt").write_text(
                broker_stdout,
                encoding="utf-8",
            )
            (output_dir / "broker-stderr.txt").write_text(
                broker_stderr,
                encoding="utf-8",
            )
            if cleanup_error is not None and active_error is None:
                raise AgentSandboxError(
                    f"agent sandbox completed but cleanup failed: {cleanup_error}"
                ) from cleanup_error


def run_hermes(
    *,
    task: Path | None,
    output_dir: Path,
    model: str = DEFAULT_MODEL,
    image: str = DEFAULT_IMAGE,
    provider: str = DEFAULT_PROVIDER,
    broker_port: int = DEFAULT_BROKER_PORT,
    max_turns: int = 100,
    timeout: int = 1800,
    read_only: bool = False,
    openshell: OpenShellClient | None = None,
) -> RunResult:
    """Compatibility wrapper for callers written before pluggable agents."""
    return run_agent(
        agent="hermes",
        task=task,
        output_dir=output_dir,
        model=model,
        image=image,
        provider=provider,
        broker_port=broker_port,
        max_turns=max_turns,
        timeout=timeout,
        read_only=read_only,
        openshell=openshell,
    )
