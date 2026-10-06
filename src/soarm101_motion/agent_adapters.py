"""Agent-specific adapters for the canonical SO-ARM101 OpenShell runtime.

The robot boundary is intentionally agent-neutral. Adapters own only harness-specific
container, provider/auth, mutable-home, prompt/tool conventions, and command construction.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from importlib import resources
from pathlib import Path
from typing import Mapping


def _asset_text(name: str) -> str:
    return resources.files("soarm101_motion.agent_assets").joinpath(name).read_text(
        encoding="utf-8"
    )


@dataclass(frozen=True)
class AgentAuthMode:
    name: str
    provider: str | None
    provider_profile_id: str | None
    provider_profile_asset: str | None
    credential_env_vars: tuple[str, ...]
    uses_login_state: bool = False

    def profile_text(self) -> str:
        if not self.provider_profile_asset:
            raise ValueError(
                f"auth mode {self.name!r} does not use an OpenShell provider profile"
            )
        return _asset_text(self.provider_profile_asset)


@dataclass(frozen=True)
class AgentAdapter:
    name: str
    image: str
    dockerfile_asset: str
    skill_asset: str
    default_model: str | None
    run_user: str
    run_group: str
    read_only_paths: tuple[str, ...]
    robot_client_binaries: tuple[str, ...]
    mutable_directories: tuple[str, ...]
    auth_modes: tuple[AgentAuthMode, ...]
    default_auth: str
    sensitive_destinations: tuple[str, ...] = ()
    managed_setup: bool = True

    def dockerfile_text(self) -> str:
        return _asset_text(self.dockerfile_asset)

    def skill_text(self) -> str:
        return _asset_text(self.skill_asset)

    def auth_mode(self, name: str | None = None) -> AgentAuthMode:
        key = self.default_auth if name is None else str(name).strip().lower()
        for mode in self.auth_modes:
            if mode.name == key:
                return mode
        choices = ", ".join(mode.name for mode in self.auth_modes)
        raise ValueError(
            f"agent {self.name!r} does not support auth mode {key!r}; "
            f"choose one of {choices}"
        )

    @property
    def provider(self) -> str | None:
        return self.auth_mode().provider

    @property
    def provider_profile_id(self) -> str | None:
        return self.auth_mode().provider_profile_id

    @property
    def credential_env_vars(self) -> tuple[str, ...]:
        return self.auth_mode().credential_env_vars

    def provider_for(self, auth: str | None = None) -> str | None:
        return self.auth_mode(auth).provider

    def provider_profile_id_for(self, auth: str | None = None) -> str | None:
        return self.auth_mode(auth).provider_profile_id

    def provider_profile_text(self, auth: str | None = None) -> str:
        return self.auth_mode(auth).profile_text()

    def credential_env_vars_for(self, auth: str | None = None) -> tuple[str, ...]:
        return self.auth_mode(auth).credential_env_vars

    def auth_names(self) -> tuple[str, ...]:
        return tuple(mode.name for mode in self.auth_modes)

    def prepare_files(
        self,
        root: Path,
        *,
        max_turns: int,
        auth: str | None = None,
    ) -> tuple[tuple[Path, str], ...]:
        del root, max_turns, auth
        return ()

    def environment(self, *, auth: str | None = None) -> dict[str, str]:
        del auth
        return {
            "HOME": "/sandbox/.home",
            "XDG_CONFIG_HOME": "/sandbox/.xdg-config",
        }

    def version_command(self) -> list[str]:
        return [self.name, "--version"]

    def extra_network_policy_lines(
        self,
        *,
        auth: str | None = None,
    ) -> tuple[str, ...]:
        self.auth_mode(auth)
        return ()

    def command(
        self,
        *,
        prompt: str,
        model: str | None,
        max_turns: int,
        auth: str | None = None,
    ) -> list[str]:
        raise NotImplementedError


@dataclass(frozen=True)
class HermesAgentAdapter(AgentAdapter):
    def prepare_files(
        self,
        root: Path,
        *,
        max_turns: int,
        auth: str | None = None,
    ) -> tuple[tuple[Path, str], ...]:
        self.auth_mode(auth)
        config = root / "hermes-config.yaml"
        config.write_text(
            "_config_version: 45\n"
            "plugins:\n  enabled: []\n  disabled: []\n"
            f"agent:\n  max_turns: {int(max_turns)}\n  budget_warning_ratio: null\n"
            "compression:\n  enabled: false\n"
            "auxiliary:\n"
            "  title_generation:\n"
            "    enabled: false\n"
            "    model_upgrade_enabled: false\n"
            "    provider: auto\n"
            '    model: ""\n',
            encoding="utf-8",
        )
        marker = root / ".no-bundled-skills"
        marker.write_text("SO-ARM101 isolated Hermes profile\n", encoding="utf-8")
        return (
            (config, "/sandbox/.hermes/config.yaml"),
            (marker, "/sandbox/.hermes/.no-bundled-skills"),
        )

    def environment(self, *, auth: str | None = None) -> dict[str, str]:
        self.auth_mode(auth)
        return {
            **super().environment(),
            "HERMES_HOME": "/sandbox/.hermes",
            "HERMES_WRITE_SAFE_ROOT": "/sandbox",
            "HERMES_ENABLE_PROJECT_PLUGINS": "0",
        }

    def command(
        self,
        *,
        prompt: str,
        model: str | None,
        max_turns: int,
        auth: str | None = None,
    ) -> list[str]:
        self.auth_mode(auth)
        chosen_model = model or self.default_model
        if not chosen_model:
            raise ValueError("Hermes requires an explicit model")
        return [
            "hermes",
            "chat",
            "--oneshot",
            "-q",
            prompt,
            "--format",
            "stream-json",
            "--max-turns",
            str(int(max_turns)),
            "--ignore-rules",
            "--toolsets",
            "hermes-cli",
            "--provider",
            "openrouter",
            "--model",
            chosen_model,
            "--reasoning",
            "none",
        ]


@dataclass(frozen=True)
class CodexAgentAdapter(AgentAdapter):
    def environment(self, *, auth: str | None = None) -> dict[str, str]:
        self.auth_mode(auth)
        return {
            **super().environment(),
            "CODEX_HOME": "/sandbox/.codex",
        }

    def extra_network_policy_lines(
        self,
        *,
        auth: str | None = None,
    ) -> tuple[str, ...]:
        selected_auth = self.auth_mode(auth)
        if selected_auth.name not in {"installed", "chatgpt"}:
            return ()
        # Normal ChatGPT OAuth is managed by Codex from its one-run auth.json copy.
        # Restrict that CLI binary to the two upstream hosts required for inference
        # and OAuth refresh. Robot traffic remains on the separately inspected broker.
        return (
            "  soarm101_codex_chatgpt:",
            "    name: soarm101_codex_chatgpt",
            "    endpoints:",
            "      - host: chatgpt.com",
            "        port: 443",
            "        protocol: tcp",
            "      - host: auth.openai.com",
            "        port: 443",
            "        protocol: tcp",
            "    binaries:",
            "      - path: /usr/local/bin/codex",
            "      - path: /opt/codex-install/packages/standalone/**/bin/codex",
            "      - path: /opt/codex-install/packages/standalone/**/codex",
        )

    def command(
        self,
        *,
        prompt: str,
        model: str | None,
        max_turns: int,
        auth: str | None = None,
    ) -> list[str]:
        del max_turns
        selected_auth = self.auth_mode(auth)

        command = ["codex"]
        if selected_auth.name == "api-key":
            # API-key mode is credential-mediated by OpenShell. Keep it on
            # HTTPS/SSE so the provider can inspect the REST connection.
            command.extend(
                (
                    "-c",
                    (
                        'model_providers.soarm101_openai_api_key={'
                        'name="OpenAI API",'
                        'base_url="https://api.openai.com/v1",'
                        'env_key="OPENAI_API_KEY",'
                        'wire_api="responses",'
                        'requires_openai_auth=false,'
                        'supports_websockets=false'
                        "}"
                    ),
                    "-c",
                    'model_provider="soarm101_openai_api_key"',
                )
            )
        elif selected_auth.name in {"installed", "chatgpt"}:
            # Login-backed Codex modes use Codex's native first-party provider.
            # OpenShell constrains the outbound destinations; the SDK does not
            # duplicate Codex's backend-routing configuration.
            pass
        else:  # pragma: no cover - auth_mode already rejects unknown values.
            raise ValueError(selected_auth.name)

        command.extend(
            (
                "exec",
                "--ephemeral",
                "--ignore-user-config",
                "--ignore-rules",
                "--skip-git-repo-check",
                "--dangerously-bypass-approvals-and-sandbox",
                "--json",
                "--color",
                "never",
                "--cd",
                "/sandbox",
            )
        )
        chosen_model = model or self.default_model
        if chosen_model:
            command.extend(("--model", chosen_model))
        command.append(prompt)
        return command



_MANIFEST_NAME_RE = re.compile(r"^[a-z0-9][a-z0-9._-]{0,63}$")
_MANIFEST_PLACEHOLDERS = {
    "{prompt}",
    "{model}",
    "{max_turns}",
    "{task_path}",
    "{skill_path}",
}
_MANIFEST_SHELL_EXECUTABLES = {
    "sh",
    "bash",
    "dash",
    "zsh",
    "fish",
    "pwsh",
    "powershell",
    "cmd",
    "cmd.exe",
}
_MANIFEST_RESERVED_ENVIRONMENT = {
    "SOARM101_BROKER_URL",
    "SOARM101_BROKER_TOKEN",
}


def _manifest_string_list(
    payload: Mapping[str, object],
    key: str,
    *,
    required: bool = False,
) -> tuple[str, ...]:
    raw = payload.get(key)
    if raw is None:
        if required:
            raise ValueError(f"adapter manifest field {key!r} is required")
        return ()
    if not isinstance(raw, list) or not all(isinstance(item, str) and item for item in raw):
        raise ValueError(f"adapter manifest field {key!r} must be a list of non-empty strings")
    return tuple(raw)


def _validate_sandbox_path(path: str, *, field: str) -> str:
    candidate = str(path).strip()
    if not candidate.startswith("/") or "/../" in candidate or candidate.endswith("/.."):
        raise ValueError(f"adapter manifest {field} entries must be absolute sandbox paths")
    return candidate


@dataclass(frozen=True)
class ManifestAgentAdapter(AgentAdapter):
    """Declarative adapter for an arbitrary CLI harness already supported by OpenShell."""

    command_argv: tuple[str, ...] = ()
    version_argv: tuple[str, ...] = ()
    static_environment: tuple[tuple[str, str], ...] = ()
    skill_body: str | None = None

    def dockerfile_text(self) -> str:
        raise ValueError(
            f"external adapter {self.name!r} uses an OpenShell image directly and has no "
            "SDK-managed Dockerfile"
        )

    def skill_text(self) -> str:
        return self.skill_body or _asset_text("robot-camera-generic-skill.md")

    def environment(self, *, auth: str | None = None) -> dict[str, str]:
        self.auth_mode(auth)
        return {
            **super().environment(),
            **dict(self.static_environment),
        }

    def version_command(self) -> list[str]:
        return list(self.version_argv)

    def command(
        self,
        *,
        prompt: str,
        model: str | None,
        max_turns: int,
        auth: str | None = None,
    ) -> list[str]:
        self.auth_mode(auth)
        values: dict[str, str | None] = {
            "{prompt}": prompt,
            "{model}": model or self.default_model,
            "{max_turns}": str(int(max_turns)),
            "{task_path}": "/sandbox/TASK.md",
            "{skill_path}": "/sandbox/SKILL.md",
        }
        command: list[str] = []
        for token in self.command_argv:
            if token in _MANIFEST_PLACEHOLDERS:
                value = values[token]
                if value is None:
                    raise ValueError(
                        f"external adapter {self.name!r} command requires {token} "
                        "but no value was supplied"
                    )
                command.append(value)
            elif "{" in token or "}" in token:
                raise ValueError(
                    "adapter manifest command placeholders must occupy a complete argv token"
                )
            else:
                command.append(token)
        return command


def load_agent_adapter_manifest(path: str | Path) -> ManifestAgentAdapter:
    """Load one external OpenShell harness adapter from a deterministic JSON manifest."""

    manifest_path = Path(path).expanduser().resolve()
    try:
        payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValueError(f"cannot read adapter manifest {manifest_path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"adapter manifest is not valid JSON: {exc}") from exc
    if not isinstance(payload, dict):
        raise ValueError("adapter manifest JSON root must be an object")
    if payload.get("schema_version") != 1:
        raise ValueError("adapter manifest schema_version must be 1")

    name = str(payload.get("name", "")).strip().lower()
    if not _MANIFEST_NAME_RE.fullmatch(name):
        raise ValueError(
            "adapter manifest name must match [a-z0-9][a-z0-9._-]{0,63}"
        )
    image = str(payload.get("image", "")).strip()
    if not image:
        raise ValueError("adapter manifest field 'image' is required")

    command_argv = _manifest_string_list(payload, "command", required=True)
    if "{prompt}" not in command_argv:
        raise ValueError("adapter manifest command must include a standalone {prompt} token")
    for token in command_argv:
        if ("{" in token or "}" in token) and token not in _MANIFEST_PLACEHOLDERS:
            raise ValueError(f"unsupported adapter manifest command placeholder {token!r}")
    if Path(command_argv[0]).name.lower() in _MANIFEST_SHELL_EXECUTABLES:
        raise ValueError(
            "adapter manifest command must invoke the agent executable directly, not a shell"
        )

    version_argv = _manifest_string_list(payload, "version_command", required=True)
    if Path(version_argv[0]).name.lower() in _MANIFEST_SHELL_EXECUTABLES:
        raise ValueError(
            "adapter manifest version_command must invoke the agent executable directly, "
            "not a shell"
        )
    robot_binaries = tuple(
        _validate_sandbox_path(value, field="robot_client_binaries")
        for value in _manifest_string_list(payload, "robot_client_binaries", required=True)
    )
    read_only_paths = tuple(
        _validate_sandbox_path(value, field="read_only_paths")
        for value in _manifest_string_list(payload, "read_only_paths")
    )
    mutable_directories_raw = _manifest_string_list(payload, "mutable_directories")
    mutable_directories = tuple(
        _validate_sandbox_path(value, field="mutable_directories")
        for value in (
            mutable_directories_raw
            or (
                "/sandbox/.home",
                "/sandbox/.xdg-config",
                "/sandbox/observations",
            )
        )
    )

    raw_environment = payload.get("environment", {})
    if not isinstance(raw_environment, dict) or not all(
        isinstance(key, str) and key and isinstance(value, str)
        for key, value in raw_environment.items()
    ):
        raise ValueError("adapter manifest environment must map non-empty strings to strings")
    reserved_environment = sorted(
        key for key in raw_environment if key in _MANIFEST_RESERVED_ENVIRONMENT
    )
    if reserved_environment:
        raise ValueError(
            "adapter manifest environment may not override broker-controlled variables: "
            + ", ".join(reserved_environment)
        )

    provider_raw = payload.get("provider")
    provider = None if provider_raw is None else str(provider_raw).strip() or None
    default_model_raw = payload.get("default_model")
    default_model = (
        None if default_model_raw is None else str(default_model_raw).strip() or None
    )

    skill_body: str | None = None
    skill_file_raw = payload.get("skill_file")
    if skill_file_raw is not None:
        skill_relative = Path(str(skill_file_raw))
        if skill_relative.is_absolute():
            raise ValueError("adapter manifest skill_file must be relative to the manifest")
        skill_root = manifest_path.parent.resolve()
        skill_path = (skill_root / skill_relative).resolve()
        if not skill_path.is_relative_to(skill_root):
            raise ValueError("adapter manifest skill_file may not escape the manifest directory")
        try:
            skill_body = skill_path.read_text(encoding="utf-8")
        except OSError as exc:
            raise ValueError(f"cannot read adapter skill file {skill_path}: {exc}") from exc

    auth_mode = AgentAuthMode(
        name="external",
        provider=provider,
        provider_profile_id=None,
        provider_profile_asset=None,
        credential_env_vars=(),
    )
    return ManifestAgentAdapter(
        name=name,
        image=image,
        dockerfile_asset="",
        skill_asset="robot-camera-generic-skill.md",
        default_model=default_model,
        run_user=str(payload.get("run_user", "10000")),
        run_group=str(payload.get("run_group", "10000")),
        read_only_paths=read_only_paths,
        robot_client_binaries=robot_binaries,
        mutable_directories=mutable_directories,
        auth_modes=(auth_mode,),
        default_auth="external",
        managed_setup=False,
        command_argv=command_argv,
        version_argv=version_argv,
        static_environment=tuple(sorted(raw_environment.items())),
        skill_body=skill_body,
    )


def resolve_agent_adapter(
    name: str,
    manifest_path: str | Path | None = None,
) -> AgentAdapter:
    """Resolve a built-in adapter or an explicit external OpenShell adapter manifest."""

    if manifest_path is None:
        return get_agent_adapter(name)
    adapter = load_agent_adapter_manifest(manifest_path)
    requested = str(name).strip().lower()
    if requested != adapter.name:
        raise ValueError(
            f"--agent {requested!r} does not match adapter manifest name {adapter.name!r}"
        )
    return adapter


HERMES_OPENROUTER = AgentAuthMode(
    name="openrouter",
    provider="soarm101-hermes-openrouter",
    provider_profile_id="soarm101-hermes-openrouter",
    provider_profile_asset="hermes-openrouter-provider.yaml",
    credential_env_vars=("OPENROUTER_API_KEY",),
)

CODEX_API_KEY = AgentAuthMode(
    name="api-key",
    provider="soarm101-codex-openai-api-key",
    provider_profile_id="soarm101-codex-openai-api-key",
    provider_profile_asset="codex-openai-provider.yaml",
    credential_env_vars=("OPENAI_API_KEY",),
)

CODEX_INSTALLED = AgentAuthMode(
    name="installed",
    provider=None,
    provider_profile_id=None,
    provider_profile_asset=None,
    credential_env_vars=(),
    uses_login_state=True,
)

CODEX_CHATGPT = AgentAuthMode(
    name="chatgpt",
    provider=None,
    provider_profile_id=None,
    provider_profile_asset=None,
    credential_env_vars=(),
    uses_login_state=True,
)


HERMES = HermesAgentAdapter(
    name="hermes",
    image="agenticforge/soarm101-hermes-openshell:local",
    dockerfile_asset="Dockerfile.hermes",
    skill_asset="robot-camera-hermes-skill.md",
    default_model="deepseek/deepseek-v4.1-flash",
    run_user="10000",
    run_group="10000",
    read_only_paths=("/opt/hermes",),
    robot_client_binaries=("/usr/bin/python3*",),
    mutable_directories=(
        "/sandbox/.hermes",
        "/sandbox/.home",
        "/sandbox/.xdg-config",
        "/sandbox/observations",
    ),
    auth_modes=(HERMES_OPENROUTER,),
    default_auth="openrouter",
)

CODEX = CodexAgentAdapter(
    name="codex",
    image="agenticforge/soarm101-codex-openshell:local",
    dockerfile_asset="Dockerfile.codex",
    skill_asset="robot-camera-codex-skill.md",
    default_model=None,
    run_user="10000",
    run_group="10000",
    read_only_paths=("/opt/codex-install",),
    robot_client_binaries=("/usr/bin/python3*",),
    mutable_directories=(
        "/sandbox/.codex",
        "/sandbox/.home",
        "/sandbox/.xdg-config",
        "/sandbox/observations",
    ),
    auth_modes=(CODEX_API_KEY, CODEX_INSTALLED, CODEX_CHATGPT),
    default_auth="api-key",
    sensitive_destinations=("/sandbox/.codex/auth.json",),
)

AGENTS: Mapping[str, AgentAdapter] = {
    HERMES.name: HERMES,
    CODEX.name: CODEX,
}


def agent_names() -> tuple[str, ...]:
    return tuple(AGENTS)


def get_agent_adapter(name: str) -> AgentAdapter:
    key = str(name).strip().lower()
    try:
        return AGENTS[key]
    except KeyError as exc:
        raise ValueError(
            f"unknown sandbox agent {name!r}; choose one of {', '.join(agent_names())}"
        ) from exc


def agent_catalog() -> list[dict[str, object]]:
    return [
        {
            "name": adapter.name,
            "image": adapter.image,
            "default_model": adapter.default_model,
            "default_auth": adapter.default_auth,
            "auth_modes": [
                {
                    "name": mode.name,
                    "provider": mode.provider,
                    "provider_profile_id": mode.provider_profile_id,
                    "credential_env_vars": list(mode.credential_env_vars),
                    "uses_login_state": mode.uses_login_state,
                }
                for mode in adapter.auth_modes
            ],
        }
        for adapter in AGENTS.values()
    ]
