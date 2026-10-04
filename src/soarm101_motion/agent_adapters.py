"""Agent-specific adapters for the canonical SO-ARM101 OpenShell runtime.

The robot boundary is intentionally agent-neutral. Adapters own only harness-specific
container, provider/auth, mutable-home, prompt/tool conventions, and command construction.
"""

from __future__ import annotations

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
    robot_client_binaries=("/usr/local/bin/python3*",),
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
