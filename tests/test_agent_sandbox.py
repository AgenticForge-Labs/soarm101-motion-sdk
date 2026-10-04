from __future__ import annotations

import hashlib
import json
import subprocess
from pathlib import Path

import pytest

from soarm101_motion import agent_sandbox
from soarm101_motion.agent_adapters import get_agent_adapter
from soarm101_motion.agent_adapters import load_agent_adapter_manifest
from soarm101_motion.cli.main import build_parser


class FakeOpenShell:
    def __init__(self) -> None:
        self.calls: list[tuple[str, ...]] = []
        self.created: dict[str, object] | None = None
        self.uploads: list[tuple[str, Path, str | None]] = []
        self.uploaded_bytes: dict[str, bytes] = {}
        self.exec_calls: list[dict[str, object]] = []
        self.downloads: list[tuple[str, Path]] = []
        self.deleted: list[str] = []

    def available(self) -> bool:
        return True

    def run(self, args, *, check=True, timeout=120, env=None):
        self.calls.append(tuple(args))
        if tuple(args) == ("status",):
            return subprocess.CompletedProcess(args, 0, stdout="connected\n", stderr="")
        if args[:2] == ["provider", "get"]:
            return subprocess.CompletedProcess(args, 0, stdout="provider\n", stderr="")
        raise AssertionError(f"unexpected OpenShell call: {args}")

    def create(self, *, name, image, policy, provider):
        self.created = {
            "name": name,
            "image": image,
            "policy": Path(policy).read_text(encoding="utf-8"),
            "provider": provider,
        }
        return {"name": name, "phase": "Ready"}

    def upload(self, name, source, destination=None):
        source_path = Path(source)
        self.uploads.append((name, source_path, destination))
        if destination is not None and source_path.is_file():
            self.uploaded_bytes[destination] = source_path.read_bytes()

    def exec(self, name, command, *, workdir="/sandbox", env=None, timeout=1800):
        self.exec_calls.append(
            {
                "name": name,
                "command": tuple(command),
                "workdir": workdir,
                "env": dict(env or {}),
                "timeout": timeout,
            }
        )
        if command and command[0] == "/bin/mkdir":
            return subprocess.CompletedProcess(command, 0, stdout="", stderr="")
        if command in (["hermes", "--version"], ["codex", "--version"]):
            return subprocess.CompletedProcess(
                command,
                0,
                stdout=f"{command[0]} 9.9.9\n",
                stderr="",
            )
        return subprocess.CompletedProcess(
            command,
            0,
            stdout='{"event":"done"}\n',
            stderr="",
        )

    def effective_policy(self, name):
        return "version: 1\nnetwork_policies: {}\n"

    def logs(self, name, *, since="2h"):
        return "synthetic openshell log\n"

    def download(self, name, sandbox_path, destination):
        destination = Path(destination)
        self.downloads.append((sandbox_path, destination))
        if sandbox_path.endswith("task-result.json"):
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(
                json.dumps(
                    {
                        "task_status": "not_finished",
                        "reason": "synthetic test",
                    }
                ),
                encoding="utf-8",
            )
        elif sandbox_path.endswith("observations"):
            # OpenShell copies the directory contents into the explicit destination;
            # it does not recreate the source basename automatically.
            destination.mkdir(parents=True, exist_ok=True)

    def delete(self, name, *, timeout=120):
        self.deleted.append(name)


class FakeBroker:
    instances: list["FakeBroker"] = []

    def __init__(self, *, port, token, event_path):
        self.port = port
        self.token = token
        self.event_path = Path(event_path)
        self.started = False
        self.stopped = False
        type(self).instances.append(self)

    def start(self):
        self.started = True
        self.event_path.parent.mkdir(parents=True, exist_ok=True)
        self.event_path.write_text("", encoding="utf-8")

    def require_authority(self):
        return {
            "authority": {"armed": True, "remaining_seconds": 300},
            "cameras": ["overhead", "wrist"],
        }

    def stop(self):
        self.stopped = True
        return "broker stdout", "broker stderr"






def test_agent_adapters_keep_harness_specific_behavior_out_of_robot_layer() -> None:
    hermes = get_agent_adapter("hermes")
    codex = get_agent_adapter("codex")

    hermes_command = hermes.command(
        prompt="test",
        model=None,
        max_turns=17,
    )
    assert hermes_command[0] == "hermes"
    assert "--max-turns" in hermes_command
    assert "deepseek/deepseek-v4.1-flash" in hermes_command

    codex_command = codex.command(
        prompt="test",
        model=None,
        max_turns=17,
    )
    assert codex_command[0] == "codex"
    assert "-c" in codex_command
    assert any(
        "supports_websockets=false" in item for item in codex_command
    )
    assert any(
        'model_provider="soarm101_openai_api_key"' == item
        for item in codex_command
    )
    assert any('env_key="OPENAI_API_KEY"' in item for item in codex_command)
    assert "exec" in codex_command
    assert "--ephemeral" in codex_command
    assert "--ignore-user-config" in codex_command
    assert "--ignore-rules" in codex_command
    assert "--skip-git-repo-check" in codex_command
    assert "--dangerously-bypass-approvals-and-sandbox" in codex_command
    assert "--json" in codex_command
    assert "--model" not in codex_command

    codex_model_command = codex.command(
        prompt="test",
        model="gpt-example",
        max_turns=17,
        auth="api-key",
    )
    assert "--model" in codex_model_command
    assert "gpt-example" in codex_model_command

    codex_chatgpt_command = codex.command(
        prompt="test",
        model=None,
        max_turns=17,
        auth="chatgpt",
    )
    codex_installed_command = codex.command(
        prompt="test",
        model=None,
        max_turns=17,
        auth="installed",
    )
    assert codex_installed_command == codex_chatgpt_command
    for login_command in (codex_installed_command, codex_chatgpt_command):
        assert not any("soarm101_chatgpt_http" in item for item in login_command)
        assert not any("model_provider=" in item for item in login_command)
        assert not any("CODEX_ACCESS_TOKEN" in item for item in login_command)

def test_openshell_create_disables_auto_provider_attachment(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    client = agent_sandbox.OpenShellClient()
    observed: list[str] = []

    def fake_run(args, *, check=True, timeout=120, env=None):
        observed.extend(args)
        return subprocess.CompletedProcess(
            args,
            0,
            stdout=json.dumps({"name": "robot-agent", "phase": "Ready"}),
            stderr="",
        )

    monkeypatch.setattr(client, "run", fake_run)
    policy = tmp_path / "policy.yaml"
    policy.write_text("version: 1\n", encoding="utf-8")

    created = client.create(
        name="robot-agent",
        image="example/hermes:test",
        policy=policy,
        provider="soarm101-hermes-openrouter",
    )

    assert created["name"] == "robot-agent"
    assert "--provider" in observed
    assert "soarm101-hermes-openrouter" in observed
    assert "--no-auto-providers" in observed

    observed.clear()
    created = client.create(
        name="robot-agent",
        image="example/codex:test",
        policy=policy,
        provider=None,
    )
    assert created["name"] == "robot-agent"
    assert "--provider" not in observed
    assert "--no-auto-providers" in observed

def test_packaged_agent_assets_are_available_and_agent_specific() -> None:
    assert "FROM nousresearch/hermes-agent:latest" in agent_sandbox.dockerfile_text(
        agent="hermes"
    )
    assert "FROM debian:bookworm-slim" in agent_sandbox.dockerfile_text(
        agent="codex"
    )
    inert_cmd = 'CMD ["/bin/sh", "-lc", "while :; do sleep 3600; done"]'
    assert inert_cmd in agent_sandbox.dockerfile_text(agent="hermes")
    assert inert_cmd in agent_sandbox.dockerfile_text(agent="codex")
    assert "CMD []" not in agent_sandbox.dockerfile_text(agent="hermes")
    assert "CMD []" not in agent_sandbox.dockerfile_text(agent="codex")
    read_only_task = agent_sandbox.read_only_validation_task_text()
    assert "Validate the isolated SO-ARM101 agent environment" in read_only_task
    assert "capture every configured camera" in read_only_task
    assert "authority.armed" in read_only_task
    assert "vision_analyze" not in read_only_task

    hermes_profile = agent_sandbox.provider_profile_text(agent="hermes")
    assert "id: soarm101-hermes-openrouter" in hermes_profile
    assert "/usr/bin/python3*" in hermes_profile
    assert "/usr/local/bin/python3*" not in hermes_profile

    codex_api_profile = agent_sandbox.provider_profile_text(
        agent="codex",
        auth="api-key",
    )
    assert "id: soarm101-codex-openai-api-key" in codex_api_profile
    assert "OPENAI_API_KEY" in codex_api_profile
    assert "CODEX_ACCESS_TOKEN" not in codex_api_profile
    assert "host: api.openai.com" in codex_api_profile

    with pytest.raises(ValueError, match="does not use an OpenShell provider"):
        agent_sandbox.provider_profile_text(
            agent="codex",
            auth="chatgpt",
        )

    with pytest.raises(ValueError, match="does not use an OpenShell provider"):
        agent_sandbox.provider_profile_text(
            agent="codex",
            auth="installed",
        )

    assert "/usr/local/bin/codex" in codex_api_profile
    assert "/opt/codex-install/packages/standalone/**/bin/codex" in codex_api_profile
    assert "/opt/codex-install/packages/standalone/**/codex" in codex_api_profile

    hermes_skill = agent_sandbox.skill_text(agent="hermes")
    hermes_copy = Path(
        "agent-as-code/skills/robot-camera-openshell-hermes/SKILL.md"
    ).read_text(encoding="utf-8")
    assert hermes_skill == hermes_copy
    assert "vision_analyze" in hermes_skill

    codex_skill = agent_sandbox.skill_text(agent="codex")
    codex_copy = Path(
        "agent-as-code/skills/robot-camera-openshell-codex/SKILL.md"
    ).read_text(encoding="utf-8")
    assert codex_skill == codex_copy
    assert "view_image" in codex_skill

    for skill in (hermes_skill, codex_skill):
        assert "python3 robotctl.py capabilities" in skill
        assert "continue observation-only work" in skill
        assert "motion cannot continue" in skill
        assert "soarm101 agent arm" not in skill
        assert "soarm101 agent go-pose" not in skill


def test_broker_policy_is_narrow_and_schema_shaped() -> None:
    policy = agent_sandbox.broker_policy_text(port=9876)
    assert "version: 1" in policy
    assert 'run_as_user: "10000"' in policy
    assert "host: host.openshell.internal" in policy
    assert "port: 9876" in policy
    assert "path: /v1/joint" in policy
    assert "path: /v1/jog" in policy
    assert "path: /v1/capture" in policy
    assert "/v1/arm" not in policy
    assert "/v1/disarm" not in policy
    assert "/v1/relax" not in policy
    assert "/v1/exec" not in policy
    assert "/opt/hermes" in policy
    assert "/usr/bin/python3*" in policy
    assert "/usr/local/bin/python3*" not in policy
    assert "hard_requirement" in policy

    read_only = agent_sandbox.broker_policy_text(
        port=9876,
        read_only=True,
        agent="hermes",
    )
    assert "path: /v1/capture" in read_only
    assert "path: /v1/state" in read_only
    assert "path: /v1/go-pose" not in read_only
    assert "path: /v1/joint" not in read_only
    assert "path: /v1/jog" not in read_only
    assert "path: /v1/gripper" not in read_only
    assert "path: /v1/sleep" not in read_only
    assert "path: /v1/stop" not in read_only

    codex_policy = agent_sandbox.broker_policy_text(
        port=9876,
        agent="codex",
    )
    assert "/opt/codex-install" in codex_policy
    assert "/usr/bin/python3*" in codex_policy
    assert "/usr/local/bin/python3*" not in codex_policy
    assert "host: chatgpt.com" not in codex_policy
    assert "host: auth.openai.com" not in codex_policy

    codex_installed_policy = agent_sandbox.broker_policy_text(
        port=9876,
        agent="codex",
        auth="installed",
    )
    assert "host: chatgpt.com" in codex_installed_policy
    assert "host: auth.openai.com" in codex_installed_policy
    assert "/usr/local/bin/codex" in codex_installed_policy

    codex_chatgpt_policy = agent_sandbox.broker_policy_text(
        port=9876,
        agent="codex",
        auth="chatgpt",
    )
    assert "host: chatgpt.com" in codex_chatgpt_policy
    assert "host: auth.openai.com" in codex_chatgpt_policy
    assert "/usr/local/bin/codex" in codex_chatgpt_policy
    assert "path: /v1/joint" in codex_chatgpt_policy


def test_agent_catalog_is_machine_readable_and_extensible() -> None:
    from soarm101_motion.agent_adapters import agent_catalog

    catalog = {item["name"]: item for item in agent_catalog()}
    assert set(catalog) == {"hermes", "codex"}
    assert catalog["hermes"]["default_auth"] == "openrouter"
    assert catalog["codex"]["default_auth"] == "api-key"
    hermes_auth = {item["name"]: item for item in catalog["hermes"]["auth_modes"]}
    codex_auth = {item["name"]: item for item in catalog["codex"]["auth_modes"]}
    assert hermes_auth["openrouter"]["credential_env_vars"] == ["OPENROUTER_API_KEY"]
    assert codex_auth["api-key"]["credential_env_vars"] == ["OPENAI_API_KEY"]
    assert codex_auth["installed"]["credential_env_vars"] == []
    assert codex_auth["installed"]["provider"] is None
    assert codex_auth["installed"]["uses_login_state"] is True
    assert codex_auth["chatgpt"]["credential_env_vars"] == []
    assert codex_auth["chatgpt"]["provider"] is None
    assert codex_auth["chatgpt"]["uses_login_state"] is True
    assert catalog["hermes"]["default_model"] == "deepseek/deepseek-v4.1-flash"
    assert catalog["codex"]["default_model"] is None


def test_agent_sandbox_cli_surface() -> None:
    parser = build_parser()

    agents = parser.parse_args(["agent", "sandbox", "agents", "--json"])
    assert agents.agent_sandbox_command == "agents"

    doctor = parser.parse_args(
        ["agent", "sandbox", "doctor", "--agent", "codex", "--json"]
    )
    assert doctor.agent_command == "sandbox"
    assert doctor.agent_sandbox_command == "doctor"
    assert doctor.agent == "codex"

    setup = parser.parse_args(
        ["agent", "sandbox", "setup", "--agent", "codex"]
    )
    assert setup.agent_sandbox_command == "setup"
    assert setup.agent == "codex"

    reauth = parser.parse_args(
        ["agent", "sandbox", "setup", "--agent", "codex", "--auth", "chatgpt", "--reauth"]
    )
    assert reauth.reauth is True

    run = parser.parse_args(
        [
            "agent",
            "sandbox",
            "run",
            "--agent",
            "codex",
            "--auth",
            "chatgpt",
            "--task",
            "TASK.md",
            "--model",
            "example/model",
        ]
    )
    assert run.agent_sandbox_command == "run"
    assert run.agent == "codex"
    assert run.auth == "chatgpt"
    assert run.task == "TASK.md"
    assert run.model == "example/model"
    installed = parser.parse_args(
        ["agent", "sandbox", "run", "--agent", "codex", "--auth", "installed", "--read-only"]
    )
    assert installed.auth == "installed"
    read_only = parser.parse_args(["agent", "sandbox", "run", "--read-only"])
    assert read_only.read_only is True
    assert read_only.task is None


def test_run_hermes_uses_only_packaged_client_skill_and_task(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = tmp_path / "TASK.md"
    task.write_text("Inspect the workspace and stop safely.\n", encoding="utf-8")
    output = tmp_path / "run"
    fake = FakeOpenShell()
    FakeBroker.instances.clear()

    monkeypatch.setattr(agent_sandbox, "BrokerProcess", FakeBroker)
    monkeypatch.setattr(
        agent_sandbox,
        "doctor",
        lambda **kwargs: agent_sandbox.DoctorResult(
            openshell=True,
            gateway=True,
            docker=True,
            image=True,
            provider=True,
            details={},
        ),
    )

    result = agent_sandbox.run_hermes(
        task=task,
        output_dir=output,
        model="example/model",
        openshell=fake,
    )

    assert result.exit_code == 0
    assert fake.created is not None
    assert fake.uploaded_bytes["/sandbox/TASK.md"].decode("utf-8") == (
        "Inspect the workspace and stop safely.\n"
    )
    assert fake.created["provider"] == agent_sandbox.DEFAULT_PROVIDER
    policy = str(fake.created["policy"])
    assert "/v1/joint" in policy
    assert "/v1/relax" not in policy

    destinations = {dest for _, _, dest in fake.uploads}
    assert "/sandbox/TASK.md" in destinations
    assert "/sandbox/SKILL.md" in destinations
    assert "/sandbox/robotctl.py" in destinations
    assert "/sandbox/.hermes/config.yaml" in destinations
    assert all("calibration" not in str(source) for _, source, _ in fake.uploads)

    hermes_call = fake.exec_calls[-1]
    command = hermes_call["command"]
    assert command[0] == "hermes"
    assert "--toolsets" in command
    assert "hermes-cli" in command
    assert "example/model" in command
    env = hermes_call["env"]
    assert env["SOARM101_BROKER_URL"].startswith("http://host.openshell.internal:")
    assert env["SOARM101_BROKER_TOKEN"]
    assert env["HERMES_HOME"] == "/sandbox/.hermes"

    assert fake.deleted == [result.sandbox]
    assert (
        "/sandbox/observations",
        output / "observations",
    ) in fake.downloads
    assert FakeBroker.instances[0].started is True
    assert FakeBroker.instances[0].stopped is True
    assert (output / "hermes-stdout.jsonl").is_file()
    assert (output / "openshell-policy.yaml").is_file()
    assert (output / "openshell-effective-policy.yaml").read_text(
        encoding="utf-8"
    ).startswith("version: 1")
    assert (output / "openshell-logs.txt").read_text(
        encoding="utf-8"
    ) == "synthetic openshell log\n"
    assert (output / "broker-stdout.txt").read_text(encoding="utf-8") == "broker stdout"
    metadata = json.loads((output / "run-metadata.json").read_text(encoding="utf-8"))
    assert metadata["schema_version"] == 3
    assert metadata["agent"] == "hermes"
    assert metadata["auth"] == "openrouter"
    assert metadata["agent_version"] == "hermes 9.9.9"
    assert metadata["model"] == "example/model"
    assert set(metadata["input_sha256"]) >= {"task", "skill", "robotctl", "policy"}






def test_run_codex_uses_shared_robot_boundary_with_codex_harness(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = tmp_path / "TASK.md"
    task.write_text("Inspect the workspace safely.\n", encoding="utf-8")
    output = tmp_path / "run"
    fake = FakeOpenShell()
    FakeBroker.instances.clear()

    monkeypatch.setattr(agent_sandbox, "BrokerProcess", FakeBroker)
    monkeypatch.setattr(
        agent_sandbox,
        "doctor",
        lambda **kwargs: agent_sandbox.DoctorResult(
            openshell=True,
            gateway=True,
            docker=True,
            image=True,
            provider=True,
            details={},
            agent="codex",
        ),
    )

    result = agent_sandbox.run_agent(
        agent="codex",
        auth="api-key",
        task=task,
        output_dir=output,
        model="gpt-example",
        openshell=fake,
    )

    assert result.agent == "codex"
    assert result.auth == "api-key"
    assert result.model == "gpt-example"
    assert fake.created is not None
    assert fake.created["image"] == "agenticforge/soarm101-codex-openshell:local"
    assert result.auth == "api-key"
    assert fake.created["provider"] == "soarm101-codex-openai-api-key"

    policy = str(fake.created["policy"])
    assert "/opt/codex-install" in policy
    assert "/usr/bin/python3*" in policy
    assert "/v1/joint" in policy

    skill = fake.uploaded_bytes["/sandbox/SKILL.md"].decode("utf-8")
    assert "view_image" in skill
    assert "vision_analyze" not in skill

    codex_call = fake.exec_calls[-1]
    command = codex_call["command"]
    assert command[0] == "codex"
    assert "exec" in command
    assert any("supports_websockets=false" in item for item in command)
    assert 'model_provider="soarm101_openai_api_key"' in command
    assert any('env_key="OPENAI_API_KEY"' in item for item in command)
    assert "--dangerously-bypass-approvals-and-sandbox" in command
    assert "--ephemeral" in command
    assert "gpt-example" in command
    env = codex_call["env"]
    assert env["CODEX_HOME"] == "/sandbox/.codex"
    assert env["SOARM101_BROKER_TOKEN"]

    assert (output / "codex-stdout.jsonl").is_file()
    assert (output / "codex-stderr.txt").is_file()
    metadata = json.loads((output / "run-metadata.json").read_text(encoding="utf-8"))
    assert metadata["schema_version"] == 3
    assert metadata["agent"] == "codex"
    assert metadata["auth"] == "api-key"
    assert metadata["agent_version"] == "codex 9.9.9"
    assert metadata["model"] == "gpt-example"
    assert set(metadata["input_sha256"]) >= {"task", "skill", "robotctl", "policy"}

def test_run_codex_chatgpt_uses_dedicated_login_state(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = tmp_path / "TASK.md"
    task.write_text("Inspect the workspace safely.\n", encoding="utf-8")
    output = tmp_path / "run-chatgpt"
    fake = FakeOpenShell()
    auth = tmp_path / "auth.json"
    auth.write_text(
        json.dumps(
            {
                "auth_mode": "chatgpt",
                "tokens": {
                    "id_token": "secret-id",
                    "access_token": "secret-access",
                    "refresh_token": "secret-refresh",
                    "account_id": "acct",
                },
                "last_refresh": "2026-10-03T00:00:00Z",
            }
        ),
        encoding="utf-8",
    )

    monkeypatch.setattr(agent_sandbox, "BrokerProcess", FakeBroker)
    monkeypatch.setattr(agent_sandbox, "codex_chatgpt_auth_path", lambda: auth)
    monkeypatch.setattr(
        agent_sandbox,
        "doctor",
        lambda **kwargs: agent_sandbox.DoctorResult(
            openshell=True,
            gateway=True,
            docker=True,
            image=True,
            provider=True,
            details={},
            agent="codex",
            auth="chatgpt",
        ),
    )

    result = agent_sandbox.run_agent(
        agent="codex",
        auth="chatgpt",
        task=task,
        output_dir=output,
        openshell=fake,
    )

    assert result.agent == "codex"
    assert result.auth == "chatgpt"
    assert fake.created is not None
    assert fake.created["provider"] is None

    assert "/sandbox/.codex/auth.json" in fake.uploaded_bytes
    copied_auth = json.loads(
        fake.uploaded_bytes["/sandbox/.codex/auth.json"].decode("utf-8")
    )
    assert copied_auth["auth_mode"] == "chatgpt"

    codex_call = fake.exec_calls[-1]
    command = codex_call["command"]
    assert not any("soarm101_chatgpt_http" in item for item in command)
    assert not any("model_provider=" in item for item in command)
    assert not any("CODEX_ACCESS_TOKEN" in item for item in command)
    assert "CODEX_ACCESS_TOKEN" not in codex_call["env"]

    policy = str(fake.created["policy"])
    assert "host: chatgpt.com" in policy
    assert "host: auth.openai.com" in policy

    metadata = json.loads((output / "run-metadata.json").read_text(encoding="utf-8"))
    assert metadata["schema_version"] == 3
    assert metadata["agent"] == "codex"
    assert metadata["auth"] == "chatgpt"
    assert metadata["provider"] is None
    assert metadata["sensitive_inputs"] == ["/sandbox/.codex/auth.json"]
    assert all(
        "auth.json" not in key
        for key in metadata["input_sha256"]
    )

def test_read_only_run_skips_human_authority_and_omits_motion_routes(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = tmp_path / "TASK.md"
    task.write_text("Inspect state and cameras only.\n", encoding="utf-8")
    output = tmp_path / "run"
    fake = FakeOpenShell()

    class ReadOnlyBroker(FakeBroker):
        def require_authority(self):
            raise AssertionError("read-only mode must not require motion authority")

        def request(self, method, path, payload=None):
            assert method == "GET"
            assert path == "/v1/capabilities"
            return {
                "ok": True,
                "result": {
                    "authority": {"armed": False},
                    "cameras": ["overhead"],
                },
            }

    monkeypatch.setattr(agent_sandbox, "BrokerProcess", ReadOnlyBroker)
    validation_calls: list[dict[str, object]] = []

    def fake_validate_read_only_evidence(**kwargs):
        validation_calls.append(dict(kwargs))
        return {"status": "passed"}

    monkeypatch.setattr(
        agent_sandbox,
        "_validate_read_only_evidence",
        fake_validate_read_only_evidence,
    )
    monkeypatch.setattr(
        agent_sandbox,
        "doctor",
        lambda **kwargs: agent_sandbox.DoctorResult(
            openshell=True,
            gateway=True,
            docker=True,
            image=True,
            provider=True,
            details={},
        ),
    )

    result = agent_sandbox.run_hermes(
        task=None,
        output_dir=output,
        read_only=True,
        openshell=fake,
    )

    assert result.exit_code == 0
    assert fake.created is not None
    assert "Validate the isolated SO-ARM101 agent environment" in (
        fake.uploaded_bytes["/sandbox/TASK.md"].decode("utf-8")
    )
    policy = str(fake.created["policy"])
    assert "/v1/capture" in policy
    assert "/v1/state" in policy
    assert "/v1/joint" not in policy
    assert "/v1/jog" not in policy
    assert "/v1/gripper" not in policy
    assert "/v1/sleep" not in policy
    assert "/v1/stop" not in policy

    hermes_call = fake.exec_calls[-1]
    prompt = hermes_call["command"][hermes_call["command"].index("-q") + 1]
    assert "hard read-only validation run" in prompt
    assert "Motion authority is not expected or required" in prompt
    assert len(validation_calls) == 1
    assert validation_calls[0]["capabilities"] == {
        "authority": {"armed": False},
        "cameras": ["overhead"],
    }


def test_read_only_evidence_requires_broker_primitives_and_verified_capture(
    tmp_path: Path,
) -> None:
    output = tmp_path / "run"
    observations = output / "observations"
    observations.mkdir(parents=True)
    image = b"synthetic-jpeg-bytes"
    image_path = observations / "overhead.jpg"
    image_path.write_bytes(image)
    digest = hashlib.sha256(image).hexdigest()

    events = tmp_path / "broker-events.jsonl"
    payloads = [
        {"action": "capabilities", "ok": True, "request": {}, "result": {}},
        {"action": "state", "ok": True, "request": {}, "result": {}},
        {
            "action": "capture",
            "ok": True,
            "request": {"camera": "overhead"},
            "result": {"sha256": digest},
        },
    ]
    events.write_text(
        "".join(json.dumps(payload) + "\n" for payload in payloads),
        encoding="utf-8",
    )

    validation = agent_sandbox._validate_read_only_evidence(
        capabilities={"cameras": ["overhead"]},
        broker_events=events,
        event_offset=0,
        output_dir=output,
        agent_exit_code=0,
    )

    assert validation["status"] == "passed"
    assert validation["missing_actions"] == []
    assert validation["missing_capture_cameras"] == []
    assert validation["unverified_capture_cameras"] == []
    saved = json.loads(
        (output / "read-only-validation.json").read_text(encoding="utf-8")
    )
    assert saved["observation_sha256"]["observations/overhead.jpg"] == digest


def test_read_only_evidence_fails_when_required_capture_is_missing(
    tmp_path: Path,
) -> None:
    output = tmp_path / "run"
    output.mkdir()
    events = tmp_path / "broker-events.jsonl"
    payloads = [
        {"action": "capabilities", "ok": True, "request": {}, "result": {}},
        {"action": "state", "ok": True, "request": {}, "result": {}},
    ]
    events.write_text(
        "".join(json.dumps(payload) + "\n" for payload in payloads),
        encoding="utf-8",
    )

    with pytest.raises(
        agent_sandbox.AgentSandboxError,
        match="missing_capture_cameras=\\['overhead'\\]",
    ):
        agent_sandbox._validate_read_only_evidence(
            capabilities={"cameras": ["overhead"]},
            broker_events=events,
            event_offset=0,
            output_dir=output,
            agent_exit_code=0,
        )
    saved = json.loads(
        (output / "read-only-validation.json").read_text(encoding="utf-8")
    )
    assert saved["status"] == "failed"


def test_run_requires_human_authority_before_agent_execution(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = tmp_path / "TASK.md"
    task.write_text("test\n", encoding="utf-8")
    output = tmp_path / "run"
    fake = FakeOpenShell()

    class UnarmedBroker(FakeBroker):
        def require_authority(self):
            raise agent_sandbox.AgentSandboxError("agent authority is not active")

    monkeypatch.setattr(agent_sandbox, "BrokerProcess", UnarmedBroker)
    monkeypatch.setattr(
        agent_sandbox,
        "doctor",
        lambda **kwargs: agent_sandbox.DoctorResult(
            openshell=True,
            gateway=True,
            docker=True,
            image=True,
            provider=True,
            details={},
        ),
    )

    with pytest.raises(agent_sandbox.AgentSandboxError, match="authority"):
        agent_sandbox.run_hermes(
            task=task,
            output_dir=output,
            openshell=fake,
        )
    assert fake.created is None


class SetupOpenShell:
    def __init__(self, *, existing_profile: bool, existing_provider: bool) -> None:
        self.existing_profile = existing_profile
        self.existing_provider = existing_provider
        self.calls: list[tuple[str, ...]] = []

    def available(self) -> bool:
        return True

    def run(self, args, *, check=True, timeout=120, env=None):
        key = tuple(args)
        self.calls.append(key)
        if key == ("status",):
            return subprocess.CompletedProcess(args, 0, stdout="connected\n", stderr="")
        if args[:2] == ["profile", "describe"]:
            return subprocess.CompletedProcess(
                args,
                0 if self.existing_profile else 1,
                stdout="profile\n" if self.existing_profile else "",
                stderr="" if self.existing_profile else "not found",
            )
        if args[:2] == ["provider", "get"]:
            return subprocess.CompletedProcess(
                args,
                0 if self.existing_provider else 1,
                stdout="provider\n" if self.existing_provider else "",
                stderr="" if self.existing_provider else "not found",
            )
        return subprocess.CompletedProcess(args, 0, stdout="", stderr="")


@pytest.mark.parametrize(
    ("existing_profile", "existing_provider", "profile_action", "provider_action"),
    [
        (False, False, "import", "create"),
        (True, True, "update", "update"),
    ],
)
def test_setup_is_idempotent(
    monkeypatch: pytest.MonkeyPatch,
    existing_profile: bool,
    existing_provider: bool,
    profile_action: str,
    provider_action: str,
) -> None:
    fake = SetupOpenShell(
        existing_profile=existing_profile,
        existing_provider=existing_provider,
    )
    monkeypatch.setattr(agent_sandbox.shutil, "which", lambda name: f"/usr/bin/{name}")
    docker_calls: list[list[str]] = []

    def fake_subprocess_run(args, **kwargs):
        docker_calls.append(list(args))
        return subprocess.CompletedProcess(args, 0, stdout="", stderr="")

    monkeypatch.setattr(agent_sandbox.subprocess, "run", fake_subprocess_run)

    agent_sandbox.setup(openshell=fake)

    assert docker_calls
    assert docker_calls[0][:2] == ["docker", "build"]
    lint_calls = [call for call in fake.calls if call[:2] == ("profile", "lint")]
    assert len(lint_calls) == 1
    assert "--global" not in lint_calls[0]
    assert any(
        call[:2] == ("profile", profile_action)
        for call in fake.calls
    )
    assert any(
        call[:2] == ("provider", provider_action)
        for call in fake.calls
    )




def test_codex_api_key_setup_uses_provider_profile(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake = SetupOpenShell(existing_profile=False, existing_provider=False)
    monkeypatch.setattr(agent_sandbox.shutil, "which", lambda name: f"/usr/bin/{name}")

    def fake_subprocess_run(args, **kwargs):
        return subprocess.CompletedProcess(args, 0, stdout="", stderr="")

    monkeypatch.setattr(agent_sandbox.subprocess, "run", fake_subprocess_run)

    agent_sandbox.setup(
        agent="codex",
        auth="api-key",
        openshell=fake,
    )

    expected = "soarm101-codex-openai-api-key"
    assert any(call[:3] == ("profile", "describe", expected) for call in fake.calls)
    assert any(call[:3] == ("provider", "get", expected) for call in fake.calls)
    create_call = next(call for call in fake.calls if call[:2] == ("provider", "create"))
    assert expected in create_call


def test_codex_chatgpt_setup_uses_device_login_not_provider(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    fake = SetupOpenShell(existing_profile=False, existing_provider=False)
    monkeypatch.setattr(agent_sandbox.shutil, "which", lambda name: f"/usr/bin/{name}")
    login_calls: list[tuple[str, bool]] = []

    def fake_subprocess_run(args, **kwargs):
        return subprocess.CompletedProcess(args, 0, stdout="", stderr="")

    def fake_login(*, image: str, reauth: bool = False) -> None:
        login_calls.append((image, reauth))

    monkeypatch.setattr(agent_sandbox.subprocess, "run", fake_subprocess_run)
    monkeypatch.setattr(agent_sandbox, "_setup_codex_chatgpt_login", fake_login)

    agent_sandbox.setup(
        agent="codex",
        auth="chatgpt",
        reauth=True,
        openshell=fake,
    )

    assert login_calls == [
        ("agenticforge/soarm101-codex-openshell:local", True)
    ]
    assert not any(call[:1] == ("profile",) for call in fake.calls)
    assert not any(call[:1] == ("provider",) for call in fake.calls)


def test_codex_chatgpt_device_login_is_dedicated_and_idempotent(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state = tmp_path / "state"
    auth_path = state / "auth.json"
    monkeypatch.setattr(agent_sandbox, "codex_chatgpt_state_dir", lambda: state)
    monkeypatch.setattr(agent_sandbox, "codex_chatgpt_auth_path", lambda: auth_path)
    monkeypatch.setattr(
        agent_sandbox,
        "host_codex_auth_path",
        lambda: tmp_path / "missing-host-auth.json",
    )

    docker_calls: list[list[str]] = []

    def fake_run(args, **kwargs):
        docker_calls.append(list(args))
        mount = args[args.index("-v") + 1]
        login_root = Path(mount.split(":", 1)[0])
        assert (login_root / "config.toml").read_text(encoding="utf-8") == (
            'cli_auth_credentials_store = "file"\n'
        )
        (login_root / "auth.json").write_text(
            json.dumps(
                {
                    "auth_mode": "chatgpt",
                    "tokens": {
                        "id_token": "id",
                        "access_token": "access",
                        "refresh_token": "refresh",
                        "account_id": "acct",
                    },
                }
            ),
            encoding="utf-8",
        )
        return subprocess.CompletedProcess(args, 0, stdout="", stderr="")

    monkeypatch.setattr(agent_sandbox.subprocess, "run", fake_run)

    agent_sandbox._setup_codex_chatgpt_login(image="codex:test")
    assert agent_sandbox._valid_codex_chatgpt_auth(auth_path)
    assert docker_calls
    assert "codex" in docker_calls[0]
    assert "login" in docker_calls[0]
    assert "--device-auth" in docker_calls[0]
    assert str(Path.home() / ".codex") not in " ".join(docker_calls[0])

    first_count = len(docker_calls)
    agent_sandbox._setup_codex_chatgpt_login(image="codex:test")
    assert len(docker_calls) == first_count

    agent_sandbox._setup_codex_chatgpt_login(image="codex:test", reauth=True)
    assert len(docker_calls) == first_count + 1


def test_codex_installed_doctor_reuses_host_login_state(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    codex_home = tmp_path / ".codex"
    codex_home.mkdir()
    auth = codex_home / "auth.json"
    auth.write_text(
        json.dumps({"auth_mode": "chatgpt", "tokens": {"access_token": "secret"}}),
        encoding="utf-8",
    )
    monkeypatch.setenv("CODEX_HOME", str(codex_home))
    monkeypatch.setattr(agent_sandbox.shutil, "which", lambda name: f"/usr/bin/{name}")

    def fake_subprocess_run(args, **kwargs):
        if args[:3] == ["docker", "version", "--format"]:
            return subprocess.CompletedProcess(args, 0, stdout="27.0\n", stderr="")
        if args[:3] == ["docker", "image", "inspect"]:
            return subprocess.CompletedProcess(args, 0, stdout="[]\n", stderr="")
        raise AssertionError(args)

    monkeypatch.setattr(agent_sandbox.subprocess, "run", fake_subprocess_run)
    fake = FakeOpenShell()

    result = agent_sandbox.doctor(
        agent="codex",
        auth="installed",
        openshell=fake,
    )
    assert result.ready is True
    assert result.auth == "installed"
    assert result.details["login_state"] == str(auth)
    assert result.details["host_codex"] == "/usr/bin/codex"
    assert not any(call[:1] == ("provider",) for call in fake.calls)


def test_run_codex_installed_uploads_one_run_copy_of_host_auth(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = tmp_path / "TASK.md"
    task.write_text("Inspect safely.\n", encoding="utf-8")
    codex_home = tmp_path / ".codex"
    codex_home.mkdir()
    auth = codex_home / "auth.json"
    auth.write_text(
        json.dumps({
            "auth_mode": "chatgpt",
            "tokens": {"access_token": "host-access", "refresh_token": "host-refresh"},
        }),
        encoding="utf-8",
    )
    monkeypatch.setenv("CODEX_HOME", str(codex_home))
    fake = FakeOpenShell()
    FakeBroker.instances.clear()

    monkeypatch.setattr(agent_sandbox, "BrokerProcess", FakeBroker)
    monkeypatch.setattr(
        agent_sandbox,
        "doctor",
        lambda **kwargs: agent_sandbox.DoctorResult(
            openshell=True,
            gateway=True,
            docker=True,
            image=True,
            provider=True,
            details={},
            agent="codex",
            auth="installed",
        ),
    )

    result = agent_sandbox.run_agent(
        agent="codex",
        auth="installed",
        task=task,
        output_dir=tmp_path / "run-installed",
        openshell=fake,
    )

    assert result.auth == "installed"
    assert fake.created["provider"] is None
    uploaded = json.loads(fake.uploaded_bytes["/sandbox/.codex/auth.json"].decode("utf-8"))
    assert uploaded["tokens"]["access_token"] == "host-access"
    assert json.loads(auth.read_text(encoding="utf-8"))["tokens"]["access_token"] == "host-access"
    metadata = json.loads(
        (tmp_path / "run-installed" / "run-metadata.json").read_text(encoding="utf-8")
    )
    assert metadata["auth"] == "installed"
    assert metadata["sensitive_inputs"] == ["/sandbox/.codex/auth.json"]
    assert not any("auth.json" in key for key in metadata["input_sha256"])


def test_codex_chatgpt_doctor_uses_dedicated_login_state(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    auth = tmp_path / "auth.json"
    auth.write_text(
        json.dumps({"auth_mode": "chatgpt", "tokens": {"access_token": "secret"}}),
        encoding="utf-8",
    )
    monkeypatch.setattr(agent_sandbox, "codex_chatgpt_auth_path", lambda: auth)
    monkeypatch.setattr(agent_sandbox.shutil, "which", lambda name: f"/usr/bin/{name}")

    def fake_subprocess_run(args, **kwargs):
        if args[:3] == ["docker", "version", "--format"]:
            return subprocess.CompletedProcess(args, 0, stdout="27.0\n", stderr="")
        if args[:3] == ["docker", "image", "inspect"]:
            return subprocess.CompletedProcess(args, 0, stdout="[]\n", stderr="")
        raise AssertionError(args)

    monkeypatch.setattr(agent_sandbox.subprocess, "run", fake_subprocess_run)
    fake = FakeOpenShell()

    result = agent_sandbox.doctor(
        agent="codex",
        auth="chatgpt",
        openshell=fake,
    )
    assert result.ready is True
    assert result.provider is True
    assert result.details["provider_name"] == "<native-codex-login>"
    assert result.details["login_state"] == str(auth)
    assert not any(call[:1] == ("provider",) for call in fake.calls)

    with pytest.raises(agent_sandbox.AgentSandboxError, match="--provider is not applicable"):
        agent_sandbox.doctor(
            agent="codex",
            auth="chatgpt",
            provider="should-not-be-used",
            openshell=fake,
        )


def test_full_control_run_requires_task(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(
        agent_sandbox,
        "doctor",
        lambda **kwargs: agent_sandbox.DoctorResult(
            openshell=True,
            gateway=True,
            docker=True,
            image=True,
            provider=True,
            details={},
        ),
    )
    with pytest.raises(agent_sandbox.AgentSandboxError, match="--task is required"):
        agent_sandbox.run_hermes(
            task=None,
            output_dir=tmp_path / "run",
            read_only=False,
            openshell=FakeOpenShell(),
        )


def test_cleanup_failure_invalidates_otherwise_successful_run(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    task = tmp_path / "TASK.md"
    task.write_text("Inspect safely.\n", encoding="utf-8")
    fake = FakeOpenShell()

    def broken_delete(name, *, timeout=120):
        raise RuntimeError("synthetic cleanup failure")

    fake.delete = broken_delete  # type: ignore[method-assign]
    monkeypatch.setattr(agent_sandbox, "BrokerProcess", FakeBroker)
    monkeypatch.setattr(
        agent_sandbox,
        "doctor",
        lambda **kwargs: agent_sandbox.DoctorResult(
            openshell=True,
            gateway=True,
            docker=True,
            image=True,
            provider=True,
            details={},
        ),
    )

    with pytest.raises(agent_sandbox.AgentSandboxError, match="cleanup failed"):
        agent_sandbox.run_hermes(
            task=task,
            output_dir=tmp_path / "run",
            openshell=fake,
        )
    assert (tmp_path / "run" / "sandbox-cleanup-error.txt").is_file()


def _write_external_adapter_manifest(
    tmp_path: Path,
    *,
    provider: str | None = None,
    command: list[str] | None = None,
) -> Path:
    manifest = tmp_path / "external-agent.json"
    payload = {
        "schema_version": 1,
        "name": "claude",
        "image": "openshell/base",
        "command": command or ["claude", "-p", "{prompt}"],
        "version_command": ["claude", "--version"],
        "robot_client_binaries": ["/usr/bin/python3*"],
        "read_only_paths": ["/usr/local/bin/claude"],
        "mutable_directories": [
            "/sandbox/.home",
            "/sandbox/.xdg-config",
            "/sandbox/observations",
        ],
        "environment": {"CLAUDE_CONFIG_DIR": "/sandbox/.home/.claude"},
    }
    if provider is not None:
        payload["provider"] = provider
    manifest.write_text(json.dumps(payload), encoding="utf-8")
    return manifest


def test_external_adapter_manifest_uses_direct_argv_and_generic_skill(
    tmp_path: Path,
) -> None:
    manifest = _write_external_adapter_manifest(
        tmp_path,
        command=["claude", "-p", "{prompt}", "--model", "{model}", "--turns", "{max_turns}"],
    )
    adapter = load_agent_adapter_manifest(manifest)

    assert adapter.name == "claude"
    assert adapter.managed_setup is False
    assert adapter.auth_names() == ("external",)
    assert adapter.command(
        prompt="Inspect safely.",
        model="example-model",
        max_turns=7,
    ) == [
        "claude",
        "-p",
        "Inspect safely.",
        "--model",
        "example-model",
        "--turns",
        "7",
    ]
    assert adapter.version_command() == ["claude", "--version"]
    assert "robotctl.py" in adapter.skill_text()
    assert adapter.environment()["CLAUDE_CONFIG_DIR"] == "/sandbox/.home/.claude"


def test_external_adapter_manifest_rejects_shell_style_placeholder_interpolation(
    tmp_path: Path,
) -> None:
    manifest = _write_external_adapter_manifest(
        tmp_path,
        command=["claude", "--prompt={prompt}"],
    )
    with pytest.raises(ValueError, match="standalone .*prompt.* token"):
        load_agent_adapter_manifest(manifest)


def test_external_adapter_policy_and_setup_use_existing_openshell_resources(
    tmp_path: Path,
) -> None:
    manifest = _write_external_adapter_manifest(tmp_path, provider="work-claude")
    fake = FakeOpenShell()

    policy = agent_sandbox.broker_policy_text(
        agent="claude",
        adapter_manifest=manifest,
        read_only=True,
    )
    assert "/usr/bin/python3*" in policy
    assert "/v1/capture" in policy
    assert "/v1/go-pose" not in policy

    result = agent_sandbox.doctor(
        agent="claude",
        adapter_manifest=manifest,
        openshell=fake,
    )
    assert result.ready is True
    assert result.details["docker"] == "not required for external OpenShell adapter"
    assert ("provider", "get", "work-claude") in fake.calls

    fake.calls.clear()
    agent_sandbox.setup(
        agent="claude",
        adapter_manifest=manifest,
        openshell=fake,
    )
    assert fake.calls == [
        ("status",),
        ("provider", "get", "work-claude"),
    ]


def test_external_adapter_runs_through_shared_broker_and_cleanup(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    manifest = _write_external_adapter_manifest(tmp_path)
    task = tmp_path / "TASK.md"
    task.write_text("Inspect safely.\n", encoding="utf-8")
    fake = FakeOpenShell()
    FakeBroker.instances.clear()
    monkeypatch.setattr(agent_sandbox, "BrokerProcess", FakeBroker)

    result = agent_sandbox.run_agent(
        agent="claude",
        adapter_manifest=manifest,
        task=task,
        output_dir=tmp_path / "run-external",
        openshell=fake,
    )

    assert result.exit_code == 0
    assert result.agent == "claude"
    assert result.auth == "external"
    assert fake.created is not None
    assert fake.created["image"] == "openshell/base"
    assert fake.created["provider"] is None
    harness_calls = [
        call for call in fake.exec_calls
        if call["command"] and call["command"][0] == "claude"
    ]
    assert len(harness_calls) >= 2
    run_call = harness_calls[-1]
    assert run_call["command"][0:2] == ("claude", "-p")
    assert "Read TASK.md and SKILL.md completely before acting." in run_call["command"][-1]
    assert run_call["env"]["SOARM101_BROKER_TOKEN"]
    assert fake.deleted == [result.sandbox]
    assert FakeBroker.instances[-1].stopped is True


def test_sandbox_cli_accepts_manifest_defined_agent_name(tmp_path: Path) -> None:
    manifest = _write_external_adapter_manifest(tmp_path)
    args = build_parser().parse_args(
        [
            "agent",
            "sandbox",
            "run",
            "--agent",
            "claude",
            "--adapter-manifest",
            str(manifest),
            "--read-only",
        ]
    )
    assert args.agent == "claude"
    assert args.adapter_manifest == str(manifest)


def test_external_adapter_manifest_rejects_shell_and_broker_env_override(
    tmp_path: Path,
) -> None:
    shell_manifest = _write_external_adapter_manifest(
        tmp_path,
        command=["/bin/sh", "-c", "{prompt}"],
    )
    with pytest.raises(ValueError, match="directly, not a shell"):
        load_agent_adapter_manifest(shell_manifest)

    payload = json.loads(shell_manifest.read_text(encoding="utf-8"))
    payload["command"] = ["claude", "-p", "{prompt}"]
    payload["environment"] = {"SOARM101_BROKER_TOKEN": "override"}
    shell_manifest.write_text(json.dumps(payload), encoding="utf-8")
    with pytest.raises(ValueError, match="broker-controlled variables"):
        load_agent_adapter_manifest(shell_manifest)


def test_external_adapter_manifest_skill_file_cannot_escape_manifest_directory(
    tmp_path: Path,
) -> None:
    adapter_dir = tmp_path / "adapter"
    adapter_dir.mkdir()
    outside = tmp_path / "outside-skill.md"
    outside.write_text("secret host text\n", encoding="utf-8")
    manifest = adapter_dir / "external-agent.json"
    manifest.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "name": "claude",
                "image": "openshell/base",
                "command": ["claude", "-p", "{prompt}"],
                "version_command": ["claude", "--version"],
                "robot_client_binaries": ["/usr/bin/python3*"],
                "skill_file": "../outside-skill.md",
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="may not escape"):
        load_agent_adapter_manifest(manifest)


def test_openshell_sandbox_names_fit_server_limit_and_preserve_agent_hint(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(agent_sandbox.secrets, "token_hex", lambda n: "12345678")
    codex_name = agent_sandbox._sandbox_name("codex")
    long_name = agent_sandbox._sandbox_name("manifest-agent-name-that-is-much-too-long")

    assert codex_name == "s101-codex-12345678"
    assert len(codex_name) <= agent_sandbox.OPENSHELL_SANDBOX_NAME_MAX_LENGTH
    assert len(long_name) <= agent_sandbox.OPENSHELL_SANDBOX_NAME_MAX_LENGTH
    assert long_name.startswith("s101-manif-")
