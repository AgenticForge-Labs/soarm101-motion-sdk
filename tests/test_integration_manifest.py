import json
from pathlib import Path

from soarm101_motion.integration import (
    DEFAULT_BASE_FRAME,
    SOFTWARE_STOP_IS_CERTIFIED_EMERGENCY_STOP,
    integration_metadata,
)


def test_integration_manifest_matches_python_metadata() -> None:
    path = (
        Path(__file__).parents[1]
        / ".agenticforge"
        / "integration-manifest.json"
    )
    manifest = json.loads(path.read_text(encoding="utf-8"))
    metadata = integration_metadata()

    assert manifest["contract_family"] == "agenticforge.robotics"
    assert manifest["contract_version"].startswith("0.2.")
    assert manifest["frames"]["default_base"] == DEFAULT_BASE_FRAME
    assert manifest["resource"]["default_id"] == metadata.resource_id
    assert manifest["motion"]["certified_emergency_stop"] is False
    assert SOFTWARE_STOP_IS_CERTIFIED_EMERGENCY_STOP is False
