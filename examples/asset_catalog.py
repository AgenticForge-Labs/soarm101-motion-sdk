"""Read configured asset metadata without opening or moving an arm."""

import json
from dataclasses import asdict

from soarm101_motion.asset_catalog import discover_assets
from soarm101_motion.poses import PoseLibrary
from soarm101_motion.sequences import SequenceLibrary
from soarm101_motion.trajectories import TrajectoryLibrary

robot_id = "so101"
assets = discover_assets(
    PoseLibrary(robot_id), TrajectoryLibrary(robot_id), SequenceLibrary(robot_id)
)
print(json.dumps([asdict(asset) for asset in assets], indent=2))
