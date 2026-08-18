from __future__ import annotations

import unittest
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
WORLD_ROOT = ROOT / "src" / "agt_sim_worlds"
SCENE_PATH = WORLD_ROOT / "worlds" / "greenhouse_v1" / "scene.yaml"
ROUTE_PATH = WORLD_ROOT / "worlds" / "greenhouse_v1" / "routes" / "row_01.yaml"
MK_MINI_PATH = WORLD_ROOT / "profiles" / "mk_mini.yaml"
AGILEX_PATH = WORLD_ROOT / "profiles" / "agilex_generic.yaml"


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise AssertionError(f"{path} must contain a mapping at the root")
    return data


def validate_pose(pose: dict, label: str) -> None:
    if not isinstance(pose, dict):
        raise AssertionError(f"{label} pose must be a mapping")
    if len(pose.get("position", [])) != 3:
        raise AssertionError(f"{label} pose.position must have 3 values")
    if len(pose.get("orientation_rpy", [])) != 3:
        raise AssertionError(f"{label} pose.orientation_rpy must have 3 values")


def validate_scene(scene: dict) -> None:
    world = scene.get("world", {})
    if world.get("frame") != "map":
        raise AssertionError("scene world.frame must be map")

    targets = scene.get("targets", [])
    ids = [target.get("id") for target in targets]
    if len(ids) != len(set(ids)):
        raise AssertionError("duplicate target id")

    for target in targets:
        target_id = target.get("id")
        if not target_id or not target.get("class"):
            raise AssertionError("target requires id and class")
        validate_pose(target.get("pose", {}), f"target {target_id}")

        viewpoints = target.get("viewpoints", [])
        if not viewpoints:
            raise AssertionError(f"target {target_id} requires at least one viewpoint")
        view_ids = [view.get("id") for view in viewpoints]
        if len(view_ids) != len(set(view_ids)):
            raise AssertionError(f"target {target_id} has duplicate viewpoint id")
        for view in viewpoints:
            validate_pose(view.get("pose", {}), f"viewpoint {target_id}/{view.get('id')}")

        manipulation_poses = target.get("manipulation_poses", [])
        if not manipulation_poses:
            raise AssertionError(
                f"target {target_id} requires at least one manipulation pose"
            )
        manipulation_ids = [pose.get("id") for pose in manipulation_poses]
        if len(manipulation_ids) != len(set(manipulation_ids)):
            raise AssertionError(
                f"target {target_id} has duplicate manipulation pose id"
            )
        for pose in manipulation_poses:
            validate_pose(
                pose.get("pose", {}),
                f"manipulation pose {target_id}/{pose.get('id')}",
            )


def validate_route(route: dict, scene: dict) -> None:
    if route.get("frame") != "map":
        raise AssertionError("route frame must be map")

    waypoints = route.get("waypoints", [])
    ids = [waypoint.get("id") for waypoint in waypoints]
    if len(ids) != len(set(ids)):
        raise AssertionError("duplicate waypoint id")

    targets = {target["id"]: target for target in scene.get("targets", [])}

    for waypoint in waypoints:
        waypoint_id = waypoint.get("id")
        pose = waypoint.get("pose", {})
        if not waypoint_id:
            raise AssertionError("waypoint requires id")
        if not all(key in pose for key in ("x", "y", "yaw")):
            raise AssertionError(f"waypoint {waypoint_id} requires x/y/yaw pose")

        behavior = waypoint.get("behavior")
        if behavior not in {"pass", "operate"}:
            raise AssertionError(f"waypoint {waypoint_id} has invalid behavior")

        if behavior == "operate":
            operation = waypoint.get("operation")
            if not isinstance(operation, dict):
                raise AssertionError(
                    f"operate waypoint {waypoint_id} requires operation binding"
                )
            target_id = operation.get("target_id")
            pose_id = operation.get("pose_id")
            pose_type = operation.get("pose_type")
            if not target_id or not pose_id or pose_type not in {
                "viewpoint",
                "manipulation",
            }:
                raise AssertionError(
                    f"operate waypoint {waypoint_id} has incomplete operation binding"
                )
            if target_id not in targets:
                raise AssertionError(
                    f"operate waypoint {waypoint_id} references unknown target"
                )

            collection_name = (
                "viewpoints" if pose_type == "viewpoint" else "manipulation_poses"
            )
            valid_pose_ids = {
                item["id"] for item in targets[target_id].get(collection_name, [])
            }
            if pose_id not in valid_pose_ids:
                raise AssertionError(
                    f"operate waypoint {waypoint_id} references unknown pose"
                )


def validate_base_profile(profile: dict) -> None:
    base = profile.get("base", {})
    if not base.get("name"):
        raise AssertionError("base profile requires name")
    if base.get("kinematics") not in {"ackermann", "differential"}:
        raise AssertionError("base profile has unsupported kinematics")
    if not isinstance(base.get("geometry_verified"), bool):
        raise AssertionError("base profile requires geometry_verified boolean")


class WorldContractTests(unittest.TestCase):
    def test_greenhouse_scene_contract(self) -> None:
        scene = load_yaml(SCENE_PATH)
        validate_scene(scene)
        self.assertEqual(3, len(scene["targets"]))

    def test_row_route_contract_and_references(self) -> None:
        scene = load_yaml(SCENE_PATH)
        route = load_yaml(ROUTE_PATH)
        validate_scene(scene)
        validate_route(route, scene)
        self.assertEqual(["P0", "P1", "P2", "P3"], [w["id"] for w in route["waypoints"]])
        self.assertEqual("operate", route["waypoints"][2]["behavior"])

    def test_base_profiles_are_controller_independent(self) -> None:
        mk_mini = load_yaml(MK_MINI_PATH)
        agilex = load_yaml(AGILEX_PATH)
        validate_base_profile(mk_mini)
        validate_base_profile(agilex)
        self.assertEqual("ackermann", mk_mini["base"]["kinematics"])
        self.assertFalse(mk_mini["base"]["geometry_verified"])
        self.assertFalse(agilex["base"]["geometry_verified"])

    def test_duplicate_target_ids_are_rejected(self) -> None:
        scene = {
            "world": {"frame": "map"},
            "targets": [{"id": "same"}, {"id": "same"}],
        }
        with self.assertRaisesRegex(AssertionError, "duplicate target id"):
            validate_scene(scene)

    def test_duplicate_waypoint_ids_are_rejected(self) -> None:
        route = {
            "frame": "map",
            "waypoints": [
                {"id": "P0", "pose": {"x": 0, "y": 0, "yaw": 0}, "behavior": "pass"},
                {"id": "P0", "pose": {"x": 1, "y": 0, "yaw": 0}, "behavior": "pass"},
            ],
        }
        with self.assertRaisesRegex(AssertionError, "duplicate waypoint id"):
            validate_route(route, {"targets": []})

    def test_missing_map_frame_is_rejected(self) -> None:
        with self.assertRaisesRegex(AssertionError, "world.frame must be map"):
            validate_scene({"world": {"frame": "odom"}, "targets": []})

    def test_missing_viewpoint_pose_is_rejected(self) -> None:
        scene = {
            "world": {"frame": "map"},
            "targets": [
                {
                    "id": "tomato_001",
                    "class": "tomato",
                    "pose": {"position": [0, 0, 1], "orientation_rpy": [0, 0, 0]},
                    "viewpoints": [{"id": "front"}],
                    "manipulation_poses": [
                        {
                            "id": "reach",
                            "pose": {"position": [0, 0, 1], "orientation_rpy": [0, 0, 0]},
                        }
                    ],
                }
            ],
        }
        with self.assertRaisesRegex(AssertionError, "pose.position must have 3 values"):
            validate_scene(scene)

    def test_operate_waypoint_requires_binding(self) -> None:
        route = {
            "frame": "map",
            "waypoints": [
                {"id": "P2", "pose": {"x": 0, "y": 0, "yaw": 0}, "behavior": "operate"}
            ],
        }
        with self.assertRaisesRegex(AssertionError, "requires operation binding"):
            validate_route(route, {"targets": []})


if __name__ == "__main__":
    unittest.main()
