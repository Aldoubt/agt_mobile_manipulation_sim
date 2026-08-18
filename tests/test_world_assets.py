from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
WORLD_DIR = ROOT / "src" / "agt_sim_worlds" / "worlds" / "greenhouse_v1"
WORLD_SDF = WORLD_DIR / "world.sdf"
SCENE_YAML = WORLD_DIR / "scene.yaml"


def parse_pose(text: str) -> list[float]:
    values = [float(value) for value in text.split()]
    if len(values) != 6:
        raise AssertionError("SDF pose must contain 6 values")
    return values


class WorldAssetTests(unittest.TestCase):
    def test_world_contains_all_semantic_targets_at_matching_positions(self) -> None:
        scene = yaml.safe_load(SCENE_YAML.read_text(encoding="utf-8"))
        root = ET.parse(WORLD_SDF).getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        models = {model.attrib["name"]: model for model in world.findall("model")}

        for target in scene["targets"]:
            target_id = target["id"]
            self.assertIn(target_id, models)
            sdf_pose = parse_pose(models[target_id].findtext("pose", default=""))
            expected = target["pose"]["position"]
            for actual_value, expected_value in zip(sdf_pose[:3], expected):
                self.assertAlmostEqual(expected_value, actual_value, places=6)

    def test_world_has_self_contained_greenhouse_geometry(self) -> None:
        root = ET.parse(WORLD_SDF).getroot()
        world = root.find("world")
        self.assertIsNotNone(world)
        names = {model.attrib["name"] for model in world.findall("model")}
        self.assertTrue({"ground", "greenhouse_left_row", "greenhouse_right_row"}.issubset(names))

        self.assertEqual([], root.findall(".//include"))
        self.assertEqual([], root.findall(".//mesh"))


if __name__ == "__main__":
    unittest.main()
