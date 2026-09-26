import importlib.util
import sys
import types
import unittest
from pathlib import Path


class FakeEnvironment:
    last_kwargs = None

    def __init__(self, **kwargs):
        type(self).last_kwargs = kwargs


class StudentProfileTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        native = types.ModuleType("_mars_rover_cpp")
        native.biome_catalog = lambda: (
            {"id": "anchor", "split": 0, "index": 0},
            {"id": "training", "split": 1, "index": 1},
            {"id": "private", "split": 2, "index": 2},
        )
        environment = types.ModuleType("mars_rover_env")
        environment.MarsRoverEnv = FakeEnvironment
        sys.modules["_mars_rover_cpp"] = native
        sys.modules["mars_rover_env"] = environment
        path = Path(__file__).parents[1] / "python/mars_rover_gui/student_profile.py"
        spec = importlib.util.spec_from_file_location("student_profile_under_test", path)
        cls.profile = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.profile)

    def test_catalog_excludes_non_public_split(self):
        self.assertEqual([item["id"] for item in self.profile.visible_catalog()], ["anchor", "training"])

    def test_environment_overrides_split_and_unset_fixed_id(self):
        args = types.SimpleNamespace(
            config="custom.yaml",
            rig="rig.yaml",
            biome_index=None,
            debug=False,
            render_width=640,
            render_height=360,
        )
        self.profile.create_environment(args)
        self.assertEqual(FakeEnvironment.last_kwargs["biome_split"], 1)
        self.assertEqual(FakeEnvironment.last_kwargs["fixed_biome_id"], -1)

    def test_selected_visible_id_is_preserved(self):
        args = types.SimpleNamespace(
            config=None,
            rig=None,
            biome_index=7,
            debug=True,
            render_width=800,
            render_height=450,
        )
        self.profile.create_environment(args)
        self.assertEqual(FakeEnvironment.last_kwargs["fixed_biome_id"], 7)
        self.assertEqual(FakeEnvironment.last_kwargs["render_mode"], "debug_rgb_array")


if __name__ == "__main__":
    unittest.main()
