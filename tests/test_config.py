import json
import os
import tempfile

import pytest

from onshape_to_robot.config import CONFIG_FILENAME, Config, _resolve_chain, deep_merge


class TestDeepMerge:
    def test_dict_union(self):
        assert deep_merge({"a": 1}, {"b": 2}) == {"a": 1, "b": 2}

    def test_nested_dict_recurses(self):
        base = {"joint_properties": {"hip": {"damping": 0.1}}}
        overlay = {"joint_properties": {"knee": {"damping": 0.2}}}
        merged = deep_merge(base, overlay)
        assert merged == {
            "joint_properties": {
                "hip": {"damping": 0.1},
                "knee": {"damping": 0.2},
            }
        }

    def test_scalar_override(self):
        assert deep_merge({"url": "a"}, {"url": "b"}) == {"url": "b"}

    def test_list_replaces_wholesale(self):
        assert deep_merge({"processors": ["A", "B"]}, {"processors": ["C"]}) == {
            "processors": ["C"]
        }

    def test_empty_overlay_is_noop(self):
        base = {"a": {"b": 1}}
        assert deep_merge(base, {}) == base

    def test_none_in_overlay_replaces(self):
        assert deep_merge({"a": {"b": 1}}, {"a": None}) == {"a": None}

    def test_does_not_mutate_inputs(self):
        base = {"nested": {"x": 1}}
        overlay = {"nested": {"y": 2}}
        deep_merge(base, overlay)
        assert base == {"nested": {"x": 1}}
        assert overlay == {"nested": {"y": 2}}


class TestResolveChain:
    def _write_config(self, path: str, data: dict):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f)

    def test_leaf_only_no_ancestors(self, tmp_path):
        child = tmp_path / "leaf"
        child.mkdir()
        cfg = child / CONFIG_FILENAME
        self._write_config(str(cfg), {"url": "x"})
        chain = _resolve_chain(str(cfg))
        assert chain == [os.path.abspath(str(cfg))]

    def test_three_level_contiguous(self, tmp_path):
        base = tmp_path / "a" / CONFIG_FILENAME
        mid = tmp_path / "a" / "b" / CONFIG_FILENAME
        leaf = tmp_path / "a" / "b" / "c" / CONFIG_FILENAME
        self._write_config(str(base), {"output_format": "urdf"})
        self._write_config(str(mid), {"mesh_format": "glb"})
        self._write_config(str(leaf), {"url": "u"})
        chain = _resolve_chain(str(leaf))
        assert chain == [
            os.path.abspath(str(base)),
            os.path.abspath(str(mid)),
            os.path.abspath(str(leaf)),
        ]

    def test_gap_stops_walk(self, tmp_path):
        # /a/o2r.json exists, /a/b/ has no config, /a/b/c/o2r.json exists.
        # Contiguous walk stops at /a/b (no config), so /a/o2r.json is NOT included.
        far = tmp_path / "a" / CONFIG_FILENAME
        leaf = tmp_path / "a" / "b" / "c" / CONFIG_FILENAME
        self._write_config(str(far), {"output_format": "urdf"})
        self._write_config(str(leaf), {"url": "u"})
        chain = _resolve_chain(str(leaf))
        assert chain == [os.path.abspath(str(leaf))]


class TestConfigLoader:
    def _write(self, path: str, data: dict):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            json.dump(data, f)

    def test_missing_config_and_no_ancestor_raises(self, tmp_path):
        with pytest.raises(Exception, match="no ancestor"):
            Config(str(tmp_path))

    def test_variant_applies_when_target_has_no_config(self, tmp_path):
        parent = tmp_path / "tmf3"
        variant_dir = parent / "extended_carriage"
        variant_dir.mkdir(parents=True)
        self._write(
            str(parent / CONFIG_FILENAME),
            {
                "output_format": "urdf",
                "url": "https://cad.onshape.com/documents/base/w/base/e/base",
                "mesh_format": "glb",
                "joint_properties": {"shared": {"damping": 0.1}},
                "variants": {
                    "extended_carriage": {
                        "url": "https://cad.onshape.com/documents/ext/w/ext/e/ext",
                        "joint_properties": {"hip": {"damping": 0.2}},
                    },
                    "short_carriage": {
                        "url": "https://cad.onshape.com/documents/sh/w/sh/e/sh",
                    },
                },
            },
        )
        cfg = Config(str(variant_dir))
        assert cfg.output_directory == os.path.abspath(str(variant_dir))
        assert cfg.variant_name == "extended_carriage"
        # Variant url overrides base url
        assert cfg.url.endswith("/e/ext")
        # mesh_format inherited from base
        assert cfg.mesh_format == "glb"
        # joint_properties deep-merged across base and variant
        assert cfg.joint_properties == {
            "shared": {"damping": 0.1},
            "hip": {"damping": 0.2},
        }
        # `variants` key is loader-only; it must never appear in the merged config
        assert "variants" not in cfg.config

    def test_variant_not_found_lists_known_variants(self, tmp_path):
        parent = tmp_path / "tmf3"
        bogus = parent / "notes"
        bogus.mkdir(parents=True)
        self._write(
            str(parent / CONFIG_FILENAME),
            {
                "output_format": "urdf",
                "url": "https://cad.onshape.com/documents/base/w/base/e/base",
                "variants": {
                    "extended_carriage": {"url": "https://cad.onshape.com/documents/ext/w/ext/e/ext"},
                },
            },
        )
        with pytest.raises(Exception, match="variants.notes"):
            Config(str(bogus))

    def test_target_config_shadows_variants_block(self, tmp_path):
        # When the target has its own o2r.json, the ancestor's variants block
        # is stripped without being applied — direct inheritance wins.
        parent = tmp_path / "tmf3"
        child = parent / "extended_carriage"
        self._write(
            str(parent / CONFIG_FILENAME),
            {
                "output_format": "urdf",
                "url": "https://cad.onshape.com/documents/base/w/base/e/base",
                "variants": {
                    "extended_carriage": {
                        "url": "https://cad.onshape.com/documents/variant/w/variant/e/variant",
                    },
                },
            },
        )
        self._write(
            str(child / CONFIG_FILENAME),
            {"url": "https://cad.onshape.com/documents/child/w/child/e/child"},
        )
        cfg = Config(str(child))
        # Child's own config wins over the ancestor's variant entry
        assert cfg.url.endswith("/e/child")
        assert cfg.variant_name is None
        assert "variants" not in cfg.config

    def test_variant_resolution_walks_multiple_levels(self, tmp_path):
        # Running the tool in a subfolder of a subfolder: the ancestor walk
        # should still find the top-level config holding the variants block.
        top = tmp_path / "rtu"
        tmf3 = top / "tmf3"
        variant_dir = tmf3 / "extended_carriage"
        variant_dir.mkdir(parents=True)
        self._write(
            str(tmf3 / CONFIG_FILENAME),
            {
                "output_format": "urdf",
                "url": "https://cad.onshape.com/documents/base/w/base/e/base",
                "variants": {
                    "extended_carriage": {
                        "url": "https://cad.onshape.com/documents/ext/w/ext/e/ext",
                    },
                },
            },
        )
        cfg = Config(str(variant_dir))
        assert cfg.url.endswith("/e/ext")

    def test_output_directory_pinned_to_child(self, tmp_path):
        parent = tmp_path / "parent"
        child = parent / "child"
        self._write(
            str(parent / CONFIG_FILENAME),
            {"output_format": "urdf", "url": "https://cad.onshape.com/documents/d/w/w/e/e"},
        )
        self._write(str(child / CONFIG_FILENAME), {"url": "https://cad.onshape.com/documents/d2/w/w2/e/e2"})
        cfg = Config(str(child))
        assert cfg.output_directory == os.path.abspath(str(child))
        # child url wins
        assert cfg.url.endswith("/e/e2")
        # inherited output_format from parent
        assert cfg.output_format == "urdf"

    def test_list_replaces_wholesale_in_real_config(self, tmp_path):
        parent = tmp_path / "p"
        child = parent / "c"
        self._write(
            str(parent / CONFIG_FILENAME),
            {
                "output_format": "urdf",
                "url": "https://cad.onshape.com/documents/d/w/w/e/e",
                "post_import_commands": ["echo a", "echo b"],
            },
        )
        self._write(
            str(child / CONFIG_FILENAME),
            {"post_import_commands": ["echo c"]},
        )
        cfg = Config(str(child))
        assert cfg.post_import_commands == ["echo c"]

    def test_dict_deep_merges_in_real_config(self, tmp_path):
        parent = tmp_path / "p"
        child = parent / "c"
        self._write(
            str(parent / CONFIG_FILENAME),
            {
                "output_format": "urdf",
                "url": "https://cad.onshape.com/documents/d/w/w/e/e",
                "joint_properties": {"hip": {"damping": 0.1}},
            },
        )
        self._write(
            str(child / CONFIG_FILENAME),
            {"joint_properties": {"knee": {"damping": 0.2}}},
        )
        cfg = Config(str(child))
        assert cfg.joint_properties == {
            "hip": {"damping": 0.1},
            "knee": {"damping": 0.2},
        }

    def test_robot_name_from_child_dir(self, tmp_path):
        parent = tmp_path / "parent"
        child = parent / "extended_carriage"
        self._write(
            str(parent / CONFIG_FILENAME),
            {"output_format": "urdf", "url": "https://cad.onshape.com/documents/d/w/w/e/e"},
        )
        self._write(str(child / CONFIG_FILENAME), {"url": "https://cad.onshape.com/documents/d/w/w/e/f"})
        cfg = Config(str(child))
        assert cfg.robot_name == "extended_carriage"
