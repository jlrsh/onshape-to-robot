from __future__ import annotations
import copy
import numpy as np
import re
import os
import commentjson as json
from functools import reduce

from .message import info

CONFIG_FILENAME = "o2r.json"


def deep_merge(base: dict, overlay: dict) -> dict:
    """
    Deep-merge two config dicts. Overlay wins on conflict. Nested dicts recurse;
    lists and scalars are replaced wholesale. Inputs are not mutated.
    """
    if not isinstance(base, dict) or not isinstance(overlay, dict):
        return copy.deepcopy(overlay)

    merged = copy.deepcopy(base)
    for key, value in overlay.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = copy.deepcopy(value)
    return merged


def _resolve_chain(config_path: str) -> list[str]:
    """
    Walk upward from config_path collecting o2r.json files in contiguous ancestor
    directories. Returns the chain in oldest-first order (base ancestor first,
    self last). Stops at the first ancestor directory without an o2r.json or at
    the filesystem root.
    """
    chain: list[str] = [os.path.abspath(config_path)]
    current_dir = os.path.dirname(os.path.abspath(config_path))

    while True:
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:
            break
        parent_config = os.path.join(parent_dir, CONFIG_FILENAME)
        if not os.path.exists(parent_config):
            break
        chain.insert(0, parent_config)
        current_dir = parent_dir

    return chain


def _find_nearest_ancestor_config(start_dir: str) -> str | None:
    """
    Walk upward from start_dir looking for the first directory that contains an
    o2r.json. Returns the config's absolute path, or None if the walk reaches
    the filesystem root without finding one. Does not inspect `start_dir` itself.
    """
    current_dir = os.path.abspath(start_dir)
    while True:
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:
            return None
        candidate = os.path.join(parent_dir, CONFIG_FILENAME)
        if os.path.exists(candidate):
            return candidate
        current_dir = parent_dir


class Config:
    def __init__(self, robot_path: str, safe: bool = False):
        self.safe: bool = safe

        abs_path = os.path.abspath(robot_path)
        if os.path.isdir(robot_path):
            target_dir = abs_path
            direct_config = os.path.join(abs_path, CONFIG_FILENAME)
        else:
            target_dir = os.path.dirname(abs_path)
            direct_config = abs_path

        # Output directory is pinned to the dir the CLI was pointed at — never
        # to an ancestor that provides the actual config. All generated URDFs,
        # pickles and assets land here.
        self.output_directory: str = target_dir

        variant_name: str | None = None
        if os.path.exists(direct_config):
            # CLI target has its own o2r.json: direct inheritance, contiguous
            # ancestor chain, variants block (if present) is NOT applied.
            self.config_file: str = direct_config
            chain = _resolve_chain(direct_config)
        else:
            # No o2r.json at the CLI target: look for an ancestor that holds a
            # `variants` block and apply the entry matching the target's
            # basename. This lets a single centralized config drive many
            # variant subdirectories.
            ancestor = _find_nearest_ancestor_config(target_dir)
            if ancestor is None:
                raise Exception(
                    f"No {CONFIG_FILENAME} at {direct_config} and no ancestor "
                    f"directory holds one. Create an {CONFIG_FILENAME} at the "
                    f"target or in a parent directory."
                )
            self.config_file = ancestor
            chain = _resolve_chain(ancestor)
            variant_name = os.path.basename(target_dir)

        loaded = []
        for path in chain:
            with open(path, "r", encoding="utf8") as stream:
                loaded.append(json.load(stream))
        merged: dict = reduce(deep_merge, loaded, {})

        # The `variants` key is loader-only metadata; it never reaches
        # read_configuration. When the CLI target has no config of its own, the
        # matching variant entry is deep-merged on top of the base config.
        variants = merged.pop("variants", {}) or {}
        if variant_name is not None:
            if not isinstance(variants, dict) or variant_name not in variants:
                known = sorted(variants.keys()) if isinstance(variants, dict) else []
                raise Exception(
                    f"No {CONFIG_FILENAME} at {direct_config}, and {self.config_file} "
                    f"has no `variants.{variant_name}` entry. "
                    f"Known variants: {known or 'none'}. "
                    f"Add an {CONFIG_FILENAME} at the target or a "
                    f"`variants.{variant_name}` block in the ancestor config."
                )
            merged = deep_merge(merged, variants[variant_name])

        self.config: dict = merged
        self.variant_name: str | None = variant_name

        self.processors: list = []
        self.read_configuration()

        if self.robot_name is None:
            self.robot_name = os.path.basename(self.output_directory)

        os.makedirs(self.output_directory, exist_ok=True)

    def to_camel_case(self, snake_str: str) -> str:
        """
        Converts a string to camel case
        """
        components = snake_str.split("_")
        return components[0] + "".join(x.title() for x in components[1:])

    def get(self, name: str, default=None, required: bool = True, values_list=None):
        """
        Gets an entry from the configuration

        Args:
            name (str): entry name
            default: default fallback value if the entry is not present. Defaults to None.
            required (bool, optional): whether the configuration entry is required. Defaults to False.
            values_list: list of allowed values. Defaults to None.
        """
        camel_name = self.to_camel_case(name)

        if name in self.config or camel_name in self.config:
            if name in self.config:
                value = self.config[name]
            else:
                value = self.config[camel_name]

            if values_list is not None and value not in values_list:
                raise Exception(
                    f"Value for {name} should be onf of: {','.join(values_list)}"
                )
            return value
        elif required and default is None:
            raise Exception(f"ERROR: missing required key {name} in config")

        return default

    def printable_version(self) -> str:
        if self.url is not None:
            return self.url
        else:
            version = f"document_id: {self.document_id}"
            if self.version_id:
                version += f" / version_id: {self.version_id}"
            elif self.workspace_id:
                version += f" / workspace_id: {self.workspace_id}"

            return version

    def parse_url(self):
        pattern = "https://(.*)/(.*)/([wv])/(.*)/e/(.*)"
        match = re.match(pattern, self.url)

        if match is None:
            raise Exception(f"Invalid URL: {self.url}")

        match_groups = match.groups()
        self.document_id = match_groups[1]
        if match_groups[2] == "w":
            self.workspace_id = match_groups[3]
        elif match_groups[2] == "v":
            self.version_id = match_groups[3]
        self.element_id = match_groups[4]

    def asset_path(self, asset_name: str) -> str:
        return f"{self.output_directory}/{self.assets_directory}/{asset_name}"

    def read_configuration(self):
        """
        Load and check configuration entries
        """

        # Robot name
        self.robot_name: str = self.get("robot_name", None, required=False)
        self.output_filename: str = self.get("output_filename", "robot")
        # Securing filename
        self.output_filename = "".join(
            c for c in self.output_filename if c.isalnum() or c in ("_", "-")
        ).rstrip()
        self.assets_directory: str = self.get("assets_directory", "assets")

        # Main settings
        self.document_id: str = self.get("document_id", required=False)
        self.version_id: str | None = self.get("version_id", required=False)
        self.workspace_id: str | None = self.get("workspace_id", required=False)
        self.element_id: str | None = self.get("element_id", required=False)

        if self.version_id and self.workspace_id:
            raise Exception("You can't specify workspace_id and version_id")

        self.url: str = self.get("url", None, required=False)
        if self.url is not None:
            self.parse_url()

        if self.url is None and self.document_id is None:
            raise Exception("You need to specify either a url or a document_id")

        self.draw_frames: bool = self.get("draw_frames", False)
        self.frame_x_forward: bool = self.get("frame_x_forward", False)

        self.assembly_name: str = self.get("assembly_name", required=False)
        self.output_format: str = self.get("output_format")
        self.configuration: str | dict = self.get("configuration", "default")
        self.ignore_limits: bool = self.get("ignore_limits", False)

        if isinstance(self.configuration, dict):
            self.configuration = ";".join(
                [f"{k}={v}" for k, v in self.configuration.items()]
            )

        # Joint specs
        self.joint_properties: dict = self.get("joint_properties", {})
        self.geom_properties: dict = self.get("geom_properties", {})
        self.no_dynamics: bool = self.get("no_dynamics", False)

        # Ignore / whitelists
        self.ignore: list[str] = self.get("ignore", {})
        if isinstance(self.ignore, list):
            self.ignore = {entry: "all" for entry in self.ignore}

        # Color override
        self.color: str | None = self.get("color", required=False)

        # Post-import commands
        self.post_import_commands: list[str] = self.get("post_import_commands", [])

        # Whether to include configuration suffix in part names
        self.include_configuration_suffix: bool = self.get(
            "include_configuration_suffix", True
        )

        # Mesh transfer format from Onshape ("stl" or "glb")
        self.mesh_format: str = self.get(
            "mesh_format", "stl", values_list=["stl", "glb"]
        )

        # When using GLB, auto-rename the default "robot" output so a parallel
        # STL run on the same directory doesn't clobber it.
        if self.mesh_format == "glb" and self.output_filename == "robot":
            self.output_filename = "robot_glb"
            print(info(f"mesh_format=glb: output_filename -> {self.output_filename}"))

        # Number of decimals to keep for small numbers
        self.round_decimals = self.get("round_decimals", 12)

        # Loading processors
        from . import processors

        loaded_modules = {}
        processors_list: list[str] | None = self.get("processors", None, required=False)
        if processors_list is None or self.safe:
            self.processors = [
                processor(self)
                for processor in processors.default_processors
                if (processor.is_safe or not self.safe)
            ]
        else:
            for entry in processors_list:
                parts = entry.split(":")

                if len(parts) == 1:
                    processor = eval(f"processors.{entry}")
                else:
                    module, cls = parts
                    if module not in loaded_modules:
                        loaded_modules[module] = __import__(module, fromlist=[cls])
                    processor = getattr(loaded_modules[module], cls)

                if processor is None:
                    raise Exception(f"ERROR: Processor {entry} not found")

                self.processors.append(processor(self))

    def round(self, object: float | list | tuple | np.ndarray):
        """
        Round the given number or list of numbers using the configuration decimals
        """
        if isinstance(object, float):
            return round(object, self.round_decimals)
        elif isinstance(object, np.ndarray):
            return object.round(self.round_decimals)
        else:
            original_type = type(object)
            return original_type(np.array(object).round(self.round_decimals))
