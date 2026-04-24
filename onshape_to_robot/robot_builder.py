import numpy as np
import os
import hashlib
import json
import fnmatch
from .geometry import Mesh
from .message import warning, info, success, error, dim, bright
from .assembly import Assembly
from .config import Config
from .robot import Part, Joint, Link, Robot, Relation, Closure
from .csg import process as csg_process


class RobotBuilder:
    def __init__(self, config: Config):
        self.config: Config = config
        self.assembly: Assembly = Assembly(config)
        self.robot: Robot = Robot(config.robot_name)

        for closure_type, frame1, frame2 in self.assembly.closures:
            self.robot.closures.append(Closure(closure_type, frame1, frame2))

        self.unique_names = {}
        self.stl_filenames: dict = {}

        for node in self.assembly.root_nodes:
            link = self.build_robot(node)
            self.robot.base_links.append(link)

    def part_is_ignored(self, name: str, what: str) -> bool:
        """
        Checks if a given part should be ignored by config
        """
        ignored = False

        # Removing <1>, <2> etc. suffix
        name = "<".join(name.split("<")[:-1]).strip()

        for entry in self.config.ignore:
            to_ignore = True
            match_entry = entry
            if entry[0] == "!":
                to_ignore = False
                match_entry = entry[1:]

            if fnmatch.fnmatch(name.lower(), match_entry.lower()):
                if (
                    self.config.ignore[entry] == "all"
                    or self.config.ignore[entry] == what
                ):
                    ignored = to_ignore

        return ignored

    def slugify(self, value: str) -> str:
        """
        Turns a value into a slug
        """
        return "".join(c if c.isalnum() else "_" for c in value).strip("_")

    def printable_configuration(self, instance: dict) -> str:
        """
        Retrieve configuration enums to replace "List_..." with proper enum names
        """
        configuration = instance["configuration"]

        if instance["configuration"] != "default":
            if "documentVersion" in instance:
                version = instance["documentVersion"]
                wmv = "v"
            else:
                version = instance["documentMicroversion"]
                wmv = "m"
            elements = self.assembly.client.elements_configuration(
                instance["documentId"],
                version,
                instance["elementId"],
                wmv=wmv,
                linked_document_id=self.config.document_id,
            )
            for entry in elements["configurationParameters"]:
                type_name = entry["typeName"]
                message = entry["message"]

                if type_name.startswith("BTMConfigurationParameterEnum"):
                    parameter_name = message["parameterName"]
                    parameter_id = message["parameterId"]
                    configuration = configuration.replace(parameter_id, parameter_name)

        return configuration

    def part_name(self, part: dict, include_configuration: bool = False) -> str:
        """
        Retrieve the name from a part.
        i.e "Base link <1>" -> "base_link"
        """
        name = part["name"]
        parts = name.split(" ")
        del parts[-1]
        base_part_name = self.slugify("_".join(parts).lower())

        if not include_configuration:
            return base_part_name

        # Only add configuration to name if its not default and not a very long configuration (which happens for library parts like screws)
        configuration = self.printable_configuration(part)
        if configuration != "default" and self.config.include_configuration_suffix:
            if len(configuration) < 40:
                parts += ["_" + configuration.replace("=", "_").replace(" ", "_")]
            else:
                parts += ["_" + hashlib.md5(configuration.encode("utf-8")).hexdigest()]

        return self.slugify("_".join(parts).lower())

    def unique_name(self, part: dict, type: str):
        """
        Get unique part name (plate, plate_2, plate_3, ...)
        In the case where multiple parts have the same name in Onshape, they will result in different names in the URDF
        """
        while True:
            name = self.part_name(part, include_configuration=True)

            if type not in self.unique_names:
                self.unique_names[type] = {}

            if name in self.unique_names[type]:
                self.unique_names[type][name] += 1
                name = f"{name}_{self.unique_names[type][name]}"
            else:
                self.unique_names[type][name] = 1
                name = name

            if name not in [frame.name for frame in self.assembly.frames]:
                return name

    def instance_request_params(self, instance: dict) -> dict:
        """
        Build parameters to make an API call for a given instance
        """
        params = {}

        if "documentVersion" in instance:
            params["wmvid"] = instance["documentVersion"]
            params["wmv"] = "v"
        else:
            params["wmvid"] = instance["documentMicroversion"]
            params["wmv"] = "m"

        params["did"] = instance["documentId"]
        params["eid"] = instance["elementId"]
        params["linked_document_id"] = self.config.document_id
        params["configuration"] = instance["configuration"]

        return params

    def get_stl_filename(self, instance: dict) -> str:
        """
        Get a STL filename unique to the instance
        """
        exact_instance = (
            instance["documentId"],
            instance["documentMicroversion"],
            instance["elementId"],
            instance["configuration"],
            instance["partId"],
        )

        if exact_instance not in self.stl_filenames:
            part_name_config = self.part_name(instance, True)
            stl_filename = part_name_config
            k = 1
            while stl_filename in self.stl_filenames.values():
                k += 1
                stl_filename = part_name_config + f"__{k}"
            if k != 1:
                print(
                    warning(
                        f'WARNING: Parts with same name "{part_name_config}", incrementing STL name to "{stl_filename}"'
                    )
                )
            self.stl_filenames[exact_instance] = stl_filename

        return self.stl_filenames[exact_instance]

    def get_stl(self, instance: dict) -> str:
        """
        Download and store mesh file. If mesh_format is "glb", fetches GLTF from
        Onshape and saves as GLB (preserving colors, materials, normals).
        Otherwise fetches binary STL.
        """
        os.makedirs(self.config.asset_path(""), exist_ok=True)

        stl_filename = self.get_stl_filename(instance)

        if self.config.mesh_format == "glb":
            filename = stl_filename + ".glb"
            self._fetch_glb(instance, stl_filename, filename)
        else:
            filename = stl_filename + ".stl"
            params = self.instance_request_params(instance)
            stl = self.assembly.client.part_studio_stl_m(
                **params,
                partid=instance["partId"],
            )
            with open(self.config.asset_path(filename), "wb") as stream:
                stream.write(stl)

        # Storing metadata for imported instances in the .part file
        stl_metadata = stl_filename + ".part"
        with open(
            self.config.asset_path(stl_metadata), "w", encoding="utf-8"
        ) as stream:
            json.dump(instance, stream, indent=4, sort_keys=True)

        return self.config.asset_path(filename)

    def _get_gltf_extract_dir(self, instance: dict) -> str:
        """
        Fetch and extract the GLTF ZIP for a Part Studio, caching per element.
        The GLTF endpoint exports ALL parts in the Part Studio as separate .gltf
        files in a ZIP archive, so we download once and reuse for all parts.
        """
        import io
        import zipfile

        # Cache key: (documentId, elementId) identifies a unique Part Studio
        cache_key = (instance["documentId"], instance["elementId"])
        if not hasattr(self, "_gltf_extract_dirs"):
            self._gltf_extract_dirs = {}

        if cache_key not in self._gltf_extract_dirs:
            params = self.instance_request_params(instance)

            # GLTF endpoint only supports workspace (w) or version (v), not microversion (m)
            if params["wmv"] == "m":
                if self.assembly.version_id:
                    params["wmvid"] = self.assembly.version_id
                    params["wmv"] = "v"
                elif self.assembly.workspace_id:
                    params["wmvid"] = self.assembly.workspace_id
                    params["wmv"] = "w"

            gltf_data = self.assembly.client.part_studio_gltf(
                **params,
                partid=instance["partId"],
            )

            # Extract ZIP to a persistent temp dir for this Part Studio
            import tempfile

            extract_dir = tempfile.mkdtemp(prefix="onshape_gltf_")
            with zipfile.ZipFile(io.BytesIO(gltf_data)) as zf:
                zf.extractall(extract_dir)

            self._gltf_extract_dirs[cache_key] = extract_dir

        return self._gltf_extract_dirs[cache_key]

    def _fetch_glb(self, instance: dict, stl_filename: str, filename: str):
        """
        Fetch GLTF from Onshape and save as GLB, preserving colors, materials,
        and normals. The GLTF endpoint returns a ZIP with all Part Studio parts
        as separate .gltf files; we match by part name and convert to GLB.
        """
        try:
            import trimesh
        except ImportError:
            print(error("ERROR: trimesh is required for GLB mesh format"))
            print(info("TIP: pip install trimesh"))
            raise

        extract_dir = self._get_gltf_extract_dir(instance)

        # Find the matching .gltf file by part name
        # Instance names may have occurrence suffixes like " <1>" — strip them
        import re

        part_name = re.sub(r"\s*<\d+>\s*$", "", instance["name"])
        gltf_files = [
            f
            for f in os.listdir(extract_dir)
            if f.endswith((".gltf", ".glb"))
        ]

        def normalize_name(s):
            """Normalize for matching: keep only alphanumeric and underscore."""
            return re.sub(r"[^a-z0-9_]", "", s.lower())

        def gltf_part_name_from_file(gf):
            base = os.path.splitext(gf)[0]
            if " - " in base:
                return base.rsplit(" - ", 1)[1]
            return base

        def strip_disambiguation(s):
            """Strip Onshape's (1), (2) etc. disambiguation suffixes."""
            return re.sub(r"\s*\(\d+\)\s*$", "", s)

        # Collect ALL candidates whose base name matches (ignoring (1), (2) suffixes)
        norm_part = normalize_name(part_name)
        candidates = []
        for gf in gltf_files:
            gltf_name = gltf_part_name_from_file(gf)
            # Match against the base name (strip disambiguation suffix)
            if normalize_name(strip_disambiguation(gltf_name)) == norm_part:
                candidates.append(gf)

        if not candidates:
            # Fallback: fuzzy match by checking if normalized part name is contained
            for gf in gltf_files:
                if norm_part in normalize_name(gf):
                    candidates.append(gf)

        if not candidates:
            raise Exception(
                f"No matching GLTF file for part '{part_name}' in archive. "
                f"Available: {gltf_files}"
            )

        # Fetch the STL for this specific partId (always correct geometry)
        params = self.instance_request_params(instance)
        stl_data = self.assembly.client.part_studio_stl_m(
            **params,
            partid=instance["partId"],
        )
        stl_mesh = trimesh.load(
            trimesh.util.wrap_as_stream(stl_data), file_type="stl", force="mesh"
        )
        stl_bounds = stl_mesh.bounds
        stl_size = stl_bounds[1] - stl_bounds[0]

        # If multiple candidates, disambiguate by comparing geometry bounds with STL
        if len(candidates) == 1:
            matched = candidates[0]
        else:
            best_match = None
            best_score = float("inf")
            for gf in candidates:
                gltf_path = os.path.join(extract_dir, gf)
                candidate_scene = trimesh.load(gltf_path)
                if isinstance(candidate_scene, trimesh.Scene) and len(candidate_scene.geometry) > 0:
                    cverts = np.concatenate([g.vertices for g in candidate_scene.geometry.values()])
                elif hasattr(candidate_scene, "vertices") and len(candidate_scene.vertices) > 0:
                    cverts = candidate_scene.vertices
                else:
                    continue
                csize = cverts.max(axis=0) - cverts.min(axis=0)
                score = np.linalg.norm(csize - stl_size)
                if score < best_score:
                    best_score = score
                    best_match = gf
            matched = best_match if best_match is not None else candidates[0]

        gltf_path = os.path.join(extract_dir, matched)
        scene = trimesh.load(gltf_path)

        # Align GLTF bounding box to match STL coordinate origin
        stl_bounds_min = stl_bounds[0]

        if isinstance(scene, trimesh.Scene) and len(scene.geometry) > 0:
            all_verts = np.concatenate(
                [g.vertices for g in scene.geometry.values()]
            )
        else:
            all_verts = scene.vertices
        gltf_bounds_min = all_verts.min(axis=0)

        offset = stl_bounds_min - gltf_bounds_min
        if np.linalg.norm(offset) > 1e-3:
            T_offset = np.eye(4)
            T_offset[:3, 3] = offset
            if isinstance(scene, trimesh.Scene):
                for geom in scene.geometry.values():
                    geom.apply_transform(T_offset)
            else:
                scene.apply_transform(T_offset)

        # Export as single GLB file (binary glTF, self-contained).
        # include_normals=True forces the NORMAL attribute; trimesh otherwise
        # skips it unless vertex_normals is already cached on the mesh, which
        # is not guaranteed after concatenate/transform round-trips.
        glb_path = self.config.asset_path(filename)
        scene.export(glb_path, file_type="glb", include_normals=True)
        print(info(f"+ Saved {stl_filename}.glb (from {matched})"))

    def get_color(self, instance: dict) -> np.ndarray:
        """
        Retrieve the color of a part
        """
        if self.config.color is not None:
            color = np.array(self.config.color)
        else:
            params = self.instance_request_params(instance)
            metadata = self.assembly.client.part_get_metadata(
                **params,
                partid=instance["partId"],
            )

            color = np.array([0.5, 0.5, 0.5, 1.0])

            # XXX: There must be a better way to retrieve the part color
            for entry in metadata["properties"]:
                if (
                    "value" in entry
                    and type(entry["value"]) is dict
                    and "color" in entry["value"]
                ):
                    rgb = entry["value"]["color"]
                    a = entry["value"]["opacity"]
                    color = np.array([rgb["red"], rgb["green"], rgb["blue"], a]) / 255.0

        return color

    def get_dynamics(self, instance: dict) -> tuple:
        """
        Retrieve the dynamics (mass, com, inertia) of a given instance
        """
        if self.config.no_dynamics:
            mass = 0
            com = [0] * 3
            inertia = [0] * 12
        else:
            if instance["isStandardContent"]:
                mass_properties = self.assembly.client.standard_cont_mass_properties(
                    instance["documentId"],
                    instance["documentVersion"],
                    instance["elementId"],
                    instance["partId"],
                    configuration=instance["configuration"],
                    linked_document_id=self.config.document_id,
                )
            else:
                params = self.instance_request_params(instance)
                mass_properties = self.assembly.client.part_mass_properties(
                    **params,
                    partid=instance["partId"],
                )

            if instance["partId"] not in mass_properties["bodies"]:
                print(
                    warning(
                        f"WARNING: part {instance['name']} has no dynamics (maybe it is a surface)"
                    )
                )
                return
            mass_properties = mass_properties["bodies"][instance["partId"]]
            mass = mass_properties["mass"][0]
            com = mass_properties["centroid"]
            inertia = mass_properties["inertia"]

            if abs(mass) < 1e-9:
                print(
                    warning(
                        f"WARNING: part {instance['name']} has no mass, maybe you should assign a material to it ?"
                    )
                )

        return mass, com[:3], np.reshape(inertia[:9], (3, 3))

    def add_part(self, occurrence: dict):
        """
        Add a part to the current link
        """
        instance = occurrence["instance"]

        if instance["suppressed"]:
            return

        if self.assembly.is_occurrence_hidden(occurrence["path"]):
            return

        if instance["partId"] == "":
            print(warning(f"WARNING: Part '{instance['name']}' has no partId"))
            return

        part_name = instance["name"]
        extra = ""
        if instance["configuration"] != "default":
            extra = dim(
                " (configuration: " + self.printable_configuration(instance) + ")"
            )
        symbol = "+"
        if self.part_is_ignored(part_name, "visual") or self.part_is_ignored(
            part_name, "collision"
        ):
            symbol = "-"
            extra += dim(" / ")
            if self.part_is_ignored(part_name, "visual"):
                extra += dim("(ignoring visual)")
            if self.part_is_ignored(part_name, "collision"):
                extra += dim(" (ignoring collision)")

        print(success(f"{symbol} Adding part {part_name}{extra}"))

        if self.part_is_ignored(part_name, "visual") and self.part_is_ignored(
            part_name, "collision"
        ):
            stl_file = None
        else:
            stl_file = self.get_stl(instance)

        # Obtain metadatas about part to retrieve color
        color = self.get_color(instance)

        # Obtain the instance dynamics
        mass, com, inertia = self.get_dynamics(instance)

        # Obtain part pose
        T_world_part = np.array(occurrence["transform"]).reshape(4, 4)

        # Adding non-ignored meshes
        meshes = []
        mesh = Mesh(os.path.relpath(stl_file, self.config.output_directory), color)
        if self.part_is_ignored(part_name, "visual"):
            mesh.visual = False
        if self.part_is_ignored(part_name, "collision"):
            mesh.collision = False
        if mesh.visual or mesh.collision:
            meshes.append(mesh)

        # Get unique part name (with _2, _3 suffixes for duplicates)
        unique_part_name = self.unique_name(instance, "part")

        # Apply geom_properties based on unique part name pattern matching
        for mesh in meshes:
            visual_properties = {}
            collision_properties = {}

            for pattern_name in self.config.geom_properties:
                if fnmatch.fnmatch(unique_part_name, pattern_name):
                    pattern_props = self.config.geom_properties[pattern_name]

                    # Check for nested visual/collision structure
                    has_nested = "visual" in pattern_props or "collision" in pattern_props

                    if has_nested:
                        visual_properties = {
                            **visual_properties,
                            **pattern_props.get("visual", {}),
                        }
                        collision_properties = {
                            **collision_properties,
                            **pattern_props.get("collision", {}),
                        }
                    else:
                        # Apply to both if not nested
                        visual_properties = {**visual_properties, **pattern_props}
                        collision_properties = {**collision_properties, **pattern_props}

            mesh.visual_properties = visual_properties
            mesh.collision_properties = collision_properties

        part = Part(
            unique_part_name,
            T_world_part,
            mass,
            com,
            inertia,
            meshes,
        )

        self.robot.links[-1].parts.append(part)

    def build_robot(self, body_id: int):
        """
        Add recursively body nodes to the robot description.
        """
        instance = self.assembly.body_instance(body_id)

        if body_id in self.assembly.link_names:
            link_name = self.assembly.link_names[body_id]
        else:
            link_name = self.unique_name(instance, "link")

        # Adding all the parts in the current link
        link = Link(link_name)
        self.robot.links.append(link)
        for occurrence in self.assembly.body_occurrences(body_id):
            if occurrence["instance"]["type"] == "Part":
                self.add_part(occurrence)
            if occurrence["fixed"]:
                link.fixed = True

        # Adding frames to the link
        for frame in self.assembly.frames:
            if frame.body_id == body_id:
                self.robot.links[-1].frames[frame.name] = frame.T_world_frame

        for children_body in self.assembly.tree_children[body_id]:
            dof = self.assembly.get_dof(body_id, children_body)
            child_body = dof.other_body(body_id)
            T_world_axis = dof.T_world_mate.copy()

            properties = self.config.joint_properties.get("default", {})
            for joint_name in self.config.joint_properties:
                if fnmatch.fnmatch(dof.name, joint_name):
                    properties = {
                        **properties,
                        **self.config.joint_properties[joint_name],
                    }

            joint = Joint(
                dof.name,
                dof.joint_type,
                link,
                None,
                T_world_axis,
                properties,
                dof.limits,
                dof.axis,
            )
            if dof.name in self.assembly.relations:
                source, ratio = self.assembly.relations[dof.name]
                joint.relation = Relation(source, ratio)

            # The joint is added before the recursive call, ensuring items in robot.joints has the
            # same order as recursive calls on the tree
            self.robot.joints.append(joint)

            joint.child = self.build_robot(child_body)

        return link
