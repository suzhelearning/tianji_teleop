"""Optional display-only object geometry shared by simulator and real monitor."""
from pathlib import Path
import xml.etree.ElementTree as ET


OBJECT_MESH_NAME = "mocap_hammer_mesh"
OBJECT_BODY_NAME = "mocap_hammer_visual"


def add_object_mesh(root: ET.Element, object_mesh: Path | None) -> None:
    """Add an invisible, nonphysical hammer template in the OBJ's own coordinates.

    Call after resolving the robot's asset directories. ReferenceOverlay places
    copies of the compiled geom at source object poses, including MuJoCo's mesh
    centering/principal-axis correction; the mocap template is never drawn.
    """
    if object_mesh is None:
        return
    path = Path(object_mesh).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"object mesh not found: {path}")
    if path.suffix.lower() != ".obj":
        raise ValueError(f"object mesh must be an OBJ file: {path}")
    world = root.find("worldbody")
    if world is None:
        raise ValueError("model has no worldbody")
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "mesh", name=OBJECT_MESH_NAME, file=str(path),
                  scale="1 1 1", refpos="0 0 0", refquat="1 0 0 0")
    body = ET.SubElement(world, "body", name=OBJECT_BODY_NAME, mocap="true",
                         pos="0 0 0", quat="1 0 0 0")
    ET.SubElement(body, "geom", name=OBJECT_BODY_NAME, type="mesh", mesh=OBJECT_MESH_NAME,
                  pos="0 0 0", quat="1 0 0 0", group="4", contype="0", conaffinity="0",
                  density="0", mass="0", rgba="1 1 1 0")
