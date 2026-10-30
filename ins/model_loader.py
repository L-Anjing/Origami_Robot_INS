"""Read physical geometry from this project's MJCF; no navigation logic."""
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def vector(element, attribute, default=None):
    text = element.get(attribute, default)
    result = np.fromstring(text, sep=" ") if text is not None else np.array([])
    if result.shape != (3,) or not np.isfinite(result).all():
        raise ValueError(f"Expected three finite values for {element.tag}.{attribute}")
    return result


def load_robot_model(path, geometry_config):
    tree = ET.parse(path).getroot()
    option = tree.find("option")
    if float(option.get("timestep")) != .001 or not np.allclose(vector(option, "gravity"), [0, 0, -9.81]):
        raise ValueError("MJCF timestep/gravity differs from data description")
    world = tree.find("worldbody")
    body = world.find("body[@name='plate0']")
    initial_root = vector(body, "pos")
    offsets, centers, sizes = [], [], []
    for node in range(6):
        if body is None or body.get("name") != f"plate{node}":
            raise ValueError("Expected serial plate0..plate5 body chain")
        if any(body.get(key) is not None for key in ("quat", "euler", "axisangle", "xyaxes", "zaxis")):
            raise ValueError("Non-identity MJCF rest rotation needs an explicit calibration model")
        geom = body.find(f"geom[@name='geom{node}']")
        if geom.get("type") != "box" or any(geom.get(k) is not None for k in ("quat", "euler", "axisangle")):
            raise ValueError("Expected axis-aligned local plate boxes")
        centers.append(vector(geom, "pos", "0 0 0"))
        sizes.append(vector(geom, "size"))  # MJCF box half-sizes
        if node < 5:
            child = body.find(f"body[@name='plate{node+1}']")
            offsets.append(vector(child, "pos"))
            joint = child.find("joint")
            if joint.get("type") != "hinge" or not np.allclose(vector(joint, "pos", "0 0 0"), 0):
                raise ValueError("Only zero-offset hinge origins are supported")
            body = child
    sizes, centers, offsets = np.array(sizes), np.array(centers), np.array(offsets)
    if not np.allclose(sizes * 2, [geometry_config["length_m"], geometry_config["thickness_m"], geometry_config["width_m"]]):
        raise ValueError("User physical dimensions differ from MJCF X-length/Y-thickness/Z-width")
    actuated = [a.get("joint") for a in tree.find("actuator")]
    if actuated != ["hinge0_1", "hinge2_3", "hinge4_5"]:
        raise ValueError("Unexpected actuator order: cannot assign u0/u1/u2")
    cracks = []
    for name in ("crack1", "crack2"):
        geom = world.find(f"geom[@name='{name}']")
        if geom.get("type") != "box":
            raise ValueError("Expected box crack reference boundaries")
        cracks.append({"name": name, "center": vector(geom, "pos"), "half_size": vector(geom, "size")})
    signs = np.array([[-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
                      [-1,-1,1], [1,-1,1], [1,1,1], [-1,1,1]])
    return {"source": str(Path(path).resolve()), "joint_offsets": offsets,
            "marker_link_offsets": np.vstack([offsets, geometry_config["logged_terminal_offset_S_m"]]),
            "nominal_link_offsets": np.tile([geometry_config["length_m"], 0., 0.], (6, 1)),
            "plate_vertices": centers[:, None, :] + sizes[:, None, :] * signs,
            "plate_centers": centers, "plate_half_sizes": sizes,
            "xml_initial_root": initial_root, "cracks": cracks,
            "physical_tip_offset": np.array([geometry_config["length_m"], 0., 0.]),
            "imu_body_offset_S": np.zeros((6, 3))}
