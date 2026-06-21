"""
Corrige os origins visuais/colisão do URDF gerado pelo SolidWorks.

O exportador calcula o origin de cada link como o inverso da junta LOCAL,
mas deveria ser o inverso de toda a cadeia cinemática até aquele link.
"""

import numpy as np
from scipy.spatial.transform import Rotation
import xml.etree.ElementTree as ET

INPUT  = "urdf/adam.urdf"
OUTPUT = "urdf/adam_fixed.urdf"


def parse_origin(elem):
    xyz = [float(v) for v in elem.get("xyz", "0 0 0").split()] if elem is not None else [0,0,0]
    rpy = [float(v) for v in elem.get("rpy", "0 0 0").split()] if elem is not None else [0,0,0]
    return np.array(xyz), np.array(rpy)


def make_T(xyz, rpy):
    T = np.eye(4)
    T[:3,:3] = Rotation.from_euler("xyz", rpy).as_matrix()
    T[:3, 3] = xyz
    return T


def inv_T(T):
    R, t = T[:3,:3], T[:3,3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


def T_to_xyz_rpy(T):
    xyz = T[:3, 3]
    rpy = Rotation.from_matrix(T[:3,:3]).as_euler("xyz")
    return xyz, rpy


def build_tree(root_elem):
    joints = {}
    child_to_joint = {}
    for j in root_elem.findall("joint"):
        name   = j.get("name")
        parent = j.find("parent").get("link")
        child  = j.find("child").get("link")
        xyz, rpy = parse_origin(j.find("origin"))
        joints[name] = dict(parent=parent, child=child, xyz=xyz, rpy=rpy)
        child_to_joint[child] = name
    return joints, child_to_joint


def fk(link_name, joints, child_to_joint, cache={}):
    if link_name in cache:
        return cache[link_name]
    if link_name not in child_to_joint:
        cache[link_name] = np.eye(4)
        return np.eye(4)
    jd = joints[child_to_joint[link_name]]
    T = fk(jd["parent"], joints, child_to_joint, cache) @ make_T(jd["xyz"], jd["rpy"])
    cache[link_name] = T
    return T


def fix(input_path, output_path):
    tree = ET.parse(input_path)
    root = tree.getroot()
    joints, child_to_joint = build_tree(root)
    cache = {}

    for link in root.findall("link"):
        name = link.get("name")
        if name not in child_to_joint:
            continue  # base_link — sem junta pai, não precisa corrigir

        T_world_link   = fk(name, joints, child_to_joint, cache)
        T_visual_correct = inv_T(T_world_link)
        xyz_new, rpy_new = T_to_xyz_rpy(T_visual_correct)

        xyz_str = " ".join(f"{v:.8f}" for v in xyz_new)
        rpy_str = " ".join(f"{v:.8f}" for v in rpy_new)

        print(f"\n{name}")
        print(f"  visual xyz: {xyz_str}")
        print(f"  visual rpy: {rpy_str}")

        for tag in ("visual", "collision"):
            for elem in link.findall(tag):
                origin = elem.find("origin")
                if origin is None:
                    origin = ET.SubElement(elem, "origin")
                origin.set("xyz", xyz_str)
                origin.set("rpy", rpy_str)

    ET.indent(tree, space="  ")
    tree.write(output_path, xml_declaration=True, encoding="utf-8")
    print(f"\nURDF corrigido salvo em: {output_path}")


if __name__ == "__main__":
    fix(INPUT, OUTPUT)
