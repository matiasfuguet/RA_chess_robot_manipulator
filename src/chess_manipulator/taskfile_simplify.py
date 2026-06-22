"""Downsample the dense RRTConnect waypoints in each Transit/Transfer block:
keep 1 of every `step`, plus the exact last one, so kautham-gui playback isn't
slowed by every waypoint. Same idea mover_robot_simplificado.py uses for the
real robot.

ktmpb_client merges every MOVE/PICK/PLACE solved while an object is attached
into one continuous Transfer block (no separate tags per action) - so a
deliberate stop like a hover waypoint can land anywhere in that block, not
just at a stride boundary. keep_joints lets the caller name UR3 joint configs
(home, hover, grasp) that must survive simplification no matter where they
fall, so a safety-relevant stop never gets silently downsampled away."""

import xml.etree.ElementTree as ET

UR3_JOINT_SLICE = slice(9, 15)


def simplify_taskfile(taskfile_path, step=20, keep_joints=None):
    tree = ET.parse(taskfile_path)
    root = tree.getroot()
    keep_joints = keep_joints or []

    def is_kept_waypoint(conf):
        values = [float(x) for x in conf.text.split()][UR3_JOINT_SLICE]
        return any(all(abs(a - b) < 1e-3 for a, b in zip(values, k)) for k in keep_joints)

    for block in root:
        if block.tag not in ("Transit", "Transfer"):
            continue
        confs = block.findall("Conf")
        if len(confs) <= step:
            continue
        kept = set(id(c) for c in confs[::step])
        kept.add(id(confs[-1]))
        kept.update(id(c) for c in confs if is_kept_waypoint(c))
        for conf in confs:
            if id(conf) not in kept:
                block.remove(conf)

    tree.write(taskfile_path, xml_declaration=True, encoding="utf-8")
