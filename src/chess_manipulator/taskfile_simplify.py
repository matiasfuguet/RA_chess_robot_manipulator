"""Downsample the dense RRTConnect waypoints in each Transit/Transfer block:
keep 1 of every `step`, plus the exact last one, so kautham-gui playback isn't
slowed by every waypoint. Same idea mover_robot_simplificado.py uses for the
real robot."""

import xml.etree.ElementTree as ET


def simplify_taskfile(taskfile_path, step=20):
    tree = ET.parse(taskfile_path)
    root = tree.getroot()

    for block in root:
        if block.tag not in ("Transit", "Transfer"):
            continue
        confs = block.findall("Conf")
        if len(confs) <= step:
            continue
        kept = set(id(c) for c in confs[::step])
        kept.add(id(confs[-1]))
        for conf in confs:
            if id(conf) not in kept:
                block.remove(conf)

    tree.write(taskfile_path, xml_declaration=True, encoding="utf-8")
