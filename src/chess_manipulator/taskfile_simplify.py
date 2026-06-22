"""Downsamples the dense per-step RRTConnect waypoints that MOVE.py/PICK.py/
PLACE.py (ktmpb_client) write into every Transit/Transfer <Conf> block - same
"keep 1 of every `step` points, always keep the exact last one" logic
mover_robot_simplificado.py uses for the real robot, applied here to the
taskfile itself so kautham-gui playback doesn't crawl through every
unsimplified waypoint.
"""

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
