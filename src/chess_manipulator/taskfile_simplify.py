"""Clean up the taskfile in three ways - all working around the same root
cause: ktmpb_client's PICK/PLACE solve their own approach/retreat independently
of whatever the previous explicit MOVE action already did, so the same hop
sometimes gets solved (and executed) twice in a row.

1. Drop whole redundant blocks. When not carrying anything, each action gets
   its own separate Transit tag - so the duplicate hop shows up as a second,
   separately-tagged block with the exact same start and end as the one right
   before it. _drop_redundant_blocks removes it entirely.

2. Drop redundant motion within a block. While carrying an object, ktmpb_client
   merges every action's path into one continuous Transfer block instead - so
   the same duplicate hop shows up *inside* one block as an exact consecutive
   repeat, or a trailing bounce back to an already-visited position (reach the
   target, then go back to hover and straight back down to the same spot).
   _drop_redundant_revisits removes both.

3. Downsample what's left. Default: keep 1 of every `step` points, plus the
   exact last one, so kautham-gui playback isn't slowed by every dense
   RRTConnect waypoint. With checkpoints_only=True (what the real-robot
   pipeline uses): split each block into one sub-block per real hop, at every
   point matching a named keep_joints config (home, hover, grasp) - the rest
   of each hop's points is interpolation Kautham already verified, not a stop
   the robot needs to make. This matters because a single Transfer block can
   span several real actions merged together (ktmpb_client keeps one tag open
   for as long as an object stays attached) - e.g. one piece's grasp pose,
   its hover, the destination's hover, and the destination's grasp pose, all
   in one block. Splitting at every named checkpoint means every resulting
   block has exactly the two endpoints of one real hop, so a downstream
   consumer can safely reduce each block to just its first and last point
   without ever skipping a hover that was buried in the middle."""

import xml.etree.ElementTree as ET

UR3_JOINT_SLICE = slice(9, 15)


def _same(a, b, tol=1e-3):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def _endpoints(confs):
    first = [float(x) for x in confs[0].text.split()][UR3_JOINT_SLICE]
    last = [float(x) for x in confs[-1].text.split()][UR3_JOINT_SLICE]
    return first, last


def _drop_redundant_blocks(root):
    prev_endpoints = None
    for block in [b for b in root if b.tag in ("Transit", "Transfer")]:
        confs = block.findall("Conf")
        if not confs:
            continue
        endpoints = _endpoints(confs)
        if prev_endpoints and _same(prev_endpoints[0], endpoints[0]) and _same(prev_endpoints[1], endpoints[1]):
            root.remove(block)
        else:
            prev_endpoints = endpoints


def _drop_redundant_revisits(confs):
    items = [(c, [float(x) for x in c.text.split()][UR3_JOINT_SLICE]) for c in confs]
    if not items:
        return []

    deduped = [items[0]]
    for c, v in items[1:]:
        if not _same(deduped[-1][1], v):
            deduped.append((c, v))

    while len(deduped) >= 3:
        last_v = deduped[-1][1]
        match = next((i for i in range(len(deduped) - 2, -1, -1) if _same(deduped[i][1], last_v)), None)
        if match is None:
            break
        deduped = deduped[: match + 1]

    return [c for c, v in deduped]


def _split_into_hops(block, confs, is_kept_waypoint):
    """One new block per real hop: from one named checkpoint to the next.
    Every checkpoint becomes the start of one sub-block and the end of the
    previous one, so nothing in between two checkpoints needs to be kept."""
    kept_ids = {id(confs[0]), id(confs[-1])}
    kept_ids.update(id(c) for c in confs if is_kept_waypoint(c))
    checkpoints = [c for c in confs if id(c) in kept_ids]

    segments = []
    for a, b in zip(checkpoints, checkpoints[1:]):
        segment = ET.Element(block.tag, block.attrib)
        for c in (a, b):
            conf = ET.SubElement(segment, "Conf")
            conf.text = c.text
        segments.append(segment)
    return segments


def simplify_taskfile(taskfile_path, step=20, keep_joints=None, checkpoints_only=False):
    tree = ET.parse(taskfile_path)
    root = tree.getroot()
    keep_joints = keep_joints or []

    def is_kept_waypoint(conf):
        values = [float(x) for x in conf.text.split()][UR3_JOINT_SLICE]
        return any(_same(values, k) for k in keep_joints)

    _drop_redundant_blocks(root)

    for block in [b for b in root if b.tag in ("Transit", "Transfer")]:
        confs = block.findall("Conf")
        for conf in confs:
            block.remove(conf)
        confs = _drop_redundant_revisits(confs)

        if checkpoints_only:
            segments = _split_into_hops(block, confs, is_kept_waypoint)
            idx = list(root).index(block)
            root.remove(block)
            for offset, segment in enumerate(segments):
                root.insert(idx + offset, segment)
            continue

        if len(confs) <= step:
            kept_confs = confs
        else:
            kept = set(id(c) for c in confs[::step])
            kept.add(id(confs[-1]))
            kept.update(id(c) for c in confs if is_kept_waypoint(c))
            kept_confs = [c for c in confs if id(c) in kept]

        for conf in kept_confs:
            block.append(conf)

    tree.write(taskfile_path, xml_declaration=True, encoding="utf-8")
