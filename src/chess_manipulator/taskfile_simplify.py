"""Limpia el taskfile que genera kautham/ktmpb sin perder los puntos importantes.

ktmpb a veces escribe movimientos repetidos (PICK/PLACE ya hacen su propia
aproximacion/retirada por dentro) y esto los quita.

"""

import xml.etree.ElementTree as ET

UR3_JOINT_SLICE = slice(9, 15)


def _same(a, b, tol=1e-3):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def _endpoints(confs):
    first = [float(x) for x in confs[0].text.split()][UR3_JOINT_SLICE]
    last = [float(x) for x in confs[-1].text.split()][UR3_JOINT_SLICE]
    return first, last


def _drop_redundant_blocks(root):
    # si no se lleva ninguna pieza, cada accion mete su propio Transit. Cuando
    # PICK/PLACE resuelve "ir al objeto" otra vez (ya resuelto por el MOVE de
    # antes) sale un segundo bloque con el mismo origen/destino: lo borramos
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
    # mientras se lleva una pieza, ktmpb junta varias acciones en un solo
    # Transfer continuo, y dentro de ese bloque aparece la misma redundancia
    # pero como repeticion seguida o como un "bote" de vuelta a un punto ya
    # visitado (subir, bajar otra vez al mismo sitio). Quitamos ambos casos
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
    # separamos el bloque en un sub-bloque por cada tramo real: cortamos en
    # cada punto con nombre (los de keep_joints) para que cada trozo tenga
    # solo los dos extremos de un movimiento de verdad
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
