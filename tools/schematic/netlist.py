"""Netlist from Altium SchDoc geometry (flat, global net labels/ports).

Pin electrical end = LOCATION + PINLENGTH in the pin's direction. Wires join
anything lying on them (ends or mid-segment). Net labels, power ports and
ports name a net and are joined across sheets by name.
"""
import collections, json, sys
from altium import records


def coord(r, k):
    v = int(r.get(f"{k}", 0) or 0)
    frac = int(r.get(f"{k}_FRAC", 0) or 0)
    return v * 100000 + frac  # 1/100000 of 10 mil units


def loc(r):
    return coord(r, "LOCATION.X"), coord(r, "LOCATION.Y")


class UF:
    def __init__(self):
        self.p = {}

    def find(self, a):
        self.p.setdefault(a, a)
        while self.p[a] != a:
            self.p[a] = self.p[self.p[a]]
            a = self.p[a]
        return a

    def union(self, a, b):
        self.p[self.find(a)] = self.find(b)


def on_segment(p, a, b):
    (x, y), (x1, y1), (x2, y2) = p, a, b
    if (x2 - x1) * (y - y1) != (y2 - y1) * (x - x1):
        return False
    return min(x1, x2) <= x <= max(x1, x2) and min(y1, y2) <= y <= max(y1, y2)


def sheet_netlist(path, uf, sheet):
    recs = records(path)
    owner = {i - 1: r for i, r in enumerate(recs)}
    designator, comment = {}, {}
    for r in recs:
        o = r.get("OWNERINDEX")
        if o is None:
            continue
        o = int(o)
        if r.get("RECORD") == "34":
            designator[o] = r.get("TEXT")
        if r.get("RECORD") == "41" and r.get("NAME", "").lower() == "comment":
            comment[o] = r.get("TEXT")

    points = []  # (point, tag)
    segments = []
    for i, r in enumerate(recs):
        kind = r.get("RECORD")
        if kind == "2":  # pin
            o = int(r.get("OWNERINDEX", -1))
            comp = owner.get(o, {})
            part = r.get("OWNERPARTID")
            if part not in (None, "-1") and comp.get("CURRENTPARTID") not in (None, part):
                continue
            if r.get("OWNERPARTDISPLAYMODE", "0") != comp.get("DISPLAYMODE", "0"):
                continue
            x, y = loc(r)
            length = coord(r, "PINLENGTH")
            orient = int(r.get("PINCONGLOMERATE", 0)) & 3
            dx, dy = [(1, 0), (0, 1), (-1, 0), (0, -1)][orient]
            end = (x + dx * length, y + dy * length)
            ref = designator.get(o, "?")
            points.append((end, ("pin", ref, r.get("DESIGNATOR"), r.get("NAME"),
                                 comment.get(o, ""))))
        elif kind == "27":  # wire
            n = int(r.get("LOCATIONCOUNT", 0))
            pts = [(coord(r, f"X{k}"), coord(r, f"Y{k}")) for k in range(1, n + 1)]
            for a, b in zip(pts, pts[1:]):
                segments.append((a, b))
        elif kind in ("25", "17"):  # net label, power port
            points.append((loc(r), ("name", r.get("TEXT"))))
        elif kind == "18":  # port: both ends
            x, y = loc(r)
            w = coord(r, "WIDTH")
            style = int(r.get("STYLE", 0))
            vertical = style in (4, 5, 6, 7)
            for p in ((x, y), (x, y + w) if vertical else (x + w, y)):
                points.append((p, ("name", r.get("NAME"))))

    key = lambda p: (sheet, p)
    for a, b in segments:
        uf.union(key(a), key(b))
    # anything (points or other wires' ends) lying on a segment joins it
    ends = [p for s in segments for p in s] + [p for p, _ in points]
    for p in ends:
        for a, b in segments:
            if on_segment(p, a, b):
                uf.union(key(p), key(a))
    tagged = []
    for p, tag in points:
        uf.find(key(p))
        if tag[0] == "name" and tag[1]:
            uf.union(key(p), ("net", tag[1]))
        tagged.append((key(p), tag))
    return tagged


def main(paths):
    uf = UF()
    tagged = []
    for path in paths:
        tagged += sheet_netlist(path, uf, path)

    nets = collections.defaultdict(lambda: {"names": set(), "pins": []})
    for node, tag in tagged:
        n = nets[uf.find(node)]
        if tag[0] == "name":
            if tag[1]:
                n["names"].add(tag[1])
        else:
            n["pins"].append(tag[1:])
    out = []
    for i, n in enumerate(nets.values()):
        if not n["pins"]:
            continue
        name = "/".join(sorted(n["names"])) or f"N{i}"
        out.append({"net": name, "pins": sorted(set(n["pins"]))})
    json.dump(sorted(out, key=lambda n: n["net"]), sys.stdout, indent=1)


if __name__ == "__main__":
    main(sys.argv[1:])
