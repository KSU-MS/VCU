"""Trace a net back to the harness connector through series parts.

python3 trace.py nets.json NET [CONNECTOR]
Walks through two-pin parts (R, L, FB, C excluded since they shunt to
ground) and op-amp buffers (IN+ -> output), stopping at power nets.
"""
import json, sys

nets = json.load(open(sys.argv[1]))
start = sys.argv[2]
conn = sys.argv[3] if len(sys.argv) > 3 else "J1"

by_name = {n["net"]: n for n in nets}
pin_net = {}
for n in nets:
    for p in n["pins"]:
        pin_net[(p[0], p[1])] = n["net"]

POWER = ("GND", "+", "Teensy 3.3v", "VCC", "3V3")


def is_power(net):
    return any(net.startswith(p) or net == p for p in POWER)


def neighbours(net):
    for ref, pin, name, comment in by_name[net]["pins"]:
        if ref[0] in "RLF" and not ref.startswith("FB") or ref.startswith("FB"):
            other = "2" if pin == "1" else "1"
            if (ref, other) in pin_net:
                yield f"{ref}({comment})", pin_net[(ref, other)]


seen, frontier, paths = {start}, [(start, [start])], []
while frontier:
    net, path = frontier.pop(0)
    for p in by_name[net]["pins"]:
        if p[0] == conn:
            paths.append(path + [f"{conn}.{p[1]}"])
    for via, nxt in neighbours(net):
        if nxt not in seen and not is_power(nxt):
            seen.add(nxt)
            frontier.append((nxt, path + [via, nxt]))

for p in paths:
    print("  " + " -> ".join(p))
if not paths:
    print("  (no path to", conn + ")")
