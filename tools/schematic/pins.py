"""Show what each pin of a part connects to: python3 pins.py nets.json U14"""
import json, sys

nets = json.load(open(sys.argv[1]))
ref = sys.argv[2]
rows = []
for n in nets:
    for p in n["pins"]:
        if p[0] == ref:
            others = [f"{q[0]}.{q[1]}" for q in n["pins"] if q[0] != ref][:6]
            key = int(p[1]) if p[1].isdigit() else 999
            rows.append((key, p[1], p[2], n["net"], others))
for _, pin, name, net, others in sorted(rows):
    print(f"{ref}.{pin:>3} {name:<22} {net[:32]:32} {' '.join(others)}")
