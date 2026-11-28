"""Minimal Altium SchDoc reader: records from the FileHeader stream."""
import sys, olefile, collections

def records(path):
    ole = olefile.OleFileIO(path)
    data = ole.openstream("FileHeader").read()
    i, out = 0, []
    while i + 4 <= len(data):
        n = int.from_bytes(data[i:i+3], "little"); kind = data[i+3]; i += 4
        blob = data[i:i+n]; i += n
        if kind != 0:
            continue
        text = blob.rstrip(b"\0").decode("latin-1")
        rec = {}
        for part in text.split("|"):
            if "=" in part:
                k, v = part.split("=", 1)
                rec[k.upper()] = v
        out.append(rec)
    return out

if __name__ == "__main__":
    for path in sys.argv[1:]:
        recs = records(path)
        idx = {i - 1: r for i, r in enumerate(recs)}  # OWNERINDEX is 0-based after header
        comps = {i: r for i, r in idx.items() if r.get("RECORD") == "1"}
        desig = {}
        comment = {}
        for r in recs:
            o = r.get("OWNERINDEX")
            if o is None: continue
            o = int(o)
            if r.get("RECORD") == "34": desig[o] = r.get("TEXT")
            if r.get("RECORD") == "41" and r.get("NAME", "").lower() == "comment": comment[o] = r.get("TEXT")
        kinds = collections.Counter(r.get("RECORD") for r in recs)
        print(f"== {path}  ({len(comps)} parts, {kinds.get('25',0)} net labels, {kinds.get('18',0)} ports, {kinds.get('17',0)} power ports)")
        for i, c in sorted(comps.items(), key=lambda kv: desig.get(kv[0]) or ""):
            print(f"  {desig.get(i,'?'):8} {c.get('LIBREFERENCE','')[:30]:30} {(comment.get(i) or c.get('COMPONENTDESCRIPTION',''))[:60]}")
        labels = sorted({r.get("TEXT") for r in recs if r.get("RECORD") in ("25","18","17") and r.get("TEXT") or r.get("NAME") and r.get("RECORD")=="18"} - {None})
        print("  nets/ports:", ", ".join(labels))
