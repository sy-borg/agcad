"""Render a document to a PNG with the headless software rasterizer.

Usage: .venv/bin/python examples/render_scene.py [document_name] [out.png]

Connects to a running engine (WRIGHT_SERVER or target/debug/wright --tcp 7860),
renders the named document with an auto-framed 3/4 camera, and writes the PNG.
Same scene + camera -> bit-identical bytes, every time.
"""

import base64
import sys

sys.path.insert(0, "python")  # noqa: E402

from wright import Wright  # noqa: E402


def main() -> None:
    name = sys.argv[1] if len(sys.argv) > 1 else None
    out = sys.argv[2] if len(sys.argv) > 2 else "render.png"

    cad = Wright.connect()
    docs = {d["name"]: d["doc"] for d in cad.list_documents()}
    if name is None:
        name, doc = next(iter(docs.items())) if docs else ("", None)
        if doc is None:
            sys.exit("no documents in the engine; draw something first")
        name = next(n for n, d in docs.items() if d == doc)
    elif name not in docs:
        sys.exit(f"unknown document {name!r}; have {sorted(docs)}")
    else:
        doc = docs[name]

    r = cad.render_offscreen(doc, width=800, height=600, path=out)
    assert r["result"] == "rendered", r
    png = base64.b64decode(r["png_base64"])
    assert png[:8] == bytes([0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A])
    print(f"rendered doc {doc} ({name}) -> {r['path']} ({len(png)} bytes, {r['width']}x{r['height']})")
    cad.close()


if __name__ == "__main__":
    main()
