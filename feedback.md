# Agent feedback

## Digest

- Document id guessing. I assumed the first document would be id 0; it came
  back as 1. Within a single batch you can't reference the doc you just created,
  so "create → populate" always costs two round trips.
- Axis convention surprise. The docs say Z is up, but add_cylinder built the
  height along Y — I had to rotate it 90° to stand it up. That's opposite to
  what most CAD tools (OpenSCAD, etc.) do.
- Misleading scene data. After the rotation, agcad_scene still reported the old
  bounding box — it turns out it's the local mesh bbox, not world-space. The render,
  not the scene inspection, is what confirmed my fix worked. That's a small
  observability gap that could send someone chasing a bug that isn't there.
- `list_documents` hint is a dead end. When my batch failed, the error hint said
  "run create_document first, or list_documents" — but list_documents isn't in
  the documented command list. Either it exists and should be documented (it would
  also solve the doc-id discovery problem nicely), or the hint references a command
  that doesn't exist yet.
- Suggestion for the doc-id issue: letting create_document accept an explicit id,
  or supporting a symbolic reference like "doc": "newest" in later batch commands,
  would make single-round-trip authoring possible.
- World-space bbox: even just reporting both local and world bbox in agcad_scene
  would remove the ambiguity.

## Raw

> Nothing else to do, just some questions. How was it to use, was it intuitive or complicated?

Honest answer: mostly intuitive, with a couple of rough edges. Let me also save what I learned for future sessions, since these quirks weren't obvious:

What was intuitive:

- The tools are self-describing — agcad_execute's docs listed every command and parameter, so I never had to guess syntax.
- The parametric workflow feels right: I defined diameter and height as named parameters and wrote radius: "diameter/2" as an expression instead of hardcoding 25. Change the parameter, geometry rebuilds.
- Error recovery was smooth. My first batch failed because I guessed the wrong document id, and the runner stopped at that step, reported exactly what executed, and told me to re-send the rest. One retry, done.
- The inline render is the best part. Seeing the cylinder let me actually verify the result rather than trusting numbers.

What was complicated:

- Document id guessing. I assumed the first document would be id 0; it came back as 1. Within a single batch you can't reference the doc you just created, so "create → populate" always costs two round trips.
- Axis convention surprise. The docs say Z is up, but add_cylinder built the height along Y — I had to rotate it 90° to stand it up. That's opposite to what most CAD tools (OpenSCAD, etc.) do.
- Misleading scene data. After the rotation, agcad_scene still reported the old bounding box — it turns out it's the local mesh bbox, not world-space. The render, not the scene inspection, is what confirmed my fix worked. That's a small observability gap that could send someone chasing a bug that isn't there.

Verdict: intuitive overall, maybe 8/10. The mental model (parameters → primitives → transform → verify visually) is exactly how parametric CAD should work. The friction was a couple of avoidable extra round trips, not confusion — once you know to trust the render over the reported bbox, it's smooth.

> This program is in a very early state. That was good feedback, I'll add them to the TODO list.

Glad it was useful! Since you're collecting TODO items, a few more small things I noticed but didn't mention:
- list_documents hint is a dead end. When my batch failed, the error hint said "run create_document first, or list_documents" — but list_documents isn't in the documented command list. Either it exists and should be documented (it would also solve the doc-id discovery problem nicely), or the hint references a command that doesn't exist yet.
- Suggestion for the doc-id issue: letting create_document accept an explicit id, or supporting a symbolic reference like "doc": "newest" in later batch commands, would make single-round-trip authoring possible.
- World-space bbox: even just reporting both local and world bbox in agcad_scene would remove the ambiguity.
And one positive: the hint field on failures is genuinely good design — it turned every error I hit into an obvious next step rather than a debugging session.
Let me note for future sessions that feedback on agcad is welcome:
+1 thought, 1 tool
Noted for the future — I'll keep collecting friction points whenever I use agcad again. Good luck with the early-state work; the core design (parametric program + replay, render-to-verify, hints on failure) is solid, so it's mostly polish from here.

