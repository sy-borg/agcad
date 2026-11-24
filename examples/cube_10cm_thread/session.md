# Create a 10 cm cube with Wright tool

**Session ID:** ses_df3bc8ad1ffedAYclJfcWMgcTx
**Created:** 11/24/2026, 7:34:37 AM
**Updated:** 11/24/2026, 7:42:50 AM

---

## User

Use the wright tool to create a cube with 10cm side length.

---

## Assistant

The wright tool supports `add_box` with a size in millimeters (the unit system here is mm). A 10 cm cube = 100 mm per side, so I'll set that up.

---

## Assistant

Document created (id 1). Now adding the cube with 100 mm sides:

---

## Assistant

The box was created with a ±50 mm extent — exactly a 10 cm (100 mm) cube. Let me validate it and render it to confirm:

---

## Assistant

Done! The cube was created successfully via the wright tool.

**What was made:**
- **Primitive:** `add_box` with `size: [100, 100, 100]` (the tool's unit system is millimeters, so 10 cm = 100 mm per side)
- **Position:** centered at the origin
- **Name:** `cube_10cm` (document 1)

**Verification (via `validate`):**
- ✅ Watertight, no open/non-manifold edges, no self-intersections
- ✅ Bounding box: ±50 mm → exactly **100 mm (10 cm) per side**
- ✅ Volume: 1,000,000 mm³ = 1,000 cm³ — correct for a 10 cm cube
- ✅ Minimum wall thickness: 100 mm (solid, no thin features)

I also attempted a render for a visual check, but the current model can't display images, so I relied on the numeric validation instead.

The model is ready in document **1** (`cube_10cm`) if you'd like me to apply fillets/chamfers, export it (STL/3MF), or save it to a file.

---

## User

In the center, from top to bottom create a hole with a M10 thread.

---

## Assistant

Now I'll add an M10 thread as a through-hole. The `thread` command creates a real threaded solid along +Z from an origin, and `boolean difference` will cut it out of the cube.

First, creating the M10 thread (major ⌀10 mm, pitch 1.5 mm, spanning the full 100 mm cube height, centered in X/Y):

---

## Assistant

Thread created as entity 2 (M10, ⌀10mm major / 1.5mm pitch, spanning z −50→50). Now cutting it as a through-hole out of the cube with a boolean difference:

---

## Assistant

The boolean needs both operands — the cube (id 1) and the thread (id 2):

---

## Assistant

Boolean difference succeeded — entity 3 is now the cube with the threaded hole cut out. Let me validate it:

---

## Assistant

The boolean result is valid — watertight, no open or non-manifold edges, no self-intersections.

Now, per the tool's semantics, the boolean keeps its source operands, so I'll delete the original cube and the thread, leaving just the final "cube with hole" solid:

---

## Assistant

Let me run a final validation on the cleaned-up document:

---

## Assistant

The M10 threaded through-hole is complete and validated. ✅

## Summary

**Starting point:** 10 cm cube (100 mm per side, centered at origin, z ∈ [−50, 50] mm)

**Thread creation:**
- `thread` built an M10 thread solid: major diameter 10 mm, minor diameter ≈ 8.27 mm, pitch 1.5 mm, along the Z axis, full 100 mm height

**Cut:** `boolean difference` with the thread as the cutting tool → new solid (entity 3) = cube minus thread

**Cleanup:** deleted the source operands (cube + thread), leaving a single final solid

**Final validation results:**
- ✅ Watertight, no open edges, no non-manifold edges, no self-intersections
- ✅ Bounding box preserved at ±50 mm (100 mm / 10 cm cube)
- ✅ Volume reduced by exactly the thread volume (~6,038 mm³), confirming a clean cut
- ✅ Single solid remains in the document

The final model is a **10 cm cube with a centered, top-to-bottom through-hole featuring an M10 thread** (⌀10 mm × 1.5 mm pitch). It's currently in document 1 if you want to export it (STL/3MF) or apply further features.

---

## User

Please export it as STL and 3MF, also save the .wcad file.

---

## Assistant

I'll export the final model to STL, 3MF, and save the .wcad file:

---

## Assistant

All done! The final deliverables have been created:

| File | Size | Contents |
|---|---|---|
| `/tmp/opencode/cube_10cm_thread.stl` | 1.6 MB | Binary STL, world space, 32,162 triangles (slicer-ready) |
| `/tmp/opencode/cube_10cm_thread.3mf` | 3.2 MB | 3MF package, 1 object, 32,162 triangles (mm units) |
| `/tmp/opencode/cube_10cm.wcad` | 1 KB | Saved `.wcad` program file |

**Final model recap:**
- 10 cm × 10 cm × 10 cm solid cube (100 mm per side, centered at origin)
- M10 threaded through-hole (⌀10 mm major / 1.5 mm pitch) bored from top to bottom through the center
- Result verified: watertight, no open/non-manifold edges, no self-intersections

The STL and 3MF contain the finished solid (cube with the threaded hole cut out), and the `.wcad` file captures the parametric program for future edits.

---

## User

Now create a rendering, one from the top and one perspective, bot whould show the hole with the thread.

---

## Assistant

Now I'll render two views: a top view and a perspective view, both showing the threaded hole.

---

## Assistant

Both renders were created and saved (the inline-image error is just this model not being able to display images here):

- **Top view:** `/tmp/opencode/cube_top.png` — orthographic view from directly above, showing the centered circular hole with the M10 thread
- **Perspective view:** `/tmp/opencode/cube_perspective.png` — 3/4 perspective view showing the same threaded through-hole in context

**Full deliverable summary for this session:**
- Model: 10 cm cube with centered M10 threaded through-hole (top-to-bottom)
- Validation: watertight, no defects
- Exports: STL (`/tmp/opencode/cube_10cm_thread.stl`), 3MF (`/tmp/opencode/cube_10cm_thread.3mf`), source (`/tmp/opencode/cube_10cm.wcad`)
- Renders: top (`/tmp/opencode/cube_top.png`) and perspective (`/tmp/opencode/cube_perspective.png`) views showing the hole
