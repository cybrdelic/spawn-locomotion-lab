# Source provenance

This is a selective source snapshot prepared on 2026-10-22 from the local physical-animation experiment. It is not a transcript export and does not include live worktree history.

- Recent controller source: the unified physical runtime and its direct actor, coordinate, goal, muscle, terrain, and bounded-body dependencies.
- Browser baseline: the working source files for the planar and full 3D procedural contact experiments. The original HTML entry pages were recovered from commit `10a1eb8f0c8251bacef1358c046d7260f9a5ae69`; the live pages had become redirects to the avatar policy view.
- Packaging edits: configurable `MIMICKIT_ROOT` replaces a machine-specific path; the transport loads only the explicitly supplied controller factory and creates its own evidence folder; Vite builds the three included entry pages; generated outputs and separately supplied model assets are ignored.
- Model derivation: `support/bounded-humanoid.xml` preserves geometry/inertia from the MimicKit humanoid through the local exported native model, replacing ball joints with bounded intrinsic X-Z-Y axes. Its contract data records the authored limits and mappings. This is source-body physics, not an avatar anatomical fit.
- Upstream checked: MimicKit commit `fbe1a36e878f036d6bf5521defb8eb3b8ab4170d`, tracked `data/assets/humanoid/humanoid.xml`, and its Apache-2.0 license.
- Screenshot: existing `3d-initial.png` from the real procedural baseline.
- GIF: seconds 3 through 11 of the existing `3d-physical-animation.mp4`, downscaled to 720 pixels wide and 12 fps without changing playback speed. It is a historical baseline demonstration, not a new MuJoCo validation run.

No raw chat transcripts, private continuation records, signed URLs, credentials, avatar meshes, motion datasets, trained weights, or ongoing training outputs were copied. Per-file source and staged hashes are retained in the separate local audit packet; machine-local paths are not embedded in this public-facing package.

