# Third-party notices

## MimicKit humanoid model and observation contract

MimicKit, Copyright 2025 Xue Bin Peng, is distributed under the Apache License 2.0. The inspected upstream checkout is https://github.com/xbpeng/MimicKit at commit `fbe1a36e878f036d6bf5521defb8eb3b8ab4170d`.

The bounded humanoid model in `support/bounded-humanoid.xml` and related contract mappings derive from its humanoid through a native model export. The local modifications replace ball joints with bounded intrinsic X-Z-Y joints and set finite actuator effort/gain contracts. The physical geometry/inertia remains based on the upstream humanoid. A copy of the upstream license is provided at [third_party/MimicKit-LICENSE](third_party/MimicKit-LICENSE).

The runtime imports MimicKit observation helpers from a separately installed checkout. Its source, datasets, downloaded assets, pretrained models, and adapted weights are not vendored here. Obtain each dependency under its applicable terms; the upstream source license is not a blanket license for every separately downloaded motion dataset or character asset.

## Browser and Python dependencies

Three.js, Rapier, Vite, Playwright, Prettier, MuJoCo, NumPy, SciPy, Numba, PyTorch, Gymnasium, and other installed upstream dependencies keep their respective licenses. Dependency source is installed through package managers and is not bundled in this snapshot.

## User code and original avatar

No new license is selected by this packaging operation. The existing JavaScript package metadata declares ISC. No original avatar model, derived avatar rig asset, or avatar recording is included because its redistribution terms were not established in this inspection.
