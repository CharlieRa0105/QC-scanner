# Point Cloud Processing (Phase 2)

> Part of [[Quality Control Scanner]] · companion to [[architecture]]

This document designs the software between "the quality gate passed" and "the
part is marked pass/fail" — turning a captured scan into a traceable
deviation result. It covers [TODO.md](../TODO.md) §2.2 (point cloud
processing) and §2.3 (CAD comparison).

**Decided 2026-07-08: deterministic algorithms only for v1.** A Gemini
deep-research report (`Automated 3D Metrology Pipeline Research.pdf`, kept at
the project root) surveyed both classical and ML approaches for every stage.
Its own conclusion for the stage that matters most — registration, which
produces the number the pass/fail decision is based on — is a **classical**
algorithm (TEASER++), chosen specifically because it is deterministic and
certifiably optimal, not because ML underperformed. This matches the
project's standing constraint: *"prefer deterministic over ML"*
([[Quality Control Scanner]], Working constraints). ML denoising (LaPDA,
diffusion models) is deferred — see [Deferred: ML denoising](#deferred-ml-denoising)
below.

## Pipeline

```
capture (.ply) → [1] clean/denoise → [2] quality gate → [3] register to CAD
  → [4] deviation analysis → heatmap + stats → pass/fail decision
```

This slots into [architecture.md](architecture.md#pipeline) between step 8
(quality gate) and the final decision — see the Architecture doc update.

### Stage 1 — Clean / denoise

Already scoped in the existing quality-gate module, kept as-is:

- **Algorithm:** Statistical Outlier Removal (SOR) with a **locally adaptive
  threshold** — compute k-NN distances via a KD-tree, derive mean/std, and set
  the retention ratio from local density variance instead of a fixed manual
  value. Removes the "re-tune per scan" problem classical SOR normally has.
- **Library:** Open3D (`remove_statistical_outlier`, k-NN via its Tensor API).
- **Module:** [src/quality_gate/outlier_check.py](../src/quality_gate/outlier_check.py)
  (existing) — cleans the cloud *and* reports the removed-ratio for the gate.
- **Not adopted for v1:** MLS/RIMLS surface smoothing. RIMLS is
  computationally heavy and depends on accurate normal estimation on raw,
  noisy scans — fragile precisely where it's needed most. Revisit only if SOR
  alone leaves visible surface noise on real scans.

### Stage 2 — Quality gate

Four checks, combined into one score in
[src/quality_gate/gate.py](../src/quality_gate/gate.py) (existing):

| Check | Algorithm | Library | Module |
|---|---|---|---|
| Density | KD-tree nearest-neighbour distance vs `voxel_size_mm` / `min_points_per_voxel` | Open3D / SciPy KD-tree | `density_check.py` (existing) |
| Outliers | SOR removed-ratio from Stage 1 | Open3D | `outlier_check.py` (existing) |
| **Surface roughness** *(new)* | PCA on local spherical neighbourhoods; eigenvalues $\lambda_0\le\lambda_1\le\lambda_2$ of the covariance matrix give $\sigma_p = \lambda_0/(\lambda_0+\lambda_1+\lambda_2)$. Flag if median $\sigma_p$ over planar regions exceeds the sensor's noise floor. | NumPy/SciPy (Open3D KD-tree for neighbourhoods) | `roughness_check.py` (**new**) |
| Holes / coverage | Compare scan coverage to the CAD's expected surface (ray-cast from the corner-reference frame) | Open3D | `hole_detection.py` (existing) |

**Roughness is a genuinely new check** — it catches noisy-but-dense scans
that density and outlier-ratio both miss (a flat surface can be dense and
have few statistical outliers while still being rough). Worth adding.

**Sensor-confidence check — not adopted.** The research report's fourth
metric (mean intensity/return vs a calibration threshold) assumes a **LiDAR**
sensor. The MIRACO Plus is **structured-light**, not LiDAR — it doesn't
expose per-point return intensity the same way. If Revopoint's SDK exposes an
equivalent per-point confidence value, add it as a fifth check later; don't
fabricate one now.

### Stage 3 — Register scan to CAD

**Decided 2026-07-08: TEASER++ global registration + ICP refine, always
(not a fallback).** Because of the corner-reference decision
([[architecture]]), the part's nominal pose relative to the arm base is
already known at scan time, so a plain-ICP-first approach was on the table as
a simpler v1 (it needs only Open3D, already in the stack). The owner's call
was to build the robust path from the start rather than add TEASER++ later
as a patch — one dependency to bring in now instead of a re-architecture
later if operator placement ever exceeds ICP's convergence radius.

| | Algorithm | Library |
|---|---|---|
| Global alignment | FPFH features + TEASER++ (Truncated Least Squares + GNC) | `teaserpp_python` |
| Refinement | Point-to-Plane ICP, initialised from the TEASER++ solution | `open3d.pipelines.registration.registration_icp` |

**Module (new):** `src/registration/align_to_cad.py`.

**Risk to close out before relying on this:** TEASER++ is a C++ library with
Python bindings, not a pure-Python package. This project has had real
friction installing vendor/compiled dependencies before (`pyrealsense2`,
`rokae_ros2`, xCore SDK) — confirm `pip install teaserpp-python` gives a
working wheel on Ubuntu 26.04 before depending on it; see
[Open questions](#open-questions).

### Stage 4 — Deviation analysis

- **Algorithm:** Cloud-to-Mesh (C2M) signed-distance projection — for each
  scan point, the perpendicular distance to the nearest CAD triangle (not
  point-to-point, which has interpolation error). Computed efficiently via a
  **Bounding Volume Hierarchy (BVH)** over the CAD mesh rather than linear
  search.
- **Library:** Open3D's `RaycastingScene` (`compute_signed_distance`) — CPU,
  no GPU required, handles millions of points against a tessellated CAD mesh.
  Sign indicates excess material (outside) vs missing material (inside).
- **Stats:** mean error, std dev, RMSE (NumPy/SciPy) against the array of
  signed distances.
- **Heatmap:** map signed distance to a colour scale (e.g. `coolwarm`) at the
  `tolerance_mm` band, write as a coloured `.ply` and/or an interactive Plotly
  HTML export for the inspection report.
- **Module (new):** `src/inspection/deviation.py`.

GD&T checks and the final pass/fail decision logic ([TODO.md](../TODO.md)
§2.3) consume this deviation array but aren't designed here — they depend on
per-feature tolerances that aren't defined yet (`scanner.tolerance_target_um`
is still `TBD` in [config/system_config.yaml](../config/system_config.yaml)).

## Software stack (v1, deterministic)

| Stage | Library |
|---|---|
| Data I/O | Open3D (Tensor API) — `.ply`/`.stl` native read/write |
| Clean/denoise | Open3D |
| Quality gate | Open3D, NumPy, SciPy (KD-tree, PCA) |
| Registration | `teaserpp_python` (TEASER++ global alignment) + Open3D (ICP refine) |
| Deviation | Open3D (`RaycastingScene`) |
| Visualization | Matplotlib / Plotly (headless heatmap export) |

No PyTorch3D, no GPU dependency for v1 — everything above runs on CPU.

## Config additions

New keys for [config/system_config.yaml](../config/system_config.yaml)
(added as `TBD` pending real-scan tuning, following the project's existing
convention of not hard-coding constants):

```yaml
quality_gate:
  # ...existing keys...
  roughness_pca_radius_mm: TBD   # neighbourhood radius for PCA surface-variation check
  max_roughness_sigma: TBD       # sensor noise floor — set from real scans

registration:
  method: icp                    # icp | icp_with_teaser_fallback — see Stage 3 decision above
  icp_max_correspondence_distance_mm: TBD

deviation:
  tolerance_mm: TBD              # reuse scanner.tolerance_target_um once resolved — don't duplicate
```

## Deferred: ML denoising

The research report's LaPDA (latent-space diffusion denoising) is left out of
v1, not rejected outright:

- It's a 2024–2026 research paper, not a shipped library — adopting it means
  reimplementing a paper or depending on an early research repo, unlike
  Open3D/TEASER++ which are mature and pip-installable.
- Supervised/learned denoisers need training data matched to the MIRACO
  Plus's actual noise profile — nothing to validate against before real
  hardware scans exist.
- Conflicts with the project's "prefer deterministic over ML" constraint for
  a system whose output is a manufacturing pass/fail decision.

Revisit if, once real scan data exists, classical SOR demonstrably leaves the
gate failing on noise the CAD comparison shows is real surface finish rather
than sensor artefact.

## Open questions

1. **TEASER++ install feasibility** — confirm `teaserpp_python` installs
   cleanly (wheel vs build-from-source) on the target Ubuntu 26.04 machines
   before relying on it as the default registration path — see Stage 3.
2. **Roughness threshold** — `max_roughness_sigma` needs the sensor's actual
   noise floor from real MIRACO Plus scans.
3. **Sensor-confidence check** — does the RevoLink SDK expose a per-point
   confidence/quality value? If yes, add as a 5th gate metric.
4. **Deviation tolerance** — blocked on `scanner.tolerance_target_um`
   (project-wide open question, not specific to this doc).
