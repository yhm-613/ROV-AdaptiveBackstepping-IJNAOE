# Adaptive Backstepping Control for a 6-DOF ROV — Code, Models and Data

Companion repository for the manuscript

> **Adaptive Backstepping Control for Six-Degree-of-Freedom ROV Trajectory Tracking under Hydrodynamic Parameter Uncertainty**
> Haomiao Yang, Guanghui Chang\*, Pan Su, Yanhang He, Xincheng Shan, Jiasen Lin
> Naval University of Engineering, Wuhan, China (\* corresponding author)

The controller replaces the two constant feedback gains of classical backstepping with
grouped six-dimensional adaptive gains (position / velocity error channels) embedded with
sigma-modification. This repository contains all Simulink models, the HIL (hardware-in-the-loop)
firmware and host models, the raw benchmark datasets behind every figure and table of the
paper, and the verification scripts.

---

## Repository layout

| Folder | Content |
|---|---|
| `01_Simulink_Models/` | Closed-loop Simulink models of the four controllers (PID / Backstepping / Adaptive / SMC baseline), step and circle-trajectory scenarios, plus the shared plant parameter file `rovmodel75kg.mat` (75 kg Chin & Lau benchmark ROV) |
| `01_Simulink_Models/SMC_R2023b/` | Boundary-layer SMC baseline exported to R2023b format (`ExportToVersion`), see its own README (Chinese) |
| `02_HIL/` | Hardware-in-the-loop workspace: STM32F407 firmware, Embedded Coder deployment model, host models, serial-link scripts, and the raw board-level datasets in `out/` |
| `03_Simulation_Results/` | `paper_metrics.json` and `smc_metrics.json` — the reference metrics used in the paper, computed from the raw datasets |
| `04_Analysis_Scripts/` | Verification and plotting scripts (see below) |

## Environment

- **MATLAB / Simulink R2024b** (datasets were produced in a single R2024b session);
  R2023b also works — the SMC baseline is provided as an R2023b-compatible export, and a
  full 64-case re-run on R2023b reproduced the reported metrics (step cases bitwise,
  circle/case RMS within <1 % except the chattering-sensitive SMC/BS nominal-circle totals,
  see `04_Analysis_Scripts/compare_repro.py` / `smc_compare.py`).
- Simulink, Control System Toolbox, Embedded Coder (HIL code generation only).
- Python 3.10+ with `numpy`, `h5py` (`.mat` files are v7.3/HDF5) for the verification scripts.
- HIL hardware (optional): YS-F4Pro board (STM32F407IGT6, 168 MHz), UART bridge @ 460800 baud.

## Reproducing the paper

### 1. Nominal simulations

Open a model in `01_Simulink_Models/<Controller>/`, run it (ode45, `MaxStep = 1e-3`,
StopTime 100 s). Each folder contains the step (`*_0.slx` / `*_step`) and circle
(`*_circle.slx`) closed-loop models; `smc_aug.m` must stay next to the SMC models
(exported PreLoadFcn loads `rovmodel75kg.mat` from the current folder).

### 2. Verify reported metrics

```bash
python 04_Analysis_Scripts/full_verify.py
```

recomputes 139 metrics (step rise/settle/overshoot/steady-state RMS, circle full-run and
steady-state RMS, mismatch and current growth percentages) from the raw datasets and
compares them against the values printed in the manuscript — 0 failures.

### 3. Raw datasets (GitHub Release)

The raw `.mat` result files (46 re-benchmark cases `RE_*`, the mismatch study
`results_m1v5`, in total ≈750 MB) are attached as ZIP assets of the
**[v1.0.0 release](../../releases)** of this repository:

| Asset | Content |
|---|---|
| `results_new_benchmark.zip` | `RE_*` single-environment R2024b runs: nominal / current (const, slow) / measurement-noise / saturation for PID, Backstepping, Adaptive, SMC + `paper_metrics.json` + `smc_metrics.json` |
| `results_m1v5_mismatch.zip` | Controller-side parameter mismatch study (±20/40/60 %), step and circle scenarios |

`eout` columns 1–3 are the **actual position** η (subtract the reference
`[2cos(0.2t), 2sin(0.2t), 2.0]` to obtain the tracking error); columns 4–6 are attitude
in degrees.

### 4. HIL experiments

Follow `02_HIL/docs/` (Chinese): firmware build in STM32CubeIDE 1.19.0, serial protocol
(104/34 B frames, CRC-16 Modbus, 460800 baud), then `scripts/run_hil_paper.m` for the
20 s / 2001-step board runs that produce `out/hil_paper_A…E.mat`.

## Script notes

- `master_repro.m`, `rerun_noise.m`, `smc_repro.m` — standalone R2023b re-run drivers used
  for the independent local reproduction campaign (they create noise/saturation/current
  model variants programmatically). They contain hard-coded local paths and are meant as
  documentation of the exact procedure; adjust paths before reuse.
- `compare_repro.py`, `smc_compare.py` — compare local re-runs against the packaged datasets.
- `redraw_circle_2d.py`, `make_graphical_abstract.py` — reproduce Figures 6–8 and the
  graphical abstract from the raw datasets.

## Data conventions

- Solver: ode45, `MaxStep = 1e-3` (current cases of the SMC baseline: 5e-4).
- Process noise: Random Number block, variance 0.001 (acceleration channel), default seed.
- Measurement noise: band-limited white noise, σ = 0.01 m (position) / 0.5° (attitude), 1 ms.
- Thruster saturation: ±100 V nominal, ±50 V in the authority-reduction study.

## License

Code and models are released under the [MIT License](LICENSE).
The raw datasets are additionally released under CC-BY-4.0.
