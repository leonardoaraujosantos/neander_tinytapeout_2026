# GitHub Actions Workflows

This document describes the CI/CD pipelines configured for the Neander-X TinyTapeout project. All workflows run on Ubuntu 24.04 and are triggered on push events or manual dispatch.

## Overview

| Workflow | File | Purpose | Trigger |
|----------|------|---------|---------|
| **test** | `test.yaml` | Run cocotb simulation tests | Every push |
| **docs** | `docs.yaml` | Generate project documentation | Every push |
| **gds** | `gds.yaml` | Build GDSII layout files | Every push |
| **fpga** | `fpga.yaml` | Generate FPGA bitstream | Manual only |

---

## 1. Test Workflow (`test.yaml`)

**Purpose:** Verifies the Verilog design using cocotb testbench simulations.

### Steps

1. **Checkout repo** - Clones the repository with all submodules
2. **Install iverilog** - Installs Icarus Verilog simulator via apt
3. **Setup Python** - Configures Python 3.11 environment
4. **Install Python packages** - Installs dependencies from `test/requirements.txt` (cocotb, etc.)
5. **Run tests** - Executes the Makefile in `test/` directory:
   - Compiles Verilog sources with iverilog
   - Runs cocotb tests
   - Checks `results.xml` for failures
6. **Test Summary** - Generates a test summary report
7. **Upload artifacts** - Saves waveform (`tb.fst`) and test results for download

### Artifacts Produced
- `test/tb.fst` - Waveform file (viewable with GTKWave or Surfer)
- `test/results.xml` - JUnit-format test results

---

## 2. Docs Workflow (`docs.yaml`)

**Purpose:** Builds project documentation using TinyTapeout's documentation action.

### Steps

1. **Checkout repo** - Clones repository with submodules
2. **Build docs** - Uses `TinyTapeout/tt-gds-action/docs@ttihp26a` to generate documentation

### Output
- Generates datasheet and documentation from `docs/info.md` and project metadata

---

## 3. GDS Workflow (`gds.yaml`)

**Purpose:** Synthesizes the Verilog design and generates GDSII layout files for chip fabrication.

### Jobs

#### 3.1 `gds` (Main Build)
Synthesizes the design using the IHP SG13G2 PDK (Process Design Kit).

**Steps:**
1. **Checkout repo** - Clones repository with submodules
2. **Build GDS** - Uses `TinyTapeout/tt-gds-action@ttihp26a`:
   - PDK: `ihp-sg13g2` (IHP 130nm SiGe BiCMOS)
   - LibreLane version: `3.0.0.dev44`
   - Runs synthesis (Yosys), place & route (OpenLane), and generates GDSII

#### 3.2 `precheck` (depends on `gds`)
Validates the generated GDS against TinyTapeout requirements.

**Steps:**
1. **Run Tiny Tapeout Precheck** - Verifies:
   - Design fits within allocated tile area
   - No DRC (Design Rule Check) violations
   - Correct pin placement and naming

#### 3.3 `gl_test` (depends on `gds`)
Runs gate-level simulation to verify post-synthesis behavior.

**Steps:**
1. **Checkout repo** - Clones repository with submodules
2. **GL test** - Runs cocotb tests against the synthesized netlist (not RTL)

#### 3.4 `viewer` (depends on `gds`)
Deploys an interactive GDS viewer to GitHub Pages.

**Steps:**
1. **Deploy viewer** - Publishes layout visualization to GitHub Pages

**Permissions required:**
- `pages: write` - Deploy to GitHub Pages
- `id-token: write` - Verify deployment source

---

## 4. FPGA Workflow (`fpga.yaml`)

**Purpose:** Generates FPGA bitstream for hardware testing on the TT ASIC Sim board.

**Note:** This workflow is disabled by default (`branches: none`). To enable:
- Remove/comment the `branches: none` line, or
- Trigger manually via `workflow_dispatch`

### Steps

1. **Checkout repo** - Clones repository with submodules
2. **FPGA bitstream** - Uses `TinyTapeout/tt-gds-action/fpga/ice40up5k@ttihp26a`:
   - Target: Lattice iCE40UP5K FPGA
   - Generates bitstream compatible with TT ASIC Sim board

---

## Workflow Dependencies

```
push event
    |
    +---> test.yaml (runs independently)
    |
    +---> docs.yaml (runs independently)
    |
    +---> gds.yaml
              |
              +---> gds job
                      |
                      +---> precheck (after gds)
                      +---> gl_test (after gds)
                      +---> viewer (after gds)
```

---

## Manual Triggers

All workflows support `workflow_dispatch`, allowing manual execution from the GitHub Actions tab:

1. Go to **Actions** tab in GitHub
2. Select the workflow
3. Click **Run workflow**
4. Select branch and confirm

---

## Common Issues

### Test Failures
- Check `results.xml` for specific test failures
- Download `tb.fst` artifact and view with GTKWave for debugging
- Ensure all source files are listed in `test/Makefile`

### GDS Build Failures
- Verify Verilog syntax with `iverilog` locally
- Check that module name matches `tt_um_cpu_leonardoaraujosantos`
- Review synthesis logs for timing/area violations

### Precheck Failures
- Ensure design fits within TinyTapeout tile constraints
- Verify correct pin naming and placement
- Check for DRC violations in the GDS
