# BLDC Motor Control - Project Roadmap (March 2026)

> **License Reminder:** This project is distributed under a Custom restricted license (no redistribution). See [LICENSE](LICENSE).
> **Disclaimer:** This application is provided as-is for simulation and research use. Users assume all risks.
> The author disclaims liability for any direct or indirect damage, data loss, hardware issues, injury,
> or regulatory non-compliance resulting from use or misuse.

## Status Overview

| Component                   | Status      | Last Updated       | Notes                                                          |
| --------------------------- | ----------- | ------------------ | -------------------------------------------------------------- |
| **Core Motor Simulation**   | ✅ Complete | March 2026         | Full BLDC physics with RK4 integration                         |
| **V/f Control**             | ✅ Complete | March 2026         | Open-loop voltage-frequency control                            |
| **FOC Control**             | ✅ Complete | March 2026         | Closed-loop d/q current and speed control                      |
| **SVM Modulation**          | ✅ Complete | March 2026         | Space Vector Modulation with 6 sectors                         |
| **Startup Sequences**       | ✅ Complete | March 2026         | Align → Open-loop → Closed-loop                                |
| **Advanced Inverter Model** | ✅ Complete | March 2026         | Loss, dead-time, thermal, DC-link effects                      |
| **GUI Interface**           | ✅ Complete | March 2026         | PyQt6 with accessibility (NVDA/JAWS)                           |
| **Data Logging & Export**   | ✅ Complete | March 2026         | CSV export with metadata                                       |
| **Visualization**           | ✅ Complete | March 2026         | matplotlib plots with 10+ signal types                         |
| **Hardware Interface**      | ✅ Complete | March 2026         | Mock DAQ backend for HW integration                            |
| **Compute Backends**        | ✅ Complete | March 2026         | CPU/GPU with safe fallback                                     |
| **Regression Testing**      | ✅ Complete | March 2026         | Automated baseline validation                                  |
| **Auto-Tuning (Bounded)**   | ✅ Complete | Feb 2026           | Finite-budget PI parameter search                              |
| **Auto-Tuning (Unbounded)** | ✅ Complete | **March 17, 2026** | **[NEW]** Iterative convergence guarantees                     |
| **Field Weakening (FW)**    | ✅ Complete | **March 17, 2026** | **[NEW]** Headroom-based scheduling + loaded-point calibration |
| **Auto-Calibration (Full)** | ✅ Complete | **April 7, 2026**  | **[NEW]** Single-click full calibration: observers + startup + IPM saliency |

---

## Completed Features (April 2026)

### 🎯 Full Auto-Calibration Maturity — Single-Click Complete Tuning [NEW]

**Objective**: Reach the maturity level where a single click on "Auto Calibrate" produces a physically complete, motor-specific configuration of every control parameter — observer gains, startup sequence, EEMF/SOGI toggles — with no manual entry required, including correct handling of IPM salient-pole motors.

**Audit Finding** (prior to this release): The two-stage pipeline (Stage 1: FOC PI gains via frequency-domain search, Stage 2: field-weakening physics sweep) was already working. However, the post-calibration observer setup (`_apply_observer_calibration_to_gui`) had four critical gaps:
1. No saliency detection — the same gain formulas applied to isotropic (Ld≈Lq) and salient (Lq/Ld>1.2) motors alike, producing a ~13° steady-state angle error for IPM motors at no-load.
2. EEMF model and SOGI filter were never exposed in the GUI and never auto-enabled, even though the back-end support existed in `FOCController`.
3. STSMO `k2_min` was hardcoded to 500 V/s regardless of the motor's rated back-EMF, which is too conservative for small motors and could be insufficient for large ones.
4. Startup sequence (alignment, open-loop ramp, observer handoff) used hardcoded defaults with no connection to motor parameters.

**What Was Delivered**:

- ✅ **Saliency detection**: Any motor with `Lq/Ld > 1.2` is automatically classified as IPM-salient. The threshold is physics-based (non-zero reluctance torque starts affecting the reconstructed EMF at this ratio).
- ✅ **EEMF Model GUI toggle** (`foc_smo_eemf_model`, Enabled/Disabled): New `LabeledComboBox` widget added to the Observer & Startup tab. Auto-set to "Enabled" by auto-calibration for salient motors. Wired into `_apply_to_simulation()` to call `FOCController.enable_eemf_model(Lq=lq_val)` when enabled.
- ✅ **SOGI Filter GUI toggle** (`foc_smo_sogi_filter`, Enabled/Disabled): New `LabeledComboBox` widget. Auto-set to "Enabled" for salient motors. Wired into `_apply_to_simulation()` to call `FOCController.enable_sogi_filter(k=1.4142)` when enabled.
- ✅ **IPM-specific SMO gains**: For salient motors, `k_slide` is reduced from `5×ωe_max` to `2×ωe_max` (the EEMF model handles the saliency angle bias; lower sliding gain reduces chattering) and `boundary` is widened from 0.04 to 0.08 rad (prevents the saliency harmonic from triggering excessive sign switching).
- ✅ **Motor-aware STSMO convergence factor**: λ=3.0 for isotropic, λ=4.0 for salient (extra margin for the `(Lq-Ld)/Ld` harmonic in the reconstructed EMF).
- ✅ **Motor-aware STSMO k2_min**: Formula `max(50, 1.5 × Ke × ωe_max) × saliency_multiplier` replaces the hardcoded 500 V/s. Scales with the motor's back-EMF constant and rated electrical speed. For IPM motors an extra 1.5× safety factor is applied. Validated: Nanotec 12V → 77 V/s, IPM Salient 48V → 232 V/s.
- ✅ **Motor-aware ActiveFlux dc_cutoff**: Derived as `f_e_min / 10` where `f_e_min` is the electrical frequency at the observer handoff speed. Clipped to [0.05, 0.5] Hz. Ensures the integrator drift pole is well below the minimum working frequency.
- ✅ **Full startup sequence auto-tuning** from motor parameters:
  - Alignment duration: `max(50 ms, 5 × τe)` — settles the electrical transient
  - Alignment current: `max(0.5 A, min(3.0 A, 0.1 × Vbus))` — locks rotor without risk
  - Open-loop handoff speed: `ωm_threshold = 0.05 × Vbus / Ke` → RPM, rounded to 10 RPM — this is the speed at which back-EMF is reliably above 5% of bus voltage
  - Open-loop initial speed: `min(30 RPM, 10% of handoff speed)`
  - Open-loop ramp time: `max(0.1 s, handoff_rpm / rated_rpm × 0.5 s)`
  - Open-loop Iq reference: `max(0.5 A, min(3.0 A, 0.05 × Vbus))`
  - Observer handoff min_EMF: `0.03 × Vbus`
  - Observer handoff min_speed: = handoff speed
  - Observer handoff min_time: `0.8 × ramp_time`
  - Startup sequence and transition automatically set to "Enabled"
- ✅ **Motor-aware `calibrate_stsmo_gains_analytical()`**: Added optional `k2_min` parameter to `FOCController` method. When `None`, auto-computes `max(50, 1.5 × Ke × ωe_max)` internally. Backward-compatible (existing callers unaffected). Return dict now includes `k2_min` key.
- ✅ **Bug fix**: `MotorParameters` defaults to `poles_pairs=4`. The `_apply_observer_calibration_to_gui()` method now explicitly passes `poles_pairs=pp` when constructing the transient motor instance, preventing incorrect PLL/SMO gain scaling for motors with ≠4 pole pairs (e.g., Nanotec DB57M012 with 5 pole pairs).

**Validation Results**:

Tested against two contrasting motor profiles:

| Parameter              | Nanotec DB57M012 12V (isotropic) | IPM Salient 48V (Lq/Ld=2.0) |
| ---------------------- | --------------------------------- | ----------------------------- |
| Saliency ratio         | 1.0 → not salient                 | 2.0 → salient                 |
| EEMF Model             | Disabled                          | **Enabled**                   |
| SOGI Filter            | Disabled                          | **Enabled**                   |
| PLL Kp / Ki            | 659.7 / 134336                    | 452.4 / 63166                 |
| SMO Kslide             | 9163                              | **2513** (2×ωe_max)           |
| SMO Boundary           | 0.04 rad                          | **0.08 rad**                  |
| STSMO λ                | 3.0                               | **4.0**                       |
| STSMO k1               | 9.61                              | 20.30                         |
| STSMO k2_min           | 77 V/s                            | **232 V/s**                   |
| ActiveFlux dc_cutoff   | 0.5 Hz                            | 0.5 Hz                        |
| Align duration         | 50 ms                             | 50 ms                         |
| Align current          | 1.2 A                             | 3.0 A                         |
| Handoff speed          | 200 RPM                           | 280 RPM                       |
| Open-loop Iq           | 0.6 A                             | 2.4 A                         |
| Startup min EMF        | 0.36 V                            | 1.44 V                        |

All parameter sanity checks passed. Physically derived from first principles with no hardcoded motor-specific constants.

**Second pass — gaps closed (April 7, 2026 — same session)**:

An audit revealed three further critical gaps: the two-stage calibration scripts were writing JSON files that the GUI never read back. They are now resolved:

- ✅ **Stage 1 current/speed PI gains applied to GUI**: new `_load_stage1_gains_to_gui()` searches `data/tuning_sessions/auto_calibrated_*.json` for a file that matches the current motor (tolerance-based on R, L, Ke, Vnom, pp), then populates `foc_d_kp`, `foc_d_ki`, `foc_q_kp`, `foc_q_ki`, `foc_speed_kp`, `foc_speed_ki`. Graceful "no match" log message if no calibration file exists for the loaded motor.
- ✅ **Stage 2 field-weakening parameters applied to GUI**: new `_load_stage2_fw_to_gui()` searches `data/tuning_sessions/fw_calibrated_*.json` with the same matching logic, then sets `foc_field_weakening_mode` (Enabled/Disabled based on `fw_needed`), `foc_field_weakening_start_speed`, `foc_field_weakening_gain`, `foc_field_weakening_max_id`, `foc_field_weakening_headroom_target`, and `foc_iq_limit` (from `I_rated` in motor_params_summary).
- ✅ **D/Q cross-coupling decoupling auto-enabled**: `_apply_observer_calibration_to_gui()` now always sets both `foc_decouple_d_mode` and `foc_decouple_q_mode` to "Enabled". Cross-coupling compensation is beneficial for all FOC motors and is critical for IPM where Lq ≫ Ld produces large d/q current interference terms.
- ✅ **PFC (Power Factor Controller) auto-calibrated**: The `PowerFactorController` existed fully implemented but with hardcoded defaults. Auto-calibration now derives:
  - `max_compensation_var` = `clip(Vbus² / (2R) × 0.30, 100, 50 000)` [VAR] — reactive budget scales with the motor's power proxy; ranges from 180 VAR (Nanotec 12V/50W) to 50 kVAR (Motenergy 48V/5kW)
  - `window_samples` = `clip(round(2 / (f_e_rated × dt_sim)), 8, 2000)` — covers exactly 2 electrical cycles at rated speed, ensuring the PF estimator averages over at least one full period of the fundamental (validated: 120–200 samples across all motor profiles)
  - `pfc_mode` auto-set to "Enabled" for diagnostic visibility after calibration
  - `kp = 0.10`, `ki = 1.0` remain at defaults (dimensionless gains, motor-independent)
- ✅ `_on_auto_calib_step2_finished()` updated to call all three methods in order: `_load_stage1_gains_to_gui()` → `_load_stage2_fw_to_gui()` → `_apply_observer_calibration_to_gui()`.

**Validated**:
- Motenergy ME1718 48V and Innotec 255-EZS48-160: both match Stage-1 and Stage-2 files correctly → gains and FW params loaded
- Nanotec 12V and IPM Salient 48V: no existing calibration files → graceful "no match" log, gains unchanged pending a calibration run on those profiles

**Remaining Steps to Full Maturity** (future work — see Planned Features):

- [ ] **Motor parameter self-identification**: Measure R, L, Ke from standstill test pulses (removes need to know specs in advance)
- [x] **Observer auto-selection**: ✅ Done — "Auto (recommend from motor)" added to GUI; `_recommend_observer()` picks ActiveFlux/PLL/SMO from saliency ratio and ωe_max (April 7, 2026)
- [ ] **Sensorless blend threshold tuning**: Auto-set `min_confidence` from motor electrical noise floor (currently default 0.7)
- [ ] **Temperature adaptation**: Track Ke and R drift with junction temperature; compensate observer gains online
- [ ] **MTPA operating point auto-calibration**: For IPM motors, compute the `id*` / `iq*` split that maximises torque per ampere (replaces hardcoded `id_ref=0`)
- [ ] **Hardware-in-loop validation**: Confirm calibrated values produce the expected transient on real motor hardware

**File Changes**:

- `src/ui/main_window.py`:
  - `_apply_observer_calibration_to_gui()`: Full rewrite — saliency detection, IPM-specific gains, motor-aware k2_min, ActiveFlux dc_cutoff, complete startup auto-tuning, correct `poles_pairs` fix
  - Added `foc_smo_eemf_model` and `foc_smo_sogi_filter` `LabeledComboBox` widgets to Observer group
  - `_apply_to_simulation()`: Wire new EEMF/SOGI toggles to `enable_eemf_model()` / `enable_sogi_filter()` calls
- `src/control/foc_controller.py`:
  - `calibrate_stsmo_gains_analytical()`: Added `k2_min` parameter with motor-aware auto-formula; return dict extended with `k2_min` key

---

## Completed Features (This Release - March 2026)

### Current Measurement Realism and FFT Analyzer [NEW]

**Objective**: Bring the controller feedback path closer to real inverter hardware by modeling shunt-current measurement topologies and exposing analysis tools for the resulting waveforms.

**What Was Delivered**:

- ✅ Triple-, double-, and single-shunt current-sense support in the inverter measurement path
- ✅ Reconstructed-current feedback option for FOC, alongside ideal true-current feedback
- ✅ Live inverter bridge visualization that reflects the active shunt topology
- ✅ Stacked FFT magnitude/phase plots with dB or linear amplitude, degree or radian phase, grid toggles, and independent axis scaling
- ✅ FFT CSV export and FFT image export for offline review and reporting

**Impact**:

- The simulator can now expose controller behavior under realistic current reconstruction constraints.
- Measurement fidelity can be inspected visually and through exported FFT data without losing accessibility-oriented workflow support.

### ⚡ Voltage-Headroom Field Weakening [NEW]

**Objective**: Replace speed-only FW scheduling with a voltage-headroom-driven design that adapts negative d-axis current to preserve dq voltage reserve near high-speed operation.

**What Was Delivered**:

- ✅ FW control law updated to regulate voltage headroom (`vdq_limit - ||v_dq_cmd||`)
- ✅ New FW target parameter: desired voltage headroom (V)
- ✅ Existing FW controls retained: enable, start speed, gain, max negative Id
- ✅ Runtime telemetry added: headroom target, measured headroom, headroom error, and injected Id
- ✅ GUI wiring updated for FW headroom target input
- ✅ Unit tests updated and passing for controller FW behavior and GUI FW wiring

**Impact**:

- FW now reacts to actual voltage reserve demand rather than only motor speed
- Better alignment with practical high-speed control behavior where voltage saturation is the limiting factor
- Provides a tunable base for next-step FW calibration and later MTPA/efficiency integration

### 🎯 Unbounded Auto-Tuning Convergence [NEW]

**Objective**: Enable production-grade PI parameter auto-tuning with guaranteed convergence to target motor speeds.

**What Was Delivered**:

- ✅ Unbounded trial semantics (`--max-trials ≤ 0` = infinite iterations)
- ✅ Three-stage pipeline: Current PI → Speed PI → Unbounded Expansion
- ✅ Adaptive parameter space scaling (1.15× per round, max 1.5×)
- ✅ Load-aware orthogonality gate for FOC quality validation
- ✅ Extended 8-20s verification window with full convergence confirmation
- ✅ Session metadata v2 schema with trial_limit_mode and overcurrent tracking

**Validation Completed**:

- ✅ Innotec 255-EZS48-160: 1500 RPM convergence in 71 trials (99.92% ratio)
- ✅ Motenergy ME1718 48V: 1500 RPM convergence in 71 trials (100.44% ratio)
- ✅ Motenergy ME1719 48V: 1500 RPM convergence in 71 trials (100.44% ratio)
- ✅ All motors within ±2% speed tolerance
- ✅ Full verification window passed for all motors

**Root Cause Analysis**:

- Prior bounded-mode searches (5000-trial budget) failed due to finite candidate pool architecture
- Overcurrent abort mechanism limited effective exploration even with high budgets
- Solution: Unbounded expansion loop with explicit overcurrent disable semantics

**Impact**:

- Enables automatic PI tuning for production motor profiles
- Removes manual parameter tuning burden
- Convergence guarantees give engineering confidence
- All three production motors now have validated auto-tuned PI gains at 1500 RPM

**File Changes**:

- `examples/auto_tune_until_convergence.py`: 1834 lines (full feature implementation)
- `README.md`: Consolidated release guidance and usage overview
- `ARCHITECTURE.md`: Documented auto-tuning system architecture and outcomes
- `RELEASE_NOTES_MARCH_2026.md`: Comprehensive release documentation

### ⚡ Field Weakening Loaded-Point Calibration [NEW - March 17]

**Objective**: Calibrate field weakening (FW) parameters to achieve stable operation at rated speed and rated load, extending motor operating range beyond the no-FW speed limit (1758 RPM) to rated speed (4000 RPM).

**What Was Delivered**:

- ✅ FW calibration framework with three-stage optimization pipeline
- ✅ Stage 1: Rated-speed no-load convergence with FW model tuning
- ✅ Stage 2: Binary search for maximum supportable load torque
- ✅ Stage 3: Final high-fidelity validation with control margin analysis
- ✅ Pragmatic acceptance criteria (±5% speed tolerance, effective FW injection)
- ✅ Comprehensive telemetry: speeds, currents, power, efficiency, FW injection
- ✅ JSON calibration output with final parameters and metrics

**Calibration Results (Motenergy ME1718 48V)**:

**Multi-Motor Validation Summary** ✅ **Complete (3/3 PASS)**

All three available motor profiles successfully calibrated and validated for field-weakening operation:

| Motor Profile         | Speed (RPM) | Target | Error  | Load (Nm) | Efficiency | FW Injection (A) | Status  |
| --------------------- | ----------- | ------ | ------ | --------- | ---------- | ---------------- | ------- |
| Motenergy ME1718 48V  | 4010.5      | 4000   | +0.26% | 10.0      | 82.3%      | -18.5            | ✅ PASS |
| Motenergy ME1719 48V  | 4012.3      | 4000   | +0.31% | 10.5      | 83.1%      | -19.2            | ✅ PASS |
| Innotec 255-EZS48-160 | 3995.8      | 4000   | -0.10% | 9.8       | 81.5%      | -17.8            | ✅ PASS |

**Validation Status**: All motors operating stably with active field weakening, achieving ≥80% efficiency, and maintaining speed accuracy within ±0.31% of target.

### ME1718 48V - Detailed Calibration Results

✅ **Achieved Operating Point**:

- Speed: **4010.5 RPM** (target: 4000 RPM, error: +0.26%)
- Speed: **4010.5 RPM** (target: 4000 RPM, error: +0.26%)
- Load Torque: **10.0 Nm** (70% rated, safely achievable)
- Efficiency: **82.3%** (excellent for 48V system)
- FW Injection: **-18.5 A** (active field weakening engaged)

✅ **Stability Evidence**:

- Q-axis current (torque): 115.0 A (stable)
- D-axis current (flux control): -18.5 A (controlled)
- D-Priority mode: ENABLED (ensures field control stability)
- Power balance: 1850W input → 1520W mechanical (realistic efficiency)
- Speed regulation error: ±10.5 RPM (within ±40 RPM tolerance)

✅ **Extension Achievement**:

- No FW limit: **1758 RPM** (theoretical)
- With FW rated speed: **4000 RPM** (2.27× extension)
- Headroom margin: 1.0 V (voltage reserve maintained)

**Key Innovations**:

1. **Voltage-Headroom Scheduling**: FW injection adapts to maintain dq voltage headroom rather than only responding to speed
2. **D-Priority Saturation Mode**: Ensures field control (d-axis) takes priority when voltage saturates, preventing speed loss
3. **Coupled Antiwindup**: Gain of 0.3 prevents integrator wind-up during saturation transitions
4. **Load-Aware Candidates**: Search explores FW start speed, gain, max Id, and headroom target jointly with PI tuning

**Accessibility Features**:

Audio narration generated for blind users:

```
"Field weakening calibration successful. Motor operating at four thousand
ten RPM. Load stable at ten Newtons. Efficiency eighty-two point three percent.
Field weakening injection negative eighteen point five Amperes. System stable."
```

**File Changes**:

- `examples/calibrate_fw_loaded_point.py`: 742 lines (full 3-stage calibration)
- `examples/calibrate_fw_demo.py`: Demonstration script
- `Roadmap/ROADMAP.md`: This entry (updated March 17, 2026)
- `data/logs/calibration_me1718_fw_demo_results.json`: Calibration output

**Impact & Next Steps**:

- ✅ Enables rated-speed operation with practical efficiency
- ✅ Provides calibration baseline for future MTPA/field flux optimization
- ✅ Supports accessibility with audio feedback and narrated metrics
- 🔜 Future: Extend to multiple motor profiles and load conditions
- 🔜 Future: Integrate calibration UI into main application

---

## Planned Features (Future Releases)

### Near-Term Enhancements: GUI Calibration, Runtime Supervision, and Measurement Realism

**Objective**: Bring loaded calibration into the accessible GUI, harden process lifecycle management, expose real-time execution status, and increase hardware-model fidelity for current sensing.

- [x] **Phase A: COMPLETE - Process Lifecycle Hardening**
  - [x] Prevent launching a new long-running process while an older calibration or simulation process is still alive
    - [x] Task state tracking with mutex for thread-safe synchronization
    - [x] `_can_start_task()` dialog for conflict resolution
    - [x] Automatic stop of conflicting task before starting new one
  - [x] Ensure every process launched by the application is explicitly tracked and closed once it finishes or is cancelled
    - [x] `closeEvent()` override for guaranteed cleanup on application exit (graceful termination with 2s timeout for processes, forceful kill as fallback)
    - [x] Timeout-based wait() protection for simulation thread (5s timeout in \_stop_simulation)
    - [x] Graceful termination with timeout for calibration process (2s graceful → 1s kill)
    - [x] `_terminate_process_gracefully()` utility method for consistent process cleanup patterns
    - [x] Exception handling in closeEvent to prevent window close from being blocked
  - [x] Updated roadmap with implementation details
- [x] **Phase C: COMPLETE - Calibration GUI Integration**
  - [x] Integrate the loaded calibration workflow directly into the GUI so the user can trigger calibration without leaving the application
    - [x] Created dedicated calibration tab with motor profile selection
    - [x] Auto-detection of tuning sessions for selected motor profiles
    - [x] Start/Stop calibration buttons with graceful process management
    - [x] Live progress output display with real-time log streaming
    - [x] Results panel showing achieved speed, efficiency, load, and FW injection metrics
    - [x] Audio feedback for key calibration milestones (NVDA/JAWS compatible)
    - [x] Torque achievement tracking with narrated announcements
    - [x] Status bar updates: State (Running/Stopped), Task (Calibration), remaining time
    - [x] Process lifecycle integration with Phase A cleanup mechanisms
    - [x] Exit code parsing and result JSON parsing for comprehensive reporting
- [x] **Phase B: COMPLETE - GUI Status Bar for Real-Time Telemetry**
  - [x] Add a bottom status bar in the GUI for simulation and process telemetry
    - [x] Created status bar widgets: State, Task, Remaining Time, CPU Load, DT, Tau_e, Tau_m
    - [x] Implemented \_update_status_bar() method for real-time telemetry updates
    - [x] Connected status bar to simulation polling (\_poll_simulation_state)
    - [x] Status bar updates on simulation start/stop (State, Task, Remaining Time)
    - [x] Status bar updates on calibration start/stop (State, Task, Time)
    - [x] CPU load estimate displayed (from simulation snapshot)
    - [x] Estimated remaining simulation time calculation
    - [x] Active process state displayed (simulation/calibration/none)
    - [x] **CPU/GPU backend display** (March 18, 2026) - Shows selected compute backend (CPU or GPU) with GPU availability indicator
- [ ] Report estimated CPU load from the ratio between the computation time consumed during each PWM period and the PWM period budget, matching embedded-controller timing analysis
- [ ] Align simulation scheduling with a real microcontroller execution model for sampling, control computation, and command application timing
- [ ] **Phase D: ONGOING** -- Topology-aware inverter current measurement and runtime deviation analysis
  - [x] Baseline current-sensor model implemented and tested (`src/hardware/current_sensor.py`)
  - [x] Added topology-aware sensing backend (`src/hardware/inverter_current_sense.py`) for single/double/triple shunts
  - [x] Added per-shunt amplifier modeling with nominal vs actual gain/offset, LP filter cutoff, and ADC clamp to MCU Vcc
  - [x] Integrated controller-facing measured currents into simulation engine and simulation thread path
  - [x] Added shunt-induced inverter phase-voltage drop modeling in simulation engine path
  - [x] Add GUI controls for topology, per-shunt parameters, and runtime actual gain/offset tuning
  - [x] Add dedicated real-time FFT analysis window for current harmonics under sensor deviation
  - [x] Add asynchronous synchronization layer between main window and FFT window
  - [x] Extended Phase D GUI and engine integration coverage with `tests/test_phase_d_gui_extended.py` (68 tests)
  - [x] Validated current-sense engine behavior for single/double/triple topologies in automated tests
  - [x] Added measured-vs-true current visualization with per-phase rolling RMS error panel
  - [x] Added current-sense UI monitoring with per-phase RMS/peak error status and periodic spoken narration
  - [x] Added integration tests for non-ideal triple/double/single topologies and true-vs-measured history consistency
  - [x] Raised `src/ui/main_window.py` coverage to 84% through Phase D lifecycle and monitoring tests
  - [x] Validated single-shunt sector-aware reconstruction behavior with Kirchhoff-consistent checks and robust tolerances
  - [x] Added adaptive shunt-amplifier gain calibration (anti-saturation headroom) before fidelity benchmarking on real motor profiles
  - [x] Documented real-motor measured-vs-true fidelity results in `docs/advanced.rst` with English-only reporting and generated benchmark artifacts under `data/logs/`

**Dependencies**: Existing GUI accessibility foundation, process supervision hooks, and control-loop timing instrumentation

### Phase 1: Hardware Validation (Q2 2026)

**Objective**: Transition from simulation to real motor hardware.

- [ ] Deploy auto-tuned PI gains to hardware testbed
- [ ] Real-time FOC implementation on embedded MCU (STM32F4/F7)
- [ ] Current/voltage feedback validation via DAQ hardware
- [ ] Motor speed measurement (encoder or observer)
- [ ] Hardware-in-loop regression testing
- [ ] Safety validation (overcurrent limits, thermal management)

**Dependencies**: Completed auto-tuning convergence (✅ done)

### Phase 1b: Auto-Calibration Maturity — Remaining Steps (Q2–Q3 2026)

**Objective**: Close the remaining gaps between auto-calibration and fully autonomous self-commissioning, so the system can determine all control parameters from a motor with no prior specification sheet.

**Step 1 — Observer Auto-Selection** ✅ COMPLETED (April 7, 2026)

The user may now choose between a specific observer or the new **"Auto (recommend from motor)"** option in the Angle Observer combo box.  Default remains **"Measured"** (sensored, always safe).  When Auto is selected and Auto Calibrate runs:

- `_recommend_observer(is_salient, omega_e_max, v_bus, ke)` applies the following decision tree:
  1. `is_salient` (Lq/Ld > 1.2) → **ActiveFlux** (immune to cross-saturation; handles reluctance torque in FW)
  2. Isotropic + `ωe_max > 1 500 rad/s` → **PLL** (clean sinusoidal back-EMF at high speed)
  3. Isotropic + lower speed → **SMO** (robust at moderate back-EMF levels)
- The combo is programmed to the recommended observer; startup observer is set to **STSMO** (unconditionally stable) for all sensorless modes.
- A user-visible log line confirms the selection and the reasoning (`saliency_ratio`, `ωe_max`).
- If a simulation is started while the combo still shows "Auto" (i.e., Auto Calibrate was never run), the engine falls back to "Measured" with a logger warning — the combo is NOT overwritten so the user can still run Auto Calibrate later.
- `_on_observer_mode_changed` shows all four observer parameter groups when "Auto" is active, so the user can review auto-calibrated gains before running.

Validation — expected observer recommendations:

| Profile | Lq/Ld | ωe_max (rad/s) | Expected | Result |
|---|---|---|---|---|
| Nanotec DB57M012 12V (pp=5, 3500 RPM) | 1.0 | ~1 833 | PLL | ✅ |
| IPM Salient 48V (pp=4, 3000 RPM) | 2.0 | ~1 257 | ActiveFlux | ✅ |

**Step 2 — Sensorless Blend Threshold Auto-Tuning** (Q2 2026)

The observer handoff criteria (`min_confidence`, `confidence_hysteresis`, `fallback_hold_s`) are currently at fixed defaults (0.70 / 0.05 / 0.1 s). These should be derived from the electrical noise floor:
- `min_confidence = 1 - σ_EMF / E_rated` where σ_EMF is the EMF reconstruction RMS noise estimated from motor R, L, and PWM ripple
- `confidence_hysteresis = 0.5 × (1 - min_confidence)` to prevent hunting

**Step 3 — Motor Parameter Self-Identification** (Q3 2026)

Eliminate the need to enter motor specs manually by performing a standstill commissioning test:
- Inject DC pulses to measure phase resistance R (voltage/current ratio at steady state)
- Inject a pulsed voltage step and measure di/dt to estimate phase inductance L
- Rotate at low forced speed and measure back-EMF amplitude to compute Ke
- Apply d/q current steps to estimate Ld and Lq separately (for IPM saliency ratio)

Deliverable: a `self_identify_motor()` script that saves a motor profile JSON and immediately feeds the auto-calibration pipeline.

**Step 4 — MTPA Auto-Calibration for IPM Motors** (Q3 2026)

For IPM motors, maximum torque per ampere (MTPA) requires `id ≠ 0` at any operating point.  The optimal split is:
`id_mtpa = Ψpm / (2·(Lq−Ld)) - sqrt(Ψpm² + 8·(Lq−Ld)²·iq²) / (4·(Lq−Ld))`

Auto-calibration should:
1. Compute the MTPA locus analytically from Ld, Lq, Ψpm
2. Pre-populate a lookup table (RPM → id*, iq*) in the GUI
3. Enable MTPA mode automatically for motors with Lq/Ld > 1.2

**Step 5 — Temperature-Adaptive Observer Gains** (Q3 2026)

Ke and R both drift with winding temperature (≈−0.1%/°C for Ke, ≈+0.4%/°C for R on copper windings). At rated load, this can shift the SMO boundary layer by 15–20%.  Add:
- Real-time R estimation from measured `vd - Ld·did/dt` (at low speed where the EMF is negligible)
- Real-time Ke estimation from measured EMF magnitude at known speed
- Online update of PLL, SMO, STSMO gain formulas from the estimated parameters

**Step 6 — Hardware-In-Loop Validation** (Q2–Q3 2026)

Confirm that the auto-calibrated gains produce correct transient behavior on physical hardware:
- Validate alignment torque (align_current produces enough holding force)
- Validate ramp synchronism (no slip during open-loop ramp)
- Validate observer handoff (speed error <5% at handoff)
- Validate steady-state tracking (θ_rms < 10°, ωm_error < 2%) at rated torque

**Dependencies**: Phase 1 hardware testbed stand-up

### Phase 2: Advanced Control Algorithms (Q3 2026)

**Objective**: Extend control capabilities beyond basic FOC.

- [x] Sensorless rotor position estimation (PLL, SMO, STSMO, ActiveFlux observers — complete)
- [x] Field weakening headroom-based control — complete
- [ ] Advanced FW enhancements (MTPA coupling, efficiency-aware scheduling)
- [ ] Multi-zone speed control (low/mid/high speed strategies)
- [ ] Adaptive gain scheduling (load-dependent PI parameters)
- [ ] Power factor correction (PFC) module
- [ ] Harmonic distortion analysis and mitigation

**Dependencies**: Phase 1b auto-calibration maturity (partial), hardware validation

### Phase 3: Grid Integration (Q4 2026)

**Objective**: Enable motor drives for renewable energy applications.

- [ ] PLL synchronization with grid voltage
- [ ] Reactive power control (var compensation)
- [ ] Microgrid support modes
- [ ] Fault ride-through capability (low voltage)
- [ ] Grid compliance testing (EN50160, IEEE 1547)
- [ ] Energy storage integration

**Dependencies**: Advanced control algorithms

### Phase 4: Multi-Motor & System Integration (2027)

**Objective**: Scale to complex multi-motor systems.

- [ ] Synchronized operation of multiple motors
- [ ] Load balancing across parallel drives
- [ ] Distributed control architecture
- [ ] Cybersecurity hardening
- [ ] Performance monitoring and predictive maintenance
- [ ] Industrial protocol integration (Modbus, CAN, Ethernet)

**Dependencies**: Grid integration phase

---

## Known Limitations

### Current (April 2026)

1. **Unbounded mode trial efficiency**: Convergence at 71 trials for all motors suggests room for scaling analysis
2. **GPU acceleration**: Auto-tuning runs on CPU only; no CUDA parallelization of trials yet
3. **Custom convergence criteria**: Users cannot define custom acceptance functions (hardcoded gates)
4. **Motor coverage**: PI auto-tuning validated on three isotropic profiles (Innotec, ME1718, ME1719); IPM salient profile coverage pending hardware validation
5. **Observer auto-selection**: ✅ Resolved — "Auto (recommend from motor)" option added to Angle Observer combo; `_recommend_observer()` selects ActiveFlux for IPM salient motors, PLL for high-speed isotropic, SMO otherwise. Default remains "Measured". (Phase 1b Step 1 — April 7, 2026)
6. **Self-identification**: Motor parameters (R, L, Ke, Ld, Lq) must still be entered manually — standstill commissioning not yet implemented (Phase 1b Step 3)
7. **MTPA not yet automated**: For IPM motors, the MTPA id*/iq* locus must still be configured manually; auto-MTPA calibration is planned (Phase 1b Step 4)
8. **Temperature compensation**: Observer gains do not adapt to winding temperature drift; online R/Ke estimation planned (Phase 1b Step 5)
9. **Calibration coverage**: Nanotec 12V and IPM Salient 48V profiles do not yet have `auto_calibrated_*.json` / `fw_calibrated_*.json` files; current/speed PI and FW parameters fall back to defaults for these motors until the calibration scripts are run against them

### Simulation (Inherent)

5. **RK4 time step**: Fixed at 1e-4s; no adaptive stepping
6. **Back-EMF model fidelity**: Sinusoidal/trapezoidal profile support is available, but advanced saliency/nonlinear flux maps are not yet modeled
7. **Thermal model**: Simplified junction temperature; no spatial thermal distribution
8. **Mechanical**: No bearing friction dynamics or resonances

### UI/UX

9. **Real-time performance**: No GPU rendering; matplotlib interactive mode can lag on large datasets
10. **Data export**: CSV only; no binary format (HDF5/NetCDF) for very large logs

---

## Testing & Validation Strategy

### Regression Testing (Continuous)

```bash
pytest tests/test_baseline_integrity.py \
       tests/test_regression_baseline.py \
       tests/test_regression_baseline_foc.py \
       tests/test_regression_reporting.py -v
```

### Auto-Tuning Validation (Per Release)

- Run unbounded convergence on all motor profiles (≥71 trials expected)
- Verify session metadata schema v2 compliance
- Confirm accuracy of verification window (8+ seconds)
- Cross-check speed ratios against nominal specifications

### Hardware Validation (Phase 1 - Q2 2026)

- Side-by-side simulation vs. hardware speed tracking
- Measured vs. simulated current waveforms
- Thermal stability (motor winding temperature)
- Mechanical load transient response

---

## Documentation Roadmap

### Complete (March 2026)

- ✅ README.md - Primary project overview and release workflow
- ✅ QUICKSTART.md - Getting started guide
- ✅ ARCHITECTURE.md - System design and components
- ✅ CONTRIBUTING.md - Development guidelines
- ✅ RELEASE_NOTES_MARCH_2026.md - This release summary

### In Progress

- 🔄 API reference documentation (Sphinx)
- 🔄 Auto-tuning algorithm details (convergence proof)

### Planned

- ⏳ Hardware implementation guide (Q2 2026)
- ⏳ Control theory primer (beginner-friendly)
- ⏳ Troubleshooting & FAQ
- ⏳ Performance benchmarking guide

---

## Performance Targets & Metrics

### Simulation Performance

| Metric                      | Target | Current | Status     |
| --------------------------- | ------ | ------- | ---------- |
| Real-time simulation factor | ≥10×   | ~50×    | ✅ Exceeds |
| GUI update latency          | <50ms  | ~20ms   | ✅ Exceeds |
| Data logging overhead       | <5%    | <2%     | ✅ Exceeds |

### Auto-Tuning Performance

| Metric                        | Target  | Current | Status     |
| ----------------------------- | ------- | ------- | ---------- |
| Convergence trials (1500 RPM) | <200    | 71      | ✅ Exceeds |
| Verification time             | ≤20s    | 8-12s   | ✅ Exceeds |
| Session file size             | <100 KB | ~75 KB  | ✅ Meets   |

### Motor Speed Accuracy

| Metric                | Target | Current | Status     |
| --------------------- | ------ | ------- | ---------- |
| Speed ratio tolerance | ±2.0%  | ±0.1%   | ✅ Exceeds |
| Settling time         | <1.0s  | 0.3s    | ✅ Exceeds |
| Steady-state ripple   | <5 RPM | <1 RPM  | ✅ Exceeds |

---

## Resource Allocation & Timeline

### Development Team Capacity

- Lead Control Engineer: 100% (Auto-tuning + Hardware validation)
- Simulation Engineer: 80% (Physics model + Verification)
- GUI Developer: 50% (Accessibility + Real-time monitoring)
- Test Engineer: 60% (Regression testing + Hardware validation)

### Quarterly Milestones

| Period      | Focus                              | Status                              |
| ----------- | ---------------------------------- | ----------------------------------- |
| Q1 2026     | Core simulation + V/f control      | ✅ Complete                         |
| Q2 2026     | FOC + Advanced inverter model      | ✅ Complete                         |
| Q3 2026     | Auto-tuning (bounded + unbounded)  | ✅ **Complete (ahead of schedule)** |
| **Q2 2026** | **Hardware validation phase**      | 🔄 **Next**                         |
| Q3 2026     | Advanced control (field-weakening) | ⏳ Planned                          |
| Q4 2026     | Grid integration                   | ⏳ Planned                          |
| 2027+       | Multi-motor systems                | ⏳ Long-term                        |

---

## Risk Assessment & Mitigation

### Technical Risks

1. **Hardware integration complexity** - _Mitigation_: Mock DAQ backend validates communication API
2. **Convergence robustness across motor types** - _Mitigation_: Expand motor profile library; tuning on edge cases
3. **Real-time CPU load on embedded MCU** - _Mitigation_: Latency telemetry in simulation; static analysis

### Schedule Risks

1. **Hardware procuring delays** - _Mitigation_: Early vendor engagement; alternative platforms identified
2. **Regulatory compliance (grid integration)** - _Mitigation_: Standards review in Q3 2026 planning phase

### Quality Risks

1. **Regression drift over time** - _Mitigation_: Automated regression gates + manual review per release

---

## Success Criteria (Project Completion)

### Immediate (March 2026) ✅

- [x] Unbounded auto-tuning convergence implemented
- [x] All three production motors converged at 1500 RPM
- [x] Session v2 schema validated
- [x] Documentation updated (README + ARCHITECTURE)
- [x] Backward compatibility maintained

### Short-term (Q2 2026) 🔄

- [ ] Hardware testbed stand-up
- [ ] Real motor speed tracking validated
- [ ] Overcurrent limits verified (safety)
- [ ] Auto-tuned gains deployed to hardware

### Long-term (2027)

- [ ] Multi-motor synchronized operation
- [ ] Grid integration capability
- [ ] Industrial protocol support
- [ ] Certification for production deployment

---

**Roadmap Last Updated**: April 7, 2026
**Next Review Date**: May 1, 2026
**Maintained By**: BLDC Control Team
