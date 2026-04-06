# SPINOTOR FOC Auto-Calibration References

## Overview

This directory contains comprehensive documentation for improving the FOC (Field-Oriented Control) auto-calibration module in the SPINOTOR project. The documentation synthesizes research from 45+ academic papers and industry technical documents to provide practical, analytical methods for PI controller gain tuning.

**Focus Areas:**
- Analytical, bandwidth-based PI tuning (replacing grid search)
- Parameter identification and online adaptation
- Anti-windup strategies for cascaded control loops
- Sensorless motor startup and control
- Field weakening control
- Robustness and stability analysis
- Validation methodologies for loaded operation

**Target Motors:**
- Motenergy ME1718 (48V, 4000 RPM, 14.3 Nm)
- Motenergy ME1719 (48V, 4000 RPM, 14.3 Nm)
- Innotec 255-EZS48-160 (48V, 3000 RPM, 26.5 Nm)

---

## Document Guide

### 1. BIBLIOGRAPHY.md (45+ References)

**Purpose:** Comprehensive bibliography of all referenced papers, organized by topic.

**Contents:**
- Core FOC implementation and auto-tuning methods
- PI gain tuning approaches (bandwidth-based, model-based, adaptive)
- Sensorless control strategies (SMO, PLL, startup methods)
- Parameter identification algorithms (MRAS, RLS, UKF)
- Field weakening control design
- Cascaded control and anti-windup strategies
- Sensorless robustness and stability analysis
- Advanced techniques (MPC, SMC, reinforcement learning)
- Practical implementation references and open-source code

**How to Use:**
- Read the abstract and "Relevance" section for each reference
- Sections organized by topic → find papers relevant to your specific problem
- "References Organized by Application Context" section at the end provides filtered lists
- URLs provided allow direct access to papers (some open-access, some require institutional access)

**Key Highlights:**
- References 3, 4, 5, 6, 7: Core bandwidth-based PI tuning methods
- References 9, 10, 11, 12: Sensorless control with startup strategies
- References 13-16: Online parameter identification
- References 20-25: Anti-windup and cascaded control design

---

### 2. AUTO_TUNING_DESIGN_GUIDE.md (30+ pages)

**Purpose:** Complete step-by-step guide for implementing analytical FOC auto-calibration.

**Contents:**

1. **Part 1: Analytical PI Tuning Foundation**
   - Current loop bandwidth design
   - PI controller formulas with examples
   - Speed loop bandwidth (1/10 ratio)
   - PWM discretization effects

2. **Part 2: Motor Parameter Identification**
   - Essential parameters (Rs, L, J, Ψm)
   - Offline measurement procedures
   - Online identification (MRAS, RLS algorithms)
   - Temperature compensation

3. **Part 3: Anti-Windup Implementation**
   - Root cause of integral windup
   - Back-calculation method with examples
   - Tracking/conditional integration
   - Implementation checklist

4. **Part 4: Validation Criteria**
   - Step response tests (current, speed)
   - Load disturbance rejection
   - Frequency domain validation (phase margin, gain margin)
   - Efficiency and thermal validation
   - Flux angle orthogonality

5. **Part 5: Gain Scheduling**
   - Speed-dependent adaptation
   - Load-dependent adaptation
   - Temperature compensation
   - Python implementation examples

6. **Part 6: Grid Search Replacement**
   - Proposed analytical tuning sequence (7 steps)
   - Detailed pseudo-code
   - Replaces 144 evaluations with 7 targeted tests

7. **Part 7: Integration with SPINOTOR**
   - Code structure for analytical tuner
   - Configuration files for target motors
   - Updated controller interface

8. **Part 8: Validation Results Template**
   - Example results document
   - How to document your tuning results

**How to Use:**
- **For Quick Start:** Read Part 1 (15 minutes) + QUICK_REFERENCE_FORMULAS.md
- **For Complete Implementation:** Work through Parts 1-8 sequentially
- **For Specific Topics:** Use section headers to jump to needed content
- **For Code:** Use pseudo-code examples as basis for Python implementation

**Key Equations:**
```
Current loop bandwidth:    ωc = R/L × 0.6
PI gains:                  Kp = 2·L·ωc·sin(φm)
                          Ki = L·ωc²·cos(φm)
Speed loop bandwidth:      ωc_speed = ωc_current / 10
Anti-windup gain:          K_aw = 1 / (Kp / Ki)
```

---

### 3. QUICK_REFERENCE_FORMULAS.md (Cheat Sheet)

**Purpose:** Compact reference of all key formulas and calculations.

**Contents:**

1. **Core PI Tuning Equations**
   - Motor transfer functions
   - PI controller formulas
   - Closed-loop relationships

2. **Motor Parameter Calculations**
   - From nameplate data
   - From resistance measurement
   - From inductance measurement
   - Inertia estimation

3. **Bandwidth and Time Constant Relationships**
   - Natural motor bandwidth
   - PWM constraints
   - Typical values for small/medium motors

4. **Phase Margin and Stability**
   - Phase margin definition
   - Gain margin
   - Overshoot vs. margin

5. **Anti-Windup Tuning**
   - Back-calculation method
   - Recovery time constant

6. **Discretization Effects**
   - Sampling period
   - Continuous-to-discrete conversion
   - Tustin/bilinear method

7. **Temperature Compensation**
   - Resistance coefficient
   - Flux linkage coefficient
   - Compensation strategies

8. **Quick Calculation Spreadsheet**
   - Template for motor parameter entry
   - Step-by-step calculation guide

9. **Testing Checklist**
   - Pre-tuning verification
   - Current loop validation
   - Speed loop validation
   - Thermal validation

10. **Common Tuning Mistakes**
    - Error table with fixes

**How to Use:**
- **During Design:** Use sections 1-3 for calculations
- **For Implementation:** Use section 8 (spreadsheet) template
- **During Testing:** Use section 9 (checklist)
- **For Debugging:** Use section 10 (common mistakes)

---

## Quick Start Guide (15 Minutes)

### Step 1: Understand the Problem (2 min)
- Current auto-tuning: 12×12 grid search = 144 evaluations
- Proposed: Analytical PI tuning = 7 targeted tests
- Motivation: Faster, more reliable, physics-based

### Step 2: Learn Core Formulas (5 min)
Read **QUICK_REFERENCE_FORMULAS.md** sections 1-2:
```
Motor bandwidth = Rs / L
PI Gains = f(ωc, phase_margin, motor_params)
Speed loop bandwidth = current_loop / 10
```

### Step 3: Calculate for Your Motor (5 min)
Use **QUICK_REFERENCE_FORMULAS.md** section 8 (spreadsheet):
1. Enter motor parameters (Rs, L, J)
2. Calculate ωn = Rs/L
3. Calculate ωc = 0.6 × ωn
4. Calculate Kp, Ki using formulas
5. Done!

### Step 4: Validate (Implementation)
Use **AUTO_TUNING_DESIGN_GUIDE.md** Part 4:
- Test current step response (< 10 ms rise time)
- Test speed step (< 20% overshoot)
- Test load rejection (< 5% speed dip)

---

## Practical Implementation Workflow

### Phase 1: Parameter Identification
```
1. Measure motor resistance: Rs = V/I (blocked rotor test)
2. Measure motor inductance: L = V/(2π·f·ΔI) (AC impedance)
3. Estimate/measure rotor inertia: J (from datasheet or acceleration test)
4. Get flux linkage from torque constant: Ψm = Ke / P_pairs
```

### Phase 2: Design PI Gains
```
1. Compute natural bandwidth: ωn = Rs/L
2. Target bandwidth: ωc = 0.6 × ωn
3. Current loop gains: Kp, Ki from bandwidth formula (60° phase margin)
4. Speed loop gains: ωc_speed = ωc/10, then compute Kp_speed, Ki_speed
5. Anti-windup gain: K_aw = Ki / Kp for each loop
```

### Phase 3: Validate and Tune
```
1. Current loop test: Apply step, verify < 10 ms rise time
2. Speed loop test: Apply step, verify < 20% overshoot
3. Load test: Apply disturbance, verify < 5% dip
4. Fine-tune if needed (reduce Kp if oscillation, reduce Ki if too slow)
5. Characterize operating region (gain scheduling table)
```

### Phase 4: Production Deployment
```
1. Document final gains
2. Implement temperature compensation (if needed)
3. Implement gain scheduling (if needed)
4. Pack gains with motor model
5. Deploy to SPINOTOR firmware
```

---

## Implementation Checklist

### Before Starting
- [ ] Motor parameters available (Rs, L, J, Ψm, pole pairs)
- [ ] PWM frequency known (typically 20 kHz)
- [ ] Current sensing accuracy verified
- [ ] Anti-windup framework exists in code

### Design Phase
- [ ] QUICK_REFERENCE_FORMULAS.md section 8 completed
- [ ] Spreadsheet shows Kp, Ki values
- [ ] Phase margin verified (45-80°)
- [ ] Gains reviewed by team member

### Implementation Phase
- [ ] New AnalyticalAutoTuner class created
- [ ] Motor parameters loaded from config
- [ ] PI gains computed analytically (not grid search)
- [ ] Anti-windup implemented with K_aw
- [ ] Temperature compensation added (if thermal variation)

### Validation Phase
- [ ] Current loop step response: < 10 ms ✓
- [ ] Speed loop step response: < 20% overshoot ✓
- [ ] Load disturbance: < 5% dip ✓
- [ ] Efficiency: > 85% ✓
- [ ] Temperature: < 100°C winding ✓
- [ ] All 3 motors tuned and documented

### Documentation Phase
- [ ] Results documented in AUTO_TUNING_DESIGN_GUIDE.md Part 8 format
- [ ] Gains table created for gain-scheduling
- [ ] Test data archived
- [ ] Code comments explain tuning methodology

---

## File Structure

```
SPINOTOR/
└── references/
    ├── README.md (this file)
    ├── BIBLIOGRAPHY.md (45+ references)
    ├── AUTO_TUNING_DESIGN_GUIDE.md (8-part guide)
    ├── QUICK_REFERENCE_FORMULAS.md (formulas + checklist)
    └── [future: motor_specific_configs/]
        ├── motenergy_me1718.yaml
        ├── motenergy_me1719.yaml
        └── innotec_255_ezs48_160.yaml
```

---

## Key Insights from Literature

### Finding 1: Bandwidth-Based Tuning
**Source:** Microchip MCAF, IEEE Papers

Current loop bandwidth should target **ωc ≈ 0.5 to 0.7 × (R/L)**.
This provides good balance between performance and robustness.

**Impact for Project:** Replaces empirical 12×12 grid with analytical formula.
Reduces tuning time from hours to minutes.

### Finding 2: Anti-Windup is Critical
**Source:** ResearchGate, Wiley Publications

Cascaded PI loops without proper anti-windup cause overshoot and oscillation
when inner loop saturates (e.g., during large speed step).

**Impact for Project:** Must implement back-calculation anti-windup.
K_aw = 1 / (Kp / Ki) provides good empirical formula.

### Finding 3: Temperature Compensation
**Source:** Motor manufacturer datasheets, IEEE papers

Stator resistance increases ~0.4%/°C (copper), changing bandwidth by ~14% from 25°C to 60°C.
Must adapt Kp/Ki to maintain bandwidth.

**Impact for Project:** Temperature sensor or resistance estimator needed.
Enables 48V motor to work reliably 0-60°C operating range.

### Finding 4: Sensorless Startup
**Source:** Microchip AN4398, Imperix docs, TI application notes

I/f (current-frequency) open-loop startup provides smooth acceleration with
constant torque. Transition to flux observer at ~20-30% speed.

**Impact for Project:** Enables reliable startup without position sensor.
Critical for motor commissioning tests.

### Finding 5: Gain Scheduling
**Source:** IEEE papers, MDPI recent publications

Gains must adapt to operating point:
- Field-weakening region reduces available voltage → reduce bandwidth
- Load variations change effective inertia → adjust speed loop gains
- Temperature changes motor parameters → continuous compensation

**Impact for Project:** Static gains insufficient for full operating range.
Implement gain scheduling table with [speed, load, temp] dependencies.

---

## Next Steps

1. **Read QUICK_REFERENCE_FORMULAS.md** (15 min)
   - Understand core equations
   - Calculate baseline gains for your motor

2. **Read AUTO_TUNING_DESIGN_GUIDE.md Part 1-2** (30 min)
   - Understand bandwidth design
   - Learn parameter measurement techniques

3. **Implement AnalyticalAutoTuner class** (4-8 hours)
   - Replace grid search with analytical computation
   - Add anti-windup
   - Add validation tests

4. **Validate on Target Motors** (4-8 hours)
   - Test current loop (step response)
   - Test speed loop (step response + load test)
   - Document results

5. **Deploy and Monitor** (ongoing)
   - Deploy gains to SPINOTOR firmware
   - Monitor performance in field
   - Add temperature compensation if needed

---

## FAQ

**Q: Why not just keep the 12×12 grid search?**
A: Grid search is slow (144 evals) and black-box. Analytical method:
- Faster (7 targeted tests)
- Physics-based (understand why gains work)
- Easier to debug
- Naturally supports gain scheduling

**Q: Do I need to measure all motor parameters?**
A: Minimum: Rs (blocked rotor test, 5 min) + L (inductance measurement, 10 min).
Everything else can be estimated from datasheet/nameplate.
Online identification will refine estimates during operation.

**Q: What if my motor isn't in the list of 3 target motors?**
A: The methodology applies to any PMSM/BLDC motor.
Follow the same steps with your motor's parameters.
Section 6.2 of AUTO_TUNING_DESIGN_GUIDE.md shows config template.

**Q: How do I validate that my tuning is good?**
A: Use the checklist in QUICK_REFERENCE_FORMULAS.md section 9.
- Current loop: < 10 ms rise time, < 10% overshoot
- Speed loop: < 20% overshoot, < 500 ms settling
- Load test: < 5% speed dip

**Q: The motor is oscillating. What should I do?**
A: First, check common mistakes in QUICK_REFERENCE_FORMULAS.md section 10.
Most likely: Ki too high (reduce by 20-50%) or missing anti-windup.
See AUTO_TUNING_DESIGN_GUIDE.md Part 3 for anti-windup implementation.

**Q: How do I handle motor temperature changes?**
A: Two approaches:
1. Static tuning at 25°C, accept ±10% bandwidth variation
2. Dynamic compensation: measure T, compute Rs(T), adapt Kp/Ki
See AUTO_TUNING_DESIGN_GUIDE.md Part 5.3 and QUICK_REFERENCE_FORMULAS.md section 7.

**Q: Can I use reinforcement learning for autotuning?**
A: Yes! See BIBLIOGRAPHY.md references 31-33 for RL-based PI tuning.
RL can learn optimal gains from simulation/experiment.
Good for complex nonlinear systems, but slower than analytical method.

---

## Contact and Support

This documentation was compiled from 45+ academic papers and industry sources.
For questions about specific references, see BIBLIOGRAPHY.md for URLs.

**Recommended Reading Order:**
1. This README (5 min)
2. QUICK_REFERENCE_FORMULAS.md sections 1-2 (10 min)
3. AUTO_TUNING_DESIGN_GUIDE.md Part 1 (20 min)
4. Then implement and iterate

---

## License and Attribution

All original technical documentation synthesizes publicly available academic papers,
industry datasheets, and open-source implementation references.
See BIBLIOGRAPHY.md for complete citation list.

**Document Generated:** 2026-03-30
**Total References:** 45+ papers and technical documents
**Target Project:** SPINOTOR FOC Control Module
**Estimated Implementation Time:** 8-16 hours to replace grid search with analytical tuning

---

## Document Roadmap

| Document | Pages | Topic | Audience |
|----------|-------|-------|----------|
| README.md | 5 | Overview + quick start | Everyone |
| QUICK_REFERENCE_FORMULAS.md | 13 | Formulas + checklist | Implementation engineers |
| AUTO_TUNING_DESIGN_GUIDE.md | 30 | Complete methodology | System engineers, researchers |
| BIBLIOGRAPHY.md | 28 | References + summaries | Research, validation |

**Total Documentation:** ~76 pages of synthesis from 45+ academic papers

---

