# FOC Auto-Tuning Design Guide for BLDC/PMSM Motors

## Executive Summary

This guide synthesizes research from 45+ academic papers and industry documentation to provide practical design methodology for improving the FOC auto-calibration module in the BLDC-Simulator project. The guide focuses on analytical, model-based tuning rather than grid search approaches.

**Target Motors:**
- Motenergy ME1718: 48V, 4000 RPM, 14.3 Nm
- Motenergy ME1719: 48V, 4000 RPM, 14.3 Nm
- Innotec 255-EZS48-160: 48V, 3000 RPM, 26.5 Nm

---

## Part 1: Analytical PI Tuning Foundation

### 1.1 Current Loop Bandwidth Design

The current loop is the critical foundation for FOC performance. Bandwidth determines responsiveness to command changes and disturbances.

#### Fundamental Bandwidth Formula

For a motor represented by **L-R dynamics** in the synchronous rotating reference frame:

```
d(i_d,q)/dt = (V_d,q - R·i_d,q - ω_e·L_q,d·i_q,d) / L_d,q
```

The natural bandwidth of the uncontrolled current is:

```
ω_n = R / L  (rad/s)
```

**This is the absolute maximum achievable closed-loop bandwidth with PI control.**

#### PI Controller Transfer Function

```
G_PI(s) = Kp + Ki/s = (Kp·s + Ki) / s
```

For a unity-feedback current loop with PI controller:

```
Open-loop: G_OL(s) = G_PI(s) × G_motor(s) = (Kp·s + Ki)/s × 1/(L·s + R)
Closed-loop: G_CL(s) = (Kp·s + Ki) / (L·s² + (R + Kp)·s + Ki)
```

#### Bandwidth-Phase Margin Relationship

For specified **phase margin φm** and **bandwidth ωc**:

```
Kp = 2·L·ωc·sin(φm)
Ki = L·ωc²·cos(φm)
```

**Practical Values:**
- **ωc_current = R/L** → Natural motor bandwidth (maximum, rarely achievable)
- **ωc_target = 0.5 to 0.7 × (R/L)** → Conservative design (most practical)
- **φm = 60°** → Standard margin (0-10% overshoot)
- **φm = 45°** → Aggressive tuning (10-20% overshoot, faster response)
- **φm = 80°** → Conservative tuning (<2% overshoot, slower response)

#### Example Calculation for Motenergy ME1718

Typical PMSM parameters (at 25°C):
```
R_s = 0.8 Ω  (stator resistance)
L = 3.5 mH   (stator inductance, average of Ld/Lq)
T_L = R/L = 0.8/0.0035 = 228.6 rad/s ≈ 36.4 Hz

Recommended ωc_current = 0.6 × 228.6 = 137 rad/s ≈ 21.8 Hz

For φm = 60°:
  Kp = 2 × 0.0035 × 137 × sin(60°) = 0.835
  Ki = 0.0035 × 137² × cos(60°) = 32.8

For φm = 45°:
  Kp = 2 × 0.0035 × 137 × sin(45°) = 0.678
  Ki = 0.0035 × 137² × cos(45°) = 46.4
```

### 1.2 Speed Loop Bandwidth Design

The speed loop is slower and must account for mechanical time constant and cascaded stability.

#### Mechanical System Model

```
J × dω/dt = T_em - T_load - B·ω

where:
  J = rotor inertia (kg·m²)
  B = viscous damping coefficient
  T_em = electromagnetic torque from current controller
  T_load = load torque (varies with speed/power)
```

#### Speed Loop Tuning Rules

The speed controller sees the current loop as first-order response with bandwidth ωc_current:

```
Effective motor transfer function seen by speed loop:
  ω(s) / ω_ref(s) = ωc_current / (T_mech·s + ωc_current)

where T_mech = J / (B + motor_damping)

Speed loop bandwidth target:
  ωc_speed = ωc_current / 10  (typical 1/10 ratio)

This ensures cascade stability margin ≈ 5-10 dB
```

#### Speed Loop PI Gains

For speed loop with same phase margin φm:

```
Kp_speed = 2·J·ωc_speed·sin(φm)
Ki_speed = J·ωc_speed²·cos(φm)
```

**Example for Motenergy (J ≈ 0.0005 kg·m²):**
```
ωc_speed = 137 / 10 = 13.7 rad/s

For φm = 60°:
  Kp_speed = 2 × 0.0005 × 13.7 × sin(60°) = 0.00594
  Ki_speed = 0.0005 × 13.7² × cos(60°) = 0.0235
```

### 1.3 PWM Sampling and Discretization Effects

All motor control runs on digital PWM at fixed sample rate f_s.

#### Current Control Loop Timing

```
Typical PWM frequency: f_pwm = 10-20 kHz (50-100 µs period)
Control sample rate:   f_s = f_pwm / prescale

For f_pwm = 20 kHz:  T_s = 50 µs
Control bandwidth must satisfy Nyquist criterion:
  ωc < ωc_nyquist = π / T_s ≈ 62,832 rad/s

Current loop bandwidth (137 rad/s) is << 62,832 rad/s ✓
(Current bandwidth is ~0.2% of Nyquist, very safe)
```

#### Discretization of PI Controller

Continuous PI controller:
```
G_PI(s) = Kp + Ki/s
```

Discretized (backward Euler):
```
G_PI(z) = Kp + Ki·T_s/(1 - z⁻¹)

Difference equation:
  u(k) = u(k-1) + (Kp + Ki·T_s)·e(k) - Kp·e(k-1)

Or equivalently:
  integral(k) = integral(k-1) + Ki·T_s·error(k)
  u(k) = Kp·error(k) + integral(k)
```

**Important**: Use bilinear (Tustin) discretization for better accuracy:
```
Tustin substitution: s → (2/T_s)·(1 - z⁻¹)/(1 + z⁻¹)

G_PI(z) = [Kp + Ki·T_s/2] + [Ki·T_s/2 - Kp]·z⁻¹ / (1 - z⁻¹)
```

---

## Part 2: Motor Parameter Identification

### 2.1 Essential Motor Parameters

For accurate auto-tuning, must identify or estimate:

1. **Electrical Parameters (Temperature-Dependent):**
   - Rs: Stator resistance (increases ~0.4%/°C for copper)
   - Ld: d-axis inductance (usually constant)
   - Lq: q-axis inductance (usually > Ld for IPM motors)
   - Ψ_m: Permanent magnet flux linkage (decreases with temperature)

2. **Mechanical Parameters:**
   - J: Rotor inertia
   - B: Viscous damping / friction
   - τ_mech: Mechanical time constant (J/B)

3. **Operating Point:**
   - Speed (ω)
   - Current (I_d, I_q)
   - Torque
   - Temperature

### 2.2 Parameter Measurement Procedures

#### Offline Static Measurements (Recommended for Baseline)

**Stator Resistance (Rs):**
```
Procedure:
  1. Block rotor (prevent rotation)
  2. Apply low DC voltage (5-10% rated)
  3. Measure phase-to-neutral voltage
  4. Measure phase current
  5. Rs = V / I (measure at several voltage levels)

Expected values for target motors: 0.5 - 1.5 Ω
```

**Inductance (Ld, Lq) - AC Impedance Method:**
```
Procedure:
  1. Apply high-frequency AC voltage (1-2 kHz)
  2. Inject current at 0° (d-axis alignment)
  3. Measure d-axis current ripple: ΔI_d
  4. Calculate: L_d = V_ac / (2π·f·ΔI_d)

  5. Repeat for q-axis (90° offset)
  6. Calculate: L_q similarly

For SURFACE-mount PMSM: L_d ≈ L_q
For INTERIOR-mount PMSM: L_q > L_d (saliency)

Expected values: 3-8 mH (surface), 2-5 mH d-axis (interior)
```

**Back-EMF Constant (Ke):**
```
Procedure:
  1. Spin motor to known speed (mechanical drive or loaded)
  2. Measure open-circuit phase voltage: V_oc
  3. Calculate: Ke = V_oc / (ω·√3)

  Or from motor nameplate/datasheet:
  Ke = T_rated / I_rated (torque/current constant)

Expected relationship: Ke = Ψ_m = 3/2 × Ψ (for 3-phase FOC)
```

#### Online Parameter Identification (During Operation)

**MRAS Algorithm (Model Reference Adaptive System):**

Compares motor model prediction with measured output:

```
Reference model: ω_ref = estimated speed from measured voltage/current
Adaptive model: ω_adapt = speed observer output

Error: e = ω_ref - ω_adapt
Adaptation law: dθ/dt = -Γ·e·ω  (where θ = unknown parameter)

Converges to correct parameter as error → 0
```

**RLS Algorithm (Recursive Least Squares):**

Continuously updates parameter estimates:

```
Online estimation: θ̂(k) = θ̂(k-1) + P(k)·φ(k)·e(k)

where:
  θ̂ = parameter estimate
  φ = regression vector (depends on measured signals)
  e = prediction error
  P = covariance matrix (updated with forgetting factor λ)

Update: P(k) = (1/λ)·[P(k-1) - P(k-1)·φ·φᵀ·P(k-1) / (λ + φᵀ·P·φ)]
```

**Advantages of Online Methods:**
- Adapt to temperature changes during operation
- Account for motor saturation effects
- No need to stop motor for measurement
- Continuous tuning refinement

### 2.3 Temperature Compensation

Motor parameters change with temperature:

**Stator Resistance Temperature Coefficient:**
```
Rs(T) = Rs(T_ref) × [1 + α_Cu·(T - T_ref)]

α_Cu ≈ 0.004 /°C for copper

Example: If Rs = 0.8 Ω @ 25°C
  At 60°C: Rs = 0.8 × [1 + 0.004×35] = 0.912 Ω  (+14%)

This changes bandwidth by ~14% → must adapt Kp/Ki
```

**Permanent Magnet Flux Linkage Temperature Coefficient:**
```
Ψ_m(T) = Ψ_m(T_ref) × [1 + α_PM·(T - T_ref)]

α_PM ≈ -0.001 to -0.002 /°C (NdFeB magnets, negative!!)

This reduces back-EMF and torque constant with temperature
```

**Adaptation Strategy:**
```
1. Measure or estimate winding temperature (via resistance observer)
2. Compute Rs(T) using temperature coefficient
3. Recompute bandwidth target: ωc = Rs(T) / L
4. Update Kp/Ki gains continuously
5. Optional: Adjust speed loop Ki for flux change
```

---

## Part 3: Anti-Windup Implementation

### 3.1 Integral Windup Problem

**Root Cause in Cascaded FOC:**

```
Normal operation:
  Speed error → Speed PI output (desired current)
  → Current controller tracks this current
  → Motor responds, speed increases
  → Error decreases

Saturation problem:
  Large speed error (e.g., load step)
  → Speed PI integrator accumulates error
  → Output command exceeds max current
  → PWM saturates at full duty cycle
  → Actual current less than command (inverted feedback)
  → Integrator continues to wind up
  → When saturation releases, integrator is very large
  → Overshoot and oscillation
```

### 3.2 Back-Calculation Anti-Windup

**Design for Cascaded FOC:**

```
Speed Loop with Back-Calc Anti-Windup:

        ┌─────────────────┐
Error ─►│ Ki/s (integral) ├─┐
   │    └─────────────────┘ │
   │                        ├─► Sum ─► Saturate ─► I_max_cmd
   └───► Kp ───────────────┤
                            │
                    ◄───────┘
                    │
        Feedback from saturated output:
        Sat_error = I_max_cmd - I_actual

        Anti-windup gain: K_aw = 1/T_i (where T_i = Kp/Ki)

        Unwind command: integral -= K_aw × Sat_error

    Implementation:
    ┌─────────────────────────────────┐
    │ if (output > limit) {           │
    │   integral -= (output - limit)  │
    │ } else if (output < -limit) {   │
    │   integral += (output + limit)  │
    │ }                               │
    └─────────────────────────────────┘
```

**Tuning Anti-Windup Time Constant:**

```
K_aw = 1 / T_aw

Where T_aw is the time constant for recovering from windup.

Typical values:
  T_aw = 2-5 × (1/ωc_loop)

For speed loop ωc = 13.7 rad/s:
  T_aw_speed = 2-5 / 13.7 = 0.15-0.37 seconds

For current loop ωc = 137 rad/s:
  T_aw_current = 2-5 / 137 = 0.015-0.036 seconds
```

### 3.3 Tracking Mode (Conditional Integration)

More aggressive alternative:

```
Tracking logic: Only integrate when error is feasible

if error·output ≤ 0:  // Error has opposite sign from output
    integral += Ki × error × T_s  // Normal integration
else:                  // Both same sign (windup condition)
    integral = clamped_value     // Hold or reduce integral
```

### 3.4 Implementation Checklist

For each PI loop (current d, current q, speed, optionally flux):

- [ ] Implement saturation limits (hard limits on command)
- [ ] Compute saturation error = desired_output - actual_limited_output
- [ ] Apply anti-windup: integral -= K_aw × sat_error
- [ ] Verify K_aw = 1 / T_i (proportional to loop gain)
- [ ] Test windup recovery: ramp from -limit to +limit, verify smooth transition
- [ ] Validate no overshoot on saturation release

---

## Part 4: Validation Criteria for Auto-Tuned Gains

### 4.1 Step Response Validation

**Test Procedure: Speed Step Response**

```
Initial: Motor at 20% rated speed, no load
Command: Step to 80% rated speed
Measure: Speed response, current response, settling time

Success Criteria:
  ✓ Rise time (10%-90%): 100-200 ms (depends on ωc_speed)
  ✓ Overshoot: <20% (more aggressive) to <5% (conservative)
  ✓ Settling time (±2%): <500 ms
  ✓ Current peak: <150% rated current
  ✓ No oscillation after setpoint reached
```

**Test Procedure: Current Step Response**

```
Initial: Motor spinning at 50% speed, no command
Command: Inject 0.5A in d-axis (flux build)
Measure: Current response, settling

Success Criteria:
  ✓ Rise time: <10 ms
  ✓ Overshoot: <10% (critical - avoid current limit)
  ✓ Settling: <50 ms
  ✓ Ripple: <5% of steady-state current
```

### 4.2 Load Disturbance Rejection

**Test Procedure: Sudden Load Torque Step**

```
Initial: Motor at constant 50% speed, 20% load torque
Disturbance: Apply sudden 50% load increase
Measure: Speed deviation, current response, recovery time

Success Criteria:
  ✓ Speed dip: <5% (depends on Ki_speed and load inertia)
  ✓ Recovery time: <200 ms
  ✓ Current: stays within ratings during transient
  ✓ No oscillation during recovery
```

### 4.3 Frequency Domain Validation

**Phase Margin and Gain Margin:**

```
Measure open-loop frequency response of tuned controller + motor

Tools:
  - Frequency sweep (inject signal at [0.1×ωc, 0.5×ωc, ωc, 2×ωc, 10×ωc])
  - Measure gain and phase at each frequency
  - Estimate crossover frequency and margins

Success Criteria:
  ✓ Phase margin: 45-80° (as designed)
  ✓ Gain margin: >6 dB
  ✓ Crossover frequency: within ±20% of target
  ✓ No phase lag >90° (would indicate instability)
```

**MATLAB Command for Frequency Response:**

```matlab
% Assuming G_ol = open-loop transfer function
% G_ol(s) = G_PI(s) × G_motor(s)

w = logspace(0, 4, 500);  % 1-10k rad/s
[mag, phase] = bode(G_ol, w);

% Find crossover frequency (where |G| = 1)
idx = find(abs(mag - 1) == min(abs(mag - 1)));
wc = w(idx);
phase_at_wc = phase(idx);

% Phase margin
PM = 180 + phase_at_wc(1);

% Gain margin
idx_180 = find(abs(phase + 180) < 5);
if ~isempty(idx_180)
    GM_dB = -mag_dB(idx_180(1));
else
    GM_dB = inf;
end
```

### 4.4 Efficiency and Heat Dissipation

**Validation During Loaded Operation:**

```
Test: Run at rated speed/torque for 30 minutes

Measure:
  - Motor winding temperature (thermocouple or resistance)
  - Phase current RMS (thermal losses ∝ I²)
  - Supply voltage and current → mechanical power
  - Calculate efficiency: P_out / P_in

Success Criteria (for 48V motor @ rated conditions):
  ✓ Efficiency: ≥85% (baseline), ≥90% (good tuning)
  ✓ Temperature rise: <50°C above ambient
  ✓ Winding temperature: <100°C (permanent magnet limit)
  ✓ Current ripple: <20% RMS
```

### 4.5 Flux Angle Orthogonality

**Validation: d-q Axis Decoupling**

```
In proper FOC:
  - Id = 0 (or small commanded value) → No flux disturbance
  - Iq = proportional to torque → Clean torque control

Poor decoupling (coupling >10%) causes:
  - Current ripple
  - Torque ripple
  - Vibration/noise
  - Reduced efficiency

Measure cross-coupling:
  1. Command Iq = I_rated, Id = 0
  2. Measure actual Id vs time
  3. Calculate cross-coupling = |I_d_measured| / I_q_commanded

Success Criteria:
  ✓ Cross-coupling: <5% (excellent)
  ✓ Cross-coupling: <10% (acceptable)
```

---

## Part 5: Gain-Scheduling and Operating Region Adaptation

### 5.1 Speed-Dependent Gain Scheduling

Motor dynamics change with speed due to:
1. Back-EMF increases → less available voltage for current control
2. PWM dead-time effects change with current direction
3. Mechanical load may be speed-dependent (friction)

```
Adaptive bandwidth formula:

V_available(ω) = V_dc - ω·Ψ_m - I_d·R  (voltage available for current loop)

ωc_actual(ω) = [V_available(ω) / (Ψ_m·L)] × ωc_nominal

At zero speed:
  ωc = V_dc / (Ψ_m·L) × ωc_nominal (maximum available)

At rated speed:
  ωc may be limited by voltage (field-weakening region)
```

**Implementation:**

```python
def compute_pi_gains(omega_electrical, V_dc=48.0, Psi_m=0.15, Rs=0.8, L=0.0035):
    """
    Compute PI gains adapted to operating point

    Args:
        omega_electrical: electrical rotor speed (rad/s)
        V_dc: DC link voltage (V)
        Psi_m: permanent magnet flux linkage (Wb)
        Rs: stator resistance (Ω)
        L: stator inductance (H)

    Returns:
        Kp_i, Ki_i, Kp_w, Ki_w: current and speed loop PI gains
    """

    # Natural bandwidth
    omega_n = Rs / L  # rad/s

    # Available voltage for current loop
    V_back_emf = omega_electrical * Psi_m
    V_cmd_margin = V_dc - V_back_emf  # margin above back-EMF

    # Limit bandwidth in field-weakening region
    if V_cmd_margin < 0.2 * V_dc:
        # Severe field-weakening: reduce bandwidth
        omega_c_target = 0.3 * omega_n
    elif V_cmd_margin < 0.5 * V_dc:
        # Moderate field-weakening: conservative tuning
        omega_c_target = 0.4 * omega_n
    else:
        # Normal region: nominal tuning
        omega_c_target = 0.6 * omega_n

    # Current loop gains (phase margin 60°)
    phi_m = 60 * np.pi / 180  # convert to radians
    Kp_current = 2 * L * omega_c_target * np.sin(phi_m)
    Ki_current = L * omega_c_target**2 * np.cos(phi_m)

    # Speed loop (ωc = 1/10 of current loop, nominal inertia)
    J = 0.0005  # kg·m² (estimate)
    omega_c_speed = omega_c_target / 10
    Kp_speed = 2 * J * omega_c_speed * np.sin(phi_m)
    Ki_speed = J * omega_c_speed**2 * np.cos(phi_m)

    return Kp_current, Ki_current, Kp_speed, Ki_speed
```

### 5.2 Load-Dependent Adaptation

If mechanical parameters (J, B) can be estimated online:

```
Updated speed loop gains:

Kp_speed = 2 × J_estimated × ωc_speed × sin(φm)
Ki_speed = J_estimated × ωc_speed² × cos(φm)

Use MRAS or load observer to estimate:
  - Effective inertia (from acceleration transients)
  - Friction/damping (from steady-state current @ constant speed)
```

### 5.3 Temperature-Based Adaptation

```python
def adapt_gains_for_temperature(Kp_ref, Ki_ref, Rs_ref, T_ref, T_actual):
    """
    Adapt PI gains for temperature change

    Args:
        Kp_ref, Ki_ref: Reference gains at T_ref
        Rs_ref: Reference stator resistance at T_ref
        T_ref: Reference temperature (°C)
        T_actual: Actual operating temperature (°C)

    Returns:
        Kp_adapted, Ki_adapted: Temperature-compensated gains
    """

    alpha_Cu = 0.004  # Temperature coefficient for copper

    # Compute resistance at actual temperature
    Rs_actual = Rs_ref * (1 + alpha_Cu * (T_actual - T_ref))

    # Resistance ratio (bandwidth changes proportionally)
    ratio = Rs_actual / Rs_ref

    # Scale gains to maintain constant bandwidth
    Kp_adapted = Kp_ref * ratio
    Ki_adapted = Ki_ref * ratio

    return Kp_adapted, Ki_adapted
```

---

## Part 6: Grid Search Replacement Algorithm

### 6.1 Proposed Analytical Tuning Sequence

Instead of 12×12 = 144 grid points, use this sequence:

```
Step 1: Offline Parameter Identification
  ├─ Measure/estimate: Rs, L, J, Ψ_m
  └─ Result: Motor parameters at reference temperature

Step 2: Compute Nominal Bandwidth
  ├─ ωc_natural = Rs / L
  ├─ ωc_current = 0.5 to 0.7 × ωc_natural
  └─ ωc_speed = ωc_current / 10

Step 3: Compute Reference Gains (Phase Margin 60°)
  ├─ Kp_current, Ki_current (from bandwidth formula)
  ├─ Kp_speed, Ki_speed (from bandwidth formula)
  └─ Result: 4 PI gains (2 per loop)

Step 4: Validate Current Loop
  ├─ Apply small current step test
  ├─ Verify: Rise time < 10 ms, no oscillation
  ├─ If unstable: reduce Kp by 20%, retry
  ├─ If too slow: increase Kp by 10%, retry
  └─ Result: Tuned current gains

Step 5: Validate Speed Loop
  ├─ Apply speed step test (20% to 80%)
  ├─ Verify: Overshoot <20%, settling <500 ms
  ├─ If overshoot: reduce Ki_speed by 20%, retry
  ├─ If oscillatory: increase damping (reduce Ki)
  └─ Result: Tuned speed gains

Step 6: Validate Under Load
  ├─ Apply load disturbance (50% increase)
  ├─ Verify: Speed dip <5%, recovery <200 ms
  ├─ If excessive ripple: reduce Ki, increase Kp
  └─ Result: Final validated gains

Step 7: Characterize Operating Region
  ├─ Test at: 20%, 50%, 80% speed
  ├─ Test at: 25%, 50%, 100% load
  ├─ Adapt gains using gain-scheduling
  └─ Result: Gain-scheduled controller
```

### 6.2 Pseudo-Code Implementation

```python
class FOCAutoTuner:
    def __init__(self, motor_model, controller_interface):
        self.motor = motor_model
        self.ctrl = controller_interface
        self.validation_log = []

    def tune_foc(self):
        """Complete auto-tuning sequence"""

        # Step 1: Parameter identification
        params = self.identify_parameters()
        self.log(f"Identified parameters: Rs={params['Rs']:.3f}Ω, L={params['L']*1e3:.2f}mH")

        # Step 2: Compute bandwidth
        omega_n = params['Rs'] / params['L']
        omega_c = 0.6 * omega_n
        self.log(f"Target bandwidth: ωc={omega_c:.1f} rad/s ({omega_c/(2*π):.1f} Hz)")

        # Step 3: Compute gains
        gains = self.compute_pi_gains_from_bandwidth(omega_c, params)
        self.ctrl.set_pi_gains(gains)
        self.log(f"Computed gains: Kp={gains['Kp_i']:.3f}, Ki={gains['Ki_i']:.1f}")

        # Step 4: Validate current loop
        self.validate_current_loop()
        if not self.validation_log[-1]['success']:
            gains = self.refine_gains_current(gains)
            self.ctrl.set_pi_gains(gains)
            self.validate_current_loop()

        # Step 5: Validate speed loop
        self.validate_speed_loop()
        if not self.validation_log[-1]['success']:
            gains = self.refine_gains_speed(gains)
            self.ctrl.set_pi_gains(gains)
            self.validate_speed_loop()

        # Step 6: Load disturbance test
        self.validate_load_rejection()
        if not self.validation_log[-1]['success']:
            self.log("WARNING: Poor load rejection, may need mechanical redesign")

        # Step 7: Characterize operating region
        self.characterize_operating_region(gains, params)

        return gains

    def identify_parameters(self):
        """Estimate motor parameters"""
        # Option A: Use offline measurements (Rs, L, J, Ψ_m)
        # Option B: Use MRAS-based online identification
        # For now, return estimates based on motor specs
        pass

    def compute_pi_gains_from_bandwidth(self, omega_c, params, phi_m=60):
        """Analytic computation of PI gains"""
        phi_m_rad = phi_m * π / 180

        # Current loop gains
        Kp_i = 2 * params['L'] * omega_c * sin(phi_m_rad)
        Ki_i = params['L'] * omega_c**2 * cos(phi_m_rad)

        # Speed loop gains
        omega_c_speed = omega_c / 10
        Kp_w = 2 * params['J'] * omega_c_speed * sin(phi_m_rad)
        Ki_w = params['J'] * omega_c_speed**2 * cos(phi_m_rad)

        return {
            'Kp_i': Kp_i, 'Ki_i': Ki_i,
            'Kp_w': Kp_w, 'Ki_w': Ki_w
        }

    def validate_current_loop(self):
        """Test current loop step response"""
        result = {
            'test': 'current_loop',
            'success': False,
            'metrics': {}
        }

        # Inject 0.5 A step in d-axis
        self.ctrl.set_speed_reference(50)  # 50% speed
        time.sleep(0.5)  # Settle

        # Apply current step
        I_cmd_old = self.ctrl.get_current_command()
        self.ctrl.set_current_command([I_cmd_old[0] + 0.5, I_cmd_old[1]])

        # Acquire response
        t, I_actual, _ = self.ctrl.capture_data(duration=0.05, fs=10e3)

        # Analyze
        result['metrics']['rise_time'] = self.compute_rise_time(t, I_actual)
        result['metrics']['overshoot'] = self.compute_overshoot(t, I_actual)
        result['metrics']['settling_time'] = self.compute_settling_time(t, I_actual)

        # Check success
        result['success'] = (
            result['metrics']['rise_time'] < 0.01 and  # 10 ms
            result['metrics']['overshoot'] < 0.1       # 10%
        )

        self.validation_log.append(result)
        return result

    # ... additional validation and refinement methods
```

---

## Part 7: Integration with BLDC-Simulator Project

### 7.1 Replacement for `adaptive_tuning.py`

Current implementation uses grid search. Proposed replacement:

```python
# File: BLDC-Simulator/foc_control/analytical_autotuner.py

import numpy as np
from scipy.signal import TransferFunction, step, bode, dB
import matplotlib.pyplot as plt

class AnalyticalFOCAutoTuner:
    """
    Replaces grid-search based tuning with analytical, bandwidth-based PI gain computation.
    Reduces calibration from 144 evaluations to ~7 targeted tests.
    """

    def __init__(self, motor_model, pwm_freq=20e3):
        self.motor = motor_model
        self.pwm_freq = pwm_freq
        self.ts = 1 / pwm_freq
        self.results = {}

    # Implementation follows pseudo-code in Section 6.2
```

### 7.2 Configuration for Target Motors

```yaml
# File: BLDC-Simulator/config/motor_autotuning_config.yaml

motors:
  motenergy_me1718:
    rated_voltage: 48.0  # V
    rated_speed: 4000    # RPM
    rated_torque: 14.3   # Nm
    rated_power: 6.0     # kW

    # Estimated parameters (obtain from datasheet or measurement)
    parameters:
      Rs: 0.80            # Ω (stator resistance, @ 25°C)
      Ld: 3.2e-3          # H (d-axis inductance)
      Lq: 3.8e-3          # H (q-axis inductance)
      Psi_m: 0.150        # Wb (flux linkage)
      J: 5.0e-4           # kg·m² (rotor inertia)
      B: 0.001            # Nm·s/rad (friction coefficient)

    # Tuning parameters
    tuning:
      phase_margin: 60    # degrees
      bandwidth_ratio: 0.6  # fraction of R/L
      anti_windup_time: 0.05  # seconds
      temperature_ref: 25  # °C
      alpha_Rs: 0.004     # temperature coefficient

autotuning_config:
  current_loop:
    test_duration: 0.05      # seconds
    test_current: 0.5        # A
    settling_threshold: 0.02 # 2% of command

  speed_loop:
    test_duration: 2.0       # seconds
    speed_step_from: 0.2     # 20% speed
    speed_step_to: 0.8       # 80% speed
    max_overshoot: 0.20      # 20% acceptable
    max_settling_time: 0.5   # seconds

  load_test:
    initial_load: 0.2        # 20% of rated torque
    step_load: 0.5           # 50% increase
    max_speed_dip: 0.05      # 5%
```

### 7.3 Updated Simulator Interface

```python
# Modified interface for auto-tuning

class FOCController:
    def __init__(self, motor_params, use_analytical_tuning=True):
        self.motor_params = motor_params

        if use_analytical_tuning:
            tuner = AnalyticalFOCAutoTuner(motor_params)
            self.pi_gains = tuner.tune_foc()
        else:
            # Fall back to old grid search if needed
            tuner = GridSearchAutoTuner(motor_params)
            self.pi_gains = tuner.tune_foc()

    def update(self, dt, speed_ref, load_torque, winding_temp):
        """
        FOC update with adaptive gain scheduling

        Args:
            dt: time step
            speed_ref: speed reference (0-1, fraction of rated)
            load_torque: mechanical load torque (Nm)
            winding_temp: estimated winding temperature (°C)
        """

        # Adapt gains for operating point
        gains_adapted = self.schedule_gains(
            speed_ref, load_torque, winding_temp
        )

        # ... rest of FOC implementation
```

---

## Part 8: Validation Results Template

Document your tuning results:

```markdown
## FOC Auto-Tuning Results for [Motor Type]

### Identified Parameters
- Rs = 0.80 Ω
- L = 3.5 mH
- J = 0.5 g·cm² = 5×10⁻⁴ kg·m²
- Ψ_m = 0.150 Wb

### Computed Gains
- Kp_current = 0.835
- Ki_current = 32.8
- Kp_speed = 0.00594
- Ki_speed = 0.0235

### Validation Results

#### Current Loop Step Response
- Rise time: 8.2 ms ✓ (target < 10 ms)
- Overshoot: 6.5% ✓ (target < 10%)
- Settling time: 24 ms ✓
- Stability margin verified

#### Speed Loop Step Response (20% → 80%)
- Rise time: 142 ms ✓
- Overshoot: 12% ✓ (target < 20%)
- Settling time: 380 ms ✓
- No oscillation

#### Load Disturbance Test
- Load step: 20% → 70% torque
- Speed dip: 3.2% ✓ (target < 5%)
- Recovery time: 185 ms ✓
- Current stayed within limit

#### Efficiency Under Load
- Measured: 88.5% at rated torque
- Temperature rise: 35°C
- Winding temp: 60°C (well below limit)

### Gain-Scheduled Adaptation
[Create table showing gains vs speed/load]

### Conclusion
Analytical PI tuning successfully replaces grid search.
Achieves 7 targeted tests vs. 144 grid evaluations.
Motor ready for deployment.
```

---

## References

See `BIBLIOGRAPHY.md` for complete citation list (45+ papers).

**Key References for This Guide:**
1. Microchip MCAF Current Loop Tuning Documentation
2. MathWorks FOC Autotuner Block Documentation
3. IEEE Xplore PI Tuning Methods for Motor Control
4. MDPI/Frontiers Recent Sensorless Observer Papers
5. ResearchGate Anti-Windup Implementation Studies

---

## Document Revision History

| Date | Version | Changes |
|------|---------|---------|
| 2026-03-30 | 1.0 | Initial guide with 8 comprehensive sections |

