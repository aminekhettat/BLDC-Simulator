# FOC PI Tuning - Quick Reference Formulas

## Core PI Tuning Equations

### Current Loop (L-R Dynamics)

**Motor Transfer Function (Synchronous Frame):**
```
G_motor(s) = 1 / (L·s + R)

Natural Bandwidth: ωn = R/L [rad/s]
Natural Frequency: fn = R/(2π·L) [Hz]
Time Constant: τ = L/R [ms]
```

**PI Controller Transfer Function:**
```
G_PI(s) = Kp + Ki/s = (Kp·s + Ki) / s
```

**Closed-Loop Transfer Function:**
```
G_CL(s) = (Kp·s + Ki) / (L·s² + (R + Kp)·s + Ki)

Standard form: G_CL(s) = ωn²·(s/Q + ωn) / (s² + 2ζ·ωn·s + ωn²)

where:
  ζ = damping ratio = (R + Kp) / (2√(L·Ki))
  ωn_closed = √(Ki/L) = bandwidth
  Q = ωn_closed / (2ζ)
```

**PI Gains from Bandwidth and Phase Margin:**

For specified closed-loop bandwidth **ωc** and phase margin **φm**:

```
Kp = 2·L·ωc·sin(φm)
Ki = L·ωc²·cos(φm)

Alternative form (using damping ratio):
  ζ_desired = sqrt(2)/2 ≈ 0.707  (phase margin 65°)
  Ki = L·ωc²
  Kp = 2·L·ωc·ζ_desired
```

**Cross-Check Verification:**

```
Actual Phase Margin = atan2( 2ζ√(1-ζ²), 1-2ζ² ) [for underdamped: ζ < √2/2]

Overshoot = exp(-ζ·π/√(1-ζ²)) × 100%

Rise Time (10-90%) ≈ 2.16 / (ζ·ωn)
Settling Time (±2%) ≈ 4.1 / (ζ·ωn)
```

### Speed Loop (Mechanical + First-Order Current Loop)

**Effective Motor Seen by Speed Loop:**
```
Speed loop sees current loop as first-order delay:
  I_actual(s) / I_command(s) = ωc_current / (s + ωc_current)

Mechanical system:
  J·dω/dt = T_em - T_load - B·ω

Combined:
  G_motor_speed(s) = (Ke·ωc_current) / (J·s·(s + ωc_current))

where Ke = torque constant [Nm/A]
```

**Speed Loop PI Gains:**

Standard tuning rule:
```
Bandwidth ratio: ωc_speed = ωc_current / 10

Kp_speed = 2·J·ωc_speed·sin(φm)
Ki_speed = J·ωc_speed²·cos(φm)

Typical values:
  J = 0.3-0.5 g·cm² = 3-5 × 10⁻⁵ kg·m² (small BLDC)
  J = 3-5 kg·cm² = 3-5 × 10⁻⁴ kg·m² (medium)
  ωc_speed ≈ 10-20 rad/s
```

---

## Motor Parameter Calculations

### From Motor Nameplate

```
Rated Electrical Power: P_elec [W]
Rated Voltage: V_rated [V]
Rated Torque: T_rated [Nm]
Rated Current: I_rated [A]
Rated Speed: N_rated [RPM]

Derived:
  ωm_rated = N_rated × π/30 [rad/s] (mechanical)
  ωe_rated = ωm_rated × Pole_Pairs [rad/s] (electrical)

  Torque Constant: Ke = T_rated / I_rated [Nm/A]
  Flux Linkage: Ψm = Ke / Pole_Pairs [Wb/rad]

  Efficiency: η = P_mech / P_elec = (T_rated × ωm) / V_rated / I_rated

  Back-EMF at rated: Eo = Ke × ωm_rated [V]
```

### From Resistance Measurement

**Stator Resistance (Blocked Rotor Test):**
```
Procedure:
  1. Block rotor (prevent rotation)
  2. Apply DC voltage V_dc (5-10% rated)
  3. Measure phase current: I_meas
  4. Rs = V_dc / I_meas  (phase-to-neutral)

Temperature correction:
  Rs(T) = Rs(T_ref) × [1 + α·(T - T_ref)]

  α_copper ≈ 0.004 /°C (stainless wire: 0.0002 /°C)
  α_aluminum ≈ 0.004 /°C
```

### From Inductance Measurement

**DC Inductance Method (AC Impedance):**
```
1. Apply high-frequency AC voltage (1-2 kHz)
2. Inject test current through a-phase (rotor at d-axis)
3. Measure phase current ripple: ΔI
4. Ld = V_ac / (2π·f·ΔI)

5. Rotate to q-axis (90°) and repeat
6. Lq = V_ac / (2π·f·ΔI_q)

For SURFACE-MOUNT PMSM:
  Ld ≈ Lq ≈ L (isotropic)

For INTERIOR-MOUNT PMSM:
  Lq > Ld (Lq/Ld ≈ 1.5 to 3)
  Use average: L_avg = (Ld + Lq) / 2
```

**Frequency-Dependent Inductance:**

At higher frequencies, inductance decreases due to skin effect.

```
L(f) ≈ L(dc) / (1 + (f/f_pole)^n)

where f_pole is pole frequency, n ≈ 0.5-1.5

For BLDC control frequencies (kHz):
  Use DC inductance value directly (error < 5%)
```

### Inertia Estimation

**From Acceleration Transient:**

```
Apply known torque, measure angular acceleration:
  J = ΔT_em / (dω/dt)

Experimental procedure:
  1. Set speed reference step from 0 to 50%
  2. Record t, ω(t)
  3. Extract dω/dt during acceleration
  4. Compute: J = (Ki_current × ΔI) / (dω/dt)

Typical values:
  Small motor (10W): J = 0.5 g·cm²
  Medium motor (100W): J = 5 g·cm²
  Large motor (1000W): J = 50 g·cm²

Conversion: 1 g·cm² = 10⁻⁵ kg·m²
```

---

## Bandwidth and Time Constant Relationships

### Natural (Uncontrolled) Motor Bandwidth

```
Current time constant: τ = L/R [seconds]

Small BLDC:
  R ≈ 1-2 Ω, L ≈ 1-3 mH
  τ ≈ 1-3 ms
  ωn ≈ 300-1000 rad/s (50-160 Hz)

Medium BLDC:
  R ≈ 0.5-1 Ω, L ≈ 3-5 mH
  τ ≈ 3-10 ms
  ωn ≈ 100-300 rad/s (16-50 Hz)

Typical practical current loop bandwidth:
  ωc_current ≈ 0.5 × ωn  (to 0.7 × ωn for aggressive)

Speed loop:
  ωc_speed = ωc_current / 10 (standard ratio)
```

### PWM Frequency Constraint

```
Nyquist frequency: f_nyquist = f_pwm / 2

For f_pwm = 20 kHz:
  f_nyquist = 10 kHz
  ωc_nyquist = 62,832 rad/s

Safe bandwidth: ωc < f_pwm / 10 (to avoid aliasing artifacts)
  ωc_max ≈ 6,283 rad/s (1 kHz)

Typical current loop: 50-500 rad/s
Typical speed loop: 5-50 rad/s
(Both >> below Nyquist, safe for 20 kHz PWM)
```

---

## Phase Margin and Stability

### Phase Margin Definition

```
At open-loop crossover frequency (where |G_OL(jωc)| = 1):
  Phase Margin = 180° + ∠G_OL(jωc)

Interpretable values:
  PM = 80°: Very conservative, slow response, no overshoot
  PM = 60°: Standard, good balance of speed/stability
  PM = 45°: Aggressive, faster but with overshoot
  PM = 30°: Very aggressive, risk of oscillation
  PM < 15°: Unstable or borderline

Overshoot vs Phase Margin (approximate):
  PM = 90° → OS ≈ 0%
  PM = 60° → OS ≈ 5-10%
  PM = 45° → OS ≈ 10-15%
  PM = 30° → OS ≈ 25-30%
  PM = 15° → OS ≈ 50%+
```

### Gain Margin

```
At phase crossover (where ∠G_OL = -180°):
  Gain Margin = 1 / |G_OL(jω_180)|

Safe values:
  GM > 2 (6 dB): Stable
  GM > 1.5 (3.5 dB): Marginal
  GM < 1.2 (1.6 dB): Risky

For well-designed PI control:
  GM > 6 dB (factor of 2)
  PM > 45°
```

---

## Anti-Windup Tuning

### Back-Calculation Method

```
Integrator unwind rate:
  K_aw = 1 / T_i = 1 / (Kp / Ki)

Saturation error detection:
  If (output_commanded > output_saturated):
    sat_error = output_commanded - output_saturated
    integral -= K_aw × sat_error

Recovery time constant:
  T_recover ≈ 1 / K_aw = Kp / Ki

Typical values:
  T_i_current = Kp / Ki ≈ 0.01-0.05 s
  T_i_speed = Kp / Ki ≈ 0.05-0.2 s

K_aw should be ~1/T_i for smooth recovery.
```

### Tracking (Conditional Integration)

```
Advanced method: Only integrate feasible errors

if (error · output) ≤ 0:
    # Error sign opposite to output → normal integration
    integral += Ki · error · dt
else:
    # Both same sign → windup condition
    # Freeze or reduce integrator
    integral = clamp(integral, limit)

This prevents accumulation during saturation.
```

---

## Discretization Effects

### Sampling Period

```
For control frequency f_s = f_pwm / prescale:
  T_s = 1 / f_s [seconds]

Typical:
  f_pwm = 20 kHz
  prescale = 1 → f_s = 20 kHz, T_s = 50 µs
  prescale = 2 → f_s = 10 kHz, T_s = 100 µs

Current loop update: T_s_current ≈ 50 µs (20 kHz)
Speed loop update: T_s_speed ≈ 1-10 ms (100-1000 Hz)
```

### Continuous to Discrete Conversion

**Forward Euler (simplest, least accurate):**
```
s → (z - 1) / T_s

G_PI(z) = Kp + Ki·T_s / (z - 1)
```

**Backward Euler (stable, moderate accuracy):**
```
s → (z - 1) / (z·T_s)

G_PI(z) = Kp + Ki·T_s / (1 - z⁻¹)

Implementation:
  integral(k) = integral(k-1) + Ki·T_s·error(k)
  output(k) = Kp·error(k) + integral(k)
```

**Tustin/Bilinear (best, recommended):**
```
s → 2(z-1) / [T_s(z+1)]

G_PI(z) = [Kp + Ki·T_s/2] + [Ki·T_s/2 - Kp]·z⁻¹ / (1 - z⁻¹)

Difference equation:
  a0 = Kp + Ki·T_s/2
  a1 = Ki·T_s/2 - Kp
  output(k) = a0·error(k) + a1·error(k-1) + output(k-1)
```

**Warping for high bandwidth (when ωc·T_s > 0.3):**
```
Pre-warp cutoff frequency:
  ωc_warped = (2/T_s) × tan(ωc·T_s/2)

Then apply Tustin with ωc_warped instead of ωc
```

---

## Temperature Compensation

### Resistance Temperature Coefficient

```
Copper: α_Cu ≈ 0.004 /°C
Aluminum: α_Al ≈ 0.004 /°C
Stainless wire: α_ss ≈ 0.0002 /°C

Rs(T) = Rs(T_ref) × [1 + α·(T - T_ref)]

Example (Copper, T_ref = 25°C):
  At 25°C: Rs = 0.80 Ω
  At 60°C (ΔT = 35°C): Rs = 0.80 × [1 + 0.004×35] = 0.912 Ω (+14%)

Bandwidth impact:
  ωn = Rs/L  increases 14%
  Kp and Ki must increase 14% to maintain same bandwidth
```

### Permanent Magnet Flux Temperature Coefficient

```
NdFeB magnets: α_PM ≈ -0.001 to -0.002 /°C (decreases with T)
SmCo magnets: α_PM ≈ -0.0003 /°C (more stable)
Ferrite: α_PM ≈ -0.002 /°C

Ψm(T) = Ψm(T_ref) × [1 + α_PM·(T - T_ref)]

Example (NdFeB, α = -0.0015 /°C):
  At 25°C: Ψm = 0.150 Wb
  At 60°C (ΔT = 35°C): Ψm = 0.150 × [1 - 0.0015×35] = 0.1442 Wb (-3.8%)

Effects:
  - Reduces back-EMF and torque constant
  - Affects field-weakening boundary
  - Less impact on current loop gains (depends on Rs, L)
```

---

## Quick Calculation Spreadsheet

### Input: Motor Parameters

```
Motor Type: _______________
Rated Power: _____ W
Rated Voltage: _____ V
Rated Speed: _____ RPM
Rated Torque: _____ Nm

Measured/Estimated:
  Rs (25°C): _____ Ω
  L (avg): _____ mH = _____ H
  J: _____ g·cm² = _____ kg·m²
  Ψm: _____ Wb
  Pole Pairs: _____
  Temperature (actual): _____ °C
```

### Calculated: Motor Dynamics

```
Natural bandwidth: ωn = Rs/L = _____ rad/s = _____ Hz
Time constant: τ = L/R = _____ ms

Electrical frequency @ rated speed:
  ωe_rated = (N_rpm × π/30) × P_pairs = _____ rad/s

Back-EMF @ rated speed:
  Eo = Ψm × ωe = _____ V
```

### Calculated: PI Gains (Phase Margin 60°)

```
Target bandwidth (60% of natural):
  ωc_current = 0.6 × ωn = _____ rad/s

Current loop gains:
  sin(60°) = 0.866, cos(60°) = 0.5
  Kp_current = 2 × L × ωc × 0.866 = _____
  Ki_current = L × ωc² × 0.5 = _____

Speed loop (1/10 bandwidth):
  ωc_speed = ωc_current / 10 = _____ rad/s
  Kp_speed = 2 × J × ωc_speed × 0.866 = _____
  Ki_speed = J × ωc_speed² × 0.5 = _____
```

### Verification: Response Characteristics

```
Expected rise time (current): 2.16 / (ζ × ωn) = _____ ms
Expected rise time (speed): 2.16 / (ζ × ωc_speed) = _____ ms
Expected overshoot: exp(-ζπ/√(1-ζ²)) = _____ %
Expected settling time: 4.1 / (ζ × ωc) = _____ ms

Overshoot (60° PM): ~5-10%
Overshoot (45° PM): ~10-15%
Overshoot (80° PM): <2%
```

---

## Testing Checklist

### Before Tuning
- [ ] Verify motor parameters (Rs, L, J from datasheet/measurement)
- [ ] Confirm voltage rating and safety limits
- [ ] Confirm current sensing accuracy
- [ ] Verify PWM frequency and sample rate
- [ ] Check anti-windup implementation exists

### Current Loop Validation
- [ ] Step response: rise time < 10 ms ✓
- [ ] Step response: overshoot < 10% ✓
- [ ] No ringing or oscillation
- [ ] Stable across 0-100% command

### Speed Loop Validation
- [ ] Step 20% → 80%: overshoot < 20% ✓
- [ ] Step 20% → 80%: settling < 500 ms ✓
- [ ] No oscillation in steady-state
- [ ] Stable load rejection (±5% speed dip) ✓

### Operating Region Characterization
- [ ] Test at 25%, 50%, 100% load
- [ ] Test at 20%, 50%, 80% speed
- [ ] Verify field-weakening region (if applicable)
- [ ] Document gain-schedule table

### Thermal Validation
- [ ] Monitor winding temperature
- [ ] Verify efficiency > 85%
- [ ] Measure current ripple
- [ ] Confirm no thermal runaway

---

## Common Tuning Mistakes

| Mistake | Symptom | Fix |
|---------|---------|-----|
| Ki too high | Oscillation, overshoot | Reduce Ki by 20-50% |
| Kp too high | Noise on output | Reduce Kp by 10-30% |
| No anti-windup | Large overshoot on step | Add back-calculation |
| Wrong L value | Unstable at high speed | Remeasure inductance |
| Ignoring sampling | Aliasing distortion | Reduce ωc below f_pwm/10 |
| Temperature ignored | Works at 25°C, fails at 60°C | Add temperature compensation |
| Load-dependent tuning | Poor load rejection | Add gain scheduling for inertia |

---

## References for Formulas

- **PI Gains from Bandwidth:** Microchip MCAF Documentation, Section 5.1.6
- **Phase Margin:** IEEE Xplore "Tuning Rules for PI Gains of FOC"
- **Discretization:** Digital Control textbooks (Franklin, Powell, Workman)
- **Temperature Compensation:** Motor manufacturer datasheets

**Last Updated:** 2026-03-30

