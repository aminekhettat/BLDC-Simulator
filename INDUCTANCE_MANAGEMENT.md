# Inductance Parameters Management in BLDC Motor Model

## Overview

This document explains how three different inductance parameters are managed and used in the BLDC motor control system: **L (phase_inductance)**, **Ld (d-axis inductance)**, and **Lq (q-axis inductance)**.

---

## Three Types of Inductance

### 1. **L (phase_inductance)** ⚡

**Status:** ✅ **ACTIVELY USED** in motor model simulation

**Purpose:**

- Phase winding inductance of the stator coils
- Used in the **trapezoidal back-EMF motor model**

**Used in:** Electrical dynamics equation

```python
# From motor_model.py (line 351)
di_dt = (voltages - phase_resistance * currents - emf) / phase_inductance
```

**How it works:**

- Controls how fast phase currents respond to applied voltage
- Larger L → slower current rise (more inductive)
- Smaller L → faster current rise (less inductive)
- Affects system stability and current ripple

**Unit:** Henries (H)  
**Default:** 0.005 H (5 mH)  
**Range:** Typically 1-50 mH for BLDC motors

**Example:**

```python
params = MotorParameters(
    phase_inductance=0.005  # 5 mH - Used in actual simulation
)
```

---

### 2. **Ld (d-axis inductance)** 📐

**Status:** ⚠️ **STORED BUT NOT USED** in current motor model

**Purpose:**

- d-axis inductance in Field-Oriented Control (FOC) coordinate frame
- Would be used for d/q-axis current transformations in advanced control algorithms

**When it's used:**

- Only when using **FOCController** with proper d/q transformations
- Currently NOT implemented in the motor model

**How it works in FOC (when implemented):**

- In FOC, 3-phase currents are transformed to direct (d) and quadrature (q) axes
- Ld controls the dynamic response of d-axis current loop
- Used in PI current controller equations: `V_d = K_pd * e_d + K_id * integral(e_d)`

**Unit:** Henries (H)  
**Default:** Defaults to `phase_inductance` (equalized)  
**Range:** Typically 0.3-2.0 mH for non-salient BLDC motors

**Example:**

```python
params = MotorParameters(
    phase_inductance=0.005,
    ld=0.0035  # d-axis inductance (different from phase L)
)
```

---

### 3. **Lq (q-axis inductance)** 📐

**Status:** ⚠️ **STORED BUT NOT USED** in current motor model

**Purpose:**

- q-axis inductance in Field-Oriented Control (FOC) coordinate frame
- Would be used for q-axis current loop control

**When it's used:**

- Only when using **FOCController** with proper d/q transformations
- Currently NOT implemented in the motor model

**How it works in FOC (when implemented):**

- Similar to Ld but for quadrature (torque-producing) axis
- Lq controls q-axis current dynamics

**Unit:** Henries (H)  
**Default:** Defaults to `phase_inductance` (equalized)  
**Range:** Typically 0.3-2.0 mH for non-salient BLDC motors

**Example:**

```python
params = MotorParameters(
    phase_inductance=0.005,
    ld=0.0035,
    lq=0.0035  # q-axis inductance
)
```

---

## Current Implementation Status

### ✅ What Works Now (Trapezoidal Model)

| Inductance | Used?  | Where                     | Effect                             |
| ---------- | ------ | ------------------------- | ---------------------------------- |
| **L**      | ✅ YES | Motor electrical dynamics | Affects current rise time & ripple |
| **Ld**     | ❌ NO  | Not used (stored only)    | No effect on simulation            |
| **Lq**     | ❌ NO  | Not used (stored only)    | No effect on simulation            |

### Code Flow

```
MotorParameters
├── phase_inductance = 0.005  ✅ Used in step()
├── ld = 0.005 (defaults to phase_inductance)  ❌ Stored, not used
└── lq = 0.005 (defaults to phase_inductance)  ❌ Stored, not used
       ↓
BLDCMotor.step()
       ↓
_calculate_derivatives()
       ↓
di_dt = (V - R*i - E) / phase_inductance  ← Only phase_inductance is used
```

---

## Why Ld/Lq Are Not Used Now

The current motor model uses a **trapezoidal back-EMF model** which:

- Models the motor in **3-phase coordinate frame** (a, b, c)
- Does NOT perform d/q-axis transformations
- Does NOT use Ld/Lq parameters

For Ld/Lq to be used, you would need:

1. **Clarke Transform** (3-phase → 2-phase alpha/beta)

   ```
   i_alpha = i_a
   i_beta = (i_a + 2*i_b) / sqrt(3)
   ```

2. **Park Transform** (alpha/beta → d/q rotating frame)

   ```
   i_d = i_alpha * cos(theta) + i_beta * sin(theta)
   i_q = -i_alpha * sin(theta) + i_beta * cos(theta)
   ```

3. **d/q-axis current dynamics** (would use Ld, Lq)
   ```python
   di_d/dt = (V_d - R*i_d + Lq*omega*i_q) / Ld
   di_q/dt = (V_q - R*i_q - Ld*omega*i_d) / Lq
   ```

---

## How to Use Ld/Lq Parameters

### To Monitor Current Values (Inspection Only)

```python
# In UI monitoring, Ld/Lq are displayed in Quick Info panel
# They show whatever was set, but don't affect motor behavior

motor = BLDCMotor(MotorParameters(
    phase_inductance=0.005,
    ld=0.004,  # Displayed in UI but not used
    lq=0.006   # Displayed in UI but not used
))
```

### To Actually Use Ld/Lq (Future Implementation)

When you implement FOCController with d/q transformations:

- Use `motor.params.ld` in d-axis current loop control
- Use `motor.params.lq` in q-axis current loop control
- These will affect current response in FOC algorithm only

---

## Best Practices

### ✅ DO

- Set `Ld` and `Lq` if you plan to implement FOC control later
- Use different Ld/Lq values for salient-pole BLDC motors where they significantly differ
- Adjust `phase_inductance` to match your motor's actual inductance

### ❌ DON'T

- Expect Ld/Lq to affect trapezoidal V/f control (they don't)
- Set `phase_inductance` to 0 (will cause division by zero)
- Confuse `phase_inductance` (3-phase model) with Ld/Lq (d/q-axis model)

---

## Example Scenarios

### Scenario 1: Basic Trapezoidal Control (Current)

```python
params = MotorParameters(
    phase_inductance=0.005,  # ✅ This will be used
    ld=0.005,                # ⚠️  Displayed but not used
    lq=0.005                 # ⚠️  Displayed but not used
)

# Ld/Lq affect nothing - only phase_inductance matters for motor
```

### Scenario 2: Preparing for FOC (Future)

```python
# Salient-pole motor with different d/q inductances
params = MotorParameters(
    phase_inductance=0.005,  # Used in trapezoidal model
    ld=0.003,                # Will be used when FOC is implemented
    lq=0.006                 # Will be used when FOC is implemented
)

# For now: only phase_inductance affects motor behavior
# In future: Ld/Lq will control FOC current loops
```

### Scenario 3: Non-Salient Motor (Isotropic)

```python
# Most small BLDC motors are non-salient (Ld ≈ Lq ≈ L)
params = MotorParameters(
    phase_inductance=0.005,  # Used in motor model
    ld=0.005,                # Same as phase_inductance
    lq=0.005                 # Same as phase_inductance
)

# All three are equal - typical for commodity BLDC motors
```

---

## Summary Table

| Feature                 | L (phase_inductance) | Ld               | Lq               |
| ----------------------- | -------------------- | ---------------- | ---------------- |
| **Current Status**      | ✅ In Use            | ⚠️ Stored        | ⚠️ Stored        |
| **Affects Simulation**  | YES                  | NO               | NO               |
| **Motor Model Type**    | Trapezoidal          | d/q-axis (FOC)   | d/q-axis (FOC)   |
| **Displayed in UI**     | Yes                  | Yes (Quick Info) | Yes (Quick Info) |
| **Affects V/f Control** | YES                  | NO               | NO               |
| **Affects FOC**         | YES (baseline)       | YES (future)     | YES (future)     |
| **Can be Changed**      | Yes                  | Yes              | Yes              |
| **Unit**                | H (Henries)          | H (Henries)      | H (Henries)      |

---

## Questions & Answers

### Q: Why set Ld/Lq if they don't do anything?

**A:** They're provided for future extensibility. When/if you implement FOC control, they'll be ready to use. Also, documenting your motor's d/q parameters is good practice.

### Q: Can I change Ld/Lq while simulating?

**A:** Yes, using the UI controls in the Parameters tab. But remember: **it won't affect current trapezoidal motor behavior**. Only `phase_inductance` affects motor dynamics.

### Q: Which should I adjust to change current rise time?

**A:** Adjust `phase_inductance` (L). Increasing L slows current rise; decreasing L speeds it up.

### Q: What if Ld ≠ Lq?

**A:** For non-salient (isotropic) BLDC motors, Ld ≈ Lq ≈ L. Differences would only matter if you implemented a salient-pole motor model.

---

## References

- BLDC Motor Model: `src/core/motor_model.py`
- Motor Parameters: `src/core/motor_model.py` (MotorParameters dataclass)
- UI Control: `src/ui/main_window.py` (\_create_parameters_tab, \_update_monitoring)
- FOC Controller (future): `src/control/foc_controller.py`
