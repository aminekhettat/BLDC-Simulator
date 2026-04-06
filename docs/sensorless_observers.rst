Sensorless Angle Observers
==========================

.. contents:: Table of Contents
   :local:
   :depth: 2

Overview
--------

SPINOTOR implements five angle observer modes, selectable from the
**Observer & Startup** tab in the GUI.  All sensorless modes reconstruct the
electrical angle from terminal voltages and currents without requiring a
physical rotor position sensor.

The observer is selected at each simulation start.  Context-sensitive parameter
widgets appear automatically depending on the active mode.

.. list-table:: Available observer modes
   :header-rows: 1
   :widths: 20 60 20

   * - Mode
     - Description
     - Best suited for
   * - Measured
     - True angle from the simulation model (reference mode)
     - Controller tuning / debugging
   * - PLL
     - Back-EMF Phase-Locked Loop
     - SPM motors, simple bring-up
   * - SMO
     - Sliding-Mode Observer (first order)
     - Disturbance-robust operation
   * - STSMO
     - Super-Twisting SMO (backward-Euler, SOGI post-filter)
     - **SPM motors (Ld≈Lq) only** — chattering-free, wide speed range
   * - ActiveFlux
     - Active Flux observer (Boldea 2009)
     - IPM motors with d/q saliency

Measured — Reference Mode
--------------------------

Uses the true rotor angle directly from the simulation model.  No estimation
or filtering is applied.

This mode does **not** represent sensorless operation; it is intended as a
reference baseline for current-loop and speed-loop tuning before switching to
one of the estimation-based modes.

**GUI parameters:** none.

**API:**

.. code-block:: python

   ctrl.set_angle_observer("Measured")

PLL — Phase-Locked Loop
------------------------

Reconstructs the back-EMF vector in the stationary (α–β) frame from the
voltage model:

.. math::

   \hat{e}_\alpha = v_\alpha - R \hat{i}_\alpha - L \frac{d\hat{i}_\alpha}{dt}, \quad
   \hat{e}_\beta  = v_\beta  - R \hat{i}_\beta  - L \frac{d\hat{i}_\beta}{dt}

A second-order PLL then tracks the angle of the reconstructed EMF vector,
yielding the estimated rotor angle :math:`\hat{\theta}_e` and electrical speed
:math:`\hat{\omega}_e`.

**Advantages:** simple two-parameter structure; well understood; fast bring-up.

**Limitations:** gain tuning is speed-dependent; susceptible to integrator drift
at very low speeds where back-EMF magnitude is small.

**GUI parameters:**

- **PLL Kp** — proportional gain of the angle tracking loop (typical 50–150).
- **PLL Ki** — integral gain (typical 1000–5000).

**API:**

.. code-block:: python

   ctrl.set_angle_observer("PLL")
   ctrl.pll_kp = 100.0
   ctrl.pll_ki = 2000.0

SMO — Sliding-Mode Observer
-----------------------------

The SMO adds a switching (sign-action) error correction term on top of the
voltage model used by the PLL.  The sign action drives the estimated current
error to zero on a sliding surface, making the observer robust against
parameter mismatches and supply-voltage disturbances.

An adjustable boundary layer softens the sign action to reduce chattering,
and a first-order LPF smooths the estimated EMF before angle extraction.

**Advantages:** better disturbance rejection than PLL; does not require fine
gain scheduling.

**Limitations:** residual chattering from the switching term; LPF introduces
a phase lag that must be compensated at high speed.

**GUI parameters:**

- **SMO Kslide** — sliding-surface gain (typical 300–1000).
- **SMO LPF Alpha** — LPF coefficient for EMF smoothing; lower = more
  filtering, more phase lag (typical 0.05–0.2).
- **SMO Boundary** — width of the boundary layer [rad] replacing hard sign
  action with a linear region (typical 0.03–0.12).

**API:**

.. code-block:: python

   ctrl.set_angle_observer("SMO")
   ctrl.smo["k_slide"]    = 500.0
   ctrl.smo["lpf_alpha"]  = 0.1
   ctrl.smo["boundary"]   = 0.05

STSMO — Super-Twisting SMO
----------------------------

The Super-Twisting algorithm (Levant 1993) is a second-order sliding-mode
technique that achieves finite-time convergence while keeping the control
signal continuous — eliminating the high-frequency chattering of first-order
SMO designs.

Mathematical Formulation
^^^^^^^^^^^^^^^^^^^^^^^^

The current estimator tracks the α-axis (mirrored for β):

.. math::

   \dot{\hat{i}}_\alpha = \frac{1}{L}\left(v_\alpha - R\hat{i}_\alpha - k_1 |e_\alpha|^{1/2} \operatorname{sign}(e_\alpha) + z_1\right)

.. math::

   \dot{z}_1 = -k_2 \operatorname{sign}(e_\alpha), \quad e_\alpha = \hat{i}_\alpha - i_\alpha

On the sliding surface :math:`e=0`, the auxiliary state :math:`z_1` converges
to the true back-EMF component.  Angle is then extracted as:

.. math::

   \hat{\theta}_e = \operatorname{arctan2}(\hat{e}_\beta,\, \hat{e}_\alpha)

Backward-Euler Integration
^^^^^^^^^^^^^^^^^^^^^^^^^^^

The simulator uses a fully implicit backward-Euler discretisation for both
the current estimator and the :math:`z_1` integrator.  The implicit formulation
lifts the forward-Euler stability constraint
:math:`k_1 < L/(\sqrt{2}\,dt) \approx 1.06\ \text{A/V}` entirely, allowing
large gains chosen solely for convergence speed without any upper stability
bound.

Speed-Adaptive k2
^^^^^^^^^^^^^^^^^^

At runtime, :math:`k_2` is not constant.  It scales with the estimated
mechanical and electrical speeds to satisfy the Levant rotating-EMF tracking
condition :math:`k_2 \geq k_e \,\omega_m \,\omega_e`:

.. math::

   k_{2,\text{eff}} = \max\!\bigl(k_{2,\text{min}},\; k_{2,\text{factor}} \cdot k_e \cdot \hat{\omega}_m \cdot \hat{\omega}_e\bigr)

A floor :math:`k_{2,\text{min}}` (default 500 V/s) prevents the gain from
collapsing to zero at standstill.

SOGI Post-Filter
^^^^^^^^^^^^^^^^^

After convergence, the :math:`z_1` components are fed through a Second-Order
Generalized Integrator (SOGI) resonant filter tuned at the estimated electrical
frequency.  The SOGI amplifies the fundamental EMF component and attenuates
chattering side-bands, yielding a clean angle estimate.  The SOGI is bypassed
below 5 rad/s (near standstill) where it cannot lock.

Analytical Gain Calibration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Gains are computed from the Levant sufficient conditions:

.. math::

   k_1 = \lambda \sqrt{E_{\max}}, \quad k_2 = \frac{\lambda^2 E_{\max}}{2}

where :math:`E_{\max} = k_e \,\omega_{m,\text{rated}}` is the rated back-EMF
amplitude and :math:`\lambda` is the convergence factor (default 3.0).

.. code-block:: python

   ctrl.enable_sensorless_emf_reconstruction()
   result = ctrl.calibrate_stsmo_gains_analytical(rated_rpm=3500.0)
   # result: {"k1": ..., "k2": ..., "e_max_v": ...}

**GUI parameters:**

- **k1** — super-twisting proportional gain (default 18, set by auto-calibrate).
- **k2_min** — minimum k2 floor [V/s] at standstill (default 500).
- **k2_factor** — speed-adaptive k2 multiplier; 1.0 = Levant theoretical
  minimum (default 1.0).
- **Rated RPM** — rated mechanical speed used for analytical calibration.
- **Auto-Calibrate** button — runs ``calibrate_stsmo_gains_analytical()``
  and populates k1, k2_min, k2_factor from motor parameters.
- **Integration Solver** — selects Backward Euler (default, unconditionally
  stable) or Forward Euler (lower-accuracy, stability-constrained).

**API:**

.. code-block:: python

   ctrl.enable_sensorless_emf_reconstruction()
   ctrl.calibrate_stsmo_gains_analytical(rated_rpm=3500.0)
   ctrl.stsmo["k1"]       = 18.0
   ctrl.stsmo["k2_min"]   = 500.0
   ctrl.stsmo["k2_factor"]= 1.0
   ctrl.set_angle_observer("PLL")   # PLL extracts angle from STSMO z1 output

Optional MRAS Resistance Correction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A Model Reference Adaptive System (MRAS) can be enabled to track slow stator
resistance drift (temperature-related):

.. code-block:: python

   ctrl.enable_mras_resistance()

When active, :math:`R` in the estimator is updated online from the real–model
current divergence.

Motor Compatibility — SPM Only
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. warning::

   The STSMO uses a **single-inductance model** (:math:`L = L_d`).  This
   assumption is exact for SPM motors (:math:`L_d = L_q`) but **invalid for
   salient IPMSM** (:math:`L_d \neq L_q`).

   On an IPM motor with strong saliency (e.g. :math:`L_q/L_d = 2`), the
   missing saliency term :math:`(L_q - L_d)\,i_d\,\omega_e` introduces a
   systematic bias on the sliding surface :math:`\sigma`.  In field-weakening
   (where :math:`i_d \ll 0`) this bias grows unboundedly, causing :math:`z_1`
   drift and eventual angle divergence.

**Recommended alternatives for salient IPMSM:**

- **ActiveFlux** (Section above) — the active-flux transformation
  :math:`\vec\psi_a = \vec\psi_s - L_d\,\vec i_s` is saliency-immune by
  construction and is the preferred observer for IPM drives and
  field-weakening studies.
- **EEMF-STSMO** (Wang et al. 2022, reference [11]) — extends the
  Super-Twisting algorithm with the :math:`L_q \cdot di/dt` correction term
  used in the EEMF model, making arctan2 extraction valid for
  :math:`L_d \neq L_q`.  This variant is not yet implemented in SPINOTOR.

ActiveFlux — Active Flux Observer
-----------------------------------

The Active Flux observer (Boldea et al. 2009) is based on the observation that
the **active flux vector**:

.. math::

   \vec{\psi}_a = \vec{\psi}_s - L_d \vec{i}_s

is always aligned with the rotor d-axis, regardless of field-weakening
:math:`i_d` injection or d/q saliency (:math:`L_d \neq L_q`).  Rotor angle is
therefore:

.. math::

   \hat{\theta}_e = \operatorname{arctan2}(\psi_{a\beta},\; \psi_{a\alpha})

Stator Flux Integration
^^^^^^^^^^^^^^^^^^^^^^^^

The stator flux :math:`\vec{\psi}_s` is obtained by integrating the back-EMF:

.. math::

   \frac{d\vec{\psi}_s}{dt} = \vec{v}_s - R\,\vec{i}_s

A drift-correcting leaky term with cutoff frequency :math:`\omega_c = 2\pi f_c`
prevents integrator offset accumulation:

.. math::

   \psi_s[k+1] = (1 - \omega_c\,dt)\,\psi_s[k] + dt\,(v_s - R\,i_s)

The cutoff frequency should be well below the minimum electrical frequency
(typically :math:`f_c < 1` Hz).

Speed Estimation
^^^^^^^^^^^^^^^^^

Electrical speed is estimated from the cross-product of consecutive active flux
samples, low-pass filtered with a coefficient of 0.9:

.. math::

   \hat{\omega}_e = \frac{\dot{\psi}_{a\beta}\,\psi_{a\alpha} - \dot{\psi}_{a\alpha}\,\psi_{a\beta}}{|\vec{\psi}_a|^2}

**Advantages:** angle estimate remains valid during field weakening; naturally
suited to IPM motors with strong saliency.

**Limitations:** performance depends on accurate :math:`L_d` and :math:`R`
values; integrator quality degrades near zero speed.

**GUI parameters:**

- **AF dc_cutoff_hz** — drift-correction pole frequency [Hz] (default 0.5 Hz).

**API:**

.. code-block:: python

   ctrl.enable_active_flux_observer(dc_cutoff_hz=0.5)

Tuning Workflow
----------------

The recommended bring-up sequence for sensorless operation:

1. **Start with Measured** — verify d/q current loops and speed loop are
   well-tuned with the true angle.
2. **Switch to PLL** — tune Kp to remove phase lag, then Ki to eliminate
   steady-state angle error at rated speed.
3. **Switch to SMO** if load-step robustness is needed; tune Kslide, then LPF
   Alpha, then Boundary.
4. **Switch to STSMO** if chattering is still visible or if wider speed range
   is required.  Use Auto-Calibrate first, then adjust k2_min for low-speed
   margin.
5. **Switch to ActiveFlux** for IPM motors or field-weakening studies where
   saliency effects matter.

In all cases:

- Enable the **sensorless blend** for smooth low-speed handoff
  (``FOCController.set_sensorless_blend()``).
- Avoid tuning inverter non-idealities until the observer is stable.
- Re-freeze regression baselines after any observer mode change.

Low-Speed Robustness — Sensorless Blend
-----------------------------------------

All sensorless modes (PLL, SMO, STSMO, ActiveFlux) participate in a
confidence-and-speed weighted blend with the measured angle at low speed:

- When back-EMF magnitude is small or speed is below a configurable threshold,
  the angle blend weight tilts toward the measured angle (open-loop reference).
- As speed and EMF confidence grow, the weight shifts toward the full observer
  angle.
- This prevents abrupt angle jumps during weak back-EMF conditions and enables
  clean open-loop → closed-loop handoff.

**Configuration:**

.. code-block:: python

   ctrl.set_sensorless_blend(
       enabled=True,
       min_speed_rpm=50.0,
       min_confidence=0.3,
   )

**Diagnostics exposed in controller state:**

- ``sensorless_blend_weight`` — current blend factor (0 = fully measured,
  1 = fully observer).
- ``theta_sensorless_raw`` — raw observer angle before blending.

References
----------

- Levant, A. (1993). Sliding order and sliding accuracy in sliding mode control.
  *International Journal of Control*, 58(6), 1247–1263.
- Boldea, I., Paicu, M.C., Andreescu, G.D., & Blaabjerg, F. (2009).
  "Active Flux" DTFC-SVM sensorless control of IPMSM.
  *IEEE Transactions on Energy Conversion*, 24(2), 314–322.
- Holtz, J. (2002). Sensorless control of induction motor drives.
  *Proceedings of the IEEE*, 90(8), 1359–1394.
- Chen, Z. et al. (2003). A novel sensorless control scheme with an adaptive
  observer. *IEEE Transactions on Industrial Electronics*, 50(4), 861–869.
