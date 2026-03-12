"""
SVM Generator Module
====================

This module implements Space Vector Modulation (SVM) for 3-phase voltage generation.

SVM is used to generate modulated voltages to drive the motor with PWM signals.
It provides efficient use of DC bus voltage and reduces harmonic distortion.

**SVM Principle:**
The reference voltage vector is synthesized using the three active vectors
(corresponding to 6 switch configurations) and zero vectors.

:author: BLDC Control Team
:version: 1.0.0

.. versionadded:: 1.0.0
    Initial SVM implementation
"""

import numpy as np


class SVMGenerator:
    """
    Space Vector Modulation Generator
    ==================================

    Generates 3-phase PWM voltages using Space Vector Modulation technique.

    **Features:**
    - Efficient voltage utilization
    - Reduced harmonic distortion
    - Support for different DC bus voltages
    - Real-time modulation calculation

    **SVM Hexagon Sectors:**
    - Sector 1: 0° to 60°
    - Sector 2: 60° to 120°
    - Sector 3: 120° to 180°
    - Sector 4: 180° to 240°
    - Sector 5: 240° to 300°
    - Sector 6: 300° to 360°

    Example:
        >>> svm = SVMGenerator(dc_voltage=48.0)
        >>> voltages = svm.modulate(magnitude=20.0, angle=np.pi/6)
        >>> print(voltages)  # [v_a, v_b, v_c]
    """

    def __init__(
        self,
        dc_voltage: float = 48.0,
        device_drop_v: float = 0.0,
        dead_time_fraction: float = 0.0,
        conduction_resistance_ohm: float = 0.0,
        switching_frequency_hz: float = 0.0,
        switching_loss_coeff_v_per_a_khz: float = 0.0,
    ):
        """
        Initialize SVM generator.

        :param dc_voltage: DC bus voltage [V], defaults to 48.0
        :type dc_voltage: float

        :raises ValueError: If dc_voltage <= 0
        """
        if dc_voltage <= 0:
            raise ValueError("DC voltage must be positive")

        self.dc_voltage = dc_voltage
        self.device_drop_v = 0.0
        self.dead_time_fraction = 0.0
        self.conduction_resistance_ohm = 0.0
        self.switching_frequency_hz = 0.0
        self.switching_loss_coeff_v_per_a_khz = 0.0
        self.phase_currents = np.zeros(3, dtype=np.float64)
        self.set_nonidealities(
            device_drop_v=device_drop_v,
            dead_time_fraction=dead_time_fraction,
            conduction_resistance_ohm=conduction_resistance_ohm,
            switching_frequency_hz=switching_frequency_hz,
            switching_loss_coeff_v_per_a_khz=switching_loss_coeff_v_per_a_khz,
        )

        # Sector boundaries (in radians)
        self.sector_angles = np.array(
            [
                0,
                np.pi / 3,
                2 * np.pi / 3,
                np.pi,
                4 * np.pi / 3,
                5 * np.pi / 3,
                2 * np.pi,
            ]
        )

        # SVM vectors (unit vectors at 0°, 60°, 120°, 180°, 240°, 300°)
        angles = np.array(
            [0, np.pi / 3, 2 * np.pi / 3, np.pi, 4 * np.pi / 3, 5 * np.pi / 3]
        )
        self.vectors = np.array(
            [[np.cos(angle), np.sin(angle)] for angle in angles], dtype=np.float64
        )

    def modulate(self, magnitude: float, angle: float) -> np.ndarray:
        """
        Generate 3-phase SVM voltages.

        **Algorithm:**
        1. Normalize reference vector to magnitude and angle
        2. Identify sector based on angle
        3. Calculate dwell times for active and zero vectors
        4. Generate modulated voltages using timing

        :param magnitude: Reference voltage magnitude [V]
        :type magnitude: float
        :param angle: Reference voltage angle [radians, 0 to 2π]
        :type angle: float
        :return: 3-phase voltages [v_a, v_b, v_c] [V]
        :rtype: np.ndarray

        .. math::
            V_{ref} = M \\cdot e^{j\\theta}

        where:
            - M: magnitude (0 to Vdc/√3)
            - θ: angle (0 to 2π)

        **Return voltages:**
            - v_a: Phase A voltage [V]
            - v_b: Phase B voltage [V]
            - v_c: Phase C voltage [V]
        """
        # Clamp magnitude to maximum (2/3 * Vdc for linear mode)
        max_magnitude = (2.0 / 3.0) * self.dc_voltage
        magnitude = np.clip(magnitude, 0, max_magnitude)

        # Normalize angle to [0, 2π)
        angle = angle % (2 * np.pi)

        # Determine sector (1-6)
        sector = int(np.floor(angle / (np.pi / 3))) + 1
        sector = np.clip(sector, 1, 6)

        # Angle within sector [0, π/3)
        angle_in_sector = angle - (sector - 1) * (np.pi / 3)

        # Normalize angle to sector
        if angle_in_sector < 0:
            angle_in_sector += np.pi / 3
        if angle_in_sector >= np.pi / 3:
            angle_in_sector -= np.pi / 3

        # Calculate dwell times using time-averaged approach
        # T1*v1 + T2*v2 + T0*v0 = Vref
        # where T1 + T2 + T0 = Ts

        # Effective voltage using 2/√3 scaling
        v_ref_norm = magnitude * np.sqrt(3) / self.dc_voltage

        # Dwell time calculations (angle-based)
        # no unused intermediates stored

        t1 = v_ref_norm * np.sin(np.pi / 3 - angle_in_sector) / np.sin(np.pi / 3)
        t2 = v_ref_norm * np.sin(angle_in_sector) / np.sin(np.pi / 3)
        t0 = 1.0 - t1 - t2

        # Clamp to valid range
        t1 = np.clip(t1, 0, 1)
        t2 = np.clip(t2, 0, 1)

        # Generate modulated voltage (time-domain PWM)
        # Using standard SVM switching sequence
        voltages = self._generate_phase_voltages(sector, t1, t2, t0)
        voltages = self._apply_nonidealities(voltages)

        return voltages

    def _generate_phase_voltages(
        self, sector: int, t1: float, t2: float, t0: float
    ) -> np.ndarray:
        """
        Generate phase voltages from dwell times and sector.

        Maps SVM timing to 3-phase voltage outputs.

        :param sector: Current sector (1-6)
        :type sector: int
        :param t1: Dwell time for first vector
        :type t1: float
        :param t2: Dwell time for second vector
        :type t2: float
        :param t0: Dwell time for zero vector
        :type t0: float
        :return: Phase voltages [v_a, v_b, v_c]
        :rtype: np.ndarray
        """
        # SVM switching patterns for each sector
        # Format: [v_a, v_b, v_c] voltages expressed as fraction of Vdc

        sector_patterns = {
            1: {"v1": [1, 0, 0], "v2": [1, 1, 0]},  # Sector 1 (0-60°)
            2: {"v1": [1, 1, 0], "v2": [0, 1, 0]},  # Sector 2 (60-120°)
            3: {"v1": [0, 1, 0], "v2": [0, 1, 1]},  # Sector 3 (120-180°)
            4: {"v1": [0, 1, 1], "v2": [0, 0, 1]},  # Sector 4 (180-240°)
            5: {"v1": [0, 0, 1], "v2": [1, 0, 1]},  # Sector 5 (240-300°)
            6: {"v1": [1, 0, 1], "v2": [1, 0, 0]},  # Sector 6 (300-360°)
        }

        pattern = sector_patterns.get(sector, sector_patterns[1])

        # Time-weighted voltage combination
        # Normalize to [-Vdc/2, Vdc/2]
        v1 = np.array(pattern["v1"], dtype=np.float64)
        v2 = np.array(pattern["v2"], dtype=np.float64)

        # Average voltage (time-domain average)
        v_avg = t1 * v1 + t2 * v2 + (t0 / 2) * (np.ones(3))

        # Convert to actual voltages centered on Vdc/2
        voltages = (v_avg * self.dc_voltage) - (self.dc_voltage / 2)

        return voltages.astype(np.float64)

    def get_maximum_voltage(self) -> float:
        """
        Get maximum achievable output voltage magnitude.

        For linear SVM mode:

        .. math::
            V_{max} = \\frac{2}{3} V_{dc}

        :return: Maximum voltage [V]
        :rtype: float
        """
        return (2.0 / 3.0) * self.dc_voltage

    def set_nonidealities(
        self,
        device_drop_v: float = 0.0,
        dead_time_fraction: float = 0.0,
        conduction_resistance_ohm: float = 0.0,
        switching_frequency_hz: float = 0.0,
        switching_loss_coeff_v_per_a_khz: float = 0.0,
    ) -> None:
        """Configure simple inverter non-idealities.

        :param device_drop_v: Per-phase effective conduction voltage drop [V]
        :param dead_time_fraction: Duty loss ratio due to dead-time [0..0.2]
        :param conduction_resistance_ohm: Effective conduction path resistance [ohm]
        :param switching_frequency_hz: Inverter switching frequency [Hz]
        :param switching_loss_coeff_v_per_a_khz:
            Voltage-loss coefficient per ampere and kHz [V/A/kHz]
        """
        if device_drop_v < 0:
            raise ValueError("device_drop_v must be non-negative")
        if dead_time_fraction < 0 or dead_time_fraction > 0.2:
            raise ValueError("dead_time_fraction must be between 0 and 0.2")
        if conduction_resistance_ohm < 0:
            raise ValueError("conduction_resistance_ohm must be non-negative")
        if switching_frequency_hz < 0:
            raise ValueError("switching_frequency_hz must be non-negative")
        if switching_loss_coeff_v_per_a_khz < 0:
            raise ValueError("switching_loss_coeff_v_per_a_khz must be non-negative")

        self.device_drop_v = float(device_drop_v)
        self.dead_time_fraction = float(dead_time_fraction)
        self.conduction_resistance_ohm = float(conduction_resistance_ohm)
        self.switching_frequency_hz = float(switching_frequency_hz)
        self.switching_loss_coeff_v_per_a_khz = float(switching_loss_coeff_v_per_a_khz)

    def set_phase_currents(self, phase_currents: np.ndarray) -> None:
        """Provide phase currents used for current-dependent voltage drops."""
        arr = np.asarray(phase_currents, dtype=np.float64)
        if arr.shape != (3,):
            raise ValueError("phase_currents must be a 3-element array")
        self.phase_currents = arr

    def _apply_nonidealities(self, voltages: np.ndarray) -> np.ndarray:
        """Apply lightweight inverter non-ideality effects to phase voltages."""
        out = np.array(voltages, dtype=np.float64, copy=True)
        if (
            self.device_drop_v > 0.0
            or self.conduction_resistance_ohm > 0.0
            or self.switching_loss_coeff_v_per_a_khz > 0.0
        ):
            total_drop = np.full(3, self.device_drop_v, dtype=np.float64)
            if self.conduction_resistance_ohm > 0.0:
                total_drop += self.conduction_resistance_ohm * np.abs(
                    self.phase_currents
                )
            if (
                self.switching_loss_coeff_v_per_a_khz > 0.0
                and self.switching_frequency_hz > 0.0
            ):
                f_khz = self.switching_frequency_hz / 1000.0
                total_drop += (
                    self.switching_loss_coeff_v_per_a_khz
                    * np.abs(self.phase_currents)
                    * f_khz
                )

            signs = np.sign(out)
            out = np.where(
                np.abs(out) > total_drop,
                out - signs * total_drop,
                0.0,
            )

        if self.dead_time_fraction > 0.0:
            out *= 1.0 - self.dead_time_fraction

        half_bus = self.dc_voltage / 2.0
        return np.clip(out, -half_bus, half_bus)

    def set_dc_voltage(self, dc_voltage: float) -> None:
        """
        Update DC bus voltage.

        :param dc_voltage: New DC voltage [V]
        :type dc_voltage: float

        :raises ValueError: If dc_voltage <= 0
        """
        if dc_voltage <= 0:
            raise ValueError("DC voltage must be positive")
        self.dc_voltage = dc_voltage


class CartesianSVMGenerator(SVMGenerator):
    """
    Cartesian SVM Generator
    =======================

    Extended SVM generator accepting Cartesian coordinates (Valpha, Vbeta)
    instead of polar form (magnitude, angle).

    Useful for direct FOC control outputs.

    Example:
        >>> svm = CartesianSVMGenerator(dc_voltage=48.0)
        >>> voltages = svm.modulate_cartesian(valpha=15.0, vbeta=8.66)
    """

    def modulate_cartesian(self, valpha: float, vbeta: float) -> np.ndarray:
        """
        Generate SVM voltages from Cartesian coordinates.

        **Clarke Transform inverse:**

        .. math::
            V_{\\alpha} = V_a - \\frac{1}{2}(V_b + V_c)

        .. math::
            V_{\\beta} = \\frac{\\sqrt{3}}{2}(V_b - V_c)

        :param valpha: Alpha-axis voltage (direct axis) [V]
        :type valpha: float
        :param vbeta: Beta-axis voltage (quadrature axis) [V]
        :type vbeta: float
        :return: 3-phase voltages [v_a, v_b, v_c] [V]
        :rtype: np.ndarray
        """
        # Convert Cartesian to polar
        magnitude = np.sqrt(valpha**2 + vbeta**2)
        angle = np.arctan2(vbeta, valpha)

        # Use standard SVM
        return self.modulate(magnitude, angle)

    def cartesian_to_threephase(self, valpha: float, vbeta: float) -> np.ndarray:
        """
        Convert Cartesian (α-β) to 3-phase voltages directly.

        Direct inverse Clarke transform:

        .. math::
            V_a = V_{\\alpha}

        .. math::
            V_b = -\\frac{1}{2}V_{\\alpha} + \\frac{\\sqrt{3}}{2}V_{\\beta}

        .. math::
            V_c = -\\frac{1}{2}V_{\\alpha} - \\frac{\\sqrt{3}}{2}V_{\\beta}

        :param valpha: Alpha-axis voltage [V]
        :type valpha: float
        :param vbeta: Beta-axis voltage [V]
        :type vbeta: float
        :return: 3-phase voltages [v_a, v_b, v_c] [V]
        :rtype: np.ndarray
        """
        sqrt3_2 = np.sqrt(3) / 2.0

        v_a = valpha
        v_b = -0.5 * valpha + sqrt3_2 * vbeta
        v_c = -0.5 * valpha - sqrt3_2 * vbeta

        return np.array([v_a, v_b, v_c], dtype=np.float64)
