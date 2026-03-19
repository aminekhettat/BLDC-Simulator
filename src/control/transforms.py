"""
Coordinate Transformations for Motor Control
===========================================

Provides Clark, Park, and Concordia transforms used in FOC algorithms.
Both forward (3-phase to two-axis) and inverse transforms are supported.

:author: BLDC Control Team
:version: 0.8.0

.. versionadded:: 0.8.0
    Initial implementation of coordinate transforms
"""

import numpy as np
from typing import Tuple


def clarke_transform(va: float, vb: float, vc: float) -> Tuple[float, float]:
    """
    Clarke transform (three-phase to stationary two-axis alpha-beta)
    assuming va + vb + vc = 0.

    :param va: Phase A voltage/current
    :param vb: Phase B voltage/current
    :param vc: Phase C voltage/current
    :return: (v_alpha, v_beta)
    """
    v_alpha = va
    v_beta = (vb - vc) / np.sqrt(3)
    return v_alpha, v_beta


def inverse_clarke(v_alpha: float, v_beta: float) -> Tuple[float, float, float]:
    """
    Inverse Clarke transform (alpha-beta to three-phase)

    :param v_alpha: Alpha-axis quantity
    :param v_beta: Beta-axis quantity
    :return: (va, vb, vc)
    """
    va = v_alpha
    vb = -0.5 * v_alpha + (np.sqrt(3) / 2.0) * v_beta
    vc = -0.5 * v_alpha - (np.sqrt(3) / 2.0) * v_beta
    return va, vb, vc


def park_transform(v_alpha: float, v_beta: float, theta: float) -> Tuple[float, float]:
    """
    Park transform (alpha-beta to rotating d-q frame).

    :param v_alpha: Alpha-axis quantity
    :param v_beta: Beta-axis quantity
    :param theta: Rotor electrical angle [rad]
    :return: (v_d, v_q)
    """
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    v_d = v_alpha * cos_t + v_beta * sin_t
    v_q = -v_alpha * sin_t + v_beta * cos_t
    return v_d, v_q


def inverse_park(v_d: float, v_q: float, theta: float) -> Tuple[float, float]:
    """
    Inverse Park transform (d-q to alpha-beta frame).

    :param v_d: d-axis quantity
    :param v_q: q-axis quantity
    :param theta: Rotor electrical angle [rad]
    :return: (v_alpha, v_beta)
    """
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    v_alpha = v_d * cos_t - v_q * sin_t
    v_beta = v_d * sin_t + v_q * cos_t
    return v_alpha, v_beta


def concordia_transform(va: float, vb: float, vc: float) -> Tuple[float, float]:
    """
    Concordia (also called symmetrical component) transform which is
    formally equivalent to Clark but keeps factor of 2/3 scaling.  Useful
    when zero-sequence components are important.

    :param va: Phase A quantity
    :param vb: Phase B quantity
    :param vc: Phase C quantity
    :return: (alpha, beta) in same units
    """
    alpha = (2.0 / 3.0) * (va - 0.5 * vb - 0.5 * vc)
    beta = (2.0 / 3.0) * ((np.sqrt(3) / 2.0) * (vb - vc))
    return alpha, beta


def inverse_concordia(alpha: float, beta: float) -> Tuple[float, float, float]:
    """
    Inverse Concordia transform (alpha-beta back to three-phase).

    :param alpha: alpha-axis quantity
    :param beta: beta-axis quantity
    :return: (va, vb, vc)
    """
    va = alpha
    vb = -0.5 * alpha + (np.sqrt(3) / 2.0) * beta
    vc = -0.5 * alpha - (np.sqrt(3) / 2.0) * beta
    return va, vb, vc
