"""
CEP (Circular Error Probable) analysis for projectile impact points.
Provides functions to compute empirical CEP50, R90, R95 radii and bias.
All calculations are based on Euclidean distance in the horizontal plane (east, north).
"""

import numpy as np


def _radial_errors(points, center):
    """
    Compute radial distances from a center point.
    Parameters
    ----------
    points : np.ndarray of shape (N, 2)
        Impact points [east, north] in meters.
    center : array-like of length 2
        Reference point [east, north] in meters.
    Returns
    -------
    np.ndarray of shape (N,)
        Radial distances in meters.
    """
    points = np.asarray(points)
    center = np.asarray(center)
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError("points must be (N, 2) array")
    if center.shape != (2,):
        raise ValueError("center must be length-2 array")
    diff = points - center
    return np.sqrt(np.sum(diff**2, axis=1))


def compute_cep_radius(points, center, percentile):
    """
    Compute empirical radius containing given percentile of impacts.
    Parameters
    ----------
    points : np.ndarray (N, 2)
        Impact points.
    center : array-like length 2
        Center about which error is measured.
    percentile : float
        Desired proportion (e.g., 0.5 for CEP50, 0.9 for R90).
    Returns
    -------
    float
        Radius in meters containing the specified percentile.
    """
    if not 0.0 < percentile <= 1.0:
        raise ValueError("percentile must be in (0, 1]")
    radii = _radial_errors(points, center)
    if radii.size == 0:
        return 0.0
    radii_sorted = np.sort(radii)
    # Nearest-rank definition: index = ceil(p * N) - 1 (0-based)
    k = int(np.ceil(percentile * radii.size)) - 1
    k = max(0, min(k, radii.size - 1))
    return float(radii_sorted[k])


def compute_cep50_r90_r95(points, center):
    """
    Compute CEP50, R90, R95 radii for given points and center.
    Returns dict with keys 'CEP50', 'R90', 'R95'.
    """
    return {
        'CEP50': compute_cep_radius(points, center, 0.5),
        'R90':   compute_cep_radius(points, center, 0.9),
        'R95':   compute_cep_radius(points, center, 0.95)
    }


def compute_bias(points, aimpoint):
    """
    Compute bias vector as mean impact minus aimpoint.
    Parameters
    ----------
    points : np.ndarray (N, 2)
        Impact points.
    aimpoint : array-like length 2
        Intended impact point.
    Returns
    -------
    np.ndarray length 2
        Bias vector (east, north) in meters.
    """
    points = np.asarray(points)
    aimpoint = np.asarray(aimpoint)
    if points.size == 0:
        return np.zeros(2)
    mean_impact = np.mean(points, axis=0)
    return mean_impact - aimpoint


def compute_precision_metrics(points):
    """
    Compute precision-centered error metrics (CEP based on mean impact point).
    Returns dict with bias, CEP50, R90, R95 where radii are centered on mean impact.
    """
    # bias relative to aimpoint will be computed elsewhere; here we assume aimpoint = (0,0) for convenience
    # but we can keep generic: compute radii around mean impact.
    # We'll compute radii around mean impact (precision).
    mean_point = np.mean(points, axis=0) if points.size > 0 else np.zeros(2)
    radii = compute_cep50_r90_r95(points, mean_point)
    bias_vec = np.mean(points, axis=0) - mean_point  # will be zero by definition
    # Actually bias relative to mean is zero; we will compute bias relative to aimpoint separately.
    return {
        'mean_impact': mean_point,
        'CEP50_precision': radii['CEP50'],
        'R90_precision': radii['R90'],
        'R95_precision': radii['R95']
    }


def compute_total_error_metrics(points, aimpoint):
    """
    Compute total error metrics (centered on aimpoint).
    Includes bias and radii centered on aimpoint.
    """
    bias = compute_bias(points, aimpoint)
    radii = compute_cep50_r90_r95(points, aimpoint)
    return {
        'aimpoint': aimpoint,
        'bias_east': bias[0],
        'bias_north': bias[1],
        'bias_magnitude': np.linalg.norm(bias),
        'CEP50_total': radii['CEP50'],
        'R90_total': radii['R90'],
        'R95_total': radii['R95']
    }