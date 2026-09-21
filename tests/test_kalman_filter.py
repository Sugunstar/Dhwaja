"""
Unit tests for Kalman filter module.
"""

import numpy as np
import sys
sys.path.insert(0, '../src')
from kalman_filter import IMUGPSKF

def test_kalman_filter():
    kf = IMUGPSKF()
    # Test initial state
    assert kf.x.shape == (6, 1)
    assert np.allclose(kf.x, 0)
    # Test predict step
    kf.predict(0.1, np.array([0.0, 0.0, 0.0]))
    # State should still be zero because no input and zero initial state
    assert np.allclose(kf.x, 0, atol=1e-6)
    # Test update with measurement
    z = np.array([1.0, 2.0, 3.0, 0.1, 0.2, 0.3]).reshape(6, 1)
    x_prev = kf.x.copy()
    kf.update(z)
    # After update, state should change
    assert not np.allclose(kf.x, x_prev, atol=1e-6)
    # And state should not be zero
    assert not np.allclose(kf.x, 0, atol=1e-6)
    print("Kalman filter test passed")

if __name__ == "__main__":
    test_kalman_filter()
    print("Kalman filter test passed!")
