import numpy as np
from filterpy.kalman import KalmanFilter

def smoothing_factor(t_e, cutoff):
    r = 2 * np.pi * cutoff * t_e
    return r / (r + 1)


def exponential_smoothing(a, x, x_prev):
    return a * x + (1 - a) * x_prev


class OneEuroFilter:
    def __init__(self, t0, x0, dx0=0.0, min_cutoff=1.0, beta=0.0,
                 d_cutoff=1.0):
        """Initialize the one euro filter."""
        # The parameters.
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)
        # Previous values.
        self.x_prev = float(x0)
        self.dx_prev = float(dx0)
        self.t_prev = float(t0)

    def __call__(self, t, x):
        """Compute the filtered signal."""
        t_e = t - self.t_prev

        # The filtered derivative of the signal.
        a_d = smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = exponential_smoothing(a_d, dx, self.dx_prev)

        # The filtered signal.
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)
        a = smoothing_factor(t_e, cutoff)
        x_hat = exponential_smoothing(a, x, self.x_prev)

        # Memorize the previous values.
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t

        return x_hat


class KalmanWrapper:
    def __init__(self, dt=0.1):
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self.dt = dt
        self.kf.F = np.array([[1, 0, 0, dt, 0, 0],
                        [0, 1, 0, 0, dt, 0],
                        [0, 0, 1, 0, 0, dt],
                        [0, 0, 0, 1, 0, 0],
                        [0, 0, 0, 0, 1, 0],
                        [0, 0, 0, 0, 0, 1]])
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, 0]])
        self.kf.P *= 1000.0  # covariance matrix
        self.kf.R *= 0.1
        self.kf.Q *= 0.01

    def update(self, z_measured):
        self.kf.predict()
        print("Kalman filter prediction step completed.")
        self.kf.update(z_measured)
        print("Kalman filter update step completed with measurement:", z_measured)
        z_filtered = self.kf.x[:3].flatten()
        if np.linalg.norm(z_filtered) == 0:
            z_filtered = np.array([1.0, 0.0, 0.0])
            print("Warning! Kalman filter output is zero, setting to default [1.0, 0.0, 0.0]")
        else:
            z_filtered /= np.linalg.norm(z_filtered)
        return z_filtered