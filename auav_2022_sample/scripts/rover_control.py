import numpy as np
import scipy.linalg

"""
This module impelments an SE2 based rover controler.
"""

def wrap(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


class SE2:
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def to_matrix(self):
        x = self.x
        y = self.y
        theta = self.theta
        cos = np.cos
        sin = np.sin
        return np.array([
            [cos(theta), -sin(theta), x],
            [sin(theta), cos(theta), y],
            [0, 0, 1]])

    @classmethod
    def from_matrix(cls, m):
        theta = np.arctan2(m[1, 0], m[0, 0])
        x = m[0, 2]
        y = m[1, 2]
        return cls(x=x, y=y, theta=theta)

    def __matmul__(self, other):
        return SE2.from_matrix(self.to_matrix()@other.to_matrix())

    def __repr__(self):
        return 'x {:g}: y: {:g} theta: {:g}'.format(self.x, self.y, self.theta)
    
    def log(self):
        # TODO: this is slow, could use analytical version
        return se2.from_matrix(scipy.linalg.logm(self.to_matrix()))
    
    def inv(self):
        # TODO: this is slow, could use analytical version
        return self.from_matrix(np.linalg.inv(self.to_matrix()))


class se2:
    
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    
    def to_matrix(self):
        x = self.x
        y = self.y
        theta = self.theta
        return np.array([
            [0, -theta, x],
            [theta, 0, y],
            [0, 0, 0]])

    @classmethod
    def from_matrix(cls, m):
        x = m[0, 2]
        y = m[1, 2]
        theta = m[1, 0]
        return cls(x=x, y=y, theta=theta)

    def __repr__(self):
        return 'x {:g}: y: {:g} theta: {:g}'.format(self.x, self.y, self.theta)
    
    def exp(self):
        # TODO this is slow, could use analytical version
        return SE2.from_matrix(scipy.linalg.expm(self.to_matrix()))


def compute_control(t, x, y, theta, ref_data):
    ref_x = ref_data['x']
    ref_y = ref_data['y']
    ref_theta = ref_data['theta']
    ref_omega = ref_data['omega']
    ref_V = ref_data['V']

    r_x = float(ref_x(t))
    r_y = float(ref_y(t))
    r_omega = float(ref_omega(t))
    r_theta = float(ref_theta(t))
    r_V = float(ref_V(t))

    X_r = SE2(r_x, r_y, r_theta)
    X = SE2(x, y, theta)
    eta = X_r.inv()@X
    chi = eta.log()

    # control parameters
    K_x = 1.0  # makes vehicle speed up to elim. along track error
    K_y_to_theta = 2.0  # makes vehicle turn to elim. cross track error
    K_theta = 4.0  # makes vehicle rotate faster for given theta error
    omega_max = np.deg2rad(180)  # deg/s max rotation rate saturation
    y_to_theta_max = np.deg2rad(90)  # y to theta_max saturation, 90 deg, perpin. to track

    v = r_V - K_x*np.real(chi.x)
    if v < 0:
        v = 0
    if v > 1:
        v = 1

    y_to_theta = K_y_to_theta*np.real(chi.y)
    if np.abs(y_to_theta > y_to_theta_max):
        y_to_theta = np.sign(y_to_theta)*y_to_theta_max

    omega = r_omega - K_theta*(np.real(chi.theta) + y_to_theta)

    if abs(omega) > omega_max:
        omega = np.sign(omega)*omega_max

    return v, omega
