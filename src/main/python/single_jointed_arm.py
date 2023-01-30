from math_util import RK4
import numpy as np
from math import *

class single_jointed_arm(object):
    def __init__(self, length, mass, kt, kv):
        # Arm length
        self.l = length

        # Payload mass
        self.m = mass

        # Moment of inertia M
        self.M = mass * length * length
        
        # Gravity constant
        self.g = -9.81

        # System identification constants
        # 
        # TODO: implement system identification procedure for determining constants
        self.kt = kt
        self.kv = kv

    # Dynamics ẋ = f(x, u)
    # 
    # The state x being
    # 
    #   [θ]
    #   [ω]
    # 
    # The input u being the voltage applied to the motor
    # 
    #   [V]
    # 
    # Let us define θ̈ as the angular acceleration of the arm, M as the moment of inertia,
    # τᵤ as the motor's applied torque, and τ(θ) as the torque applied due to gravity. The 
    # dynamics of the system can be described as:
    # 
    #   θ̈ = M⁻¹(τᵤ + τ(θ))
    # 
    # The applied torque is a function of voltage V
    # 
    #   V = kᵥω + kₜτᵤ
    #   τᵤ = kₜ⁻¹(V - kᵥω)
    def dynamics(self, x, u):
        theta = x(0)
        omega = x(1)
        voltage = u(0)

        tau_u = (voltage - self.kv * omega) / self.kt
        tau_g = self.g * self.m * cos(theta)
        return np.array([
            omega,
            (tau_u + tau_g) / self.M
        ])