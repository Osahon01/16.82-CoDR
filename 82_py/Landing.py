import math
import numpy as np


"""
Model for predicting landing runway needed

Inputs
------

W : Weight of the plane WITHOUT fuel
C_D : Drag coefficient
C_L : Lift coefficient
mu : Coefficient of friction when braking
S : Reference area, wing area
T : Forward Thrust (+) or Reverse Thrust (-) value during landing
V : Velocity at touchdown

"""
class LandingDistance:
    def __init__(
            self,
            W,
            C_D,
            C_L,
            mu,
            S,
            T,
            V,
    ):

        """
        Initializing these values just in case this code gets expanded on later in the project

        """
        self.W = W
        self.C_D = C_D
        self.C_L = C_L
        self.mu = mu
        self.S = S
        self.T = T
        self.V = V

        self.g = 9.80665
        self.rho = 1.225

    def landing_distance(self):

        return self.W/(self.g*self.rho*self.S*(self.C_D-self.mu*self.C_L))*np.log(1-(self.rho*self.S*self.V**2*(self.C_D-self.mu*self.C_L))/(2*(self.T-self.mu*self.W)))

"""
Example case

if __name__ == "__main__":
    model = LandingDistance(W=34000, C_D=0.3, C_L=0.9, mu=0.4, S=23, T=2300, V=40)

    print(model.landing_distance())
"""
