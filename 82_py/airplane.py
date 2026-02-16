# Imported libraries
import math
import numpy as np
from ambiance import Atmosphere

# Our models imported
from cruise_model import CruiseModel
from takeoff_model import TakeoffModel

# Vectors to sweep over
v_cruises = np.linspace(40, 120, 10)  # m/s
ARs = np.linspace(5, 15, 10)  # Aspect ratio sweep

# Design parameters (FIXED)
CLTO = 10
CDTO = 1
W = 19000 * 4.445  # N (converted from lbs)
S_W = 50  # kg/m^2
T_W = 0.3
h_cruise = 3048.0  # 10,000 ft in meters
gamma = math.radians(15.0)  # Climb angle in radians
eta_battery = 0.95
eta_generator = 0.92
eta_v_prop = 0.7
eta_add_prop = 0.7
epsilon_battery = 250.0 * 3600.0  # 250 Wh/kg converted to J/kg

# Takeoff Model Constants
aoa = 0
e = 0.8
CD0 = 0.0335 # Hard coded from cruise drag model for now, but will need to be part of a loop eventually

class Airplane:
    def __init__(
        self,
        v_cruise,
        AR
    ):
        self.v_cruise = v_cruise
        self.AR = AR
        self.S = W / S_W  # Wing area (m^2)
        self.T = W * T_W  # Takeoff thrust (N)

    def run_cruise_model(self):
        cruise = CruiseModel(
            h_cruise=h_cruise,
            s_ref=self.S,
            weight=self.W,
            v_cruise=self.v_cruise,
            h_cruise=h_cruise,
            aoa=aoa, 
            aspect_ratio=self.AR,
            e=e,
            thrust=None,
            Cd0=0.03
        )
        return cruise

    