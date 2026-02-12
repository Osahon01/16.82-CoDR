"""Simple Model for cruise. NOTE: May need to install package(s) using pip"""

import numpy as np
from ambiance import Atmosphere
from pint import UnitRegistry
from takeoff_model import Takeoff_model

# NOTE: Once takeoff code is published, I will read in inputs and update lift model accordingly
# TODO: Biruk will add CD tradeoff study and conduct Cessna studies to calibrate a CD model (induced and parasitic)

ureg = UnitRegistry()


class CruiseModel:
    def __init__(self, s_wet, weight, v_cruise, h_cruise, aoa) -> None:
        self.atm = Atmosphere(h=h_cruise)
        self.s_wet = s_wet
        self.weight = weight
        self.v_cruise = v_cruise
        self.h_cruise = h_cruise
        self.density = self.atm.density
        self.aoa = aoa

    def get_CL(self):
        q = 0.5 * self.density * (v_cruise**2)
        L = self.weight * np.cos(
            self.aoa
        )  # assumes I have weight as a function of time
        return L / (q * self.s_wet)

    # TODO: Do we want cruise shaft power?


# Runner script
if __name__ == "__main__":
    # Design Variables
    takeoff_cls = Takeoff_model(
        T_W_takeoff, W_S, W, P_shaft_TO, CLTO, CDTO, CD0, AR, e
    )  # TODO: parameters need to be defined somewhere
    s_wet = takeoff_cls.S
    weight = takeoff_cls.weight  # TODO: Need this to be implemented in takeoff model; function of time or altitude

    # End of takeoff (eot) parameters; TODO: Need this to be implemented in takeoff model
    v_cruise = takeoff_cls.v_eot  # assumed constant
    h_in = takeoff_cls.h_eot  # NOTE: Ensure that term is in meters
    h_end = 10000 * ureg("ft").to_base_units().magnitude
    h_cruise = np.linspace(h_in, h_end) if h_end > h_in else np.array(h_in)

    # TODO: Optionally allow for variable gamma over trajectory, but will need to define variable as time or height -- will affect setup
    # ...
    gamma = 0.0  # (unit: radians) - Can add feature to vary aoa_cruise down the line
    cruise_cls = CruiseModel(s_wet, weight, v_cruise, h_cruise, aoa=gamma)
