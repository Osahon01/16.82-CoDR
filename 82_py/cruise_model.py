"""Simple Model for cruise. NOTE: May need to install package(s) using pip"""

import numpy as np
import matplotlib.pyplot as plt
from ambiance import Atmosphere
from pint import UnitRegistry
from takeoff_model import Takeoff_model

# NOTE: Once takeoff code is published, I will read in inputs and update lift model accordingly
# TODO: Biruk will add CD tradeoff study and conduct Cessna studies to calibrate a CD model (induced and parasitic)

ureg = UnitRegistry()


class CruiseModel:
    def __init__(
        self, s_ref, weight, v_cruise, h_cruise, aoa, aspect_ratio, e, thrust
    ) -> None:
        self.atm = Atmosphere(h=h_cruise)
        self.s_ref = s_ref
        self.weight = weight  # newtons
        self.v_cruise = v_cruise
        self.h_cruise = h_cruise
        self.density = self.atm.density
        self.aoa = aoa  # radians
        self.aspect_ratio = aspect_ratio
        self.e = e
        self.q = 0.5 * self.density * (self.v_cruise**2)
        self.thrust = thrust

    def cl(self):
        L = self.weight * np.cos(
            self.aoa
        )  # assumes I have weight as a function of time
        return L / (self.q * self.s_ref)

    def cd_induced(self):
        # see eqn: https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/induced.html
        return (self.cl() ** 2) / (np.pi * self.aspect_ratio * self.e)

    def cd_form(self):
        # Assume gamma=0, and T=D
        return self.thrust / (self.q * self.s_ref)

    def cd_total(self):
        # Current model will not account for viscous drag (not significant in cd calculations)
        return self.cd_induced() + self.cd_form()

    # TODO: Do we want cruise shaft power?


# Runner script
if __name__ == "__main__":
    # Design Variables
    aspect_ratio = np.linspace(..., ...)  # TODO: desired AR sweep range
    e = ...
    T_W_takeoff, W_S, W, P_shaft_TO, CLTO, CDTO, CD0 = ...
    takeoff_cls = Takeoff_model(
        T_W_takeoff, W_S, W, P_shaft_TO, CLTO, CDTO, CD0, AR=aspect_ratio, e=e
    )  # TODO: parameters need to be defined somewhere
    s_ref = takeoff_cls.S
    weight = takeoff_cls.weight  # TODO: Need this to be implemented in takeoff model; function of time or altitude

    # End of takeoff (eot) parameters; TODO: Need this to be implemented in takeoff model
    v_cruise = takeoff_cls.v_eot  # assumed constant
    h_in = takeoff_cls.h_eot  # NOTE: Ensure that term is in meters
    h_end = 10000 * ureg("ft").to_base_units().magnitude
    h_cruise = np.linspace(h_in, h_end) if h_end > h_in else np.array(h_in)
    thrust = takeoff_cls.get_T_cruise(v_cruise)

    # TODO: Optionally allow for variable gamma over trajectory, but will need to define variable as time or height -- will affect setup
    # ...
    gamma = 0.0  # (unit: radians) - Can add feature to vary aoa_cruise down the line
    cruise_cls = CruiseModel(
        s_ref,
        weight,
        v_cruise,
        h_cruise,
        aoa=gamma,
        aspect_ratio=aspect_ratio,
        e=e,
        thrust=thrust,
    )

    PLOT = False
    if PLOT:
        plt.figure()
        plt.plot(aspect_ratio, cruise_cls.cd_form(), color="red")
        plt.plot(aspect_ratio, cruise_cls.cd_induced(), color="blue")
        plt.plot(aspect_ratio, cruise_cls.cd_total(), color="black")
        plt.title("CD vs AR")
        plt.xlabel("AR")
        plt.ylabel("CD")
        plt.legend()
        plt.grid()
        plt.show()

    print(f"{50 * '='}\nBest AR: {np.round(min(cruise_cls.cd_total(), 2))}\n{50 * '='}")
