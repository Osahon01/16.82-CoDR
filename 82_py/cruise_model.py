"""Simple Model for cruise. NOTE: May need to install package(s) using pip"""

import numpy as np
import matplotlib.pyplot as plt
from ambiance import Atmosphere
from pint import UnitRegistry
from power_gen_usage import AircraftConfig, DEPSizingModel
from cruise_drag_model import parastic_drag, AR
from CoDR_equations import V_CRUISE

# NOTE: Once takeoff code is published, I will read in inputs and update lift model accordingly
# TODO: Biruk will add CD tradeoff study and conduct Cessna Caravan studies to calibrate a CD model (induced and parasitic)

ureg = UnitRegistry()


class CruiseModel:
    def __init__(self, s_ref, weight, v_cruise, h_cruise, AR, e, Cd0) -> None:
        self.atm = Atmosphere(h=h_cruise)
        self.s_ref = s_ref
        self.weight = weight  # newtons
        self.v_cruise = v_cruise
        self.h_cruise = h_cruise
        self.density = self.atm.density
        self.AR = AR
        self.e = e
        self.q = 0.5 * self.density * (self.v_cruise**2)
        # self.thrust = thrust
        self.Cd0 = Cd0

    def cl(self):
        L = self.weight  # assumes I have weight as a function of time
        return L / (self.q * self.s_ref)

    def cd_induced(self):
        # see eqn: https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/induced.html
        return (self.cl() ** 2) / (np.pi * self.AR * self.e)

    def cd_parasitic(self):
        return self.Cd0

    def cd_total(self):
        # Current model will not account for viscous drag (not significant in cd calculations)
        return self.cd_induced() + self.cd_parasitic()

    def drag_total(self):
        return self.cd_total() * self.q * self.s_ref


# Runner script
if __name__ == "__main__":
    # Design Variables
    e = 0.7  # TODO: determine if this value is a reasonable guess
    Cd0 = parastic_drag()

    # TODO: Update with actual values by calling relevant class
    cruise_cls = CruiseModel(
        s_ref=10,  # TODO: update to varied model
        weight=1000,  # TODO: update to varied model
        v_cruise=V_CRUISE,  # # TODO: update to varied model; # should sweep over an array of cruise values
        h_cruise=AircraftConfig.alt_cruise_m,
        AR=AR,
        e=e,
        Cd0=Cd0,
    )

    CD_total = cruise_cls.cd_total()
    L_over_D = cruise_cls.cl() / CD_total

    PLOT = True
    if PLOT:
        plt.figure()
        plt.plot(AR, L_over_D)
        plt.xlabel("Aspect Ratio")
        plt.ylabel("L/D")
        plt.title("Cruise L/D vs AR")
        plt.grid()
        plt.show()
