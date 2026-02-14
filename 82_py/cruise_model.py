"""Simple Model for cruise. NOTE: May need to install package(s) using pip"""

import numpy as np
import matplotlib.pyplot as plt
from ambiance import Atmosphere
from pint import UnitRegistry
from takeoff_model import TakeoffModel
from cruise_drag_model import parastic_drag

# NOTE: Once takeoff code is published, I will read in inputs and update lift model accordingly
# TODO: Biruk will add CD tradeoff study and conduct Cessna Caravan studies to calibrate a CD model (induced and parasitic)

ureg = UnitRegistry()


class CruiseModel:
    def __init__(self, s_ref, weight, v_cruise, h_cruise, AR, e, thrust, Cd0) -> None:
        self.atm = Atmosphere(h=h_cruise)
        self.s_ref = s_ref
        self.weight = weight  # newtons
        self.v_cruise = v_cruise
        self.h_cruise = h_cruise
        self.density = self.atm.density
        self.AR = AR
        self.e = e
        self.q = 0.5 * self.density * (self.v_cruise**2)
        self.thrust = thrust
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

    # TODO: Do we want cruise shaft power?


# Runner script
if __name__ == "__main__":
    # Design Variables
    AR = np.linspace(5, 15, 100)  # TODO: desired AR sweep range
    e = 0.7  # TODO: determine if this value is a reasonable guess
    Cd0 = parastic_drag()

    ## if TakeoffModel used:
    #     # Takeoff Model values
    #     T_W_takeoff, W_S, W, P_shaft_TO, CLTO, CDTO, CD0 = [10,10,10,10,10,10,10]
    #     takeoff_cls = Takeoff_model(
    #         T_W_takeoff, W_S, W, P_shaft_TO, CLTO, CDTO, CD0, AR=AR, e=e
    #     )  # TODO: parameters need to be defined somewhere
    #     s_ref = takeoff_cls.S
    #     weight = takeoff_cls.weight  # TODO: Need this to be implemented in takeoff model; function of time or altitude

    #     # End of takeoff (eot) parameters; TODO: Need this to be implemented in takeoff model
    #     v_cruise = takeoff_cls.v_eot  # assumed constant
    #     h_in = takeoff_cls.h_eot  # NOTE: Ensure that term is in meters
    #     h_end = 10000 * ureg("ft").to_base_units().magnitude
    #     h_cruise = np.linspace(h_in, h_end) if h_end > h_in else np.array(h_in)
    #     thrust = takeoff_cls.get_T_cruise(v_cruise)

    #     cruise_cls = CruiseModel(
    #         s_ref,
    #         weight,
    #         v_cruise,
    #         h_cruise,
    #         AR=AR,
    #         e=e,
    #         thrust=thrust,
    #         Cd0=Cd0
    #     )

    # TODO: Update with actual values by calling relevant class
    cruise_cls = CruiseModel(
        s_ref=10,
        weight=1000,
        v_cruise=100,
        h_cruise=3000,
        AR=AR,
        e=e,
        thrust=5000,
        Cd0=Cd0,
    )

    CD_total = cruise_cls.cd_total()
    L_over_D = cruise_cls.cl() / CD_total

    PLOT = True
    if PLOT:
        # plt.figure()
        # plt.plot(AR, cruise_cls.cd_parasitic(), color="red", label='CD_par')
        # plt.plot(AR, cruise_cls.cd_induced(), color="blue", label='CD_ind')
        # plt.plot(AR, cruise_cls.cd_total(), color="black", label='CD_tot')
        # plt.title("CD vs AR")
        # plt.xlabel("AR")
        # plt.ylabel("CD")
        # plt.legend()
        # plt.grid()
        # plt.show()

        plt.figure()
        plt.plot(AR, L_over_D)
        plt.xlabel("Aspect Ratio")
        plt.ylabel("L/D")
        plt.title("Cruise L/D vs AR")
        plt.grid()
        plt.show()
