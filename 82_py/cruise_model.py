"""Simple Model for cruise. NOTE: May need to install package(s) using pip"""

import numpy as np
import matplotlib.pyplot as plt
from ambiance import Atmosphere
from pint import UnitRegistry
from power_gen import AircraftConfig, DEPSizingModel
from cruise_drag_model import parastic_drag
from CoDR_equations import V_CRUISE

# NOTE: Once takeoff code is published, I will read in inputs and update lift model accordingly

ureg = UnitRegistry()


class CruiseModel:
    def __init__(
        self, s_ref, weight, v_cruise, h_cruise, aoa, e, thrust, Cd0, AR
    ) -> None:
        self.atm = Atmosphere(h=h_cruise)
        self.s_ref = s_ref
        self.weight = weight  # newtons
        self.v_cruise = v_cruise
        self.h_cruise = h_cruise
        self.density = self.atm.density
        self.aoa = aoa  # radians
        self.e = e
        self.q = 0.5 * self.density * (self.v_cruise**2)
        self.thrust = thrust
        self.Cd0 = Cd0
        self.AR = AR

    def cl(self):
        L = self.weight * np.cos(
            self.aoa
        )  # assumes I have weight as a function of time
        return L / (self.q * self.s_ref)

    # def cd_induced(self):
    #     # see eqn: https://www.grc.nasa.gov/www/k-12/VirtualAero/BottleRocket/airplane/induced.html
    #     return (self.cl() ** 2) / (np.pi * self.aspect_ratio * self.e)
    def cd_induced(self):
        return (self.cl() ** 2) / (np.pi * self.AR * self.e)

    # TODO: determine whether this number should be estimated by cruise or takeoff teams
    def cd_parasitic(self):
        # Assume gamma=0, and T=D
        ## cd_parasitic = cd_form + cd_skin_friction
        # return self.thrust / (self.q * self.s_ref)
        return self.Cd0

    def cd_total(self):
        # Current model will not account for viscous drag (not significant in cd calculations)
        return self.cd_induced() + self.cd_parasitic()

    # TODO: Do we want cruise shaft power?


# Runner script
if __name__ == "__main__":
    # Design Variables
    AR = np.linspace(5, 15, V_CRUISE_VEC.shape[0])  # TODO: desired AR sweep range
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
        Cd0=CD0,
    )

    PLOT = False
    if PLOT:
        plt.figure()
        # plt.plot(aspect_ratio, cruise_cls.cd_parasitic(), color="red")
        plt.plot(aspect_ratio, cruise_cls.cd_induced(), color="blue")
        # plt.plot(aspect_ratio, cruise_cls.cd_total(), color="black")
        plt.title("CD vs AR")
        plt.xlabel("AR")
        plt.ylabel("CD")
        plt.legend()
        plt.grid()
        plt.show()

    print(f"{50 * '='}\nBest AR: {np.round(min(cruise_cls.cd_total(), 2))}\n{50 * '='}")
