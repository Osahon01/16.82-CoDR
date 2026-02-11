"""Simple Model for cruise."""

import numpy as np
from ambiance import Atmosphere
from pint import UnitRegistry

# NOTE: May need to install package using pip

# NOTE: Once takeoff code is published, I will read in inputs and update lift model accordingly
# Biruk will add CD tradeoff study and conduct Cessna studies to calibrate a CD model (induced and parasitic)

ureg = UnitRegistry()

# Design Variables
h_end = 10000 * ureg("ft").to_base_units()
gamma_fn = lambda t: (h_end - h_in) / t  # assuming I know time from Zach's power model

atm = Atmosphere(
    h=...
)  # should be in meters; ensure that you use SI units for this using ureg
rho = atm.density
q = 0.5 * rho * (v**2)  # ASSUMPTION: cruise velocity is constant; relies
L = W_fn * np.sin(gamma_fn)  # assumes I have weight as a function of time
CL = L / (q * S)

# We want cruise shaft power
