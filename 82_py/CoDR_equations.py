import math as m
import numpy as np
import matplotlib.pyplot as plt
from pint import UnitRegistry

# NOTE: You may need to install pyommo anad pint using pip

# Constants, see syllabus for constraints
ureg = UnitRegistry()  # keeps track of units
g = 9.81 * ureg("m/s^2").magnitude
N = 9  # 7-9 passengers
PAY = 100 * ureg("kg").magnitude  # per passenger
RANGE = 1500 * ureg("miles").to("m").magnitude
V_CRUISE = 125 * ureg("m/s").magnitude
X_TAKEOFF = 300 * ureg("ft").to("m").magnitude  # max takeoff distance; can be lower
# PAY_VOL = keep similar to those of similar-sized  aircrafts
