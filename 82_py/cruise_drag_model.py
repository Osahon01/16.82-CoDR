import numpy as np
import matplotlib.pyplot as plt
from ambiance import Atmosphere

# (Assumed) Constants
pi = np.pi
AR = np.linspace(5, 15, 500)  # TODO: desired AR sweep range
e = 0.7

# Aircraft parameters (Caravan-like)
W = 39000  # N
S = 25.96  # m^2
h = 3000  # meters (~10,000 ft)

# Density as a function of atmosphere
atm = Atmosphere(h)
rho = atm.density[0]

# Velocity sweep (m/s)
# NOTE: Let's be consistent with cruise velocity calls across scripts
V_CRUISE_VEC = np.linspace(
    40, 120, 500
)  # Note: Cessna Caravan has cruise speed of about 96 m/s
q = 0.5 * rho * V_CRUISE_VEC**2
CL = W / (q * S)  # Required CL for level flight


# Caravan Parastic Drag Calc
def parastic_drag():
    S_caravan = 25.96
    AR_caravan = 9.702
    P_caravan = 503e3  # W
    eta_prop = 0.8  # assumed
    V_cruise_caravan = 96  # m/s
    D_caravan = P_caravan * eta_prop / V_cruise_caravan
    q_caravan = 0.5 * rho * V_cruise_caravan**2

    Cl_total_caravan = 35600 / (q_caravan * S_caravan)
    Cd_total_caravan = D_caravan / (q_caravan * S_caravan)
    CD_i_caravan = Cl_total_caravan**2 / (pi * AR_caravan * e)
    CD0 = Cd_total_caravan - CD_i_caravan
    return CD0


CD0 = parastic_drag()
# print(CD0)


# Our Drag components
CDi = CL**2 / (pi*AR*e)
D_i = q * S * CDi
D_p = q * S * CD0
D_total = D_i + D_p


# Plot
PLOT = False
if PLOT:
    plt.figure(figsize=(8, 6))
    plt.plot(V_CRUISE_VEC, D_i, label="Induced Drag")
    plt.plot(V_CRUISE_VEC, D_p, label="Parasite Drag")
    plt.plot(V_CRUISE_VEC, D_total, label="Total Drag", linewidth=2)
    plt.xlabel("Velocity (m/s)")
    plt.ylabel("Drag (N)")
    plt.title("Drag vs Velocity")
    plt.legend()
    plt.grid()
    plt.show()


min_index = np.argmin(D_total)


# Find minimum drag speed
def v_opt():
    return round(V_CRUISE_VEC[min_index], 2)


def AR_opt():
    return round(AR[min_index], 2)


print(f"{40 * '='}\nVelocity at Minimum Cruise Drag: {v_opt()}, m/s\n{40 * '='}\n")
print(f"{40 * '='}\nAR at Minimum Cruise Drag: {AR_opt()}\n{40 * '='}")
