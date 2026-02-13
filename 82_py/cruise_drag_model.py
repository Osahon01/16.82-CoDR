import numpy as np
import matplotlib.pyplot as plt
from ambiance import Atmosphere

# TODO: find parastic drag through wind tunnel test data 

# Constants
pi = np.pi
AR = 9.702
e = 0.7
CD0 = 0.032

# Aircraft parameters (Caravan-like)
W = 39000        # N
S = 25.96        # m^2
h = 3000         # meters (~10,000 ft)

# Atmosphere
atm = Atmosphere(h)
rho = atm.density[0]

# Velocity sweep (m/s)
V = np.linspace(40, 120, 500) #Note: Cessna Caravan has cruise speed of about 96 m/s

q = 0.5 * rho * V**2

# Required CL for level flight
CL = W / (q * S)

# Drag components
CDi = CL**2 / (pi * AR * e)

D_i = q * S * CDi
D_p = q * S * CD0

D_total = D_i + D_p

# Plot
plt.figure(figsize=(8,6))
plt.plot(V, D_i, label="Induced Drag")
plt.plot(V, D_p, label="Parasite Drag")
plt.plot(V, D_total, label="Total Drag", linewidth=2)
plt.xlabel("Velocity (m/s)")
plt.ylabel("Drag (N)")
plt.title("Drag vs Velocity")
plt.legend()
plt.grid()
plt.show()

# Find minimum drag speed
min_index = np.argmin(D_total)
print("Velocity at Minimum Drag:", V[min_index], "m/s")

