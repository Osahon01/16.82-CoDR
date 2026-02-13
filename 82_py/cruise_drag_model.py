import numpy as np
import matplotlib.pyplot as plt
from ambiance import Atmosphere

# Constants
pi = np.pi
AR = 9.7
e = 0.7
CD0 = 0.032

# Aircraft parameters (Caravan-like)
W = 39000        # N
S = 26           # m^2
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

Di = q * S * CDi
Dp = q * S * CD0

D_total = Di + Dp

# Plot
plt.figure(figsize=(8,6))
plt.plot(V, Di, label="Induced Drag")
plt.plot(V, Dp, label="Parasite Drag")
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

